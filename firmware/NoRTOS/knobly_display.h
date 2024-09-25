#include <Wire.h>
#include "avrc2560_core.h"

#define DSP_SWITCHES_INT 2
#define ENC_PC_PORT 2


#define ENABLE_SWITCHES_INT (EIMSK |= (1<<DSP_SWITCHES_INT))
#define ENABLE_ENC_INTS (PCICR |= (1<<ENC_PC_PORT))

#define DISABLE_SWITCHES_INT (EIMSK &= ~(1<<DSP_SWITCHES_INT))
#define DISABLE_ENC_INTS (PCICR &= ~(1<<ENC_PC_PORT))

#define DSKY_LED_RAM_BASE 0x00
#define DSKY_KEY_RAM_BASE 0x40

#define INTERFACE_BUFFER_SIZE 64

#define KEYBYTES 6  //must be 6 to read all keys and clear int bit
#define CHARSET_SIZE 86

//    x   x   x   x
//      x   x   x
//ledBitBuffer[*_FLAGS] bit
//    4   2   1   0
//      3   5   6
#define RED_FLAGS 13
#define GREEN_FLAGS 15

#define LED_STAT_BASE 12
#define DOWNBNK_BASE 2
#define UPBNK_BASE 0
#define ENTERKEY_BASE 1
#define ENCODERKEYS_BASE 4

#define LED_BIT_BUF_SIZE 16
#define ASCII_NULL 0
#define ASCII_0 48
#define ASCII_9 57
#define ASCII_A 65
#define ASCII_F 70// Wire Master Writer

#define ASCII_Z 90
#define ASCII_HEX_SKIP (ASCII_A - ASCII_9 - 1)

enum ascii_bound{
  ascii_null =  ASCII_NULL,
  ascii_0 = ASCII_0,
  decimal = ASCII_9,
  ascii_A = ASCII_A,
  hex = ASCII_F,
  alpha = ASCII_Z
};

byte add(byte a,byte b){ return a+b;}
byte sub(byte a,byte b){ return a-b;}

unsigned char reverse(unsigned char b) {
   b = (b & 0b11110000) >> 4 | (b & 0b00001111) << 4;
   b = (b & 0b11001100) >> 2 | (b & 0b00110011) << 2;
   b = (b & 0b10101010) >> 1 | (b & 0b01010101) << 1;
   return b;
}
unsigned char layoutBugFix(unsigned char b) {

  //get middle 2 bits
  byte mask = b & 0b00011000;
  //remove middle 2 bits
  b^=mask;
  //reverse mask 
  mask = reverse(mask);
  //reset middle 2 bits
  return b^=mask;
}

class KnoblyDisplay {
public:
  KnoblyDisplay(byte i_address, HardwareSerial* Serialx,TwoWire* theWire = &Wire) {
    address = i_address;
    _wire = theWire;
    _wire->begin();
    _serial= Serialx;
  }

  ~KnoblyDisplay() {
    _wire->end();
  }

  void init() {
     //enable read from encoder port
     DDRJ = 0x00;
     PORTJ = 0x00;
     DDRK = 0x00;
     PORTK = 0x00;
     //enable pinchange interupt on all of portj (PC8:15)
     PCICR = 0b00000100;
     PCMSK2 = 0xFF;
     
    //init device
    delay(20);  //delay at least one "frame time" for key switches to stablize
    getSwitchStatus();

     _wire->beginTransmission(address);
    _wire->write(0x21); //System Setup Register: Turn on System oscillator
    _wire->endTransmission();
    _wire->beginTransmission(address);
    _wire->write(0x81); //Display Setup Register: Turn on display, no blinking
    _wire->endTransmission();
    _wire->beginTransmission(address);
    _wire->write(0xA1); //ROW/INT Set Register : int enable, active low'
    _wire->endTransmission();
    _wire->beginTransmission(address);
    //_wire->write(0b11100000); //dimming command: min 16 lvls between min and max
      _wire->write(0b11100100);
    //_wire->write(0b11101111); //dimming command: Max
    _wire->endTransmission();

    getSwitchStatus();
    clearFrame();
    delay(20);  //delay at least one "frame time" for key switches to stablize

  }

  void sendFrame() {

      _wire->beginTransmission(address);
      _wire->write(DSKY_LED_RAM_BASE);
      #pragma unroll
      for (word i = 0; i < LED_BIT_BUF_SIZE; i++) { //send digit frame buffer
        _wire->write(ledBitBuffer[i]);
        //_wire->write(0xff);
      }
      _wire->endTransmission();

  }

  void clearFrame() {
      _wire->beginTransmission(address);
      _wire->write(DSKY_LED_RAM_BASE);
      #pragma unroll
      for (word i = 0; i < LED_BIT_BUF_SIZE; i++) {
        _wire->write(0x00);
      }
      _wire->endTransmission();
  }

  int getSwitchStatus() {
    int j = 0;
    risingStateChange = false;
      
    _wire->beginTransmission(address);  // transmit to device #0
    _wire->write(DSKY_KEY_RAM_BASE);
    _wire->endTransmission();  // stop transmitting
    
    _wire->requestFrom(address, KEYBYTES, true);
    // client may send less than requested
    while (_wire->available()) {
      //reverse byte to match LED layout on display board
      next[j] = reverse(_wire->read());  // Receive a byte as character
      j++;
    }

   // /*
    for (int i = 0;i<j;i++){
      _serial->print(next[i]);
      _serial->print(",");
    }
    _serial->println();
     //L: 1 <-> 128 :R
     //8 switch bank down on byte 0
     //8 switch bank up on byte 2
     //enter key on byte 1
     //encoder buttons on byte 4 (16,32,64,128)
   //  _serial->print("downbank:");
   //  _serial->println(*downbank);
   // */


    //set bits from switches
    ledBitBuffer[LED_STAT_BASE+DOWNBNK_BASE] |= *downbank;
    ledBitBuffer[LED_STAT_BASE+UPBNK_BASE] |= *upbank;

    //turn off any identical switch bits
    byte identical = ledBitBuffer[LED_STAT_BASE+DOWNBNK_BASE] & ledBitBuffer[LED_STAT_BASE+UPBNK_BASE];
    ledBitBuffer[LED_STAT_BASE+DOWNBNK_BASE]^= identical;
    ledBitBuffer[LED_STAT_BASE+UPBNK_BASE] ^= identical;



   //extract knob keys
   byte e = reverse(*encoderbtns);
     _serial->print("encoder btn:");
     _serial->println(e);
    ledBitBuffer[GREEN_FLAGS] = layoutBugFix(e);




    //clear up/down banks on enter
    if(*enter){
      ledBitBuffer[LED_STAT_BASE+DOWNBNK_BASE] = 0;
      ledBitBuffer[LED_STAT_BASE+UPBNK_BASE] = 0;
      ledBitBuffer[GREEN_FLAGS] = 0;
    }
    return 1; 
  }

  void  byte2string(byte val, char* str, byte len) {
    byte devisor = 1;
    byte clean_val = val;
    switch(len){
      case 2:
      devisor = 0x10;
      break;
      case 3:
      devisor = 10;
      break;
      default:
      len = 1;
      break;
    }
    for (byte i=len; i>0; --i ){
      byte idx = i-1;
      str[idx] = (clean_val % devisor);
      clean_val =  (clean_val - str[idx])/devisor;
      str[idx] += ASCII_0;
      str[idx] += ((len ==2)&& str[idx] >ASCII_9 && str[idx] <ASCII_A) ? ASCII_HEX_SKIP : 0;
    }
  }

  void setXDigits(char* str, byte count, byte starting) {
    for (byte i = starting; i < starting+count; i++) {
      setDigit(i, str[i]);
    }
  }

  void setDigit(byte digit, byte ch) {
    byte ascii_char = ch - ASCII_0;
    digitsDisplayState[digit] = ch;
    ledBitBuffer[digit*2] = alphanum_charset[2*ascii_char];
    ledBitBuffer[(digit * 2) + 1] = alphanum_charset[(2*ascii_char)+1];
  }

  //generalized increment/decrement digit, with max value of <rollover>, using add/sub function pointer
  bool inc_decDigit(byte digit, ascii_bound rollover, byte (*op)(const byte a, const byte b )) {
    byte nextChar = tableGapSkip(  op(digitsDisplayState[digit], 1) % (rollover+1), op ,rollover);
    bool ret = (op == &add)? (nextChar<digitsDisplayState[digit]): (nextChar>digitsDisplayState[digit]);
    setDigit(digit, nextChar);
    return ret;
  }
  //generilized increment/decrement of entire display 
  void inc_decDisplay(ascii_bound rollover, byte (*op)(const byte a, const byte b )){
    bool carry;
    byte i = 5;
    do{
      carry = inc_decDigit(i, rollover, op);
      i--;
      
    }while(carry&&i<5);
  }
  
  byte encoder_read(){
    _serial->println("knob turned:");
    _serial->println(PINK,BIN);
    *enter = PINK;
    byte pa = PINK;
    int v;
    int out;
    for (byte i=0;i<4;i++){
      v = 0b00000011 & (pa >>(i*2));
      out = QEM[enc_state[i] * 4 + v]; //out holds in the +-1 value of rotation
      enc_state[i] = v ;
      switch(out){
        case 0:
          break;
        case 1:     
          inc_decDigit(i+2, hex,&add);
          break;
        case -1:
          inc_decDigit(i+2, hex,&sub);
          break;
        default:
          break;
      }
    }
        return 0;

  }


  void rxFromModule(){
    //called from module code
    //deposit data in the input buffer
    //set "new data flag"
  }

  void txToModule(){
    //called from display code
    
  }


  byte kd_output_buffer[INTERFACE_BUFFER_SIZE] = {0};
  byte kd_input_buffer[INTERFACE_BUFFER_SIZE] = {0};
  byte* kd_input_read_ptr = kd_input_buffer;
  byte* kd_input_write_ptr = kd_input_buffer;

  byte* kd_output_read_ptr = kd_output_buffer;
  byte* kd_output_write_ptr = kd_output_buffer;

  byte switchBufferA[KEYBYTES] = { 0 };
  byte switchBufferB[KEYBYTES] = { 0 };
  byte* next = switchBufferA;
  byte* prior = switchBufferB;
  byte* downbank = next+DOWNBNK_BASE;
  byte* upbank = next+UPBNK_BASE;
  byte* enter = next+ENTERKEY_BASE;
  byte* encoderbtns = next+ENCODERKEYS_BASE;
  byte digitsDisplayState[6] = { 0 };  //store the ascii value of 14 seg display elements
  byte ledBitBuffer[LED_BIT_BUF_SIZE] = { 0 }; //byte strings for display, first 12 bytes store 14segment display bits, last 4 store LEDs
  int QEM [16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Quadrature Encoder Matrix
  int enc_state[4]={0,0,0,0};
  byte address;
  bool risingStateChange=false;
  TwoWire* _wire;
  Stream* _serial;

private:

  byte alphanum_charset[CHARSET_SIZE] = {
  0b00111111,0b010001,    //0
  0b110,0b1,              //1
  0b11011,0b100010,       //2
  0b1111,0b10,            //3
  0b000100110,0b100010,   //4
  0b000101101,0b100010,   //5
  0b000111101,0b100010,   //6
  0b00000001,0b10001,     //7
  0b111111,0b100010,      //8
  0b100111,0b100010,      //9
  0b0, 0b0,               //unsupported ascii gap
  0b0, 0b0,
  0b0, 0b0,
  0b0, 0b0,
  0b0, 0b0,
  0b0, 0b0,
  0b0, 0b0,
  0b110111,0b100010,      //A
  0b10001111,0b1010,      //B
  0b111001,0b0,           //C
  0b10001111,0b1000,      //D
  0b111001,0b0100000,     //E
  0b110001,0b0100000,     //F
  0b111101,0b10,          //G
  0b110110,0b100010,      //H
  0b10001001,0b1000,      //I
  0b00011110, 0b0,        //J
  0b00110000,0b100101,    //K
  0b111000,0b0,           //L
  0b01110110,0b1,         //M
  0b01110110,0b100,       //N
  0b00111111,0b0,         //0
  0b110011,0b100010,      //P
  0b00111111,0b100,       //Q
  0b110011,0b100110,      //R
  0b000101101,0b100010,   //S
  0b10000001,0b1000,      //T
  0b00111110,0b0,         //U
  0b110000,0b10001,       //V
  0b110110,0b10100,       //W
  0b01000000,0b010101,    //X
  0b01000000,0b001001,    //X
  0b1001,0b10001,         //Z
  };

  char tableGapSkip(byte ch, byte (*op)(const byte a, const byte b ), ascii_bound rollover){
    char ret =ch;
    if(ch>=ASCII_NULL && ch<ASCII_0){
      ret = (op == &add)? ASCII_0 : rollover;
    }
    if(ch>ASCII_9 && ch<ASCII_A){
      ret = (op == &add)? ASCII_A : ASCII_9;
    }
    return ret;
  }

};

#ifndef KNOBLY_DISPLAY_H
#define KNOBLY_DISPLAY_H
#include <Wire.h>
#include "avrc2560_core.h"
#include "DigitEnums.h"
#include "DigitDisplay.h"
#include "Charset.h"


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

    //create digit display opjects
    for (byte i = 0; i<6; ++i)
    {    
      digits[i] = new DigitDisplay(ledBitBuffer + (2*i));
    }
  }

  ~KnoblyDisplay() {
    _wire->end();
    for (byte i = 0; i<6; ++i)
    {    
      free(digits[i]);
    }
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

    enc_state[0]=0;
    enc_state[1]=0;  
    enc_state[2]=0;  
    enc_state[3]=0;  

    //reset buffer   
    for (word i = 0; i < LED_BIT_BUF_SIZE; i++) {
        ledBitBuffer[i]=0;
    }
    sendFrame();
  }

  void sendFrame() {
      for(byte i=0; i<6;++i){
        digits[i]->populateDisplay();
      }
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

    /*
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
    */


    //set bits from switches
    ledBitBuffer[LED_STAT_BASE+DOWNBNK_BASE] |= *downbank;
    ledBitBuffer[LED_STAT_BASE+UPBNK_BASE] |= *upbank;

    //turn off any identical switch bits
    byte identical = ledBitBuffer[LED_STAT_BASE+DOWNBNK_BASE] & ledBitBuffer[LED_STAT_BASE+UPBNK_BASE];
    ledBitBuffer[LED_STAT_BASE+DOWNBNK_BASE]^= identical;
    ledBitBuffer[LED_STAT_BASE+UPBNK_BASE] ^= identical;



   //extract knob keys
   byte e = reverse(*encoderbtns);
   //_serial->print("encoder btn:");
   //_serial->println(e);
   raw_e |= e;
   ledBitBuffer[GREEN_FLAGS] |= layoutBugFix(e);


    //compute LED checksum, if anything has been lit since last enter (where it is cleared)
    byte checksum = 0;
    for (byte i=0; i<4;++i){
      checksum |= (0 ^ ledBitBuffer[LED_STAT_BASE+i]);
    }
    
    //only enter if led status has changed
    if(*enter && checksum!=0){

      //ignore falling edge
      if (enterKeyFirstEdge){
        enterKeyFirstEdge =false;
          
      }
      //do work on rising edge
      else{
        //copy enc en/dis to enc_active_mask
        for (byte i = 0; i< 4; ++i){
            //if ist bit of raw_e is toggle the ith digit
            if ((raw_e & (1<<i))!=0 ){
              digits[i]->togglePower();
            }
        }
        //clear led buffer
        ledBitBuffer[LED_STAT_BASE+DOWNBNK_BASE] = 0;
        ledBitBuffer[LED_STAT_BASE+UPBNK_BASE] = 0;
        ledBitBuffer[GREEN_FLAGS] = 0;
        enterKeyFirstEdge =true;
      }
    }
    return 1; 
  }


  void setXDigits(char* str, byte count, byte starting) {
    for (byte i = starting; i < starting+count; i++) {
      setDigit(i, str[i]);
    }
  }

  void setDigit(byte digit, byte ch) {
    digits[digit]->setChar(ch);
  }

  //generalized increment/decrement digit, with max value of <rollover>, using add/sub function pointer
  bool inc_decDigit(byte digit, digitOp op) {
            _serial->println(op);
    return digits[digit]->incdec(op);
  }
  //generilized increment/decrement of entire display 
  void inc_decDisplay(digitOp op){
    bool carry;
    byte i = 5;
    do{
      carry = inc_decDigit(i,op);
      i--;
      
    }while(carry&&i<5);
  }
  
  byte encoder_read(){
        _serial->println("-----------------");
    _serial->println("knob turned:");
    _serial->println(PINK,BIN);
    byte pa = reverse(PINK);
    int v;
    int out;
    for (byte i=0;i<4;i++){
      v = 0b00000011 & (pa >>(i*2));
      out = QEM[enc_state[i] * 4 + v]; //out holds in the +-1 value of rotation
      enc_state[i] = v ;
      switch(out){
        case 1:     
          inc_decDigit(i+2,ADD);
          break;
        case -1:
          inc_decDigit(i+2, SUB);
          break;
        case 0:         
        default:
          break;
      }
    }
            _serial->println("-----------------");
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
  byte ledBitBuffer[LED_BIT_BUF_SIZE] = { 0 }; //byte strings for display, first 12 bytes store 14segment display bits, last 4 store LEDs
  byte switchCheckpointBitBuffer[4] = { 0 }; //store switch bits on enter key. only accept another enter key if these have changed in some way
  int QEM [16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Quadrature Encoder Matrix
  int enc_state[4]={0,0,0,0};
  byte raw_e=0;
  bool enterKeyFirstEdge = true;
  byte address;
  TwoWire* _wire;
  Stream* _serial;
  DigitDisplay* digits[6];


private:

};

#endif//KNOBLY_DISPLAY_H

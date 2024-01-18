

//cmd line for flashing a fresh board
//avrdude.exe "-CC:\Users\tompl\AppData\Local\Arduino15\packages\arduino\tools\avrdude\6.3.0-arduino17/etc/avrdude.conf" -v -V -patmega2560 -cstk500v2 -Pusb -U lfuse:w:0xff:m -U hfuse:w:0x98:m -U efuse:w:0xff:m 

//#include <Arduino_FreeRTOS.h>
#include <Wire.h>
#include <avr/interrupt.h>
#include <avr/io.h>

#define SRCLK 7   //PH4 = arduino7
#define SRDATA 6  //PH3 =arduino6
//measure pb0, pin 19 of compairitor
#define DSKY_LED_RAM_BASE 0x00
#define DSKY_KEY_RAM_BASE 0x40
#define KEYBYTES 6
#define DEBUG 1
#define CHARSET_SIZE 86
#define LED_STAT_BASE 12
#define LED_BIT_BUF_SIZE 16
#define ASCII_NULL 0
#define ASCII_0 48
#define ASCII_9 57
#define ASCII_A 65
#define ASCII_F 70// Wire Master Writer

#define ASCII_Z 90
#define ASCII_HEX_SKIP (ASCII_A - ASCII_9 - 1)

//ISR related global data 
volatile byte direction = 0;
volatile byte indata = 0; //data recived from host
volatile byte outData = 0; // data staged from sending to host 
volatile byte err = 0;
volatile bool hostIOOccured = false;
volatile bool dskyIOOccured = false;

int QEM [16] = {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0}; // Quadrature Encoder Matrix
int enc_state[4]={0,0,0,0};


enum ascii_bound{
  ascii_null =  ASCII_NULL,
  ascii_0 = ASCII_0,
  decimal = ASCII_9,
  ascii_A = ASCII_A,
  hex = ASCII_F,
  alpha = ASCII_Z
};

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

byte add(byte a,byte b){ return a+b;}
byte sub(byte a,byte b){ return a-b;}

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

void ioaddrSet(byte addr) {
  byte i;
  for (i = 0; i < 8; i++) {
    digitalWrite(SRDATA, 0);
    digitalWrite(SRCLK, HIGH);
    digitalWrite(SRCLK, LOW);
  }
  for (i = 0; i < 8; i++) {
    digitalWrite(SRDATA, (addr & 128) != 0);
    addr <<= 1;
    digitalWrite(SRCLK, HIGH);
    digitalWrite(SRCLK, LOW);
  }
  digitalWrite(SRCLK, HIGH);
  digitalWrite(SRCLK, LOW);
}

class DSKY {
public:
  DSKY(byte i_address, TwoWire* theWire = &Wire) {
    address = i_address;
    _wire = theWire;
    _wire->begin();
  }
  ~DSKY() {
    _wire->end();
  }
  void init() {
    //init device
    Serial2.println("");
    Serial2.println("init dsky");
    Serial2.println(address, HEX);
    
    delay(20);  //delay at least one "frame time" for key switches to stablize
    getSwitchStatus();

    _wire->beginTransmission(address);
    _wire->write(0x21); //System Setup Register: Turn on System oscillator
    _wire->endTransmission();
    _wire->beginTransmission(address);
    _wire->write(0x81); //Display Setup Register: Turn on display, no blinking
    _wire->endTransmission();
    _wire->beginTransmission(address);
    _wire->write(0xA1); //ROW/INT Set Register : int enable, active low
    _wire->endTransmission();
    _wire->beginTransmission(address);
    _wire->write(0b11100000); //dimming command: min 16 lvls between min and max
    //_wire->write(0b11101111); //dimming command: Max
    _wire->endTransmission();

    clearFrame();
    //setXDigits("101010",6,0);
    sendFrame();
  }
  void sendFrame() {
    _wire->beginTransmission(address);
    _wire->write(DSKY_LED_RAM_BASE);
    for (word i = 0; i < LED_BIT_BUF_SIZE; i++) { //send digit frame buffer
      _wire->write(ledBitBuffer[i]);
      //_wire->write(0xff);
    }
    _wire->endTransmission();
  }
  void clearFrame() {
    _wire->beginTransmission(address);
    _wire->write(DSKY_LED_RAM_BASE);
    for (word i = 0; i < LED_BIT_BUF_SIZE; i++) {
      _wire->write(0x00);
    }
    _wire->endTransmission();
  }

  void getSwitchStatus() {
    int j = 0;
    risingStateChange = false;
    _wire->beginTransmission(address);  // transmit to device #0
    _wire->write(DSKY_KEY_RAM_BASE);
    _wire->endTransmission();  // stop transmitting
    _wire->requestFrom(address, KEYBYTES, true);
    // client may send less than requested
    //Serial2.println("SWbankUp,X,SWbankDwn,X,Enc1,Enc2");
    while (_wire->available()) {
      next[j] = _wire->read();  // Receive a byte as character
     // Serial2.print(next[j]);
      //Serial2.print(",");
      j++;
    }
    //Serial2.println();
    return;
    for (byte i = 0;i<KEYBYTES;++i){
      int delta = next[i]^prior[i] ;
      if(delta ==0){
        continue;
      }
      else if(delta >0 &&next[i]>prior[i]){
          prior[i]= next[i];
          ledBitBuffer[LED_STAT_BASE+i] ^= prior[i]; // toggle leds to go with state change
          risingStateChange = true;
      }
      else {
          prior[i]= next[i];
      }
    }
    /*
    for(byte i=0;i<LED_BIT_BUF_SIZE;i++){
      Serial2.print(ledBitBuffer[i],HEX);
      Serial2.print(",");
    }
    Serial2.println();
    */
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
  
  //void setSwitchStatusLed(byte switchId, int status){
  //ledBitBuffer[12] = 0xff; //bank 0 a-f
  //ledBitBuffer[13] =0xff; //bank 0 g-p
  //ledBitBuffer[14]=0xff; //bank 1 a-f
  //ledBitBuffer[15]=0xff;////bank 0 g-p
  //  }

  byte switchBufferA[KEYBYTES] = { 0 };
  byte switchBufferB[KEYBYTES] = { 0 };
  byte* next = switchBufferA;
  byte* prior = switchBufferB;
  byte digitsDisplayState[6] = { 0 };  //store the ascii value of 14 seg display elements
  byte ledBitBuffer[LED_BIT_BUF_SIZE] = { 0 }; //byte strings for display, first 12 bytes store 14segment display bits, last 4 store LEDs 
  byte address;
  bool risingStateChange=false;
  TwoWire* _wire;
};

/*
// these 2 functions will be removed in next revision of board
byte unhashdata(byte d) {
  byte fivetoone = (d & 0b11111000) >> 2;
  byte sevensix = (d & 0b00000110) << 5;
  byte zero = (d & 0b00000001);
  return (fivetoone | sevensix | zero);
}
byte hashdata(byte d) {
  byte fivetoone = (d & 0b00111110) << 2;
  byte sevensix = (d & 0b11000000) >> 5;
  byte zero = (d & 0b00000001);
  return (fivetoone | sevensix | zero);
}
*/

byte encoder_read(){
  byte pa = PINA;
  int v;
  int out;
  for (byte i=0;i<4;i++){
    v = 0b00000011 & (pa >>(i*2));
    out = QEM[enc_state[i] * 4 + v];
    enc_state[i] = v;
    Serial2.print(out);
    Serial2.print(",");
  }
  Serial2.println();
}

ISR(INT3_vect) {
  /*
  Init State:
  shift register Q0-7 contain the address that the avr will listen on, set at boot
  ~{PortIOREquest} pulled up by mcu internal resistor
  ~{IO_add_comp} is held low, flip flop passing ioreq out to wait
  A0-7 + ~{IORQ} are non-matching 

  IO sequence:
  1. cpu brings iorq rd/wt low with matching port address on bus
  2. portIOrequest goes low after matching addr[2:7] with sr[2:7]  and gets latched onto waitline with next clk pulse
  2.5 avr falling edge intterrupt trigger on portIOrequest
  3. avr checks RD and WR lines to determine what request is being made
  
  Timing Notes:
  Single 8bit Read, from iorq going low to wait and busreq release : between 8 and 9 us (58 to 66 clock cycles, 15 to 17 machine cycles )
  Single 8bit Write, from iorq going low to wait release : between 8 and 6.5 us (44 to 48 clock cycles, 11 to 12 machine cycles)

  will need to retime after next revision and removal of hash/unhash functions

  */

  //report IO to rest of AVR
  hostIOOccured = true;


  //read ~WR and ~RD pins
  err = 0;
  byte _WRPin = (PINB & (1 << PB5)) >> PB5;
  byte _RDPin = (PINB & (1 << PB6)) >> PB6;

  if (_WRPin == 0 && _RDPin == 1) {
    /*
      if write:
        1. avr data pins default to input/high Z and collects value
        2. avr tristate decoding logic by brining ~{IO_add_comp} low, wait goes high from pullup on cpu card, latch is set high on next clk pluse
      */
    //read data bus
    indata = PINK;
    // inline asm to make to compiler and timing behave
    asm("sbi %0, %1 \n"
        "nop\n"
        "nop\n"
        "nop\n"
        "nop\n"
        "nop\n"
        "nop\n"
        "nop\n"
        "nop\n"
        "nop\n"
        "cbi %0, %1 \n"
        :
        : "I"(_SFR_IO_ADDR(PORTD)), "I"(PORTD7));
    return;
  } else if (_RDPin == 0 && _WRPin == 1) {
    /*
      if read:
        4. avr sets databus to output , and output low, then  sets value
        5. avr then asserts the busreq line to essentially stall the cpu in tristate after the databus has been read
        6. avr tristate decoding logic by brining ~{IO_add_comp} high , wait goes high from pullup on cpu card, latch is set high on next clk pluse
        7. cpu reads data lines now that wait is clear
        8. cpu sees asserted bus req at begining of last m cycle of read
        9. cpu tristates its pins
        10. avr Poll on busack, proceed after it goes low
        11. avr tristate the databus
        12. avr reenable address decoding by brining ~{IO_add_comp} low
        13. avr release busreq
      */
    //set data lines to output
    DDRK = 0xFF;
    PORTK = 0x00;
    //output true data
    PORTK = outData;
    //set busreq to output
    DDRH = DDRH | (1 << PH6);

    //assert bus req low
    PORTH = PORTH & ~(1 << PH6);
    //avr tristate decoding logic by brining ~{IO_add_comp} high
    asm("sbi %0, %1 \n"
        :
        : "I"(_SFR_IO_ADDR(PORTD)), "I"(PORTD7));
    // wait for cpu to read data, this will be indicated by bus ack going low on pinh5, use avrlibc/io for stibility 
    loop_until_bit_is_clear(PINH, 5);

    //trisatate data bus
    DDRK = 0x00;
    PORTK = 0x00;
    // reanable addr decode
    asm("cbi %0, %1 \n"
        :
        : "I"(_SFR_IO_ADDR(PORTD)), "I"(PORTD7));
    // release busreq and set to high Z
    PORTH = PORTH | (1 << PH6);
    //set busreq to input
    DDRH = DDRH & ~(1 << PH6);
  } else {
    err = 1;
  }
}

//general global data
DSKY* dsky;
byte val = 0;

void setup() {
  //7,6,5,4,3,2,1,0
  // CLK,Rd,WR,MRQ,prog_miso,prog_mosi,prog_sck,m1
  DDRB = 0x00;
  //PD = ~IO_addr_comp, ~IORQ, NMI, INT, ~portIOReq, ~i2c int,SDA,SCL
  DDRD = 0b10000011;
  //PJ = A7 - A0
  DDRJ = 0x00;
  //PL = AF - A8
  DDRL = 0x00;
  //PK = D5,D4,D3,D2,D1,D7,D6,D0 (set to high Z state via Portk =0)
  DDRK = 0x00;
  PORTK = 0x00;
  //~WAIT,BusRQ,BusAck,port_sr_clk,port_sr_data,Halt,uart2_tx,uart2_rx
  DDRH = 0b00011011;
  //X,X,man_clock_toggle,man_clock_en,pg3,pg2,pg1,pg0
  DDRG = 0b00110000;

  ioaddrSet(0b11001000);  //203

  Serial2.begin(9600);    // Start serial2 for output
  delay(1000);
  dsky = new DSKY(0x70);  // Create DSKY display object
  dsky->init();           // Init display
  //set up port IO interrup ctrl,
  EIMSK = 0b00000000;          //disable INT3&INT2 before setting its mode
  EICRA = 0b10100000;          //INT3&INT2 triggers on FALLING EDGE
  EIMSK = 0b00001000;          //enable INT3
  PORTD = PORTD | (0 << PD7);  //force ~IO_addr_comp (pd7) low, enabling wait latching

  // set Port A to input, and enable pull ups
  DDRA = 0x00;
  PORTA = 0xFF;

  //set unused user ports to output
  DDRC = 0xFF;
  DDRE = 0xFF;
  DDRF = 0xFF;
  DDRG = DDRG | 0b00001111;
  PORTA = 0xFF;
  PORTC = 0xFF;
  PORTE = 0xFF;
  PORTF = 0xFF;
  PORTG = PORTG | 0b00001111;
/*
  xTaskCreate(RefreshDSKY, // Task function
              "Refresh", // Task name
              128, // Stack size 
              NULL, 
              1, // Priority
              NULL );
              */
}

void loop() {
  //read inputs
  char temp[3] ={0}; 
  for (byte i =0;i<=255;i++){
  encoder_read();

  //if(dskyIOOccured){
  dsky->getSwitchStatus();
  dskyIOOccured = false;
  //}
  dsky->byte2string(i, temp, 3);
  dsky->setXDigits(temp, 3, 0);
  dsky->sendFrame();
  }
  
  //if(dsky->risingStateChange||hostIOOccured){
  //  //collect data from ISR region is there is any
  //  dsky->byte2string(indata, temp, 3);
  // dsky->setXDigits(temp, 3, 0);
  //dsky->sendFrame();
  //  hostIOOccured = false;
  //}
  
}


#ifndef ACRV2560CORE_H
#define ACRV2560CORE_H

//cmd line for flashing a fresh board
//avrdude.exe "-CC:\Users\tompl\AppData\Local\Arduino15\packages\arduino\tools\avrdude\6.3.0-arduino17/etc/avrdude.conf" -v -V -patmega2560 -cstk500v2 -Pusb -U lfuse:w:0xff:m -U hfuse:w:0x98:m -U efuse:w:0xff:m 

#include <avr/interrupt.h>
#include <avr/io.h>

/**
 * Address Bus (J/L)
 */
#define ADDRLO_DDR DDRJ
#define ADDRLO_PORT PORTJ
#define ADDRLO_PIN PINJ

#define ADDRHI_DDR DDRL
#define ADDRHI_PORT PORTL
#define ADDRHI_PIN PINL

#define GET_ADDRLO ADDRLO_PIN
#define SET_ADDRLO(addr) ADDRLO_PORT = (addr)

#define GET_ADDRHI ADDRHI_PIN
#define SET_ADDRHI(addr) ADDRHI_PORT = (addr)

#define GET_ADDR (GET_ADDRLO | (GET_ADDRHI << 8))
#define SET_ADDR(addr) (SET_ADDRLO((addr) & 0xFF), SET_ADDRHI((addr) >> 8))

#define ADDR_INPUT (ADDRLO_DDR = 0x00, ADDRHI_DDR = 0x00)
#define ADDR_OUTPUT (ADDRLO_DDR = 0xFF, ADDRHI_DDR = 0xFF)
#define ADDR_TSTATE  (ADDR_INPUT, ADDRLO_PORT=0x00,ADDRHI_PORT=0x00)
/**
 * Data Bus (K)
 */
#define DATA_DDR DDRK
#define DATA_PORT PORTK
#define DATA_PIN PINK

#define DATA_INPUT DATA_DDR = 0x00
#define DATA_OUTPUT DATA_DDR = 0xFF
#define DATA_TSTATE (DATA_INPUT, DATA_PORT = 0x00)

#define GET_DATA DATA_PIN
#define SET_DATA(data) DATA_PORT = (data)

/**
 * Port B Map
 */

#define M1 (1 << PINB0)
#define SPI_CLK (1 << PINB1)
#define SPI_MOSI (1 << PINB2)
#define SPI_MISO (1 << PINB3)
#define MREQ (1 << PINB4)
#define WR (1 << PINB5)
#define RD (1 << PINB6)
#define CLK (1 << PINB7)

/**
 * Port B Manipulation
 */

#define GET_M1 (PINB & M1)
#define GET_MREQ (PINB & MREQ)
#define GET_WR (PINB & WR)
#define GET_RD (PINB & RD)
#define GET_CLK (PINB & CLK)

/**
 * Port D Map
 */
#define I2C_SCL  (1 << PIND0)
#define I2C_SDA (1 << PIND1)
#define I2C_INT (1 << PIND2)
#define PORT_IO_REQUEST (1 << PIND3)
#define INT (1 << PIND4)
#define NMI (1 << PIND5)
#define IORQ (1 << PIND6)
#define IO_ADDR_COMP (1 << PIND7)

/**
 * Port D Manipulation
 */
#define GET_PORT_IO_REQUEST (PIND & PORT_IO_REQUEST)
#define GET_INT (PIND & INT)
#define GET_NMI (PIND & NMI)
#define GET_IORQ (PIND & IORQ)

#define IO_ADDR_COMP_HI PIND |= IO_ADDR_COMP
#define IO_ADDR_COMP_LO PIND &= ~IO_ADDR_COMP

/**
 * Port G Map
 */
#define MAN_CLK_EN (1 << PING4)
#define MAN_CLK_TOG (1 << PING5)

#define MAN_CLK_EN_HI PORTG |= MAN_CLK_EN
#define MAN_CLK_EN_LO PORTG &= ~MAN_CLK_EN

#define MAN_CLK_TOG_HI PORTG |= MAN_CLK_TOG
#define MAN_CLK_TOG_LO PORTG &= ~MAN_CLK_TOG

/**
 * Port H Map
 */
#define UART2_RX  (1 << PINH0)
#define UART2_TX (1 << PINH1)
#define HALT (1 << PINH2)
#define PORT_SR_DATA (1 << PINH3)
#define PORT_SR_CLK (1 << PINH4)
#define BUSACK (1 << PINH5)
#define BUSRQ (1 << PINH6)
#define WAIT (1 << PINH7)


/**
 * Port H Manipulation
 */

#define GET_HALT (PINH & HALT)
#define GET_BUSACK (PINH & BUSACK)
#define GET_BUSRQ (PINH & BUSRQ)
#define GET_WAIT  (PINH & WAIT)

#define PORT_SR_DATA_HI PORTH |= PORT_SR_DATA
#define PORT_SR_DATA_LO PORTH &= ~PORT_SR_DATA

#define PORT_SR_CLK_HI PORTH |= PORT_SR_CLK
#define PORT_SR_CLK_LO PORTH &= ~PORT_SR_CLK
#define PORT_SR_CLK_TOG (PORT_SR_CLK_HI, PORT_SR_CLK_LO)


#define BUSRQ_HI PORTH |= BUSRQ
#define BUSRQ_LO PORTH &= ~BUSRQ

/**
 * DDR defaults
 */
// CLK,Rd,WR,MRQ,prog_miso,prog_mosi,prog_sck,m1
#define DDRB_DEFAULT 0x00
//PD = ~IO_addr_comp, ~IORQ, NMI, INT, ~portIOReq, ~i2c int,SDA,SCL
#define DDRD_DEFAULT 0b10000011
//~WAIT,BusRQ,BusAck,port_sr_clk,port_sr_data,Halt,uart2_tx,uart2_rx
#define DDRH_DEFAULT 0b00011011
//X,X,man_clock_toggle,man_clock_en,pg3,pg2,pg1,pg0
#define DDRG_DEFAULT 0b00110000


//ISR related global data 
volatile byte direction = 0;
volatile byte indata = 0; //data recived from host
volatile byte outData = 0; // data staged from sending to host 
volatile byte err = 0;
volatile bool hostIOOccured = false;

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
  byte _WRPin = GET_WR;
  byte _RDPin = GET_RD;

  if (_WRPin != WR && _RDPin == RD) {
    /*
      if write:
        1. avr data pins default to input/high Z and collects value
        2. avr tristate decoding logic by brining ~{IO_add_comp} low, wait goes high from pullup on cpu card, latch is set high on next clk pluse
      */
    //read data bus
    indata = GET_DATA;
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
  } else if (_RDPin != RD && _WRPin == WR) {
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
    DATA_OUTPUT;
    //clear databus
    SET_DATA(0x00);
    //output true data
    SET_DATA(outData);
    //set busreq to output
    DDRH = DDRH | BUSRQ;

    //assert bus req low
    BUSRQ_LO;
    //avr tristate decoding logic by brining ~{IO_add_comp} high
    asm("sbi %0, %1 \n"
        :
        : "I"(_SFR_IO_ADDR(PORTD)), "I"(PORTD7));
    // wait for cpu to read data, this will be indicated by bus ack going low on pinh5, use avrlibc/io for stibility
    loop_until_bit_is_clear(PINH, 5);

    //trisatate data bus
    DATA_TSTATE;
    // reanable addr decode
    asm("cbi %0, %1 \n"
        :
        : "I"(_SFR_IO_ADDR(PORTD)), "I"(PORTD7));
    // release busreq and set to high Z
    PORTH = PORTH | BUSRQ;
    //set busreq to input
    DDRH = DDRH & ~BUSRQ;
  } else {
    err = 1;
  }
}

void ioaddrSet(byte addr) {
  byte i;
  for (i = 0; i < 8; i++) {
    PORT_SR_DATA_LO;
    PORT_SR_CLK_TOG;
  }
  for (i = 0; i < 8; i++) {
    //digitalWrite(SRDATA, (addr & 128) != 0);
    ((addr & 128) != 0) ? PORT_SR_DATA_HI : PORT_SR_DATA_LO;
    addr <<= 1;
    PORT_SR_CLK_TOG;
  }
  PORT_SR_CLK_TOG;
}

void initRCBus(){
  DATA_TSTATE;
  ADDR_TSTATE;
  DDRB = DDRB_DEFAULT;
  DDRD = DDRD_DEFAULT;
  DDRH = DDRH_DEFAULT;
  DDRG = DDRG_DEFAULT;
  ioaddrSet(0b11001000);  //203
}

void enableRCbusinterrupt(){
    //set up port IO interrup ctrl,
  EIMSK = 0b00000000;          //disable INT3&INT2 before setting its mode
  EICRA = 0b10100000;          //INT3&INT2 triggers on FALLING EDGE
  EIMSK = 0b00001100;          //enable INT3
  IO_ADDR_COMP_LO;  //force ~IO_addr_comp (pd7) low, enabling wait latching
}

void initUserPorts(){
  //set unused user ports to output
  //DDRA = 0xFF;
  DDRC = 0xFF;
  DDRE = 0xFF;
  DDRF = 0xFF;
  DDRG = DDRG | 0b00001111;
  PORTA = 0xFF;
  PORTC = 0xFF;
  PORTE = 0xFF;
  PORTF = 0xFF;
  PORTG = PORTG | 0b00001111;
}

#endif /*ACRV2560CORE_H*/

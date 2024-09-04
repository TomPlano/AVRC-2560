
#include <avr/interrupt.h>
#include <avr/io.h>
#include "avrc2560_core.h"
#include "knobly_display.h"

#define KEYINT 1
#define KNOBINT 3

//Global objects
KnoblyDisplay* dsp_ptr;
byte interupt;
byte keyint;
byte knobint;
void setup() {
  Serial2.begin(9600);
  interupt = 0;

  initRCBus();
  initUserPorts();
  dsp_ptr = new KnoblyDisplay(0x70,&Serial2);
  dsp_ptr->init();
  enableRCbusinterrupt();


}

ISR(INT2_vect) {
  EIMSK ^= 0b00000100; 
  interupt =1;
  keyint =1;
}

ISR(PCINT1_vect){
  PCICR ^= 0b00000010;
  interupt =1;
  knobint =1;
}

  


void loop() {
  if(interupt){
    serve_interrupt();
  }  
}


void TaskInput()
{
  //Serial2.println("switch input");
  dsp_ptr->getSwitchStatus();
  dsp_ptr->sendFrame();
}


void TaskEncRead()
{
  Serial2.println("encoder input");
  dsp_ptr->encoder_read();
  dsp_ptr->sendFrame();
}

void serve_interrupt(){
  if(keyint){
    TaskInput(); 
    keyint = 0;
  }
  if(knobint){
    TaskEncRead();
    knobint = 0;
  } 
    interupt =0;
    EIMSK |= 0b00000100;
    PCICR |= 0b00000010;

}

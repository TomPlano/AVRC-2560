
#include <avr/interrupt.h>
#include <avr/io.h>
#include "avrc2560_core.h"
#include "knobly_display.h"


//Global objects
KnoblyDisplay* dsp_ptr;
byte interupt;
byte keyint;
byte knobint;
void setup() {
  Serial2.begin(9600);
  interupt = 0;
  

  initRCBus(false);
  initUserPorts();
  //allHighZ();
  dsp_ptr = new KnoblyDisplay(0x70,&Serial2);
  dsp_ptr->init();
  enableRCbusinterrupt();


}

ISR(INT2_vect) {
  DISABLE_SWITCHES_INT;
  interupt =1;
  keyint =1;
}

ISR(PCINT2_vect){
  DISABLE_ENC_INTS;
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
  Serial2.println("switch input");
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
    interupt = 0;
    ENABLE_SWITCHES_INT;
    ENABLE_ENC_INTS;

}

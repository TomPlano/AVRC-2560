
// Include Arduino FreeRTOS library
#include <Arduino_FreeRTOS.h>
#include "avrc2560_core.h"
#include "knobly_display.h"

/**
 * Task handlers
 * https://www.freertos.org/a00019.html#xTaskHandle
 */

#define NUM_TASKS 3
TaskHandle_t t_input;
TaskHandle_t t_debug;
TaskHandle_t t_encRead;
TaskHandle_t* t_handleList[NUM_TASKS] = {&t_input, &t_debug, &t_encRead};


byte enc_port_int_store;


KnoblyDisplay* dsp;
void setup() {
  Serial2.begin(9600);
  initRCBus();
  enableRCbusinterrupt();
  initUserPorts();
  dsp = new KnoblyDisplay(0x70,&Serial2);
  dsp->init();
  /**
   * Task creation
   */
  
  xTaskCreate(TaskInput, // Task function
              "Input", // Task nam
              1024, // Stack size 
              (void*) dsp, 
              0, // Priority
              &t_input); // Task handler
 
  xTaskCreate(TaskDbgMon,
              "DbgMon",
              1024,
              (void*) dsp, 
              0,
              &t_debug);

  xTaskCreate(TaskEncRead,
              "EncRead",
              1024,
              (void*) dsp,
              0,
              &t_encRead);

}

ISR(INT2_vect) {
  EIMSK ^= 0b00000100;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR( t_input, &xHigherPriorityTaskWoken );
  if (xHigherPriorityTaskWoken) {
    taskYIELD();
  }
}

ISR(PCINT1_vect){
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR( t_encRead, &xHigherPriorityTaskWoken );
  if (xHigherPriorityTaskWoken) {
    taskYIELD();
  }
}

  


void loop() {}



void TaskDbgMon(void *pvParameters)
{
  KnoblyDisplay* dsp_ptr = (KnoblyDisplay*)pvParameters; 
  char temp[6]={ASCII_0,ASCII_0,ASCII_0,ASCII_0,ASCII_0,ASCII_0};
  dsp_ptr->setXDigits(temp, 6, 0);
  for (;;)
  {
    Serial2.println("======== Tasks status ========");
    Serial2.print("Tick count: ");
    Serial2.print(xTaskGetTickCount());
    Serial2.print(", Task count: ");
    Serial2.print(uxTaskGetNumberOfTasks());

    Serial2.println();
    Serial2.println();

    for (int i = 0; i<NUM_TASKS; i++){
      Serial2.print("- TASK ");
      Serial2.print(pcTaskGetName(*t_handleList[i])); // Get task name with handler
      Serial2.print(", High Watermark: ");
      Serial2.print(uxTaskGetStackHighWaterMark(*t_handleList[i]));
      Serial2.println();
    }
    
    Serial2.println();
    dsp_ptr->inc_decDisplay(decimal, add);
    dsp_ptr->sendFrame();
    vTaskDelay( 5000 / portTICK_PERIOD_MS );
  }
}


void TaskInput(void *pvParameters)
{
  KnoblyDisplay* dsp_ptr = (KnoblyDisplay*)pvParameters; 
  for (;;)
  {
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {
      dsp_ptr->getSwitchStatus();
      dsp_ptr->sendFrame();
      EIMSK |= 0b00000100;
    }
  }
}


void TaskEncRead(void *pvParameters)
{
  KnoblyDisplay* dsp_ptr = (KnoblyDisplay*)pvParameters;
  for (;;)
  {
     if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {
      dsp_ptr->encoder_read();
      dsp_ptr->sendFrame();
    }
  }
}

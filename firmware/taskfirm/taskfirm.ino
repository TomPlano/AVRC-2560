
// Include Arduino FreeRTOS library
#include <Arduino_FreeRTOS.h>
#include "avrc2560_core.h"
#include "knobly_display.h"

/**
 * Task handlers
 * https://www.freertos.org/a00019.html#xTaskHandle
 */
TaskHandle_t t_handleList[4];
TaskHandle_t taskDrawHandle;
KnoblyDisplay* dsp;
void setup() {
  Serial2.begin(9600);
  initRCBus();
  enableRCbusinterrupt();
  initUserPorts();
  dsp = new KnoblyDisplay(0x70,&Serial2,&taskDrawHandle);
  dsp->init();
  /**
   * Task creation
   */
  xTaskCreate(TaskInput, // Task function
              "Input", // Task nam
              1024, // Stack size 
              (void*) dsp, 
              0, // Priority
              &t_handleList[0]); // Task handler

  xTaskCreate(TaskDbgMon,
              "DbgMon",
              1024,
              NULL, 
              0,
              &t_handleList[1]);

  xTaskCreate(TaskEncRead,
              "EncRead",
              1024,
              (void*) dsp,
              0,
              &t_handleList[2]);
  
  xTaskCreate(TaskDraw, // Task function
              "Draw", // Task nam
              1024, // Stack size 
              (void*) dsp, 
              0, // Priority
              &taskDrawHandle); // Task handler

}

ISR(INT2_vect) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR( t_handleList[0], &xHigherPriorityTaskWoken );
  if (xHigherPriorityTaskWoken) {
    taskYIELD();
  }
}
  


void loop() {}



void TaskDbgMon(void *pvParameters)
{
  (void) pvParameters;

  for (;;)
  {
    Serial2.println("======== Tasks status ========");
    Serial2.print("Tick count: ");
    Serial2.print(xTaskGetTickCount());
    Serial2.print(", Task count: ");
    Serial2.print(uxTaskGetNumberOfTasks());

    Serial2.println();
    Serial2.println();

    for (int i = 0; i<3; i++){
      Serial2.print("- TASK ");
      Serial2.print(pcTaskGetName(t_handleList[3])); // Get task name with handler
      Serial2.print(", High Watermark: ");
      Serial2.print(uxTaskGetStackHighWaterMark(t_handleList[i]));
      Serial2.println();
    }
    
    Serial2.println();
    
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
    }
  }
}

void TaskDraw(void *pvParameters)
{
  KnoblyDisplay* dsp_ptr = (KnoblyDisplay*)pvParameters;  
  for (;;)
  {
    if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {
      dsp_ptr->sendFrame();
    }
  }
}

void TaskEncRead(void *pvParameters)
{
  KnoblyDisplay* dsp_ptr = (KnoblyDisplay*)pvParameters;
  char temp[4]={ASCII_0,ASCII_0,ASCII_0,ASCII_0};
  dsp_ptr->setXDigits(temp, 4, 2);
  for (;;)
  {
    dsp_ptr->encoder_read();
    vTaskDelay( 25/ portTICK_PERIOD_MS ); 
  }
}

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <esp_system.h>

// Task handlers/////////////////////////////////////////////////////////
TaskHandle_t task1HandleLEFTsHARRPIR;
TaskHandle_t task2HandleRIGHTsHARRPIR;
TaskHandle_t task3HandleMainSHARRP_IR;
TaskHandle_t taskHandleReciveData;
// Queue handler/////////////////////////////////////////////////////////
QueueHandle_t sending_IR1_Value;
QueueHandle_t sending_IR2_Value;
QueueHandle_t sending_MAIN_IR_Value;

void task1_LEFT_Sharp_IR(void *parameter) {
  while (true) {
    float volts_1 = analogRead(36)*0.001220703125; 
    float distance_1 = 13*pow(volts_1, -1); // worked out from datasheet graph
    xQueueSend(sending_IR1_Value, &distance_1, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10));
    //xQueueReset(sendingValue);
   // Serial.print("sensor_LEFT Task_1: ");
   // Serial.print(distance_1);
    //Serial.print("\t"); 
    //vTaskDelay(50);
  }
}

void task2_RIGHT_Sharp_IR(void *parameter) {
  while (true) {
    float volts_2 = analogRead(39)*0.001220703125; 
    float distance_2 = 13*pow(volts_2, -1); // worked out from datasheet graph
    xQueueSend(sending_IR2_Value, &distance_2, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10));
    //Serial.print("sensor_RIGHT Task_2:");
    //Serial.print(distance_2);
    //Serial.print("\t"); 
    //vTaskDelay(50);
    
    
  }
}

void task3_MAIN_Sharp_IR(void *parameter) {
  while (true) {
    float volts_3 = analogRead(34)*0.001220703125; 
    float distance_3 = 13*pow(volts_3, -1); // worked out from datasheet graph
    xQueueSend(sending_MAIN_IR_Value, &distance_3, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10));
    //Serial.print("sensor_RIGHT Task_2:");
    //Serial.print(distance_2);
    //Serial.print("\t"); 
    //vTaskDelay(50);
    
    
  }
}

void task_ReciveData(void *parameter) {
  while (true) {
    float IR_1 , IR_2 , IR_MAIN;

    bool forward_ok = false , left_ok = false , right_ok = false;

     xQueueReceive(sending_IR1_Value, &IR_1, portMAX_DELAY);
     xQueueReceive(sending_IR2_Value, &IR_2, portMAX_DELAY);
     xQueueReceive(sending_MAIN_IR_Value, &IR_MAIN, portMAX_DELAY);
     Serial.print("\t"); 
     Serial.print(IR_1);
     Serial.print("\t"); 
     Serial.print(IR_2);
     Serial.print("\t"); 
     Serial.print(IR_MAIN);
     Serial.print("\t"); 


      if(IR_1<10){
        left_ok = false;
        //Serial.print("Left wall detected");
        }
      else{
        left_ok = true;
        //Serial.print("NO Left wall detected");
        }
       // Serial.print("\t"); 
      if(IR_2<10){
        right_ok = false;
        //Serial.print("Right wall detected");
        }
      else{
        right_ok = true;
        //Serial.print("NO Right wall detected");
        }
        //Serial.print("\t"); 
      if(IR_MAIN<10){
        forward_ok = false;
        //Serial.print("Front wall detected");
        }
      else{
        forward_ok = true;
        //Serial.print("NO Front wall detected");
        }

      Serial.print("\t"); 

      if(forward_ok == true && left_ok == true && right_ok == true){
        Serial.println("go forward");
      }
      if(forward_ok == true && left_ok == true && right_ok == false){
        Serial.println("go forward or left");
      }
      if(forward_ok == true && left_ok == false && right_ok == true){
        Serial.println("go forward or right");
      }
      if(forward_ok == true && left_ok == false && right_ok == false){
        Serial.println("go forward only");
      }
      if(forward_ok == false && left_ok == true && right_ok == true){
        Serial.println("go left or right");
      }
      if(forward_ok == false && left_ok == true && right_ok == false){
        Serial.println("go left");
      }
      if(forward_ok == false && left_ok == false && right_ok == true){
        Serial.println("go right");
      }
      if(forward_ok == false && left_ok == false && right_ok == false){
        Serial.println("go back");
      }

      


     vTaskDelay(pdMS_TO_TICKS(10));



    

  }
}



void setup() {
  Serial.begin(115200);

  sending_IR1_Value = xQueueCreate(1, sizeof(float));
  sending_IR2_Value = xQueueCreate(1, sizeof(float));
  sending_MAIN_IR_Value = xQueueCreate(1, sizeof(float));

  // Create Task 1 with task handler
  xTaskCreate(task1_LEFT_Sharp_IR, "Task1_LEFT_Sharp_IR", 2000, NULL, 0, &task1HandleLEFTsHARRPIR);
  // Create Task 2 with task handler
  xTaskCreate(task2_RIGHT_Sharp_IR, "Task2_RIGHT_Sharp_IR", 2000, NULL, 1, &task2HandleRIGHTsHARRPIR);

  xTaskCreate(task3_MAIN_Sharp_IR, "Task3_MAIN_Sharp_IR", 2000, NULL, 1, &task3HandleMainSHARRP_IR);

  xTaskCreate(task_ReciveData, "Task_ReciveData", 6000, NULL, 1, &taskHandleReciveData);
}



void loop() {

  // Empty loop
}









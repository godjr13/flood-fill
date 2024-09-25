#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <esp_system.h>
#include <driver/gpio.h>
#include <math.h>

// Motor control pins
#define AIN1 5   // Motor 1 control pin
#define BIN1 19  // Motor 2 control pin
#define AIN2 17  // Motor 1 direction pin
#define BIN2 18  // Motor 2 direction pin
#define PWMA 12  // Motor 1 PWM pin
#define PWMB 13  // Motor 2 PWM pin
#define STBY 14  // Standby pin for both motors

TaskHandle_t encoderTaskHandle = NULL; // Task handle for processing encoder counts
TaskHandle_t MOTOR_Control_TaskHandle;
void MOTOR1Task(void *pvParameters);
void MOTOR1Task(void *pvParameters);

// Mutex for encoder counts /////////////////////////////////////////////////
SemaphoreHandle_t encoderMutex;

// Queue handler/////////////////////////////////////////////////////////
QueueHandle_t sending_u_Value;
QueueHandle_t sending_u_1_Value;
QueueHandle_t sending_ENCO1_Value;
QueueHandle_t sending_ENCO2_Value;


// Encoder pins and counts /////////////////////////////////////////////////
#define ESP_INTR_FLAG_DEFAULT 0
#define ENCODER_PIN_A1 GPIO_NUM_15  // Channel A of the first encoder
#define ENCODER_PIN_B1 GPIO_NUM_2   // Channel B of the first encoder
#define ENCODER_PIN_A2 GPIO_NUM_4   // Channel A of the second encoder
#define ENCODER_PIN_B2 GPIO_NUM_27  // Channel B of the second encoder

volatile long pulse_count1 = 0; //Pulse count for the first encoder
volatile long pulse_count2 = 0; //Pulse count for the second encoder


//PID motor control task
void task_MOTOR_Control(void *parameter) {
    int target = 1000;
    long previousTime = 0, previousTime_1 = 0;
    float ePrevious = 0, ePrevious_1 = 0;
    float eIntegral = 0, eIntegral_1 = 0;
    float u, u_1;
    float kp = 0.125;
    float kd = 0.0000;
    float ki = 0.0000000;
    volatile long ENCO1, ENCO2;

    float EN1 ,EN2;

    while (true) {
        //Lock the encoder mutex to safely access pulse counts
        if (xSemaphoreTake(encoderMutex, portMAX_DELAY) == pdTRUE) {
            Serial.print("Encoder 1 : ");
            Serial.print(pulse_count1);
            Serial.print("\tEncoder 2: ");
            Serial.print(pulse_count2);
            ENCO1 = pulse_count1;
            ENCO2 = pulse_count2;
            xSemaphoreGive(encoderMutex);
        }

        EN1 = ENCO1;
        EN2 = ENCO2;

        //Motor 1 Control
        long currentTime = micros();
        float deltaT = ((float)(currentTime - previousTime)) / 1.0e6;
        int e = abs(ENCO1 - target);
        float eDerivative = (e - ePrevious) / deltaT;
        eIntegral += e * deltaT;
        u = (kp * e) + (kd * eDerivative) + (ki * eIntegral);
        previousTime = currentTime;
        ePrevious = e;

          Serial.print("\t"); 
          Serial.print(u);
    

        u = constrain(abs(u), 0, 255);  // Ensures 'u' is within 0 to 255

        //Motor 2 Control
        long currentTime_1 = micros();
        float deltaT_1 = ((float)(currentTime_1 - previousTime_1)) / 1.0e6;
        int e_1 = abs(ENCO2 - target);
        float eDerivative_1 = (e_1 - ePrevious_1) / deltaT_1;
        eIntegral_1 += e_1 * deltaT_1;
        u_1 = (kp * e_1) + (kd * eDerivative_1) + (ki * eIntegral_1);
        previousTime_1 = currentTime_1;
        ePrevious_1 = e_1;

        Serial.print("\t"); 
        Serial.print(u_1);

        u_1 = constrain(abs(u_1), 0, 255);  // Ensures 'u_1' is within 0 to 255

        //Send values to the queue with checks
        if (xQueueSend(sending_u_Value, &u, pdMS_TO_TICKS(10)) != pdPASS) {
            Serial.println("Failed to send u to queue.");
        }
        if (xQueueSend(sending_u_1_Value, &u_1, pdMS_TO_TICKS(10)) != pdPASS) {
            Serial.println("Failed to send u_1 to queue.");
        }

        if (xQueueSend(sending_ENCO1_Value, &EN1, pdMS_TO_TICKS(10)) != pdPASS) {
            Serial.println("Failed to send ENCO1 to queue.");
        }
        
         if (xQueueSend(sending_ENCO2_Value, &EN2, pdMS_TO_TICKS(10)) != pdPASS) {
            Serial.println("Failed to send ENCO2 to queue.");
        }
        

        vTaskDelay(pdMS_TO_TICKS(50));  // Added delay to prevent overwhelming the system
    }
}

//Motor 1 control task that receives encoder and control signal from the queue
void MOTOR1Task(void *pvParameters) {
    float U_1;
    float ENCO1;
   
    while (true) {
        

        // Check queue before receiving
        if (xQueueReceive(sending_u_Value, &U_1, pdMS_TO_TICKS(10)) == pdPASS && xQueueReceive(sending_ENCO1_Value, &ENCO1, pdMS_TO_TICKS(10)) == pdPASS) {
            Serial.print("\t"); 
            Serial.print(ENCO1);
            
            
            if(ENCO1<1000.00){       Motor1_fwd(U_1); }
            else if(ENCO1 =1000.00){ Motor1_brake(); }
            else if(ENCO1>1000.00){  Motor1_rev(U_1); }
            else{                    Motor1_brake(); }

            
            
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

//Motor 2 control task
void MOTOR2Task(void *pvParameters) {
    float U_2;
    float ENCO2;
    while (true) {
        // Check queue before receiving
        if (xQueueReceive(sending_u_1_Value, &U_2, pdMS_TO_TICKS(10)) == pdPASS && xQueueReceive(sending_ENCO2_Value, &ENCO2, pdMS_TO_TICKS(10)) == pdPASS) {
            
            Serial.print("\t"); 
            Serial.println(ENCO2);
            

            
            if(ENCO2<1000.00){        Motor2_fwd(U_2); }
            else if(ENCO2 =1000.00){  Motor2_brake(); }
            else if(ENCO2>1000.00){   Motor2_rev(U_2); }
            else{                     Motor2_brake(); }

        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}


// Interrupt Service Routine for Encoder 1 Channel A
void IRAM_ATTR encoder_isr_handler_A1(void *arg) {
  if (gpio_get_level(ENCODER_PIN_B1) == 0) {
    pulse_count1++;  // Clockwise rotation
  } else {
    pulse_count1--;  // Counterclockwise rotation
  }
  xTaskNotifyFromISR(encoderTaskHandle, 0, eNoAction, NULL); // Notify the task
}

// Interrupt Service Routine for Encoder 2 Channel A
void IRAM_ATTR encoder_isr_handler_A2(void *arg) {
  if (gpio_get_level(ENCODER_PIN_B2) == 0) {
    pulse_count2++;  // Clockwise rotation
  } else {
    pulse_count2--;  // Counterclockwise rotation
  }
  xTaskNotifyFromISR(encoderTaskHandle, 0, eNoAction, NULL); // Notify the task
}

// Task to process encoder pulses
void encoder_task(void *arg) {
  long last_count1 = 0, last_count2 = 0; // Variables to store the last known pulse counts
  while (true) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for a notification from ISR
    // Safely update last count using a mutex
    if (xSemaphoreTake(encoderMutex, portMAX_DELAY) == pdTRUE) {
      last_count1 = pulse_count1;
      last_count2 = pulse_count2;
      xSemaphoreGive(encoderMutex);
    }
  }
}





void setup() {
  Serial.begin(115200);
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);

  sending_u_Value = xQueueCreate(1, sizeof(float));
  sending_u_1_Value = xQueueCreate(1, sizeof(float));
  sending_ENCO1_Value = xQueueCreate(1, sizeof(float));
  sending_ENCO2_Value = xQueueCreate(1, sizeof(float));

  
  // Create mutex for encoder counts
  encoderMutex = xSemaphoreCreateMutex();


  xTaskCreate(task_MOTOR_Control, "Task_MOTOR_Control", 6000, NULL, 1, &MOTOR_Control_TaskHandle);
  xTaskCreate(MOTOR1Task, "MOTOR1Task", 1000, NULL, 0, NULL);
  xTaskCreate(MOTOR2Task, "MOTOR2Task", 1000, NULL, 0, NULL);


  // Configure encoder pins as inputs
  gpio_set_direction(ENCODER_PIN_A1, GPIO_MODE_INPUT);
  gpio_set_direction(ENCODER_PIN_B1, GPIO_MODE_INPUT);
  gpio_set_direction(ENCODER_PIN_A2, GPIO_MODE_INPUT);
  gpio_set_direction(ENCODER_PIN_B2, GPIO_MODE_INPUT);

  
  

  // Set interrupts on the rising edge for both encoders' Channel A
  gpio_set_intr_type(ENCODER_PIN_A1, GPIO_INTR_POSEDGE);
  gpio_set_intr_type(ENCODER_PIN_A2, GPIO_INTR_POSEDGE);

  // Install interrupt service and attach the handlers
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  gpio_isr_handler_add(ENCODER_PIN_A1, encoder_isr_handler_A1, NULL);
  gpio_isr_handler_add(ENCODER_PIN_A2, encoder_isr_handler_A2, NULL);

  // Create the task to handle pulse count processing
  xTaskCreate(
      encoder_task,      // Task function
      "encoder_task",    // Task name
      4096,              // Stack size
      NULL,              // Task input parameter
      10,                // Priority
      &encoderTaskHandle // Task handle
  );


}

void loop() {
  // put your main code here, to run repeatedly:

}


// Motor control functions
void Motor1_fwd(int speed) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(STBY, HIGH);
  analogWrite(PWMA, speed);
}

void Motor1_rev(int speed) {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(STBY, HIGH);
  analogWrite(PWMA, speed);
}

void Motor1_brake() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  digitalWrite(STBY, HIGH);
  analogWrite(PWMA, 0);
}

void Motor_standby() {
  digitalWrite(STBY, LOW);
}

void Motor2_fwd(int speed) {
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  digitalWrite(STBY, HIGH);
  analogWrite(PWMB, speed);
}

void Motor2_rev(int speed) {
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  digitalWrite(STBY, HIGH);
  analogWrite(PWMB, speed);
}

void Motor2_brake() {
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
  digitalWrite(STBY, HIGH);
  analogWrite(PWMB, 0);
}

void forward(int speed) {
  Motor1_fwd(speed);
  Motor2_fwd(speed);
}

void back(int speed) {
  Motor1_rev(speed);
  Motor2_rev(speed);
}

void left(int speed) {
  int temp = abs(speed) / 2;
  Motor1_rev(temp);
  Motor2_fwd(temp);
}

void right(int speed) {
  int temp = abs(speed) / 2;
  Motor1_fwd(temp);
  Motor2_rev(temp);
}

void brake() {
  Motor1_brake();
  Motor2_brake();
}
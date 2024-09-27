#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <esp_system.h>
#include <driver/gpio.h>
#include <math.h>

//motor driver pins
#define AIN1 5
#define BIN1 19
#define AIN2 17
#define BIN2 18
#define PWMA 12
#define PWMB 13
#define STBY 14

//motor task handlers
TaskHandle_t encoderTaskHandle = NULL; // Task handle for processing encoder counts
TaskHandle_t MOTOR_Control_TaskHandle;
void MOTOR1Task(void *pvParameters);


//IR Task handlers/////////////////////////////////////////////////////////
TaskHandle_t task1HandleLEFTsHARRPIR;
TaskHandle_t task2HandleRIGHTsHARRPIR;
TaskHandle_t task3HandleMainSHARRP_IR;
TaskHandle_t taskHandleReciveData;

//floodfill Task handlers/////////////////////////////////////////////////////////
void FloodFillTask(void *pvParameters);


// Mutex for encoder counts /////////////////////////////////////////////////
SemaphoreHandle_t encoderMutex;

//Motor Queue handler/////////////////////////////////////////////////////////
QueueHandle_t sending_u_Value;
QueueHandle_t sending_u_1_Value;
QueueHandle_t sending_ENCO1_Value;
QueueHandle_t sending_ENCO2_Value;

//IR Queue handler/////////////////////////////////////////////////////////
QueueHandle_t sending_IR1_Value;
QueueHandle_t sending_IR2_Value;
QueueHandle_t sending_MAIN_IR_Value;
//wall_num Queue handler/////////////////////////////////////////////////////////
QueueHandle_t sending_wall_num;
//motor target Queue handler/////////////////////////////////////////////////////////
QueueHandle_t sending_target_left_wheel;
QueueHandle_t sending_target_right_wheel;

//foolfill Queue handler/////////////////////////////////////////////////////////
QueueHandle_t sending_direction;

// Encoder pins and counts /////////////////////////////////////////////////
#define ESP_INTR_FLAG_DEFAULT 0
#define ENCODER_PIN_A1 GPIO_NUM_15  // Channel A of the first encoder
#define ENCODER_PIN_B1 GPIO_NUM_2   // Channel B of the first encoder
#define ENCODER_PIN_A2 GPIO_NUM_4   // Channel A of the second encoder
#define ENCODER_PIN_B2 GPIO_NUM_27  // Channel B of the second encoder

volatile long pulse_count1 = 0; // Pulse count for the first encoder
volatile long pulse_count2 = 0; // Pulse count for the second encoder


#define Q_size (16*16)
#define INT_MAX 2147483647


// Point struct definition 
typedef struct{
  int x,y;
} Point;

// Queue struct definition
typedef struct{
  Point points[Q_size]; // defines an array of the point data type 
  int front, rear;
} Queue;

// Data type for minimum distance array for the check nearby cells function
typedef struct{
  Point cells[8];
  int min;
} Manhattan_min;

// Maze info
Point current_cell = {0, 16};
Point start = {0, 16};
Point goal = {8, 8};
Point invalid_cell = {-1,-1};

// Maze matrix
int maze[16][16] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

// Manhattan distance maze
int manhattan_maze[16][16] = {
            {14,13,12,11,10,9,8,7,7,8,9,10,11,12,13,14},
            {13,12,11,10,9,8,7,6,6,7,8,9,10,11,12,13},
            {12,11,10,9,8,7,6,5,5,6,7,8,9,10,11,12},
            {11,10,9,8,7,6,5,4,4,5,6,7,8,9,10,11},
            {10,9,8,7,6,5,4,3,3,4,5,6,7,8,9,10},
            {9,8,7,6,5,4,3,2,2,3,4,5,6,7,8,9},
            {8,7,6,5,4,3,2,1,1,2,3,4,5,6,7,8},
            {7,6,5,4,3,2,1,0,0,1,2,3,4,5,6,7},
            {7,6,5,4,3,2,1,0,0,1,2,3,4,5,6,7},
            {8,7,6,5,4,3,2,1,1,2,3,4,5,6,7,8},
            {9,8,7,6,5,4,3,2,2,3,4,5,6,7,8,9},
            {10,9,8,7,6,5,4,3,3,4,5,6,7,8,9,10},
            {11,10,9,8,7,6,5,4,4,5,6,7,8,9,10,11},
            {12,11,10,9,8,7,6,5,5,6,7,8,9,10,11,12},
            {13,12,11,10,9,8,7,6,6,7,8,9,10,11,12,13},
            {14,13,12,11,10,9,8,7,7,8,9,10,11,12,13,14}
};

int explored[16][16] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

// Function declarations moved outside the task
int move(Point new_cell);
void enqueue(Queue *q, Point p);
Point dequeue(Queue *q);
int check_wall();
Manhattan_min* check_nearby_cells(Point cell);

// Move function
int move(Point new_cell) {
  int direction;
  //choosing the direction
  if(new_cell.x == current_cell.x + 1 && new_cell.y == current_cell.y){
    direction = 1;
  }else if(new_cell.x == current_cell.x + 1 && new_cell.y == current_cell.y - 1){
    direction = 2;
  }else if(new_cell.x == current_cell.x && new_cell.y == current_cell.y - 1){
    direction = 3;
  }else if(new_cell.x == current_cell.x - 1 && new_cell.y == current_cell.y - 1){
    direction = 4;
  }else if(new_cell.x == current_cell.x - 1 && new_cell.y == current_cell.y){
    direction = 5;
  }else if(new_cell.x == current_cell.x - 1 && new_cell.y == current_cell.y + 1){
    direction = 6;
  }else if(new_cell.x == current_cell.x && new_cell.y == current_cell.y + 1){
    direction = 7;
  }else if(new_cell.x == current_cell.x + 1 && new_cell.y == current_cell.y + 1){
    direction = 8;
  }
  // ... [Direction logic unchanged]
  return direction;
}

// Enqueue function
void enqueue(Queue *q, Point p) {
  if ((q->rear + 1) % Q_size == q->front) {
    return;
  } else {
    q->rear = (q->rear + 1) % Q_size;
    q->points[q->rear] = p;
  }
}

// Dequeue function
Point dequeue(Queue *q) {
  Point p = {-1, -1}; 
  if (q->front == q->rear) {
    return p;
  } else {
    q->front = (q->front + 1) % Q_size;
    return q->points[q->front];
  }
}


// Checking nearby cells
Manhattan_min* check_nearby_cells(Point cell) {
  int min = 16;
  int nearby_cell_val[8];
  Point min_cells[8];
  Manhattan_min manhattan_min; 
  Point nearby_cells[8] = {
    {cell.x + 1, cell.y}, {cell.x + 1, cell.y - 1}, {cell.x, cell.y - 1},
    {cell.x - 1, cell.y - 1}, {cell.x - 1, cell.y}, {cell.x - 1, cell.y + 1},
    {cell.x, cell.y + 1}, {cell.x + 1, cell.y + 1}
  };

  for (int i = 0; i < 8; i++) {
    int x = nearby_cells[i].x;
    int y = nearby_cells[i].y;
    if (x >= 0 && x < 16 && y >= 0 && y < 16) {
      nearby_cell_val[i] = manhattan_maze[x][y];
    } else {
      nearby_cell_val[i] = -1; // Invalid cell
    }
  }

  for (int i = 0; i < 8; i++) {
    if (nearby_cell_val[i] != -1 && nearby_cell_val[i] < min) {
      min = nearby_cell_val[i];
    }
  }

  for (int i = 0; i < 8; i++) {
    manhattan_min.cells[i] = (nearby_cell_val[i] == min) ? nearby_cells[i] : invalid_cell;
  }
  manhattan_min.min = min;

  return &manhattan_min;
}



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
    int wall_num;

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
        wall_num = 1;
      }
      if(forward_ok == true && left_ok == true && right_ok == false){
        Serial.println("go forward or left");
        wall_num = 3;
      }
      if(forward_ok == true && left_ok == false && right_ok == true){
        Serial.println("go forward or right");
        wall_num = 2;
      }
      if(forward_ok == true && left_ok == false && right_ok == false){
        Serial.println("go forward only");
        wall_num = 5;
      }
      if(forward_ok == false && left_ok == true && right_ok == true){
        Serial.println("go left or right");
        wall_num = 4;
      }
      if(forward_ok == false && left_ok == true && right_ok == false){
        Serial.println("go left");
        wall_num = 7;
      }
      if(forward_ok == false && left_ok == false && right_ok == true){
        Serial.println("go right");
        wall_num = 6;
      }
      if(forward_ok == false && left_ok == false && right_ok == false){
        Serial.println("go back");
        wall_num = 8;
      }

      xQueueSend(sending_wall_num, &wall_num, portMAX_DELAY);  


     vTaskDelay(pdMS_TO_TICKS(10));



    

  }
}

void task_MOTOR_Control(void *parameter) {
    int target_left_wheel = 0 , target_right_wheel = 0 , wall_num = 0 , dir;
    long previousTime = 0, previousTime_1 = 0;
    float ePrevious = 0, ePrevious_1 = 0;
    float eIntegral = 0, eIntegral_1 = 0;
    float u, u_1;
    float kp = 0.325;
    float kd = 0.00005;
    float ki = 0.0000000;
    volatile long ENCO1, ENCO2 ,counts_per_cell = 465 , counts_per_90_turn = 200 , counts_per_180_turn = 400;

    float EN1 ,EN2;

    while (true) {
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
        
        xQueueReceive(sending_direction, &dir, portMAX_DELAY);

        switch (dir) {
            case 1:
                target_left_wheel = ENCO1 + counts_per_cell;
                target_right_wheel = ENCO2 + counts_per_cell;
                break;
            case 2:
                target_left_wheel = ENCO1 + counts_per_cell;
                target_right_wheel = ENCO2 + counts_per_cell;
                break;
            case 3:
                target_left_wheel = ENCO1 + counts_per_cell;
                target_right_wheel = ENCO2 + counts_per_cell;
                break;
            case 4:
                target_left_wheel = ENCO1 - counts_per_90_turn;
                target_right_wheel = ENCO2 + counts_per_90_turn;
                break;
            case 5:
                target_left_wheel = ENCO1 + counts_per_cell;
                target_right_wheel = ENCO2 + counts_per_cell;
                break;
            case 6:
                target_left_wheel = ENCO1 + counts_per_90_turn;
                target_right_wheel = ENCO2 - counts_per_90_turn;
                break;
            case 7:
                target_left_wheel = ENCO1 - counts_per_90_turn;
                target_right_wheel = ENCO2 + counts_per_90_turn;
                break;
            case 8:
                target_left_wheel = ENCO1 - counts_per_180_turn;
                target_right_wheel = ENCO2 + counts_per_180_turn;
                break;
            default:
                Serial.println("Unknown wall number");
                break;
        }
        

        // Motor 1 Control
        long currentTime = micros();
        float deltaT = ((float)(currentTime - previousTime)) / 1.0e6;
        int e = abs(ENCO1 - target_left_wheel);
        float eDerivative = (e - ePrevious) / deltaT;
        eIntegral += e * deltaT;
        u = (kp * e) + (kd * eDerivative) + (ki * eIntegral);
        previousTime = currentTime;
        ePrevious = e;

          Serial.print("\t"); 
          Serial.print(u);
    

        u = constrain(abs(u), 0, 255);  // Ensures 'u' is within 0 to 255

        // Motor 2 Control
        long currentTime_1 = micros();
        float deltaT_1 = ((float)(currentTime_1 - previousTime_1)) / 1.0e6;
        int e_1 = abs(ENCO2 - target_right_wheel);
        float eDerivative_1 = (e_1 - ePrevious_1) / deltaT_1;
        eIntegral_1 += e_1 * deltaT_1;
        u_1 = (kp * e_1) + (kd * eDerivative_1) + (ki * eIntegral_1);
        previousTime_1 = currentTime_1;
        ePrevious_1 = e_1;

        Serial.print("\t"); 
        Serial.print(u_1);

        u_1 = constrain(abs(u_1), 0, 255);  // Ensures 'u_1' is within 0 to 255

        // Send values to the queue with checks
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
        if (xQueueSend(sending_target_left_wheel, &target_left_wheel, pdMS_TO_TICKS(10)) != pdPASS) {
            Serial.println("Failed to send ENCO2 to queue.");
        }
        if (xQueueSend(sending_target_right_wheel, &target_right_wheel, pdMS_TO_TICKS(10)) != pdPASS) {
            Serial.println("Failed to send ENCO2 to queue.");
        }
        

        vTaskDelay(pdMS_TO_TICKS(50));  // Added delay to prevent overwhelming the system
    }
}

void MOTOR1Task(void *pvParameters) {
    float U_1 , U_2;
    float ENCO1 ,ENCO2;
    int target_left_wheel , target_right_wheel ;
       
   
    while (true) {
        

        // Check queue before receiving
        if (xQueueReceive(sending_u_Value, &U_1, pdMS_TO_TICKS(10)) == pdPASS && xQueueReceive(sending_ENCO1_Value, &ENCO1, pdMS_TO_TICKS(10)) == pdPASS && xQueueReceive(sending_target_left_wheel, &target_left_wheel, pdMS_TO_TICKS(10))  == pdPASS && xQueueReceive(sending_u_1_Value, &U_2, pdMS_TO_TICKS(10)) == pdPASS && xQueueReceive(sending_ENCO2_Value, &ENCO2, pdMS_TO_TICKS(10)) == pdPASS && xQueueReceive(sending_target_right_wheel, &target_right_wheel, pdMS_TO_TICKS(10))== pdPASS) {
            Serial.print("\t"); 
            Serial.print(ENCO1);
            Serial.print("\t"); 
            Serial.println(ENCO2);
    
        }
        vTaskDelay(pdMS_TO_TICKS(10));

         if(ENCO1 < target_left_wheel){       Motor2_fwd(U_1); }
         else if(ENCO1 == target_left_wheel){ Motor2_brake(); }
         else if(ENCO1 > target_left_wheel){  Motor2_rev(U_1); }
            //else{                    Motor1_brake(); }

         if(ENCO2 < target_right_wheel){        Motor1_fwd(U_2); }
         else if(ENCO2 == target_right_wheel){  Motor1_brake(); }
         else if(ENCO2 > target_right_wheel){   Motor1_rev(U_2); }
            //else{                     Motor2_brake(); }   
    }
}

// flood fill task function
void FloodFillTask(void *pvParameters) {
  int dir;
  Queue q;
  q.front = q.rear = 0;
  enqueue(&q, start);
  int distance;
  int walls;

  while (q.front != q.rear) {
    Point current = dequeue(&q);

    if (current.x == goal.x && current.y == goal.y) {
      return;
    }

    distance = manhattan_maze[current.x][current.y];
    Manhattan_min* nearby_min = check_nearby_cells(current);
    xQueueReceive(sending_wall_num, &walls, portMAX_DELAY);

    switch(walls){
        case 1:
          break;
        case 2:
          nearby_min->cells[4] = invalid_cell;
          break;
        case 3:
          nearby_min->cells[0] = invalid_cell;
          break;
        case 4:
          nearby_min->cells[6] = invalid_cell;
          break;
        case 5:
          nearby_min->cells[4] = invalid_cell;
          nearby_min->cells[0] = invalid_cell;
          break;
        case 6:
          nearby_min->cells[4] = invalid_cell;
          nearby_min->cells[5] = invalid_cell;
          nearby_min->cells[6] = invalid_cell;
          break;
        case 7:
          nearby_min->cells[0] = invalid_cell;
          nearby_min->cells[6] = invalid_cell;
          nearby_min->cells[7] = invalid_cell;
          break;
        case 8:
          nearby_min->cells[0] = invalid_cell;
          nearby_min->cells[4] = invalid_cell;
          nearby_min->cells[5] = invalid_cell;
          nearby_min->cells[6] = invalid_cell;
          nearby_min->cells[7] = invalid_cell;
          break;
      }

    if (distance > nearby_min->min) {
      for (int i = 0; i < 8; i++) {
        if (nearby_min->cells[i].x != invalid_cell.x || nearby_min->cells[i].y != invalid_cell.y) {
          enqueue(&q, nearby_min->cells[i]);
        }
      }
    }

    Point new_cell = dequeue(&q);
    dir = move(new_cell);
    xQueueSend(sending_direction, &dir, pdMS_TO_TICKS(10));
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

  sending_IR1_Value = xQueueCreate(1, sizeof(float));
  sending_IR2_Value = xQueueCreate(1, sizeof(float));
  sending_MAIN_IR_Value = xQueueCreate(1, sizeof(float));

  sending_wall_num = xQueueCreate(1, sizeof(int));

  sending_target_left_wheel = xQueueCreate(1, sizeof(int));
  sending_target_right_wheel = xQueueCreate(1, sizeof(int));

  
  // Create mutex for encoder counts
  encoderMutex = xSemaphoreCreateMutex();

 //floodfill queue and task
  sending_direction = xQueueCreate(1, sizeof(float));
  xTaskCreate(FloodFillTask, "TheFloodFillTask", 10000, NULL, 2, NULL);

  xTaskCreate(task_MOTOR_Control, "Task_MOTOR_Control", 7000, NULL, 0, &MOTOR_Control_TaskHandle);
  xTaskCreate(MOTOR1Task, "MOTOR1Task", 1000, NULL, 0, NULL);
 
  xTaskCreate(task1_LEFT_Sharp_IR, "Task1_LEFT_Sharp_IR", 2000, NULL, 1, &task1HandleLEFTsHARRPIR);
  xTaskCreate(task2_RIGHT_Sharp_IR, "Task2_RIGHT_Sharp_IR", 2000, NULL, 1, &task2HandleRIGHTsHARRPIR);
  xTaskCreate(task3_MAIN_Sharp_IR, "Task3_MAIN_Sharp_IR", 2000, NULL, 1, &task3HandleMainSHARRP_IR);
  xTaskCreate(task_ReciveData, "Task_ReciveData", 6000, NULL, 0, &taskHandleReciveData);


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
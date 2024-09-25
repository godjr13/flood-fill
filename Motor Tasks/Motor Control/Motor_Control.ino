#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <esp_system.h>
#include <driver/gpio.h>
#include <math.h>


void FloodFillTask(void *pvParameters);

QueueHandle_t sending_deraction;

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
  // Choosing the direction logic
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

// Wall checking function
int check_wall() {
  int wall_val = 0;
  // Wall checking logic
  return wall_val;
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

// Main task function
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
    walls = check_wall();

    switch(walls){
        case 1:
          break;
        case 2:
          nearby_min->cells[4] = invalid_cell;
        case 3:
          nearby_min->cells[0] = invalid_cell;
        case 4:
          nearby_min->cells[6] = invalid_cell;
        case 5:
          nearby_min->cells[4] = invalid_cell;
          nearby_min->cells[0] = invalid_cell;
        case 6:
          nearby_min->cells[4] = invalid_cell;
          nearby_min->cells[5] = invalid_cell;
          nearby_min->cells[6] = invalid_cell;
        case 7:
          nearby_min->cells[0] = invalid_cell;
          nearby_min->cells[6] = invalid_cell;
          nearby_min->cells[7] = invalid_cell;
        case 8:
          nearby_min->cells[0] = invalid_cell;
          nearby_min->cells[4] = invalid_cell;
          nearby_min->cells[5] = invalid_cell;
          nearby_min->cells[6] = invalid_cell;
          nearby_min->cells[7] = invalid_cell;
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
    xQueueSend(sending_deraction, &dir, pdMS_TO_TICKS(10));
  }
}

// Setup and loop functions
void setup() {
  Serial.begin(115200);
  sending_deraction = xQueueCreate(1, sizeof(float));
  xTaskCreate(FloodFillTask, "TheFloodFillTask", 10000, NULL, 2, NULL);
}

void loop() {
  // Empty loop
}

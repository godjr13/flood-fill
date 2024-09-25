
//sensors
#define tof
#define ir_left
#define ir_right
#define ir_left45
#define ir_right45

#define Q_size (16*16)
#define INT_MAX 2147483647

//distance set points
int tof_sp, irleft_sp, irright_sp, irr45_sp, irl45_sp;

//Point struct definition 
typedef struct{
  int x,y;
} Point;

//Queue struct definition
typedef struct{
  Point points[Q_size]; //defines an array of the point data type 
  int front, rear;
} Queue;

//data type for minimum distance array for the check nearby cells function
typedef struct{
  Point cells[8];
  int min;
}Manhattan_min;

// Maze info
Point current_cell = {0, 16};
Point start = {0, 16};
Point goal = {8, 8};
//Point *explored_cells[Q_size] = {(Point *)malloc(sizeof(Point))}; //dynamic array of all the explored cordinates
Point invalid_cell = {-1,-1};

//maze matrix
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
};//zero represents open paths and 1 represents closed paths

//manhattan distance maze
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


//movement
void move(int direction){
  //marking explored cells
  explored[current_cell.x][current_cell.y] = 1;
  switch(direction){
    case 1://up 
      current_cell.y++;
      break;
    case 2: //down 
      current_cell.y--;
      break;
    case 3: //left 
      current_cell.x--;
      break;
    case 4: //right
      current_cell.x++;
      break;
    case 5: //diagonal up-left
      current_cell.x--;
      current_cell.y++;
    case 6: //diagonal up-right
      current_cell.x++;
      current_cell.y++;
    case 7: //diagonal down-left
      current_cell.x--;
      current_cell.y--;
    case 8: //diagonal down-right
      current_cell.x++;
      current_cell.y--;
  }


}

//queue
// Enqueue a point to the queue
void enqueue(Queue *q, Point p) {
  // Check if the queue is full
  if ((q->rear + 1) % Q_size == q->front) {
    // Queue is full, handle the overflow (e.g., stop adding or expand the queue)
    return;
  } else {
    // Move rear and add the new point
    q->rear = (q->rear + 1) % Q_size;
    q->points[q->rear] = p;
  }
}

// Dequeue a point from the queue
Point dequeue(Queue *q) {
  Point p = {-1, -1};  // Default invalid point to return if queue is empty
  
  // Check if the queue is empty
  if (q->front == q->rear) {
    return p;
  } else {
    // Move front and return the dequeued point
    q->front = (q->front + 1) % Q_size;
    return q->points[q->front];
  }
}

//other functions
int check_wall(){
  
  /* wall senarios : 
    Scenario 1: no walls
    Scenario 2: wall left
    Scenario 3: wall right
    Scenario 4: wall front
    Scenario 5: wall left and right
    Scenario 6: wall left and front
    Scenario 7: wall right and front
    Scenario 8: wall left,right and front */
  int wall_val;
  return wall_val;
}

Manhattan_min* check_nearby_cells(Point cell){
  int min = 16;
  int nearby_cell_val[8];
  Point min_cells[8];
  Manhattan_min manhattan_min; 
  Point nearby_cells[8] = {
    {cell.x + 1, cell.y},       // Right
    {cell.x + 1, cell.y - 1},   // Down-right
    {cell.x, cell.y - 1},       // Down
    {cell.x - 1, cell.y - 1},   // Down-left
    {cell.x - 1, cell.y},       // Left
    {cell.x - 1, cell.y + 1},   // Up-left
    {cell.x, cell.y + 1},       // Up
    {cell.x + 1, cell.y + 1}    // Up-right
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
  
  //find the  minimum manhattan distances
  for (int i = 0; i < 8; i++){
    if(nearby_cell_val[i] != -1){
      if(nearby_cell_val[i] < min){
        min = nearby_cell_val;
      }
    }
  }

  //find the minimum cells
  for (int i = 0; i < 8; i++){
    if(nearby_cell_val[i] == min){
      min_cells[i] = nearby_cells[i];
    }else{
      min_cells[i] = invalid_cell;
    }
  }
  for (int i = 0; i < 8; i++){
    manhattan_min.cells[i] = min_cells[i];
  }
  
  manhattan_min.min = min;

  return &manhattan_min;
}


//floodfill
void floodfill(){
  Queue q;
  q.front = q.rear = 0;
  enqueue(&q , start);
  int distance;
  int walls;
  Point min_cells;
  while(q.front != q.rear){ //checks if the queue is empty
    Point current = dequeue(&q); // Get the front cell

    // Check if we reached the goal
    if (current.x == goal.x && current.y == goal.y) {
      //do something to indicate that the mouse reached the goal
      return;
    } else{
      distance = manhattan_maze[current.x][current.y]; //gets the manhattan distance value
      
      //check nearby cells
      Manhattan_min* nearby_min = check_nearby_cells(current);

      //check wall scenario
      walls = check_wall();
      //eliminate unavailable paths
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

      if(distance > nearby_min->min){
        for(int i = 0; i < 8; i++){
          if (nearby_min->cells[i].x != invalid_cell.x || nearby_min->cells[i].y != invalid_cell.y) {
            enqueue(&q , nearby_min->cells[i]); //adds cell to the queue
          }
        }
      }else if(distance = nearby_min->min){
        //increment the explored cell values
        for(int i = 0; i < 16; i++){
          for(int j = 0; j < 16; j++){
            if(explored[i][j] == 1){
              manhattan_maze[i][j]++;
            }
          }
          enqueue(&q , nearby_min->cells[i]); //adds cell to the queue
        }
      } 
    }
    //move to the next cell in queue
  }
}



int main(){
 
} 

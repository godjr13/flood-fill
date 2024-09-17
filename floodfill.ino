
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


// Maze info
Point current_cell = {0, 16};
Point start = {0, 16};
Point goal = {8, 8};

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

int explored_cells[16][16] = {
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
  explored_cells[current_cell.x][current_cell.y] = 1;
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

int* find_min_indices(int* values, int size, int* num_indices) {
    if (size <= 0) {
        *num_indices = 0;
        return NULL;
    }

    //Find the minimum value
    int min_value = values[0];
    for (int i = 1; i < size; i++) {
        if (values[i] < min_value) {
            min_value = values[i];
        }
    }

    //Count the number of occurrences of the minimum value
    *num_indices = 0;
    for (int i = 0; i < size; i++) {
        if (values[i] == min_value) {
            (*num_indices)++;
        }
    }

    // Allocate memory for the indices array
    int* indices = (int*)malloc(*num_indices * sizeof(int));
    if (indices == NULL) {
        *num_indices = 0;
        return NULL; // Return NULL if memory allocation fails
    }

    //Collect the indices of the minimum value
    int index = 0;
    for (int i = 0; i < size; i++) {
        if (values[i] == min_value) {
            indices[index++] = i;
        }
    }

    return indices;
}

int* check_nearby_cells(Point cell){
    int nearby_cell_val[8];
    int num_indices;
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
    
    int * indices = find_min_indices(nearby_cell_val, 8, &num_indices);
    return indices;
    //return nearby_cell_val;
}


//floodfill
void floodfill(){
  Queue q;
  q.front = q.rear = 0;

  enqueue(&q , start);

  while(q.front != q.rear){ //checks if the queue is empty
    Point current = dequeue(&q); // Get the front cell

    // Check if we reached the goal
    if (current.x == goal.x && current.y == goal.y) {
      //do something to indicate that the mouse reached the goal
      return;
    }
  

  }
}



int main(){
 /* //searching algorithm
  while(current_cell != goal){
    int wall = check_wall();
    
    switch(wall){

      case 1: //no walls
        check_nearby_cells(current_cell);
        
        break;

      case 2: //wall left
        maze[current_cell[0]-1] [current_cell[1]] = 1; //set the left cell as a closed path
        break;

      case 3: //wall right
        maze[current_cell[0]+1] [current_cell[1]] = 1;
        break;

      case 4: //wall front
        maze[current_cell[0]] [current_cell[1]+1] = 1;
        break;

      case 5: //wall left and right
        maze[current_cell[0]-1] [current_cell[1]] = 1;
        maze[current_cell[0]+1] [current_cell[1]] = 1;
        break;

      case 6: //wall left and front
        maze[current_cell[0]] [current_cell[1]+1] = 1;
        maze[current_cell[0]-1] [current_cell[1]] = 1;
        break;

      case 7: //wall right and front
        maze[current_cell[0]+1] [current_cell[1]] = 1;
        maze[current_cell[0]] [current_cell[1]+1] = 1;
        break;

      case 8: //wall left,right and front
        maze[current_cell[0]] [current_cell[1]+1] = 1;
        maze[current_cell[0]-1] [current_cell[1]] = 1;
        maze[current_cell[0]+1] [current_cell[1]] = 1;
        break;

    }
  }*/

} 

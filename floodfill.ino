
//sensors
#define tof
#define ir_left
#define ir_right
#define ir_left45
#define ir_right45

//distance set points
int tof_sp, irleft_sp, irright_sp, irr45_sp, irl45_sp;

int cell_val; //current cell value (set starting cell as current cell in the begining..)
int current_cell[2] = {0,16}; 
int start[2] = {0,16};
int goal[2] = {8,8};
int Q_size = 16*16;

//Point struct definition 
typedef struct{
  int x,y;
} Point;

//Queue struct definition
typedef struct{
  Point points[Q_size]; //defines an array of the point data type 
  int front, rear;
} Queue;


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
            {14,13,12,11,10,9,8,7,7,8,9,10,11,12,13,14}};

//explored cells
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
  explored_cells[current_cell[0]][current_cell[1]] = 1; //updates explored cells
  switch(direction){
    case 1://up 
      current_cell[1]++;
      break;
    case 2: //down 
      current_cell[1]--;
      break;
    case 3: //left 
      current_cell[0]--;
      break;
    case 4: //right
      current_cell[0]++;
      break;
  }

  //marking explored cells
  explored_cells[current_cell[0]][current_cell[1]] = 1;
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

int check_nearby_cells(int cell[2]){
  //pass in the current cell index into this function
  int nearby_cell_val[8];
  //nearby cell indexes
  int nearby_cells[8][2] = {
    {cell[0] + 1, cell[1]},       //Right
    {cell[0] + 1, cell[1] - 1},   //Down-right
    {cell[0], cell[1] - 1},       //Down
    {cell[0] - 1, cell[1] - 1},   //Down-left
    {cell[0] - 1, cell[1]},       //Left
    {cell[0] - 1, cell[1] + 1},   //Up-left
    {cell[0], cell[1] + 1},       //Up
    {cell[0] + 1, cell[1] + 1}    //Up-right
  };
  
  for (int i = 0; i < 8; i++) {
    int x = nearby_cells[i][0];
    int y = nearby_cells[i][1];
    if (x >= 0 && x < 16 && y >= 0 && y < 16) {
      nearby_cell_val[i] = manhattan_maze[x][y];
    }else {
      nearby_cell_val[i] = -1; // Invalid cell
    }
  }
  return nearby_cell_val;
}

//floodfill
void floodfill(){

}

int main(){
  //searching algorithm
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
  }
}
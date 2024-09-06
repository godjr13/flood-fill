
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

//maze matrix
int maze[16][16] = {{14,13,12,11,10,9,8,7,7,8,9,10,11,12,13,14},
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

//movement
void move_forward(){
}

void turn_left(){
}

void turn_right(){
}

void turn_back(){
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
  int i,j;
  i = cell[1];
  j = cell[2];
  
  int nearby_cell = {}; //cell index queue
  int nearby_cell_val = {}; //distance values
  
  for(i; i<=i+1; i++){
    for(j; j<=j+1; j++){

    }
  }

  for(i; i<=i+1; i--){
    for(j; j<=j+1; j--){

    }
  }
}

int main(){
  //searching algorithm
  if(cell_val != 0){
    int wall = check_wall();
    
    switch(wall){

      case 1: //no walls
        break;

      case 2: //wall left
        break;

      case 3: //wall right
        break;

      case 4: //wall front
        break;

      case 5: //wall left and right
        break;

      case 6: //wall left and front
        break;

      case 7: //wall right and front
        break;

      case 8: //wall left,right and front
        break;

    }
  }
}
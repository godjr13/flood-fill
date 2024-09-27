int old, new;

//resets the encoder when the wall scenario changes
void reset_enc(int wall){
  new = wall;
  if(old != new){
    // encoder_variableA = 0;
    // encoder_variableB =0;
  }
  //records the encoder value 
  new = old;
}


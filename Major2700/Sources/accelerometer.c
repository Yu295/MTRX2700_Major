#include "accelerometer.h"

void convertUnits(AccelRaw *raw_data, AccelScaled *scaled_data){
    
    scaled_data->x = (float)(raw_data->x)/250;
    scaled_data->y = (float)(raw_data->y)/250;
    scaled_data->z = (float)(raw_data->z)/250;
}


// calculate elevation from initial accelerometer reading taken while stationary
float findElevation (AccelScaled *scaled_data) {
  
  return atanf((scaled_data->z)/(sqrt((scaled_data->y)*(scaled_data->y)+(scaled_data->x)*(scaled_data->x))));
}

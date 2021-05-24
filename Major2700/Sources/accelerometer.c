#include "accelerometer.h"
#include "l3g4200d.h"
#include "iic.h"
#include <math.h>

void convertUnits(AccelRaw *raw_data, AccelScaled *scaled_data){
    scaled_data->x = (float)(raw_data->x)/250;
    scaled_data->y = (float)(raw_data->y)/250;
    scaled_data->z = (float)(raw_data->z)/250;
}

void getDisplacement(float *prevVel, float *prevDisp) {
  AccelRaw raw_accel;
  AccelScaled scaled_accel;
  
  float elevation;
  float newVel, newDisp, currAccel;
  float dt = 25; // time step (ms)
  float g = 9.81;
   
  getRawDataAccel(&raw_accel);
  convertUnits(&raw_accel, &scaled_accel);
  elevation = atanf((scaled_accel.z)/(sqrt((scaled_accel.y)*(scaled_accel.y)+(scaled_accel.x)*(scaled_accel.x))));
  
  scaled_accel.z += g*sinf(elevation); // remove component due to gravity
  scaled_accel.z *= g/1000; // change to mm/s^2
  
  newVel = (scaled_accel.z*cosf(elevation)) * dt + (*prevVel); // get component in world Z axis
  newDisp = newVel*dt + (*prevDisp);
  
  *prevVel = newVel;
  *prevDisp = newDisp;
  
  return;   
}

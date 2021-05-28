#include "accelerometer.h"
#include "l3g4200d.h"
#include "iic.h"
#include <math.h>

void convertUnits(AccelRaw *raw_data, AccelScaled *scaled_data){
    scaled_data->x = (float)(raw_data->x)/250;
    scaled_data->y = (float)(raw_data->y)/250;
    scaled_data->z = (float)(raw_data->z)/250;
}

void getDisplacement(float *prevDisp, float *prevVel, float *accel) {
  AccelRaw raw_accel;
  AccelScaled scaled_accel; // acceleration is in g's ( *9.81 m/s^2)
  
  float elevation;
  float newVel, newDisp; // in mm/s and mm respectively
  float dt = 0.01; // time step (s)
  float g = 9.81; // m/s^2
  float accel_noise = 0.30272; // tolerance to remove measurement noise (mm/s^2)
   
  getRawDataAccel(&raw_accel);
  convertUnits(&raw_accel, &scaled_accel);
  elevation = atanf((scaled_accel.z)/(sqrt((scaled_accel.y)*(scaled_accel.y)+(scaled_accel.x)*(scaled_accel.x))));
  
  // change to m/s^2, then mm/s^2
  scaled_accel.z *= g*1000; 
  scaled_accel.x *= g*1000;
  
  // remove component due to gravity
  scaled_accel.z -= g*sinf(elevation)*1000; 
  scaled_accel.x -= g*cosf(elevation)*1000;
  
  // change readings from body to world frame
  scaled_accel.z *= cosf(elevation);
  scaled_accel.x *= sinf(elevation);
  
  // resolve readings to world Z axes
  *accel = scaled_accel.z - scaled_accel.x - accel_noise;
  if (fabs(*accel) < accel_noise) {
    if ((*accel)*(*prevVel) > 0) {
      *accel *= -1;
    }
  }
  
  // perform Euler integration twice
  newVel = (*accel) * dt + (*prevVel); 
  newDisp = newVel*dt + (*prevDisp);
  
  // return new values for next Euler integration
  *prevVel = newVel;
  *prevDisp = newDisp;
  
  return;   
}

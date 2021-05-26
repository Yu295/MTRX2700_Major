#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */

// need this for string functions
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pll.h"
#include "simple_serial.h"
#include "l3g4200d.h"
#include "servo.h"
#include "lidar.h"

// pan servo after the user has been prompted to stop to begin the mapping process
void panServo(char *buffer);

// get IMU and LIDAR data after the PTU has turned to a new position
void getMeasurements(char *buffer, char azimuth);

void main(void) {
  
  int error_code = NO_ERROR;
  unsigned char buffer[100];
  
  int DEFAULT_ELEVATION = (int)(-asinf((float)HEIGHT_OFF_GROUND/(float)NOMINAL_LIDAR)*180.0/acosf(-1));
  int DEFAULT_AZIMUTH = 0;
  
  // make sure the board is set to 24MHz
  PLL_Init();
  
  // initialise the simple serial
  SCI1_Init(BAUD_9600);
  
  // initialise the sensor suite
  error_code = iicSensorInit();
  
  // write the result of the sensor initialisation to the serial
  if (error_code == NO_ERROR) {
    sprintf(buffer, "NO_ERROR\n");
    // SCI1_OutString(buffer);
  } else {
    sprintf(buffer, "ERROR %d\n", error_code);
    // SCI1_OutString(buffer);    
  }
  
  timer_config();
  PWMConfig();
  
	EnableInterrupts;
  
  for(;;) {
    // stay here until an obstacle is detected in front
    // return to default configuration after done panning
    turnToElevationAzimuth(DEFAULT_ELEVATION, DEFAULT_AZIMUTH, NULL, NULL, NONE);
    delay(3000);
  
    panServo(buffer);
    
    _FEED_COP(); /* feeds the dog */
  } /* loop forever */
  
  /* please make sure that you never leave main */
}

// pans in small increments by increasing elevation then increasing azimuth angle
void panServo(char *buffer) {
  char elevation, azimuth; 
  SERVO_STATE result;
  unsigned char prevDutyE = 0, prevDutyA = 0;
  
  for (elevation = MIN_PAN_ELEVATION; elevation <= MAX_PAN_ELEVATION; elevation++) {
    
    // keep increasing elevation angle until the servo actually moves
    result = turnToElevationAzimuth(elevation, MIN_PAN_AZIMUTH, &prevDutyE, &prevDutyA, ELEVATION);
    
    if (result == SUCCESSFUL_TURN) { 
        getMeasurements(buffer, MIN_PAN_AZIMUTH);
        SCI1_OutString(buffer); 
    
    } else if (result == DUPLICATE_CONFIG) {
      continue;
    }
    
    // pan across the range of azimuth angles at a fixed elevation
    for (azimuth = MIN_PAN_AZIMUTH; azimuth <= MAX_PAN_AZIMUTH; azimuth++) {
      
      result = turnToElevationAzimuth(elevation, azimuth, &prevDutyE, &prevDutyA, AZIMUTH);
      
      if (result == SUCCESSFUL_TURN) {
        getMeasurements(buffer, azimuth);         
        SCI1_OutString(buffer);     
      } else if (result == DUPLICATE_CONFIG) {
        continue;
      }
    }
  }
  
  return;
}

void getMeasurements(char *buffer, char azimuth) {
    int i;
    unsigned long minDist; // taking 5 readings per orientation and recording the minimum
    unsigned long groundDist; // how far the ground is expected to be at the given elevation
    AccelRaw read_accel;
    AccelScaled scaled_accel;
    MagRaw read_magnet;
    MagScaled magnet_average;
    Orientation orientations;
    float conversion = 180.0/(float)acos(-1);  // conversion factor from rad to deg
    
    // initialise average magnetometer readings
    magnet_average.x = 0;
    magnet_average.y = 0;
    magnet_average.z = 0;
    
    // take 10 magnetometer readings per orientation and average
    for (i = 0; i < 10; ++i) {
      getRawDataMagnet(&read_magnet);
      magnet_average.x += 0.1*(float)read_magnet.x;
      magnet_average.y += 0.1*(float)read_magnet.y;
      magnet_average.z += 0.1*(float)read_magnet.z;  
    }
    
    // find actual elevation and heading using IMU
    getRawDataAccel(&read_accel);
    convertUnits(&read_accel, &scaled_accel);
    findOrientation(&orientations, &scaled_accel, &magnet_average);
          
    groundDist = getGroundDistance(-orientations.e); // compute how far the ground should be 
    minDist = groundDist + 1; // initialise minimum distance to be greater than expected ground distance
    delay(400);
    
    // take 5 LIDAR readings and record the minimum. If the minimum is still greater than groundDist,
    // then chances are there are no objects in that direction
    for (i = 0; i < 5; ++i) {
      TIE |= TIE_C1I_MASK;
      delay(100);
      TIE &= ~TIE_C1I_MASK;
      
      if (distance < minDist) {
        minDist = distance;  
      }
    } 
    
    // returns string in format <measured elevation>,<set azimuth>,<LIDAR measurement>,<expected ground distance>
    sprintf(buffer, "%d,%d,%lu,%lu\n", (int)(orientations.e*conversion), azimuth, minDist, groundDist);
    
    return;  
}
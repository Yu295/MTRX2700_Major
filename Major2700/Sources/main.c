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

#define TRUE 1
#define FALSE 0

// pan servo after the user has been prompted to stop to begin the mapping process
void panServo(char *buffer, char FLAT_OFFSET);

// get IMU and LIDAR data after the PTU has turned to a new position
void getMeasurements(char *buffer, char openElevation, char FLAT_OFFSET, char azimuth, char panning);

void main(void) {
  
  int error_code = NO_ERROR;
  unsigned char buffer[50];
  char FLAT_OFFSET = 0;
  char DEFAULT_ELEVATION = (char)(-asinf((float)HEIGHT_OFF_GROUND/(float)NOMINAL_LIDAR)*180.0/acosf(-1));
  char DEFAULT_AZIMUTH = 0;
  float displacement = 0, velocity = 0, accel = 0;
  AccelRaw read_accel;
  AccelScaled scaled_accel; 
  Orientation orientations;
  
  // make sure the board is set to 24MHz
  PLL_Init();
  
  // initialise the simple serial
  SCI1_Init(BAUD_9600);
  
  // initialise the sensor suite
  error_code = iicSensorInit();
  
  // write the result of the sensor initialisation to the serial
  if (error_code == NO_ERROR) {
    sprintf(buffer, "NO_ERROR\n");
    SCI1_OutString(buffer);
  } else {
    sprintf(buffer, "ERROR %d\n", error_code);
    SCI1_OutString(buffer);    
  }
  
  timer_config();
  PWMConfig();
  
	EnableInterrupts;
	
	// get the pitch offset if the surface isn't flat
  turnToElevationAzimuth(0, 0, 0, NULL, NULL, NONE);
  delay(1000);
  getRawDataAccel(&read_accel);
  convertUnits(&read_accel, &scaled_accel);
  findOrientation(&orientations, &scaled_accel, ELEVATION_ONLY, NULL);
  //FLAT_OFFSET = (char)((orientations.e)*180.0/acosf(-1));
  FLAT_OFFSET = 0;
  sprintf(buffer, "Offset (deg): %d\n", FLAT_OFFSET);
  SCI1_OutString(buffer);
  
  DEFAULT_ELEVATION -= FLAT_OFFSET;
  delay(1000);
  
  for(;;) {
  
    // stay here until an obstacle is detected in front
    // return to default configuration after done panning
    turnToElevationAzimuth(DEFAULT_ELEVATION, FLAT_OFFSET, DEFAULT_AZIMUTH, NULL, NULL, NONE);
    
    /* Trying to get displacement from accelerometer
    getDisplacement(&displacement, &velocity, &accel);
    sprintf(buffer, "a: %.5f, v: %.5f, d: %.5f\n", accel, velocity, displacement);
    getRawDataAccel(&read_accel);
    convertUnits(&read_accel, &scaled_accel);
    sprintf(buffer, "ax: %.5f, ay: %.5f, az: %.5f\n", scaled_accel.x, scaled_accel.y, scaled_accel.z);
    sprintf(buffer, "%f\n", accel);
    */
    delay(10);
  
    panServo(buffer, FLAT_OFFSET);
    
    _FEED_COP(); /* feeds the dog */
  } /* loop forever */
  
  /* please make sure that you never leave main */
}

// pans in small increments by increasing elevation then increasing azimuth angle
void panServo(char *buffer, char FLAT_OFFSET) {
  char elevation, azimuth; 
  SERVO_STATE result;
  unsigned char prevDutyE = 0, prevDutyA = 0;
  
  for (elevation = MIN_PAN_ELEVATION - FLAT_OFFSET; elevation <= MAX_PAN_ELEVATION - FLAT_OFFSET; elevation++) {
    
    // keep increasing elevation angle until the servo actually moves
    result = turnToElevationAzimuth(elevation, FLAT_OFFSET, MIN_PAN_AZIMUTH, &prevDutyE, &prevDutyA, ELEVATION);
    
    if (result == SUCCESSFUL_TURN) { 
        delay(1500);
        getMeasurements(buffer, elevation, FLAT_OFFSET, MIN_PAN_AZIMUTH, TRUE);
        SCI1_OutString(buffer); 
    
    } else if (result == DUPLICATE_CONFIG) {
      continue;
    }
    
    // pan across the range of azimuth angles at a fixed elevation
    for (azimuth = MIN_PAN_AZIMUTH; azimuth <= MAX_PAN_AZIMUTH; azimuth++) {
      
      result = turnToElevationAzimuth(elevation, FLAT_OFFSET, azimuth, &prevDutyE, &prevDutyA, AZIMUTH);
      
      if (result == SUCCESSFUL_TURN) {
        delay(1000);
        getMeasurements(buffer, elevation, FLAT_OFFSET, azimuth, TRUE);         
        SCI1_OutString(buffer);     
      }
    }
  }
  
  return;
}

void getMeasurements(char *buffer, char openElevation, char FLAT_OFFSET, char azimuth, char panning) {
    int i;
    unsigned long minDist; // taking 5 readings per orientation and recording the minimum
    unsigned long groundDist; // how far the ground is expected to be at the given elevation
    AccelRaw read_accel;
    AccelScaled scaled_accel;
    Orientation orientations;
    float conversion = 180.0/(float)acos(-1);  // conversion factor from rad to deg
    
    // take 10 LIDAR readings and record the minimum. If the minimum is still greater than groundDist,
    // then chances are there are no objects in that direction
    
    groundDist = getGroundDistance(-orientations.e); // compute how far the ground should be 
    minDist = groundDist + 1; // initialise minimum distance to be greater than expected ground distance
    
    for (i = 0; i < 10; ++i) {
      TIE |= TIE_C1I_MASK;
      delay(20);
      TIE &= ~TIE_C1I_MASK;
      
      if (distance < minDist) {
        minDist = distance;  
      }
    } 
    
    // find actual elevation using IMU
    if (panning) {  
      getRawDataAccel(&read_accel);
      convertUnits(&read_accel, &scaled_accel);
      sprintf(buffer, "ax: %.5f, ay: %.5f, az:%.5f\n", scaled_accel.x, scaled_accel.y, scaled_accel.z);
      SCI1_OutString(buffer);
      findOrientation(&orientations, &scaled_accel, ELEVATION_ONLY, NULL);
      
      // returns string in format <measured elevation>,<ideal elevation>,<set azimuth>,<LIDAR measurement>,<expected ground distance>
      sprintf(buffer, "%d,%d,%d,%lu,%lu\n", (int)(orientations.e*conversion), openElevation + FLAT_OFFSET, azimuth, minDist, groundDist);
    } else {
      // returns string in format <LIDAR measurement>,<expected ground distance>
      sprintf(buffer, "%lu,%lu\n", minDist, groundDist);
    }  
    return;  
}
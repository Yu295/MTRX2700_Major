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
void panServo(char *buffer);

// get IMU and LIDAR data after the PTU has turned to a new position
char getMeasurements(char *buffer, char openElevation, char azimuth, char panning);

void main(void) {
  
  int error_code = NO_ERROR;
  char buffer[100];
  unsigned long minDistance;
  
  int DEFAULT_ELEVATION = (int)(-asinf((float)HEIGHT_OFF_GROUND/(float)NOMINAL_LIDAR)*180.0/acosf(-1));
  int DEFAULT_AZIMUTH = 0;
  float displacement = 0, velocity = 0, accel = 0;
  AccelRaw read_accel;
  AccelScaled scaled_accel;
  
  // make sure the board is set to 24MHz     
  PLL_Init();
  
  // initialise the simple serial
  SCI1_Init(BAUD_9600);
  
  // initialise the sensor suite
  error_code = iicSensorInit();
  
  // write the result of the sensor initialisation to the serial
  if (error_code == NO_ERROR) {
    //sprintf(buffer, "NO_ERROR\n");
    //SCI1_OutString(buffer);
  } else {
    //sprintf(buffer, "ERROR %d\n", error_code);
    //SCI1_OutString(buffer);    
  }
  
  timer_config();
  PWMConfig();
  
	EnableInterrupts;
  turnToElevationAzimuth(0, 0, NULL, NULL, NONE);
  for(;;) {
  
    // stay here until an obstacle is detected in front
    // return to default configuration after done panning
    turnToElevationAzimuth(DEFAULT_ELEVATION, DEFAULT_AZIMUTH, NULL, NULL, NONE);
    delay(1000);
    
    // keep reading distances until an obstacle is detected 
    
    while(getMeasurements(buffer,DEFAULT_ELEVATION,DEFAULT_AZIMUTH,FALSE));
    SCI1_OutString(buffer); // send through to serial to indicate to MATLAB to play voice prompts  
    flushBuffer(buffer);
    SCI1_InString(buffer); // wait until MATLAB is done playing voice prompts
    
    if (*buffer == '1') {
      Orientation orientations;
      MagRaw read_magnet;
      MagScaled avg_magnet;
      panServo(buffer); // start panning and sending data to MATLAB mapping/guidance module
      turnToElevationAzimuth(0, 0, NULL, NULL, NONE); // set PTU to flat orientation to get magnetometer readings
      flushBuffer(buffer); 
      SCI1_InString(buffer); // wait until MATLAB is done with mapping

      /************** GUIDANCE ****************/      
      
      while(*buffer != '3') {
        int i;
        avg_magnet.x = 0;
        avg_magnet.y = 0;
        avg_magnet.z = 0;
        
        
        for (i = 0; i < 10; ++i) {
          getRawDataMagnet(&read_magnet);
          avg_magnet.x += 0.1 * (float)read_magnet.x;
          avg_magnet.y += 0.1 * (float)read_magnet.y;
          avg_magnet.z += 0.1 * (float)read_magnet.z;
        }
        
        /*getRawDataMagnet(&read_magnet);
        avg_magnet.x = (float)read_magnet.x;
        avg_magnet.y = (float)read_magnet.y;
        avg_magnet.z = (float)read_magnet.z;*/
        
        getRawDataAccel(&read_accel); // get accelerometer reading
        convertUnits(&read_accel, &scaled_accel); // scale accelerometer reading
        findOrientation(&orientations, &scaled_accel, BEARING, &avg_magnet); // get current bearing
        sprintf(buffer, "%d\n", (int)(orientations.h * 180.0/acosf(-1)));
        delay(500);
        SCI1_OutString(buffer); // send current bearing through serial to MATLAB
        flushBuffer(buffer);
        SCI1_InString(buffer);  // wait for signal to determine if user is facing the right way
      }
    }
    
    /*SCI1_InString(buffer);
    SCI1_OutString(buffer);*/
    _FEED_COP(); /* feeds the dog */
  } /* loop forever */
  
  /* please make sure that you never leave main */
}

// pans in small increments by increasing elevation then increasing azimuth angle
void panServo(char *buffer) {
  char elevation, azimuth; 
  SERVO_STATE result;
  unsigned char prevDutyE = 0, prevDutyA = 0;
  
  for (elevation = MIN_PAN_ELEVATION; elevation <= MAX_PAN_ELEVATION; elevation ++) {
    
    // keep increasing elevation angle until the servo actually moves
    result = turnToElevationAzimuth(elevation, MIN_PAN_AZIMUTH, &prevDutyE, &prevDutyA, ELEVATION);
    
    if (result == SUCCESSFUL_TURN) { 
        delay(1500);
        getMeasurements(buffer, elevation, MIN_PAN_AZIMUTH, TRUE);;
        SCI1_OutString(buffer); 
    
    } else if (result == DUPLICATE_CONFIG) {
      continue;
    }
    
    // pan across the range of azimuth angles at a fixed elevation
    for (azimuth = MIN_PAN_AZIMUTH; azimuth <= MAX_PAN_AZIMUTH; azimuth++) {
      
      result = turnToElevationAzimuth(elevation, azimuth, &prevDutyE, &prevDutyA, AZIMUTH);
      
      if (result == SUCCESSFUL_TURN) {
        delay(400);
        getMeasurements(buffer, elevation, azimuth, TRUE);         
        SCI1_OutString(buffer);     
      }
    }
  }
  
  sprintf(buffer, "5\n");
  SCI1_OutString(buffer);
  return;
}

// returns TRUE if an obstacle is detected and FALSE otherwise
char getMeasurements(char *buffer, char openElevation, char azimuth, char panning) {
    int i;
    unsigned long minDist; // taking 5 readings per orientation and recording the minimum
    unsigned long groundDist; // how far the ground is expected to be at the given elevation
    AccelRaw read_accel;
    AccelScaled scaled_accel;
    Orientation orientations;
    float conversion = 180.0/(float)acos(-1);  // conversion factor from rad to deg
    
    // take 10 LIDAR readings and record the minimum. If the minimum is still greater than groundDist,
    // then chances are there are no objects in that direction
    
    getRawDataAccel(&read_accel);
    convertUnits(&read_accel, &scaled_accel);
    findOrientation(&orientations, &scaled_accel, ELEVATION_ONLY, NULL);
    
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
      // writes to buffer string in format <measured elevation>,<set elevation>,<set azimuth>,<LIDAR measurement>,<expected ground distance>
      sprintf(buffer, "%d,%d,%d,%lu,%lu\n", (int)(orientations.e*conversion), openElevation, azimuth, minDist, groundDist);
      return TRUE;
    }
    
    // writes to buffer string in format <LIDAR measurement>,<expected ground distance>
    sprintf(buffer, "%lu,%lu\n", minDist, groundDist);
    return (minDist > groundDist);
}
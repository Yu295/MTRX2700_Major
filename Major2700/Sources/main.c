/************************** Major Project 2021 ***************************
*                                                                        *
* Audio-Based Guided Navigation for Elderly/Visually Impaired People     *
*                                                                        *
* Authors: Jason Lai, Reihana Tsao, Yujiao Cao                           *
*                                                                        *
* Date: 06/05/2021                                                       *
**************************************************************************/

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

// whether or not to pan
typedef enum {
  NOT_PANNING = 0,
  PANNING
} IS_PANNING;

// pan servo after the user has been prompted to stop to begin the mapping process
void panServo(char *buffer);

// get IMU and LIDAR data after the PTU has turned to a new position
char getMeasurements(char *buffer, char openElevation, char azimuth, IS_PANNING panning);

void main(void) {
  
  int error_code = NO_ERROR;
  char buffer[100];            // buffer to store strings sent through SCI1
  unsigned long minDistance;   // LIDAR distance reading
  
  // set the default elevation to be s.t. the ground returns a reading of approx NOMINAL_LIDAR if no obstacles are detected
  int DEFAULT_ELEVATION = (int)(-asinf((float)HEIGHT_OFF_GROUND/(float)NOMINAL_LIDAR)*180.0/acosf(-1));
  
  // look forward by default
  int DEFAULT_AZIMUTH = 0; 
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
  
  timer_config(); // configure E-Clock timer for use by LIDAR and PWM block
  PWMConfig();    // configure individual clocks for PWM block
  
	EnableInterrupts;

  for(;;) {
  
    // turn to the default configuration
    turnToElevationAzimuth(DEFAULT_ELEVATION, DEFAULT_AZIMUTH, NULL, NULL, NONE);
    
    // make sure the PTU stops moving before beginning to take measurements
    delay(1000);
    
    // keep reading distances until an obstacle is detected  
    while(getMeasurements(buffer, DEFAULT_ELEVATION, DEFAULT_AZIMUTH, NOT_PANNING));
    
    /**************** VOICE PROMPTS ***********/
    SCI1_OutString(buffer);                                                 // send data to serial to indicate to MATLAB to play prompts  
    flushBuffer(buffer);
    SCI1_InString(buffer);                                                  // wait until MATLAB is done playing voice prompts
    
    /**************** MAPPING ***************/
    if (*buffer == START_MAPPING_FLAG) {
      Orientation orientations;
      MagRaw read_magnet;
      MagScaled avg_magnet;
      panServo(buffer);                               // start panning and sending data to MATLAB mapping/guidance module
      turnToElevationAzimuth(0, 0, NULL, NULL, NONE); // set PTU to flat orientation to get magnetometer readings
      delay(400);                                     // short pause to give the PTU enough time to move
      
      getRawDataAccel(&read_accel);                   // get accelerometer reading and scale it
      convertUnits(&read_accel, &scaled_accel);       
      orientations.e = findElevation(&scaled_accel);  // find current elevation for use in bearing calculation
      SCI1_InString(buffer);                          // wait until MATLAB is done with mapping

      /************** GUIDANCE ****************/      
      
      while(*buffer != DONE_TURNING_FLAG) {
        int i;
        avg_magnet.x = 0;
        avg_magnet.y = 0;
        avg_magnet.z = 0;
        
        // taking 10 magnetometer readings per orientation and averaging
        for (i = 0; i < 10; ++i) {
          getRawDataMagnet(&read_magnet);
          avg_magnet.x += 0.1 * (float)read_magnet.x;
          avg_magnet.y += 0.1 * (float)read_magnet.y;
          avg_magnet.z += 0.1 * (float)read_magnet.z;
        }
        
        // comment out next 3 lines if using the initial elevation only
        /*getRawDataAccel(&read_accel);     
        convertUnits(&read_accel, &scaled_accel);
        orientations.e = findElevation(&scaled_accel);*/
           
        orientations.b = findBearing(orientations.e, &avg_magnet);            // get current bearing
        sprintf(buffer, "%d\n", (int)(orientations.b * 180.0/acosf(-1)));
        delay(500);                                                           // make sure MATLAB is ready to receive serial data before sending
        SCI1_OutString(buffer);                                               // send current bearing through serial to MATLAB      
        SCI1_InString(buffer);                                                // wait for signal to determine if user is facing the right way
      }
    }
    
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
        delay(1500);
        getMeasurements(buffer, elevation, MIN_PAN_AZIMUTH, PANNING);
        SCI1_OutString(buffer); 
    
    } else if (result == DUPLICATE_CONFIG) {
      continue;
    }
    
    // pan across the range of azimuth angles at a fixed elevation
    for (azimuth = MIN_PAN_AZIMUTH; azimuth <= MAX_PAN_AZIMUTH; azimuth++) {
      
      result = turnToElevationAzimuth(elevation, azimuth, &prevDutyE, &prevDutyA, AZIMUTH);
      
      if (result == SUCCESSFUL_TURN) {
        delay(400);
        getMeasurements(buffer, elevation, azimuth, PANNING);  // get the distance, elevation and azimuth measurements       
        SCI1_OutString(buffer);     
      }
    }
  }
  
  sprintf(buffer, DONE_PANNING_FLAG); // send flag to indicate no more mapping data is to be read in
  SCI1_OutString(buffer);
  return;
}

// returns TRUE if an obstacle is detected and FALSE otherwise
char getMeasurements(char *buffer, char openElevation, char azimuth, IS_PANNING panning) {
    int i;
    unsigned long minDist;                     // taking 10 readings per orientation and recording the minimum
    unsigned long groundDist;                  // how far the ground is expected to be at the given elevation
    AccelRaw read_accel;
    AccelScaled scaled_accel;
    Orientation orientations;
    float conversion = 180.0/(float)acos(-1);  // conversion factor from rad to deg
    
    // take 10 LIDAR readings and record the minimum. If the minimum is still greater than groundDist,
    // then chances are there are no objects in that direction
    
    getRawDataAccel(&read_accel);
    convertUnits(&read_accel, &scaled_accel);
    orientations.e = findElevation(&scaled_accel);
    
    groundDist = getGroundDistance(-orientations.e); // compute how far the ground should be 
    minDist = groundDist + 1;                        // initialise minimum distance to be greater than expected ground distance
    
    for (i = 0; i < 10; ++i) {
      TIE |= TIE_C1I_MASK;
      delay(20);
      TIE &= ~TIE_C1I_MASK;
      
      if (distance < minDist && distance > MIN_RANGE) {
        minDist = distance;  
      }
    } 
    
    // find actual elevation using IMU
    if (panning == PANNING) {  
      
      // writes to buffer string in format <measured elevation>,<set elevation>,<set azimuth>,<LIDAR measurement>,<expected ground distance>
      sprintf(buffer, "%d,%d,%d,%lu,%lu\n", (int)(orientations.e*conversion), openElevation, azimuth, minDist, groundDist);
      return TRUE;
    }
    
    // writes to buffer string in format <LIDAR measurement>,<expected ground distance>
    sprintf(buffer, "%lu,%lu\n", minDist, groundDist);
    return (minDist > groundDist);
}
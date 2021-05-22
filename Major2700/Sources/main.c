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

void panServo(void);

void main(void) {

  AccelRaw read_accel;
  AccelScaled scaled_accel;
  
  GyroRaw read_gyro;
  
  MagRaw read_magnet;
  MagScaled norm_magnet;
  MagScaled magnet_average;
  
  Orientation orientations;
  
  unsigned long i;
  char j;
  
  int error_code = NO_ERROR;
  unsigned char buffer[100];
  float conversion = 180.0/(float)acos(-1);
  
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
  
  for(;;) {
    
    turnToElevationAzimuth(0, 0, NULL, NULL, NONE);
    magnet_average.x = 0;
    magnet_average.y = 0;
    magnet_average.z = 0;
    for (j = 0; j < 10; ++j) {
      getRawDataMagnet(&read_magnet);
      normaliseMagnet(&norm_magnet, &read_magnet);
      magnet_average.x += 0.1*(float)read_magnet.x;
      magnet_average.y += 0.1*(float)read_magnet.y;
      magnet_average.z += 0.1*(float)read_magnet.z;  
    }
    getRawDataAccel(&read_accel);    convertUnits(&read_accel, &scaled_accel);
    findInitOrientation(&orientations, &scaled_accel, &magnet_average);
    
    sprintf(buffer, "mx: %.2f my: %.2f mz:%.2f e:%.2f a:%.2f\n", magnet_average.x, magnet_average.y, magnet_average.z, orientations.e*conversion, orientations.a*conversion);
    SCI1_OutString(buffer);
    //panServo();
    
    _FEED_COP(); /* feeds the dog */
  } /* loop forever */
  
  /* please make sure that you never leave main */
}

// pans in small increments by increasing elevation then increasing azimuth angle
void panServo(void) {
  char elevation, azimuth; 
  unsigned char buf[50];
  SERVO_STATE result;
  unsigned char prevDutyE = 0, prevDutyA = 0;
  long dt;
  int ovB, ovC;
  
  for (elevation = MIN_PAN_ELEVATION; elevation <= MAX_PAN_ELEVATION; elevation++) {
    
    // keep increasing elevation angle until the servo actually moves
    result = turnToElevationAzimuth(elevation, MIN_PAN_AZIMUTH, &prevDutyE, &prevDutyA, ELEVATION);
    
    if (result == SUCCESSFUL_TURN) {
      int i;
      unsigned long minDist = MAX_RANGE;
        
        delay(1000); // give it enough time to move to the left
        
        for (i = 0; i < 5; ++i) {
          TIE |= TIE_C1I_MASK;
          delay(100);
          TIE &= ~TIE_C1I_MASK;
          
          if (distance < minDist) {
            minDist = distance;  
          }
        }
        
        //sprintf(buf, "0,0,%u,%u,%ld,%u,%u,%lu\n", time_1, time_2, dt, ovB, ovC, distance);
        sprintf(buf, "%d,%d,%lu\n", elevation, azimuth, minDist); 
        SCI1_OutString(buf); 
    
    } else if (result == DUPLICATE_CONFIG) {
      continue;
    }
    
    // pan across the range of azimuth angles at a fixed elevation
    for (azimuth = MIN_PAN_AZIMUTH; azimuth <= MAX_PAN_AZIMUTH; azimuth++) {
      
      result = turnToElevationAzimuth(elevation, azimuth, &prevDutyE, &prevDutyA, AZIMUTH);
      
      if (result == SUCCESSFUL_TURN) {
        int i;
        unsigned long minDist = MAX_RANGE;
        
        delay(400);
        
        for (i = 0; i < 5; ++i) {
          TIE |= TIE_C1I_MASK;
          delay(100);
          TIE &= ~TIE_C1I_MASK;
          
          if (distance < minDist) {
            minDist = distance;  
          }
        }
        
        //sprintf(buf, "0,0,%u,%u,%ld,%u,%u,%lu\n", time_1, time_2, dt, ovB, ovC, distance);
        sprintf(buf, "%d,%d,%lu\n", elevation, azimuth, minDist); 
        SCI1_OutString(buf);
      
      } else if (result == DUPLICATE_CONFIG) {
        continue;
      }
    }
  }
  
  // return to default configuration after done panning
  //turnToElevationAzimuth(DEFAULT_ELEVATION, DEFAULT_AZIMUTH, &prevDutyE, &prevDutyA, NONE);
  //delay(1000);
  
  return;
}
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

void main(void) {

  AccelRaw read_accel;
  AccelScaled scaled_accel;
  
  GyroRaw read_gyro;
  
  MagRaw read_magnet;
  Orientation orientations;
  
  unsigned long i = 0;
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
    sprintf(buffer, "NO_ERROR");
    SCI1_OutString(buffer);
  } else {
    sprintf(buffer, "ERROR %d");
    SCI1_OutString(buffer);    
  }
  
  PWMConfig();
	EnableInterrupts;
  
  getRawDataMagnet(&read_magnet);
  getRawDataAccel(&read_accel);
  convertUnits(&read_accel, &scaled_accel);
  findInitOrientation(&orientations, &scaled_accel, &read_magnet);
  sprintf(buffer, "Angles (deg) Elevation: %.2f, Azimuth: %.2f\n", orientations.e*conversion, orientations.a*conversion);
  SCI1_OutString(buffer);
  //sprintf(buffer, "Read in x: %.2f, y: %.2f, z: %.2f\n", scaled_accel.x, scaled_accel.y, scaled_accel.z);
  //SCI1_OutString(buffer);	
  
  for(;;) {
    PWMConfig();  
    _FEED_COP(); /* feeds the dog */
  } /* loop forever */
  
  /* please make sure that you never leave main */
}

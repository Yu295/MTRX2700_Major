#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */

// need this for string functions
#include <stdio.h>
#include "lidar.h"
#include "pll.h"
#include "simple_serial.h"
#include "l3g4200d.h"


void main(void) {

  AccelRaw read_accel;
  AccelScaled scaled_accel;

  GyroRaw read_gyro;
  
  MagRaw read_magnet;
  
  long i;
  int error_code = NO_ERROR;
  unsigned char buffer[64];

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
  
  // configure timer for lidar operation
  timer_config();
  
	EnableInterrupts;
  
  for(;;) {
    
    #ifndef SIMULATION_TESTING
    
    // read the raw values
    getRawDataGyro(&read_gyro);
    getRawDataAccel(&read_accel);
    getRawDataMagnet(&read_magnet);
    
    #else
    read_gyro.x = 123; read_gyro.y = 321; read_gyro.z = 3000;
    read_accel.x = 124; read_accel.y = 421; read_accel.z = 3001;
    read_magnet.x = 125; read_magnet.y = 521;read_magnet.z = 3002;
    
    #endif 
    
    // convert the acceleration to a scaled value
    convertUnits(&read_accel, &scaled_accel);    
    
    // format the string of the sensor data to go the the serial
    sprintf(buffer, "%.2f, %.2f, %.2f, %d, %d, %d, %d, %d, %d \r\n", scaled_accel.x, scaled_accel.y, scaled_accel.z, read_gyro.x, read_gyro.y, read_gyro.z, read_magnet.x, read_magnet.y, read_magnet.z);
    
    // output the data to serial
    SCI1_OutString(buffer);
    
    /*if (time_flag) {
      
      sprintf(buffer, "\ndistance: %lu, time_1: %u, time_2: %u, overflow: %u \n" ,distance, time_1, time_2, overflow);
    } else {
      sprintf(buffer, "\nToo far! Object should be within 2.73m!\n");
    }
    
    SCI1_OutString(buffer);
    for(i = 0; i < 99999; ++i);
    i = 0;*/
    
    _FEED_COP(); /* feeds the dog */
  } /* loop forever */
  
  /* please make sure that you never leave main */
}

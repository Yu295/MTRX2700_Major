/** \file iic.c
 * @brief Functions to read, initialise and test sensors using IIC
 * Most of this code was downloaded from the course website and modified - link in report
 */

/*
 *Implementing code that utilises the IIC. 
 *The functions used in this file come from the file "iicAuxillary".
*/

// Serial port should be set to 9600 baud rate
// check if serial port on Dragon12 is configured somewhere else in our main program
// check if the function checking the e-clock frequency is needed here
// Check if we need timer and alarm functions - if we do copy them over

#include "l3g4200d.h"


#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */

#include "l3g4200d_definitions.h"
#include <math.h>


// structure containing the config parameters for the accelerometer
typedef struct ACCELEROMETER_CFG_STRUCT {
  uint8_t power_ctl_register;
  uint8_t power_ctl_value;
  uint8_t data_format_register;
  uint8_t data_format_value;
} ACCELEROMETER_CFG_STRUCT;

ACCELEROMETER_CFG_STRUCT accelerometer_cfg = {ADXL345_POWER_CTL, 0x08, ADXL345_DATA_FORMAT, 0x08};
  

// structure containing the config parameters for the gyroscope  
typedef struct GYRO_CFG_STRUCT {
  uint8_t ctl_register;
  uint8_t ctl_value;
} GYRO_CFG_STRUCT;

GYRO_CFG_STRUCT gyro_cfg = {L3G4200D_CTRL_REG1, 0x0f};


// structure containing the config parameters for the magnetometer
typedef struct MAG_CFG_STRUCT {
  uint8_t ctl_register;
  uint8_t ctl_value;
} MAG_CFG_STRUCT;

MAG_CFG_STRUCT mag_cfg = {HM5883_MODE_REG, 0x00};


// initialise functions for each sensor
IIC_ERRORS accel_init(void);
IIC_ERRORS magnet_init(void);
IIC_ERRORS gyro_init(void);


// Get the raw magnetic data from the sensor
IIC_ERRORS getRawDataMagnet(MagRaw *raw_data)
{        
    iic_request_data(magnet_wr, HM5883_DATAX0);
    return iic_read_data(magnet_rd, (uint8_t *)raw_data, sizeof(MagRaw));
}


// Get the raw acceleration data from the sensor
IIC_ERRORS getRawDataAccel(AccelRaw *raw_data)
{
    iic_request_data(accel_wr, ADXL345_DATAX0);
    return iic_read_data(accel_rd, (uint8_t *)raw_data, sizeof(AccelRaw));
}


// Get the raw gyro data from the sensor
IIC_ERRORS getRawDataGyro(GyroRaw *raw_data)
{
    iic_request_data(gyro_wr, L3G4200D_OUT_XYZ_CONT);    
    return iic_read_data(gyro_rd, (uint8_t *)raw_data, sizeof(GyroRaw));
}



// Initialise each sensor
IIC_ERRORS iicSensorInit()
{
   volatile IIC_ERRORS error_code = NO_ERROR;
   
   // start the iic running
   iicinit(IIC_100KHZ);
   
   // Set up each of the sensors
   error_code = gyro_init();
   if (error_code != NO_ERROR)
     return error_code;
    
   error_code = accel_init();
   if (error_code != NO_ERROR)
     return error_code;
    
   error_code = magnet_init();
   if (error_code != NO_ERROR)
     return error_code;
}




// initialise the accelerometer
IIC_ERRORS accel_init(void)
{
  return iic_send_data(accel_wr, (uint8_t*)&accelerometer_cfg, sizeof(ACCELEROMETER_CFG_STRUCT));
}



// initialise the gyros
IIC_ERRORS gyro_init(void)
{
  return iic_send_data(gyro_wr, (uint8_t*)&gyro_cfg, sizeof(GYRO_CFG_STRUCT));
}



// initialise the magnetometers
IIC_ERRORS magnet_init(void)
{
  return iic_send_data(magnet_wr, (uint8_t*)&mag_cfg, sizeof(MAG_CFG_STRUCT));
}


// calculate elevation and azimuth angles from initial accelerometer and magnetometer readings taken while stationary
void findInitOrientation(Orientation *orientations, AccelScaled *scaled_data, MagRaw *mag_data) {
  float x, y;
  float norm = sqrtf((mag_data->x)*(mag_data->x) + (mag_data->y)*(mag_data->y) + (mag_data->z)*(mag_data->z));
  float norm_x = (mag_data->x) / norm;
  float norm_y = (mag_data->y) / norm;
  float norm_z = (mag_data->z) / norm;
  float a_rolling = atanf((scaled_data->y)/((scaled_data->z)*(scaled_data->z)+(scaled_data->x)*(scaled_data->x))); 
  
  orientations->e = atanf((scaled_data->z)/((scaled_data->y)*(scaled_data->y)+(scaled_data->x)*(scaled_data->x)));
  
  //z = (mag_data->z)*cosf(orientations->e) + (mag_data->y)*sinf(orientations->e)*sinf(a_rolling) + (mag_data->x)*cosf(a_rolling)*sinf(orientations->e);
  //y = (mag_data->y)*cosf(a_rolling) - (mag_data->x)*sinf(a_rolling);
  x = (mag_data->z)*cosf(orientations->e);
  y = (mag_data->y);
  orientations->y = y;
  orientations->x = x;
  orientations->a = atan2f(y, x);
  return;     
}

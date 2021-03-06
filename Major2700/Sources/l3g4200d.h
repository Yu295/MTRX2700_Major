#ifndef L3G4200D_HEADER
#define L3G4200D_HEADER

// file l3g4200d.h
//   public interface for the l3g4200d sensor suite
//   this interface allows you to initalise the sensors
//   and read the raw data values
//   
//   these values require further scaling according to the 
//   datasheets

#include "iic.h"
#include "accelerometer.h"

// data structures containing the raw values
typedef struct GyroRaw {
  int x;    // x rotational velocity
  int y;    // y rotational velocity
  int z;    // z rotational velocity
} GyroRaw;

typedef struct MagRaw {
  int x;    // x B field density (Ga)
  int y;    // y B field density (Ga)
  int z;    // z B field density (Ga)
} MagRaw;

// data structure containing magnetometer data stored as floats (for average computation)
typedef struct MagScaled {
  float x;
  float y;
  float z;
} MagScaled;

// data structure containing orientation values
typedef struct Orientation {
  float e;      // elevation (rad) between -pi/2 and pi/2
  float b;      // bearing (rad) between 0 and 2*pi
} Orientation;

// Initialise each sensor
IIC_ERRORS iicSensorInit();

// Get the raw acceleration data from the sensor
IIC_ERRORS getRawDataAccel(AccelRaw *raw_data);


// Get the raw magnetic data from the sensor
IIC_ERRORS getRawDataMagnet(MagRaw *raw_data);


// Get the raw gyro data from the sensor
IIC_ERRORS getRawDataGyro(GyroRaw *raw_data);


// Calculate bearing from elevation and magnetometer reading
float findBearing (float elevation, MagScaled *mag_data);

#endif
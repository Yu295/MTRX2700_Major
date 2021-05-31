#ifndef ACCELEROMETER_HEADER
#define ACCELEROMETER_HEADER


// data structures containing the raw values
typedef struct AccelRaw {
  int x;
  int y;
  int z;
} AccelRaw;


// data structures containing the raw values
typedef struct AccelScaled {
  float x;
  float y;
  float z;
} AccelScaled;

#define M_TO_MM 1000

void convertUnits(AccelRaw *raw_data, AccelScaled *scaled_data);
void getDisplacement(float *prevVel, float *prevDisp, float *accel);

#endif
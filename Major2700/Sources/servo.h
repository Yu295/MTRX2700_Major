#ifndef SERVO_HEADER
#define SERVO_HEADER

// rotational degree of freedom to check for duplication during servo panning
typedef enum {
  ELEVATION,
  AZIMUTH,
  NONE  
} ANGLE;

// status of servo motion
typedef enum {
  SUCCESSFUL_TURN,
  INVALID_ELEVATION,
  INVALID_AZIMUTH,
  DUPLICATE_CONFIG
} SERVO_STATE;

// desired ranges of motion of the PTU (measured with respect to the board) 
#define MIN_PAN_ELEVATION -45
#define MAX_PAN_ELEVATION 45
#define MIN_PAN_AZIMUTH -90
#define MAX_PAN_AZIMUTH 90


void PWMConfig(void);
SERVO_STATE turnToElevationAzimuth(char elevation, char azimuth, unsigned char *prevDutyE, unsigned char *prevDutyA, ANGLE duplicate);

#endif
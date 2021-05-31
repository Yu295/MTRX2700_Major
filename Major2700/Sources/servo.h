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
#define CALIBRATION_NIGHTY 102   // calibration factor at -90 and 90 degrees
#define CALIBRATION_ZERO 1.5     // calibration factor at 0 dergees
#define PWMPER_CONVERSION 12.75  //PWMPERx/20 convertion factor



void PWMConfig(void);
SERVO_STATE turnToElevationAzimuth(char elevation, char azimuth, unsigned char *prevDutyE, unsigned char *prevDutyA, ANGLE duplicate);

#endif
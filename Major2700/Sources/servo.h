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
#define MAX_PAN_ELEVATION 30
#define MIN_PAN_AZIMUTH -90
#define MAX_PAN_AZIMUTH 90
#define CALIBRATION_SLOPE_INV 102   // inverse of calibration slope
#define CALIBRATION_ZERO 1.5     // calibration constant at 0 dergees
#define PWMPER_CONVERSION 12.75  //PWMPERx/20 conversion factor

// configures the channels PWM7 and PWM5 (connected to PTU) for servo control
void PWMConfig(void);

// turns the PTU to an orientation specified by elevation and azimuth angles
// accounts for duplicate configurations due to rounding by accepting the previous duty ratios calculated and angle type
// to check for duplication
SERVO_STATE turnToElevationAzimuth(char elevation, char azimuth, unsigned char *prevDutyE, unsigned char *prevDutyA, ANGLE duplicate);

#endif
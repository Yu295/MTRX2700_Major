#ifndef SERVO_HEADER
#define SERVO_HEADER

typedef enum {
  ELEVATION,
  AZIMUTH,
  NONE  
} ANGLE;

typedef enum {
  SUCCESSFUL_TURN,
  INVALID_ELEVATION,
  INVALID_AZIMUTH,
  DUPLICATE_CONFIG
} SERVO_STATE;

#define MIN_PAN_ELEVATION -90
#define MAX_PAN_ELEVATION 90
#define MIN_PAN_AZIMUTH -90
#define MAX_PAN_AZIMUTH 90

void PWMConfig(void);
void calibrateDist(void);
SERVO_STATE turnToElevationAzimuth(char elevation, char azimuth, unsigned char *prevDutyE, unsigned char *prevDutyA, ANGLE duplicate);

#endif
#ifndef LIDAR_HEADER
#define LIDAR_HEADER

extern volatile unsigned long distance;
extern volatile unsigned int time_1;
extern volatile unsigned int time_2;
extern volatile unsigned int obstacle;

#define MAX_RANGE 2730 // maximum lidar range in mm. Beyond this, readings are unreliable.
#define HEIGHT_OFF_GROUND 1050 // rollator seat height (mm)

void timer_config(void);
void check_obstacle(void);

unsigned long getGroundDistance(float declination);

__interrupt void TC1_ISR(void);
__interrupt void TOF_ISR(void);



#endif



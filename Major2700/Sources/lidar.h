#ifndef LIDAR_HEADER
#define LIDAR_HEADER

extern volatile unsigned long distance;  // LIDAR distance reading (mm)
extern volatile unsigned int time_1;     // timer reading for rising edge
extern volatile unsigned int time_2;     // timer reading for falling edge
extern volatile unsigned int obstacle;

#define MAX_RANGE 2730 // maximum lidar range in mm. Beyond this, readings are unreliable.
#define NOMINAL_LIDAR 1930 // to calculate the default elevation angle
#define HEIGHT_OFF_GROUND 750 // rollator seat height (mm)

// configure TC1 for input capture
void timer_config(void);

// compute the expected distance reading of the ground for reference
unsigned long getGroundDistance(float declination);

// interrupt service routine for input capture in TC1
__interrupt void TC1_ISR(void);

// interrupt service routine to count number of timer overflows
__interrupt void TOF_ISR(void);

#endif



#ifndef LIDAR_HEADER
#define LIDAR_HEADER

extern volatile unsigned long distance;  // LIDAR distance reading (mm)
extern volatile unsigned int time_1;     // timer reading for rising edge
extern volatile unsigned int time_2;     // timer reading for falling edge
extern volatile unsigned int obstacle;

#define MIN_RANGE 100 // minimum lidar range in mm. Readings any closer are likely to be noise or the PTU's platform
#define MAX_RANGE 2730 // maximum lidar range in mm. Beyond this, readings are unreliable.
#define NOMINAL_LIDAR 2000 // to calculate the default elevation angle
#define HEIGHT_OFF_GROUND 1050 // rollator seat height (mm)
#define LIDAR_OFFSET 100
#define OVERFLOW_FACTOR 65536
#define E_CLOCK 24

// configure TC1 for input capture
void timer_config(void);

// compute the expected distance reading of the ground for reference
unsigned long getGroundDistance(float declination);

// interrupt service routine for input capture in TC1
__interrupt void TC1_ISR(void);

// interrupt service routine to count number of timer overflows
__interrupt void TOF_ISR(void);

#endif



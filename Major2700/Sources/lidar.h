#ifndef LIDAR_HEADER
#define LIDAR_HEADER

extern volatile unsigned long distance;
extern volatile unsigned int time_1;
extern volatile unsigned int time_2;

#define MAX_RANGE 2730

void timer_config(void);
__interrupt void TC1_ISR(void);
__interrupt void TOF_ISR(void);



#endif



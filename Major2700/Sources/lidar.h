#ifndef LIDAR_HEADER
#define LIDAR_HEADER

extern volatile unsigned long distance;
//extern volatile char read_flag; // whether the LIDAR value should be used
extern volatile unsigned int time_1;
extern volatile unsigned int time_2;

#define MAX_RANGE 2730

void timer_config(void);
//void lidar_capture(long *a, unsigned int *b, unsigned int *c);
//void LCD_setup(void);
__interrupt void TC1_ISR(void);
__interrupt void TOF_ISR(void);



#endif



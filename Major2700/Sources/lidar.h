#ifndef LIDAR_HEADER
#define LIDAR_HEADER


extern volatile unsigned long distance;
extern volatile unsigned int time_1;
extern volatile unsigned int time_2;
extern volatile char time_flag;
extern volatile unsigned int overflow;

void timer_config(void);
//void LCD_setup(void);
__interrupt void TC1_ISR(void);
__interrupt void TOF_ISR(void);



#endif



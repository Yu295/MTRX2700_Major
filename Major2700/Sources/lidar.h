#ifndef LIDAR_HEADER
#define LIDAR_HEADER

extern volatile unsigned long distance;
//extern volatile char read_flag; // whether the LIDAR value should be used

void timer_config(void);
void lidar_capture(void);
//void LCD_setup(void);
//__interrupt void TC1_ISR(void);
__interrupt void TOF_ISR(void);



#endif



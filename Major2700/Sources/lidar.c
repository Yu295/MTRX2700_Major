#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include <math.h>

#include "l3g4200d.h"
#include "lidar.h"

volatile unsigned int overflow; // number of overflows
volatile long time_diff;        // pulse width of LIDAR reading (ms)
volatile char edge_flag; // whether to read in a rising or falling edge

unsigned long getGroundDistance(float declination) {
  
  // not looking towards the ground if declination is non-positive, so set reference distance as the LIDAR's max range
  if (declination <= 0) {
    return MAX_RANGE;
  }
  
  return (unsigned long)((float)HEIGHT_OFF_GROUND / sinf(declination));
}

void timer_config(void){

   _asm SEI;
    
   TSCR1 |= 0x90;  //Enable timer, fast flag clear bit enabled
   TIOS_IOS1 = 0; // Input capture enabled at TC1
   TCTL4 = 0x04; // Capture on rising edge, then configure to capture on falling edge
   TFLG1 &= ~TFLG1_C1F_MASK; //Clear C1F
   ICOVW = 0x02; // prohibit overwriting C1F
   ICSYS = 0;    // disable queueing
   DLYCT = 0x11; // remove initial noise
   
   _asm CLI;
   
   return; 

}

void check_obstacle(void){
    
   TIE |= TIE_C1I_MASK;
   delay(100);
   
   
   if  (distance  < 1930) {  //1.93 metres while the PTU is at an azimuth of 15 degrees by default
   
   obstacle = 1;
    
   TIE &= ~TIE_C1I_MASK; 

   }
   return;
}

/********change it to interrupt, to calculate pulse width  *********/

#pragma CODE_SEG __NEAR_SEG NON_BANKED

__interrupt void TC1_ISR(void) { 
   
   
   while(!(TFLG1_C1F & 1));    //Wait for the first rising edge.
   
   if (edge_flag) {
      
      time_1 = TC1; // time captured on rising edge
      TCTL4 = 0x08; // set up next interrupt for falling edge
      overflow = 0;  // overflow count
      edge_flag = 0; // for next time an interrupt is triggered 
      TSCR2 = 0x80; // turn on the overflow interrupt - start counting
   
   } else {
     
      TSCR2 = 0; // turn off the overflow interrupt - done counting
      time_2 = TC1; // time captured on falling edge
      TCTL4 = 0x04; // set up next interrupt for rising edge
      edge_flag = 1; // for the next LIDAR measurement
      
      if (time_2 < time_1) {
          overflow -= 1;
      }
      
      time_diff = (long)time_2 - (long)time_1;
      distance = (unsigned long) (((long)overflow * 65536 + time_diff)/24 - 100);    
   }
   
   return;  
}

__interrupt void TOF_ISR(void) { 
  
    TFLG2 = 0x80; // clear TOF flag
    ++overflow;   // increment overflow count
    
    return; 
}
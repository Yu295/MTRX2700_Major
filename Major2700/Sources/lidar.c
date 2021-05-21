#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */

#include "l3g4200d.h"
#include "lidar.h"


volatile unsigned int overflow;
volatile long time_diff;
volatile char time_flag;
volatile char edge_flag; // whether to read in a rising or falling edge

void timer_config(void){

   _asm SEI;
    
   TSCR1 |= 0x90;  //Enable timer, fast flag clear bit enabled
   TIOS_IOS1 = 0; // Input capture enabled at TC1
   TCTL4 = 0x04; // Capture on rising edge, then configure to capture on falling edge
   TFLG1 &= ~TFLG1_C1F_MASK; //Clear C1F
   ICOVW = 0x02; // prohibit overwriting C1F
   ICSYS = 0;    // disable queueing
   DLYCT = 0x11; // remove initial noise
   //read_flag = 0;  
   //DLYCT
   
   _asm CLI;
   
   return; 

}

/*
void lidar_capture(long *a, unsigned int *b, unsigned int *c){
   unsigned long i = 0;
   _asm SEI;
   
   TCTL4 = 0x04; // set up next interrupt for rising edge
   TSCR2 = 0x80;  // Enable timer overflow interrupt, and a prescaler of 1
   TFLG1_C1F = 0;
   while(!(TFLG1_C1F & 1));    // Wait for the first rising edge.   
   
    
   time_1 = TC1; // time captured on rising edge
   
   TCTL4 = 0x08; // set up next interrupt for falling edge
   
   _asm CLI;    
   
   //TSCR2_TOI = 1;  //Enable timer overflow interrupt, and a prescaler of 1
   
  
   //edge_flag = 0;
  
   //time_flag = 1;
   overflow = 0;
   
   while(!(TFLG1_C1F & 1));
   
   *c = overflow;         
   TSCR2 &= ~0x80; 
   time_2 = TC1; // time captured on falling edge
   
       
   
        
   //edge_flag = 1;
        
   //time_flag = 0;
  
   if (time_2 < time_1) {
       overflow -= 1;
   } else {
    
       overflow -= 0;
   }
   time_diff = (long)time_2 - (long)time_1;
  
  
   distance = ((long)overflow * 65536 + time_diff)/24;
 
   //distance = ((long)time_diff)/24-100;
 
   if (distance > 2730){
   
    time_flag = 0;

   } 
   
   *a = time_diff;
   *b = overflow;     
   //}

} */



/********change it to interrupt, to calculate pulse width  *********/

#pragma CODE_SEG __NEAR_SEG NON_BANKED

__interrupt void TC1_ISR(void) { 
   
   
   while(!(TFLG1_C1F & 1));    //Wait for the first rising edge.
   
   if (edge_flag) {
      
      time_1 = TC1; // time captured on rising edge
      
      
      TCTL4 = 0x08; // set up next interrupt for falling edge
      
      overflow = 0;  // overflow count
      
      edge_flag = 0; // for next time an interrupt is triggered
      
      time_flag = 1;
      
      TSCR2 = 0x80; // turn on the overflow interrupt - start counting
   
   } else {
      
      //overflow = 0;
      TSCR2 = 0; // turn off the overflow interrupt - done counting
      
      time_2 = TC1; // time captured on falling edge
      
      TCTL4 = 0x04; // set up next interrupt for rising edge
      
      edge_flag = 1; // for the next LIDAR measurement
      
      //time_flag = 0;
      
      if (time_2 < time_1) {
          overflow -= 1;
      }
      time_diff = (long)time_2 - (long)time_1;
      
      distance = (unsigned long) (((long)overflow * 65536 + time_diff)/24 - 100);
     
      //distance = ((long)time_diff)/24-100;
      
   }
   
}

__interrupt void TOF_ISR(void) { 
  
    TFLG2 = 0x80;
    overflow++; 
}
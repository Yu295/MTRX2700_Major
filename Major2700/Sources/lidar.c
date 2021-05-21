#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */

#include "l3g4200d.h"
#include "lidar.h"

volatile unsigned int time_1;
volatile unsigned int time_2;
volatile int overflow;
volatile int time_diff;
volatile char time_flag;
volatile char edge_flag; // whether to read in a rising or falling edge

void timer_config(void){

   _asm SEI;
    
   TSCR1_TFFCA = 1;  //Enable timer, fast flag clear bit enabled
   TIOS_IOS1 = 0; // Input capture enabled at TC1
   TIE &= ~TIE_C1I_MASK; // disable the input capture interrupt at channel one (change to |= TIE_C1I_MASK to enable)
   TSCR2_TOI = 1;  //Enable timer overflow interrupt, and a prescaler of 1
   TCTL4 = 0x04; //Capture on rising edge, then condigure to capture on falling edge
   TFLG1 = 0x02; //Clear C1F
   //read_flag = 0;  
   //DLYCT
   return; 

}


void lidar_capture(void){
 
   while(!(TFLG1_C1F & 1));    //Wait for the first rising edge.   
   
   //if (read_flag) {
     
     if (edge_flag) {
        
        time_1 = TC1; // time captured on rising edge
        
        //TSCR2_TOI = 1;  //Enable timer overflow interrupt, and a prescaler of 1
        
        TCTL4 = 0x08; // set up next interrupt for falling edge
        
        edge_flag = 0;
        
        time_flag = 1;
        
        overflow = 0;

     
     } else {
           
        time_2 = TC1; // time captured on falling edge
        
        TCTL4 = 0x04; // set up next interrupt for rising edge
        
        edge_flag = 1;
        
        //time_flag = 0;
        
        if (time_2 < time_1) {
            overflow -= 1;
        }
        time_diff = time_2 - time_1;
        
        distance = ((long)overflow * 65536 + (long)time_diff)/24-100;
       
        //distance = ((long)time_diff)/24-100;
       
        if (distance > 2730){
         
         time_flag = 0;
     
        } 
        
      }
   //}

}

/*

__interrupt void TC1_ISR(void) { 
    
  
} */

#pragma CODE_SEG __NEAR_SEG NON_BANKED

__interrupt void TOF_ISR(void) { 
  
    TFLG2 = TFLG2 | TFLG2_TOF;
    overflow++; 
}
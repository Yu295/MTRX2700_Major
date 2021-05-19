#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */

#include "l3g4200d.h"
#include "lidar.h"



volatile int time_diff;

volatile int edge_flag;

void timer_config(void){

   _asm SEI;
    
   TSCR1_TFFCA = 1;  //Enable timer, fast flag clear bit enabled
   TSCR2_TOI = 1;  //Enable timer overflow interrupt, and a prescaler of 1
   TIOS_IOS1 = 0; // Input capture enabled at TC1
   TIE_C1I = 1; // Enable the input capture interrupt at channel zero
   
   TCTL4 = 0x04; //Capture on rising edge, then condigure to capture on falling edge
   TFLG1 = 0x02; //Clear C1F
   //ICOVW = 0x01; //Prohibit overwrite at TC0
   //DLYCT
   
   _asm CLI; 

}



/********change it to interrupt, to calculate pulse width  *********/

#pragma CODE_SEG __NEAR_SEG NON_BANKED

__interrupt void TC1_ISR(void) { 
  
  
   //TFLG1 = TFLG1_C1F; //Clears TFLG1 if fast flag clear is not working
   
   
   while(!(TFLG1_C1F & 1));    //Wait for the first rising edge.
   
   if (edge_flag) {
      
      time_1 = TC1; // time captured on rising edge
      
      TCTL4 = 0x08; // set up next interrupt for falling edge
      
      edge_flag = 0;
      
      time_flag = 1;

   
   } else {
      
      overflow = 0;
      
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
   
}


__interrupt void TOF_ISR(void) { 
  
    TFLG2 = TFLG2 | TFLG2_TOF;
    overflow ++; 
}
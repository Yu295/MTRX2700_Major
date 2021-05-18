#include "servo.h"
#include "derivative.h"

// configure PWM7 and PWM5 (connected to PTU)
void PWMConfig(void) {
  PWMCTL = 0; // all separate PWM channels
  PWMCLK = (PWMCLK_PCLK5_MASK | PWMCLK_PCLK7_MASK); // select scaled PWM clocks
  PWMPOL = (PWMPOL_PPOL5_MASK | PWMPOL_PPOL7_MASK); // PWM outputs start at HIGH
  PWMCAE = 0;     // PWM output is left-aligned
  PWMPRCLK = 0x77; // set clock A and B prescaler to 128    
  PWMSCLA = 75;    // use SA/SB, and scale these by 256
  PWMSCLB = 75;    // so total prescaling of 128*75*2 = 19200
                  // i.e. SA/SB has a period of 0.8 ms               
  PWMPER5 = 25;   // use PWMPER = 25 so PWM has a period of 20 ms
  PWMPER7 = 25;
  PWMDTY5 = 1;  // left: 0.9/20 * 25, forward: 1.5/20 * 25, right: 2.1/20 * 25
  PWMDTY7 = 1;  // PWM duty cycle = PWMDTYx / PWMPERx 
  PWME = (PWME_PWME5_MASK | PWME_PWME7_MASK);  
}
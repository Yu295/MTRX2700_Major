#include "servo.h"
#include "derivative.h"

// configure PWM7 and PWM5 (connected to PTU)
void PWMConfig(void) {
  PWMCTL = 0; // all separate PWM channels
  PWMCLK = (PWMCLK_PCLK5_MASK | PWMCLK_PCLK7_MASK); // select scaled PWM clocks
  PWMPOL = (PWMPOL_PPOL5_MASK | PWMPOL_PPOL7_MASK); // PWM outputs start at HIGH
  PWMCAE = 0;     // PWM output is left-aligned
  PWMPRCLK = 0x77;   // Period of 2ms so ratio of 48,000
                  // set clock A and B prescaler to 128 and use SA/SB so prescaled by 256
                  // use PWMPER = 188
  PWMPER5 = 188;
  PWMPER7 = 188;
  PWMDTY5 = 9;  // left: 0.9/20 * 188, forward: 1.5/20 * 188, right: 2.1/20 * 188
  PWMDTY7 = 9;  // PWM duty cycle = PWMDTYx / PWMPERx 
  PWME = (PWME_PWME5_MASK | PWME_PWME7_MASK);  
}
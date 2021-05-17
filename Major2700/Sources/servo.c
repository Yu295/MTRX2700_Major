#include "servo.h"
#include "derivative.h"

// configure PWM7 and PWM5 (connected to PTU)
void PWMConfig(void) {
  PWMCTL |= 0xCC; // select 16 bit PWM67 and PWM45
                  // 45 is controlled by PWMCH4, 67 is controlled by PWMCH6
  PWMCLK |= 0; // select PWM clock A
  PWMPOL |= (PWMPOL_PPOL4_MASK || PWMPOL_PPOL6_MASK); // PWM outputs start at HIGH
  PWMCAE = 0;     // PWM output is left-aligned
  PWMPRCLK = 4;   // Period of 20ms so ratio of 480,000
                  // set clock A prescaler to 16, use PWMPER = 30,000
  PWMPER4 = 30000;
  PWMPER6 = 30000;
  PWMDTY4 = 1350;  // left: 0.9/20 * 30000, forward: 1.5/20 * 30000, right: 2.1/20 * 30000
  PWMDTY6 = 1350;  // PWM duty cycle = PWMDTYx / PWMPERx 
  PWME |= (PWME_PWME4_MASK || PWME_PWME5_MASK || PWME_PWME6_MASK || PWME_PWME7_MASK);  
}
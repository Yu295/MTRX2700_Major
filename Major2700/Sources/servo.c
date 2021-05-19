#include "servo.h"
#include "derivative.h"


// configure PWM7 and PWM5 (connected to PTU)
void PWMConfig(void) {
  PWMCTL = 0; // all separate PWM channels
  PWMCLK = (PWMCLK_PCLK5_MASK | PWMCLK_PCLK7_MASK); // select scaled PWM clocks
  PWMPOL = (PWMPOL_PPOL5_MASK | PWMPOL_PPOL7_MASK); // PWM outputs start at HIGH
  PWMCAE = 0;     // PWM output is left-aligned
  PWMPRCLK = 0x66; // set clock A and B prescaler to 64    
  PWMSCLA = 15;    // use SA/SB, and scale these by 15
  PWMSCLB = 15;    // so total prescaling of 64*15*2 = 1920
                  // i.e. SA/SB has a period of 0.08 ms               
  PWMPER5 = 255;   // use PWMPER = 255 so PWM has a period of 20.48 ms
  PWMPER7 = 255;
  PWMDTY5 = 19;  // left: 0.9/20 * 255, forward: 1.5/20 * 255, right: 2.1/20 * 255
  PWMDTY7 = 0;  // PWM duty cycle = PWMDTYx / PWMPERx 
  PWME = (PWME_PWME5_MASK | PWME_PWME7_MASK);  
}

// angle in degrees and assumed to be between +/- 90
void turnToElevationAzimuth(char elevation, char azimuth) {
  unsigned char dutyE, dutyA;
  double ratioE, ratioA;
  
  if (elevation < -90 || elevation > 90) {
    return;
  }
  
  // angle of -90: 0.9, 0: 1.5, +90: 2.1
  ratioE =  (double)elevation / 150 + 1.5;
  dutyE = (unsigned char)(ratioE * 12.75);
  PWMDTY5 = dutyE;
  
  // angle of -90: ? 0: 1.5, +90: ? 
  ratioA = (double)azimuth / 102 + 1.5;
  dutyA = (unsigned char)(ratioA * 12.75);
  PWMDTY7 = dutyA;
  
  return;
}
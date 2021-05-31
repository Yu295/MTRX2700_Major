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
  PWMPER7 = 255;   // resolution given by  1/255 * 20 * 102 = 8 degree increments
  PWMDTY5 = 19;  // left: 0.9/20 * 255, forward: 1.5/20 * 255, right: 2.1/20 * 255
  PWMDTY7 = 0;  // PWM duty cycle = PWMDTYx / PWMPERx 
  PWME = (PWME_PWME5_MASK | PWME_PWME7_MASK);
  
  return;  
}

// angles in degrees and assumed to be between +/- 90
SERVO_STATE turnToElevationAzimuth(char elevation, char azimuth, unsigned char *prevDutyE, unsigned char *prevDutyA, ANGLE duplicate) {
  unsigned char dutyE, dutyA; // value to set PWMDTYx
  double ratioE, ratioA;      // intermediate value to calculate PWMDTYx
  
  // reject invalid angles
  if (elevation < (MIN_PAN_ELEVATION) || elevation > (MAX_PAN_ELEVATION)) {
    return INVALID_ELEVATION;
  
  } else if (azimuth < MIN_PAN_AZIMUTH || azimuth > MAX_PAN_AZIMUTH) {
    return INVALID_AZIMUTH;
  }
  
  // Calibration values: angle of -90: 0.62, 0: 1.5, +90: 2.38 
  ratioE = (double)elevation / CALIBRATION_NIGHTY + CALIBRATION_ZERO;
  dutyE = (unsigned char)(ratioE * PWMPER_CONVERSION); // conversion factor of PWMPERx/20
  
  
  // Calibration values: angle of -90: 0.62 0: 1.5, +90: 2.38 
  ratioA = (double)azimuth / CALIBRATION_NIGHTY + CALIBRATION_ZERO;
  dutyA = (unsigned char)(ratioA * PWMPER_CONVERSION); // conversion factor of PWMPERx/20
  
  // precision lost during cast operation so servo may end up not moving from the previous position
  // check if the relevant servo will actually end up moving from the previous position
  if ((duplicate == ELEVATION) && (dutyE == *prevDutyE)) {
    return DUPLICATE_CONFIG;
  
  } else if ((duplicate == AZIMUTH) && (dutyA == *prevDutyA)) {
    return DUPLICATE_CONFIG;
  
  }
  
  // move the servos
  PWMDTY5 = dutyE;
  PWMDTY7 = dutyA;
  
  // store the current duty ratios as references
  *prevDutyE = dutyE;
  *prevDutyA = dutyA;
                
  return SUCCESSFUL_TURN;
}
#include "servo.h"
#include "iic.h"
#include "derivative.h"
#include "lidar.h"
#include "simple_serial.h"
#include <stdio.h>


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
  unsigned char dutyE, dutyA;
  double ratioE, ratioA;
  
  // reject invalid angles
  if (elevation < MIN_PAN_ELEVATION || elevation > MAX_PAN_ELEVATION) {
    return INVALID_ELEVATION;
  } else if (azimuth < MIN_PAN_AZIMUTH || azimuth > MAX_PAN_AZIMUTH) {
    return INVALID_AZIMUTH;
  }
  
  // angle of -90: 0.9, 0: 1.5, +90: 2.1
  ratioE = (double)elevation / 150 + 1.5;
  dutyE = (unsigned char)(ratioE * 12.75);
  
  
  // angle of -90: 0.62 0: 1.5, +90: 2.38 
  ratioA = (double)azimuth / 102 + 1.5;
  dutyA = (unsigned char)(ratioA * 12.75); // precision lost during cast operation so servo may end up not moving from the previous position
  
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

void calibrateDist(void) {
  unsigned char buf[50];
  long dt;
  int ovB, ovC;
  
  turnToElevationAzimuth(0, 0, NULL, NULL, NONE);
  
  delay(400);
  //lidar_capture(&dt, &ovB, &ovC);
  do {
    TIE |= TIE_C1I_MASK;
    delay(100);
    TIE &= ~TIE_C1I_MASK;
  } while (distance > 2730);
  
  //sprintf(buf, "0,0,%u,%u,%ld,%u,%u,%lu\n", time_1, time_2, dt, ovB, ovC, distance);
  sprintf(buf, "0,0,%u,%u,%lu\n", time_1, time_2, distance); 
  SCI1_OutString(buf);
  return;
}
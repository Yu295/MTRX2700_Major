# MTRX2700 Major Project 
# Audio-Based Guided Navigation for Elderly Blind People 

## Jason Lai, Reihana Tsao, Yujiao Cao

### Introduction
This design is for an audio-based guidance system for elderly and/or visually-impaired people, which performs obstacle detection and path-tracing, and provides voice instructions for navigation purposes. The system continuously scans for obstacles in front of the user. As soon as a potential obstacle is detected, the user shall receive a voice prompt to stop in place. The system then begins to pan the immediate surroundings and maps the corresponding area to obtain a safe path in the form of suitable angles to turn. The user shall then be instructed to begin turning until a subsequent prompt is received to walk forward in their current direction.

The developed software is designed to interface with the 68HCS12 Microcontroller and provided pan-and-tilt unit (PTU). 

### System Flow Chart


### Modular Design

**Servomotor Module**

The PTU is fitted with two HiTec HS-422 Standard Deluxe Servos, providing two degrees of freedom as shown in ?.
Thus, the PTU's orientation in the body frame can be completely specified by its elevation angle $\theta \in [-45,\,45]$ and azimuth angle $\phi \in [-90, 90]$.

Specifying the PTU to achieve a given orientation can be achieved by controlling the respective servos to turn to these angles. Pulse-width modulation (PWM) from the 68HCS12 PWM block is used to provide actuation to the servos. The following functions in the module ```servo.h``` have been implemented to facilitate such control:

```c
// comment goes here
void PWMConfig(void);
```
Before use, the 68HCS12 PWM block must be configured. The servos to control elevation and azimuth are configured to use 8-bit PWM channels 5 and 7 respectively. A period $T=20$ms is typically required for servo operation. Since the 68HCS12 E-clock timer is configured to operate at 24 MHz (with no pre-scaling taking place), this implies the total scaling required is:

$$\text{prescaler} \times (\text{PWMPER} + 1) = \frac{20\times 10^{-3}}{\frac{1}{24\times 10^6}} = 480,000 $$

The 68HCS12 PWM block offers the capability to set PWMCTL to concatenate channels 4-5 and 6-7. This way, two 16-bit channels are formed, allowing ```PWMPER``` to take values from 0 to $2^{16}-1$. However, doing so will deactivate PWM outputs to channels 5 and 7, thus this approach was not implemented.

To maximise the servos' resolution, ```PWMPER``` should be large, hence:
$$\text{PWMPER} = 255$$
If a period $T=20.48$ms is used instead, this implies
$$\text{prescaler} = 491520 \implies \text{prescaler} = 1920$$
This prescaling can be achieved by the following:
1. Prescaling Clocks A/B by a factor of 64 through setting ```PWMPRCLK```.
2. Opting to use the scaled clocks SA/SB by setting ```PWMCLK```, further prescaling the PWM clocks by a factor of 2.
3. Scaling SA/SB by a factor of 15 by setting ```PWMSCLA``` and ```PWMSCLB```.

The above analysis is only valid for left-aligned PWM output beginning HIGH hence ```PWMCAE``` and ```PWMPOL``` are set to ensure this. 

```c
// comment goes here
SERVO_STATE turnToElevationAzimuth(char elevation, char azimuth, unsigned char *prevDutyE, unsigned char *prevDutyA, ANGLE duplicate);
```

**IMU Module**

The PTU is fitted with a 10 Degree of Freedom Inertial Measurement Unit (IMU) Module that comes with the following sensor suite:
1. ADXL345 Digital Accelerometer, which provides the absolute acceleration with respect to the body frame.
2. HML5883 3-Axis Digital Compass IC, which provides the components of Earth's magnetic field in the body frame.
3. L3G4200D MEMS Motion Sensor: 3-Axis Digital Output Gyroscope, which provides the angular velocity with respect to the body frame.

which provides the rate of rotation, direction of Earth's magnetic field and absolute acceleration, after manipulating the raw data, we can obtain the azimuth and elevation angles of the system at the current position. 

**Lidar Module**

The Lidar-Lite Sensor version 2 provides the distance to the first object being detected. The lidar interacts with the HCS12 Dragon Board through pin PT1, by capturing the pulse width of the PWM signal being send to PT1, the measured distance to the obstacle can be calculated by a 1msec/metre realationship.

**Serial Module**

The serial module is designated to transmit real-time orientations of the system, distance to the object, and the program flow indication between CodeWarrior and MATLAB.


**MATLAB Module**

The MATLAB module has two portions, serial data receiving and transmitting, and mapping of the environment. 
The serial portion will be polling until the distance to the object is transmitted from CodeWarrior to MATLAB, then a voice instruction will be played through PC to inform the user that there are obstacles in the walkway and then asks the user to stop. Afterwards, the PTU will be triggered to start panning through . During the scanning procedure, the serial port transmits the real-time orientations of the system along with the distance to the object, and stores the information into a matrix that can be used for environement mapping. 




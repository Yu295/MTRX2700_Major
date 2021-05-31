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
Thus, the PTU's orientation in the body frame can be completely specified by its elevation angle ![equation](https://latex.codecogs.com/gif.latex?%5Cinline%20%5Ctheta%5Cin%5B-45%5Eo%2C45%5Eo%5D) and azimuth angle ![equation](https://latex.codecogs.com/gif.latex?%5Cinline%20%5Cphi%5Cin%5B-90%5Eo%2C90%5Eo%5D).

Specifying the PTU to achieve a given orientation can be achieved by controlling the respective servos to turn to these angles. Pulse-width modulation (PWM) from the 68HCS12 PWM block is used to provide actuation to the servos. The following functions in the module ```servo.h``` have been implemented to facilitate such control:

```c
// comment goes here
void PWMConfig(void);
```
Before use, the 68HCS12 PWM block must be configured. The servos to control elevation and azimuth are configured to use 8-bit PWM channels 5 and 7 respectively. A period ![equation](https://latex.codecogs.com/gif.latex?%5Cinline%20T%3D20) ms is typically required for servo operation. Since the 68HCS12 E-clock timer is configured to operate at 24 MHz (with no pre-scaling taking place), this implies the total scaling required is:

![equation](https://latex.codecogs.com/gif.latex?%5Ctext%7Bprescaler%7D%5Ctimes%28%5Ctext%7BPWMPER%7D&plus;1%29%3D%5Cfrac%7B20%5Ctimes10%5E%7B-3%7D%7D%7B%5Cfrac%7B1%7D%7B24%5Ctimes10%5E6%7D%7D%3D480%2C000)

The 68HCS12 PWM block offers the capability to set PWMCTL to concatenate channels 4-5 and 6-7. This way, two 16-bit channels are formed, allowing ```PWMPER``` to take values from 0 to ![equation](https://latex.codecogs.com/gif.latex?%5Cinline%202%5E%7B16%7D-1). However, doing so will deactivate PWM outputs to channels 5 and 7, thus this approach was not implemented.

To maximise the servos' resolution, ```PWMPER``` should be large, hence:

![equation](https://latex.codecogs.com/gif.latex?%5C%20%5Ctext%7BPWMPER%7D%3D255)

If a period ![equation](https://latex.codecogs.com/gif.latex?%5C%20T%3D20.48) ms is used instead, this implies

![equation](https://latex.codecogs.com/gif.latex?%5Ctext%7Bprescaler%7D%5Ctimes%28%5Ctext%7BPWMPER%7D&plus;1%29%3D%5Cfrac%7B20.48%5Ctimes%2010%5E%7B-3%7D%7D%7B%5Cfrac%7B1%7D%7B24%5Ctimes10%5E6%7D%7D%5Cimplies%5Ctext%7Bprescaler%7D%3D1920)

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

The Lidar-Lite Sensor version 2 provides the distance to the first object being detected. The lidar interacts with the 68HCS12 Dragon Board through pin PT1, by capturing the pulse width of the PWM signal being send to PT1, the measured distance to the obstacle can be calculated by a 1msec/metre realationship. Thus, the following functions in ```lidar.h``` are implemented to measure the distance.
```c
void timer_config(void);
```
The timer block of the HCS12 board is intended to measure the duration of a pulse, the timer is enabled with a prescaler of 1, since the input capture register is 16-bit, the maximum pulse width that can be captured is 2.73 msec. 
The register ```TIOS``` is in control of either implementing the input cpature or the output compare function, since the PWM signal is generted at PT1, the input capture function at channel 1 must be enabled. As the PWM signal came with a measurement period, the edge configuration is initialized as rising edge, and configured as falling edge after the firts edge detection to ensure the accuracy of the measurement.
```c
__interrupt void TC1_ISR(void);
```
The capturing of the PWM signal is interpreatated as an interrupt. To filter the noise in the measuremnet, the ```TIE``` register is only enabled for 10 readings at each position, and the minimal value is the designated distance. The pulse width is measured by capturing the ```TC1``` value at the risting edge, and subtracted by the ```TC1``` value at the next falling edge. Despite the limitation of underestimating the distance at some point, it is usually functional with an accuracy of. 

**Serial Module**

The serial module is designated to transmit real-time orientations of the system, distance to the object, and the program flow indication between CodeWarrior and MATLAB. Other than the serial functions being provided, two functions is facilitated to read from serial.
```c
void SCI1_InString(char *buffer);
```
This function polls the ```RDRF``` bit in the register ```SCI1SR1```, and reads until a new line character indicating the end of the string, and stores the information in buffer. And to keep consistency of the C strings, a terminating character ```\0``` has been generated.
```c
void flushBuffer(char *buffer);
```
This function clears the content in side ```buffer```, ensuring that oly the required information is being stored in to it.

**MATLAB Serial Module**

The MATLAB module has two portions, serial data receiving and transmitting, and mapping of the environment. 
The serial portion will be polling until the distance to the object is transmitted from CodeWarrior to MATLAB, then a voice instruction will be played through PC to inform the user that there are obstacles in the walkway and then asks the user to stop. Afterwards, the PTU will be triggered to start panning through . During the scanning procedure, the serial port transmits the real-time orientations of the system along with the distance to the object, and stores the information into a matrix that can be used for environement mapping. 
The following functions are created to interact between MATLAB and CodeWarrior through the serial port.

```matlab
readLidar.m
```
This function is implemented such that it will be polling until a reading of the distance is sent by CodeWarrior through the serial port. Then the voice instruction is played to stop the user.

```matlab
sendSerial.m
```
This function is used to send information to the serial port, which is the most important function in the interaction of the two programmes. 

 ```matlab
 readSerial.m
 ```
 After stopping the user, the ```panServo``` function will be implemented in the C Portion, afterwards, the readings of the distance, elevation and azimuth will come through to MATLAB. This function also stores the data into a matrix for the mapping later on.
 
 ```matlab
 readMagnet.m
 ```
 When the mapping of the environment is successfully accomplished, the user is instructed to turn to the correct elevation that has been caculated in the mapping function. In this function, the magenetometer reading is transmitted through from CodeWarior so that when the correct elevation is achieved, the voice instruction will guide the user to go forwards.
 
 **Matlab Mapping and Guidance Module**
 
Mapping of the environment takes the serial readings of distance, elevation and azimuth from the ```readSerial.m``` function and translates them to the Cartesian coordinates x,y,z, which are then plotted in a 3D scatter plot. The conversions are calculated using 3D trigonometry. 



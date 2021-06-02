# MTRX2700 Major Project 
# Audio-Based Guided Navigation for Elderly Blind People 

## Jason Lai, Reihana Tsao, Yujiao Cao

## Introduction
This design is for an audio-based guidance system for elderly and/or visually-impaired people, which performs obstacle detection and path-tracing, and provides voice instructions for navigation purposes. The system continuously scans for obstacles in front of the user. As soon as a potential obstacle is detected, the user shall receive a voice prompt to stop in place. The system then begins to pan the immediate surroundings and maps the corresponding area to obtain a safe path in the form of suitable angles to turn. The user shall then be instructed to begin turning until a subsequent prompt is received to walk forward in their current direction.

The developed software is designed to interface with the 68HCS12 Microcontroller and provided pan-and-tilt unit (PTU). This harwdware will require connection via SCI to an external device (e.g. laptop) that will execute the associated MATLAB program. As such, **it is highly recommended that the setup is mounted on a flat surface, with the PTU facing forwards.**

## System Flow Chart


## Servomotor Module (C)

The PTU is fitted with two HiTec HS-422 Standard Deluxe Servos, providing two degrees of freedom as shown in ?.
Thus, the PTU's orientation in the body frame can be completely specified by its elevation angle ![equation](https://latex.codecogs.com/gif.latex?%5Cinline%20%5Ctheta%5Cin%5B-45%5Eo%2C45%5Eo%5D) and azimuth angle ![equation](https://latex.codecogs.com/gif.latex?%5Cinline%20%5Cphi%5Cin%5B-90%5Eo%2C90%5Eo%5D).

Specifying the PTU to achieve a given orientation can be achieved by controlling the respective servos to turn to these angles. Pulse-width modulation (PWM) from the 68HCS12 PWM block is used to provide actuation to the servos. The following functions in the module ```servo.h``` have been implemented to facilitate such control:

```c
void PWMConfig(void);
```
Before use, the 68HCS12 PWM block must be configured. The servos to control elevation and azimuth are configured to use 8-bit PWM channels 5 and 7 respectively. A period ![equation](https://latex.codecogs.com/gif.latex?%5Cinline%20T%3D20) ms is typically required for servo operation. Since the 68HCS12 E-clock timer is configured to operate at 24 MHz (with no pre-scaling taking place), this implies the total scaling required is:

![equation](https://latex.codecogs.com/gif.latex?%5Ctext%7Bprescaler%7D%5Ctimes%28%5Ctext%7BPWMPER%7D&plus;1%29%3D%5Cfrac%7B20%5Ctimes10%5E%7B-3%7D%7D%7B%5Cfrac%7B1%7D%7B24%5Ctimes10%5E6%7D%7D%3D480%2C000)

The 68HCS12 PWM block offers the capability to set ```PWMCTL``` to concatenate channels 4-5 and 6-7. This way, two 16-bit channels are formed, allowing ```PWMPER``` to take values from 0 to ![equation](https://latex.codecogs.com/gif.latex?%5Cinline%202%5E%7B16%7D-1). However, doing so will deactivate PWM outputs to channels 5 and 7, thus this approach was not implemented.

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
SERVO_STATE turnToElevationAzimuth(char elevation, char azimuth, unsigned char *prevDutyE, unsigned char *prevDutyA, ANGLE duplicate);
```
This function is responsible for generating the PWM signal that provides actuation to turn the PTU to an orientation specified by ```elevation``` and ```azimuth```. This is achieved by setting ```PWMDTY``` to a value corresponding to these respective angles, with a linear relationship determined through a calibration process. The following table summarises the calibration points used:

| Angle (deg) | Ratio (E) | PWMDTY (E) | Ratio (A) | PWMDTY (A) |
|-|-|-|-|-|
|-90|0.62|8|0.62|8|
|0|1.5|19|1.5|19|
|+90|2.38|30|2.38|30|  

Note both servos were observed to have identical calibration points. ```PWMDTY``` was calculated using the following:

![equation](https://latex.codecogs.com/gif.latex?%5Ctext%7BPWMDTY%7D%3D%5Ctext%7BPWMPER%7D%5Ctimes%5Cfrac%7B%5Ctext%7Bratio%7D%7D%7BT%7D%3D%5Cunderbrace%7B%5Cfrac%7B%5Ctext%7BPMWPER%7D%7D%7BT%7D%7D_%7B%5Ctext%7Bconversion%20factor%7D%7D%5Ctimes%5Cunderbrace%7B%28mx&plus;b%29%7D_%7B%5Ctext%7Bratio%7D%7D)

Where ```PWMPER=255```, ```T=20.48``` (approximated to be 20 ms in the final implementation), ```m=0.0098``` and ```b=1.5``` based on the calibration points used.

The result is finally rounded to the nearest integer before being assigned to ```PWMDTY```. Such rounding means that in general, there will be multiple configurations that lead to the same ```PWMDTY``` values, meaning the PTU does not move.

To address this, the extra arguments ```prevDutyE```, ```prevDutyA``` and ```duplicate``` are passed, storing the previous duty ratios and type of angle to check for duplication respectively. If a duplicate configuration is realised, the function returns ```DUPLICATE_CONFIG``` and no actuation occurs. To bypass these checks, the pre-defined constants ```NULL``` and ```NONE``` can be passsed in place of ```prevDutyx``` and ```duplicate``` respectively.

Successful actuation to a new configuration leads to ```SUCCESSFUL_TURN``` being returned whereas angle arguments outside the defined range will lead to the ```INVALID``` flags being returned.

## IMU Module (C)

The PTU is fitted with a 10 Degree of Freedom Inertial Measurement Unit (IMU) Module that comes with the following sensor suite:
1. ADXL345 Digital Accelerometer, which provides the absolute acceleration with respect to the body frame.
2. HML5883 3-Axis Digital Compass IC, which provides the components of Earth's magnetic field in the body frame.
3. L3G4200D MEMS Motion Sensor: 3-Axis Digital Output Gyroscope, which provides the angular velocity with respect to the body frame.

These sensors can also be used to form closed-loop control over the PTU's orientation. It was however observed that open-loop control provided sufficiently precise control over the PTU's orientation. Nevertheless, to improve the accuracy of the mapping process, the set elevation and azimuth angles, alongside the measured elevation are transmitted via SCI to the Mapping and Guidance module. 

### Accelerometer
The accelerometer can be used to calculate the current elevation (in the body frame) provided the user is standing still. This way, the components of acceleration sensed by the ADXL345 will be solely due to gravity.
The following equation can be used to compute elevation:

![equation](https://latex.codecogs.com/gif.latex?%5Ctheta%3D%5Carctan%5Cbiggl%28%5Cfrac%7Ba_z%7D%7B%5Csqrt%7Ba_x%5E2&plus;a_y%5E2%7D%7D%5Cbiggr%29)

This equation assumes the following:
- **The IMU was stationary when the measurement was taken**. This is ensured while panning by introducing a small delay after each motion.
- **Roll of the PTU in the body frame is 0.** This is reasonable to assume as there is no degree-of-freedom along this axis.
- **Elevation and roll of the PTU in the world frame is 0.** This assumes that the user is not on an incline/decline (otherwise elevation would be non-zero) and that the PTU is mounted on a horizontal surface (otherwise roll would be non-zero). This is reasonable to assume as the target application is in an indoor environment.

This calculation is implemented in ```findOrientations``` which shall be discussed in further detail in the next section.

### Magnetometer
The magnetometer can be used to calculate the current bearing in the world frame. Before doing so, the HML5883 needs to be configured to provide reasonable readings. In particular, the default sensor field range in the provided I2C and L3G4200D modules was ![equation](https://latex.codecogs.com/gif.latex?%5Cinline%20%5Cpm%200.88%5C%2C%5Ctext%7BGa%7D). 

However, it was observed during testing that frequent overflows of the magnetometer readings occurred. This was deduced by slowly rotating the IMU one revolution on a fixed platform and plotting the readings along each axis as a function of time. A sinusoidal waveform would be expected if no overflows occur, hence the range was increased to ![equation](https://latex.codecogs.com/gif.latex?%5Cinline%20%5Cpm%208.1%5C%2C%5Ctext%7BGa%7D) to achieve this, at the cost of less precise readings. This required setting ```CRB``` (Configuration Register B) during setup of the HML5883 to modify the device gain as follows:

```c
#define HM5883_CFG_REG_B 0x01

// change magnetometer gain to allow readings of up to 8.1 Ga to prevent overflow 
MAG_CFG_STRUCT mag_cfg_b = {HM5883_CFG_REG_B, 0xE0};

// initialise the magnetometers
IIC_ERRORS magnet_init(void) {
  iic_send_data(magnet_wr, (uint8_t*)&mag_cfg, sizeof(MAG_CFG_STRUCT));
  return iic_send_data(magnet_wr, (uint8_t*)&mag_cfg_b, sizeof(MAG_CFG_STRUCT));
}
```
Once appropriate readings are returned, the following equations can be used to calculate bearing:

![equation](https://latex.codecogs.com/gif.latex?%5Cbegin%7Balign*%7D%20y%26%3Dm_y%5C%5Cz%26%3Dm_z%5Ccos%5Ctheta-m_x%5Csin%5Ctheta%5C%5C%5Cbeta%26%3D%5Cbegin%7Bcases%7D90%2C%20%26%20z%3D0%2C%5C%2Cy%3E0%5C%5C270%2C%20%26%20z%3D0%2C%5C%2Cy%3C0%5C%5C%20180-%5Carctan%5Cbigl%28%5Cfrac%7By%7D%7Bz%7D%5Cbigr%29%2C%20%26z%3C0%5C%5C%20-%5Carctan%5Cbigl%28%5Cfrac%7By%7D%7Bz%7D%5Cbigr%29%2C%20%26z%3E0%2C%5C%2Cy%3C0%5C%5C%20360%20-%5Carctan%5Cbigl%28%5Cfrac%7By%7D%7Bz%7D%5Cbigr%29%2C%20%26z%3E0%2C%5C%2Cy%3E0%5Cend%7Bcases%7D%5Cend%7Balign*%7D)

The returned bearing will be an angle in the interval ![equation](https://latex.codecogs.com/gif.latex?%5Cinline%20%5B0%2C360%29). 
As these equations rely on the previously measured elevation, similar assumptions are made. The following additional assumptions are made:
- **The user is isolated from strong magnetic fields.** This includes compasses embedded in laptop/mobile devices and other sources of electromagnetic interference. This way, the magnetometer reading will only be reflective of the strength of Earth's magnetic field.
- **The PTU azimuth angle is zero i.e. it is facing forwards.** This ensures the bearing is reflective of the direction in which the user intends to head towards. 
```c
void findOrientation(Orientation *orientations, AccelScaled *scaled_data, ORIENTATION_MEASUREMENT measurement, MagScaled *mag_data);
```
This function accepts scaled acceleration readings (i.e. accelerations in terms of g) in ```scaled_data``` and magnetometer readings in ```mag_data```. The resulting orientations determined by the equations above are returned in the ```orientations``` struct. The additional argument ```measurement``` can be set to ```BEARING``` or ```ELEVATION_ONLY``` to toggle the calculation of bearing. 

### Gyroscope
The final implementation does not use measurements from the gyroscope. In theory, such measurements would have helped with providing closed-loop control over the PTU's rotation. However, as discussed previously, the data was deemed too noisy to provide reliable orientations. **This is exacerbated by the fact that the gyroscope data must be integrated to obtain orientation, magnifying even the smallest errors.** 

## LiDAR Module (C)

The LiDAR-Lite Sensor version 2 provides the distance to the first object being detected. The LiDAR interacts with the 68HCS12 Dragon Board through pin ```PT1```. By capturing the pulse width of the PWM signal sent to ```PT1```, the measured distance to the obstacle can be calculated by a 1msec/metre relationship. Thus, the following functions in ```lidar.h``` are implemented to measure the distance:
```c
void timer_config(void);
```
The timer block of the 68HCS12 board is intended to measure the duration of a pulse. As the timer is enabled without prescaling, and the input capture register is 16-bit, the maximum pulse width that can be captured is 2.73 msec.

The register ```TIOS``` is in control of either implementing the input capture or the output compare function, since the PWM signal is generated at ```PT1```, the input capture function at channel 1 must be enabled. As the PWM signal also includes a brief sensor measurement period, the channel is configured to initially capture a rising edge, then a falling edge only after the first edge detection to ensure the accuracy of the measurement. This is achieved by setting ```TCTL4``` as needed.
```c
__interrupt void TC1_ISR(void);
```
The capturing of the PWM signal is interpreted as an interrupt. To filter measurement noise, the ```TIE``` register is only enabled for 10 readings at each position, and the minimal value (above a pre-determined noise threshold) is the designated distance. The pulse width is measured by capturing the ```TC1``` value at the rising edge, and subtracting this from the ```TC1``` value at the next falling edge. Despite the limitation of underestimating the distance sometimes, it is usually functional with an accuracy of.
The captured distance is based on the time elapsed for the laser to travel between the object and the sensor, there is a limitation for this methodology:
- **The dected obstacles are assumed to be non-black objects.** This is because that black absorbs more light than other colours, thus if the object is black, the laser cannot recognize it.

## Serial Module (C)

The serial module is designated to transmit real-time orientations of the system, distance to the object, and the program flow indication between CodeWarrior and MATLAB. Two additional functions were implemented to read from ```SCI1```:
```c
void SCI1_InString(char *buffer);
```
This function polls the ```RDRF``` bit in the register ```SCI1SR1```, and reads until a newline (```LF```) character indicating the end of the string, and stores the string in ```buffer```. To maintain consistency with strings in C, a terminating character ```\0``` is also added. The following assumptions are made in this function:
- **The incoming string is terminated by a newline character.** This is ensured by constructing strings in the Serial MATLAB module to always include a terminating newline character.
- **The incoming string does not overflow the buffer.** This is ensured by allocating a sufficiently large buffer.
```c
void flushBuffer(char *buffer);
```
This function clears the contents of ```buffer``` before a new string is read in, to ensure that only the new information is being stored into it.

## Serial Module (MATLAB)

The Serial module in MATLAB can be split as follows:
- receiving LIDAR data for obstacle detection
- receiving the full suite of sensor data for mapping
- receiving magnetometer data for guidance
- transmitting relevant flags to synchronise program flow.

The respective functions below have been implemented to facilitate these interactions between the C and MATLAB portions of the program via ```SCI1```.

```matlab
function data = readLidar(SerialPort)
```
This function is implemented such that it will be polling until a reading of the distance is sent by CodeWarrior through ```SCI1```. This will only occur if an obstacle has been detected.A voice instruction is then played through the laptop to inform the user there are obstacles in the walkway before asking them to stop.

```matlab
function sendSerial(SerialPort, str)
```
This function is important for synchronising the two portions of the program, via transmission of relevant flags. As aforementioned, these flags are sent as strings terminated by newline characters. These flags act as indicators of what the C portion should execute next.  For example, this function is called to trigger the PTU to start panning the environment. 

 ```matlab
function data = readSerial(SerialPort)
 ```
 After stopping the user, the ```panServo``` function will be implemented in the C Portion. During the scanning procedure, the serial port transmits the real-time orientations of the system along with the distance to the object, and stores the information into a matrix that can be used for environment mapping.
 
 ```matlab
function angleMatch = readMagnet(SerialPort, angleToTurn)
 ```
 When the mapping of the environment is successfully accomplished, the user is instructed to turn to the correct bearing that has been calculated in the mapping function. In this function, the magnetometer reading is transmitted through from CodeWarior so that when the correct elevation is achieved, the voice instruction will guide the user to go forwards.
 
## Audio Module (MATLAB)
```matlab
function playPrompt(message)
```
The audio module is in control of guidance and navigation. This function initializes the system speaker in the PC and can b
either
e called with the message required as an parameter in other functions. For guidance purpose, there are three voice instructions to the user; for navigation, instructions are made to navigate the user to turn to the correct direction as well as inform the overshoot while the user is turinng. The following assumption is made in this function.
- **The user is sensible to diretional left and right.** Since the prompts direct the user to turn to left or right, it is assumed that the user starts turning in the right direction.

 ## Mapping and Guidance Module (MATLAB)
 
Mapping of the environment takes the serial readings of distance, elevation and azimuth from the ```readSerial``` function and translates them to the Cartesian coordinates ```x```, ```y``` and ```z```, which are then plotted in a 3D scatter plot. The serial provides intended elevation, actual elevation, azimuth, LiDAR distance, and estimated ground distance. The conversions are calculated using 3D trigonometry shown below. The values taken to be converted are filtered to remove noise and eliminate ground readings using estimated ground distance. Within a 50 - 90 cm distance range, the readings of the LiDAR compared to the area being scanned displayed too much noise. This was the same for values larger than 200cm. Therefore, these values were filtered out to leave behind the narrower, more accurate range. 

![Image of Trig Calcs](https://github.com/Yu295/MTRX2700_Major/blob/main/2700calcs.jpg)

Once the Cartesian coordinates are found, the actual distance between two successive scanned points are found. This is done by first finding the horizontal distance from the origin to two successive points. These distances are labelled ```d1``` and ```d2```, where ```d1 < d2```. The angle subtending the two points is labelled theta. The calculations below then give the front on gap width available for the user to walk through.

![Image of Trig Calcs 2](https://github.com/Yu295/MTRX2700_Major/blob/main/2700calcs2.jpg)

However, these gaps occur at different elevations. Therefore, only the base elevation gaps are taken. The elevation used to reference this is the intended elevation provided, rather than actual elevation, as there is variance in actual data. A gap index is established which corresponds to the available gaps, and the width of these gaps are saved in a ```baseGap``` array. The angle range of the gap is also saved in a matrix. Then, the data points (obstacles) scanned in from other elevations are looped through. Gap indexes are removed from valid gaps when obstacles are detected which do not leave enough width for the user to move through.

If there is more than one gap available, the gap closest to the forwards (positive x) direction is selected. The turn instruction angle is then output through serial. If there is no valid gap, then the turn instruction angle is output as 180 in order to turn the user around. Voice instructions are provided to navigate the user around obstacles.

The system was tested with sample LiDAR data taken from preliminary scans, with known obstacles scanned.


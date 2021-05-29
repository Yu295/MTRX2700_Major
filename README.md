# MTRX2700 Major Project 
# Audio-Based Guided Navigation for Elderly Blind People 

## Jason Lai, Reihana Tsao, Yujiao Cao

### Introduction
This design is an ausio-based guidance system for elderly blind people, which performs obstacle detection and path tracing in daily uses, and provides voice instructions for navigation purposes. The system operates constantly for obstacle detection in the users's path, when there is obstacle found in the way, the system starts to scan the environment and map the corresponding area to obtain a safe path and navigate the user to it through sound information.

### System Flow Chart


### Modular Deisgn

**PTU Module**
The PTU is fitted with a 10 Degree of Freedom Inertial Measurement Unit (IMU) Module which provides the rate of rotation, direction of Earth's magnetic field and absolute acceleration, after manipulating the raw data, we can obtain the azimuth and elevation angles of the system at the current position. 

**Lidar Module**
The Lidar-Lite Sensor version 2 provides the distance to the first object being detected. The lidar interacts with the HCS12 Dragon Board through pin PT1, by capturing the pulse width of the PWM signal being send to PT1, the measured distance to the obstacle can be calculated by a 1msec/metre realationship.

**Serial Module**
The serial module is designated to transmit real-time orientations of the system, distance to the object, and the program flow indication between CodeWarrior and MATLAB.

**Servo Motor Module**
The servo motor is controlled by ultilising the PWM block of the HCS12 Dragon Board. 

**MATLAB Module**
The MATLAB module has two portions, serial data receiving and transmitting, and mapping of the environment. 
The serial portion will be polling until the distance to the object is transmitted from CodeWarrior to MATLAB, then a voice instruction will be played through PC to inform the user that there are obstacles in the walkway and then asks the user to stop. Afterwards, the PTU will be triggered to start panning through . During the scanning procedure, the serial port transmits the real-time orientations of the system along with the distance to the object, and stores the information into a matrix that can be used for environement mapping. 





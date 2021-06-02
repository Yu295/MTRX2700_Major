#ifndef SIMPLESERIAL_HEADER
#define SIMPLESERIAL_HEADER

#define BAUD_300      0
#define BAUD_600      1
#define BAUD_1200     2
#define BAUD_2400     3
#define BAUD_4800     4
#define BAUD_9600     5
#define BAUD_19200    6
#define BAUD_38400    7
#define BAUD_57600    8
#define BAUD_115200   9

#define STOP_LIDAR_FLAG '0'
#define START_MAPPING_FLAG '1'
#define DONE_TURNING_FLAG '3'
#define DONE_PANNING_FLAG "5\n"

// initialise SCI1
void SCI1_Init(unsigned short baudRate);

// Output single character
void SCI1_OutChar(char);  

// Output null terminated string 
void SCI1_OutString(char *buffer);

// Read newline terminated string from SCI1 and store a null-terminated version into buffer
void SCI1_InString(char *buffer); 

#endif
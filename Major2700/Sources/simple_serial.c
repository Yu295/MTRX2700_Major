// simple serial interface

#include "simple_serial.h" 
#include "derivative.h"        /* derivative information */

// empties null-terminated buffer before reading in a new string
void flushBuffer(char *buffer);

// initialise SCI1
void SCI1_Init(unsigned short baudRate) {
  
  SCI1BDH = 0;

  switch(baudRate){
	case BAUD_300:
	  SCI1BDH=19;
  	SCI1BDL=136;
	  break;
	case BAUD_600:
	  SCI1BDH=9;
  	SCI1BDL=196;
	  break;
	case BAUD_1200:
	  SCI1BDH=4;
  	SCI1BDL=226;
	  break;
	case BAUD_2400:
	  SCI1BDH=2;
  	SCI1BDL=113;
	  break;
	case BAUD_4800:
    SCI1BDH=1;
    SCI1BDL=56;
	  break;
	case BAUD_9600:
    SCI1BDH=0;
    SCI1BDL=156;
	  break;
	case BAUD_19200:
    SCI1BDH=0;
    SCI1BDL=78;
	  break;
	case BAUD_38400:
    SCI1BDH=0;
    SCI1BDL=39;
	  break;
	case BAUD_57600:
    SCI1BDH=0;
    SCI1BDL=26;
	  break;
	case BAUD_115200:
    SCI1BDH=0;
    SCI1BDL=13;
	  break;
  }
  
  SCI1CR1 = 0;
  SCI1CR2 = 0x0C; 
}
          
// Output single character
void SCI1_OutChar(char data) {
 
  while((SCI1SR1 & SCI1SR1_TDRE_MASK) == 0);
  SCI1DRL = data;  
}

#define NULL_CHARACTER 0x00
#define LINE_FEED 0x0A
#define CARRIAGE_RETURN 0x0D

// Output null terminated string 
void SCI1_OutString(char *buffer) {

  while(*buffer) {
  
    SCI1_OutChar(*buffer);
    buffer++; 
  }  
}

// Read a string ends in a new line character
void SCI1_InString(char *buffer) {
  unsigned char count = 0;                  // keep track of where to insert char into buffer
  volatile char c = 0;                      // current char
  
  flushBuffer(buffer);                      // empty the buffer if not already empty
  
  // read until a newline character
  while (c != LINE_FEED) {
    
    while(!(SCI1SR1 & SCI1SR1_RDRF_MASK));  // poll the RDRF bit until a char is ready to be read
    c = (char)SCI1DRL;                      // read the char
    buffer[count] = c;                      // store char correctly in buffer
    ++count; 
  }
  
  buffer[count] = 0;                        // add terminating NULL char for consistency with C strings
  return;
}

// Clears the content of buffer
void flushBuffer(char *buffer) {
  while (*buffer) {
    *buffer = 0;
    ++buffer;
  }
  return;
}



/* ***************************************************************************** */
/* You can use this file to define the low-level hardware control fcts for       */
/* LED, button and LCD devices.                                                  */ 
/* Note that these need to be implemented in Assembler.                          */
/* You can use inline Assembler code, or use a stand-alone Assembler file.       */
/* Alternatively, you can implement all fcts directly in master-mind.c,          */  
/* using inline Assembler code there.                                            */
/* The Makefile assumes you define the functions here.                           */
/* ***************************************************************************** */


#ifndef	TRUE
#  define	TRUE	(1==1)
#  define	FALSE	(1==2)
#endif

#define	PAGE_SIZE		(4*1024)
#define	BLOCK_SIZE		(4*1024)

#define	INPUT			 0
#define	OUTPUT			 1

#define	LOW			 0
#define	HIGH			 1


// APP constants   ---------------------------------

// Wiring (see call to lcdInit in main, using BCM numbering)
// NB: this needs to match the wiring as defined in master-mind.c

#define STRB_PIN 24
#define RS_PIN   25
#define DATA0_PIN 23
#define DATA1_PIN 10
#define DATA2_PIN 27
#define DATA3_PIN 22

// -----------------------------------------------------------------------------
// includes 
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/types.h>
#include <time.h>

// -----------------------------------------------------------------------------
// prototypes

int failure (int fatal, const char *message, ...);

// -----------------------------------------------------------------------------
// Functions to implement here (or directly in master-mind.c)

/* this version needs gpio as argument, because it is in a separate file */
void digitalWrite (uint32_t *gpio, int pin, int value) {
  /* ***  COMPLETE the code here, using inline Assembler  ***  */
}

// adapted from setPinMode
void pinMode(uint32_t *gpio, int pin, int mode /*, int fSel, int shift */) {
  /* ***  COMPLETE the code here, using inline Assembler  ***  */
}

void writeLED(uint32_t *gpio, int led, int value) {
  /* ***  COMPLETE the code here, using inline Assembler  ***  */
}

int readButton(uint32_t *gpio, int button) {
  /* ***  COMPLETE the code here, using inline Assembler  ***  */
}

void waitForButton(uint32_t *gpio, int button) {
  /* ***  COMPLETE the code here, just C no Assembler; you can use readButton ***  */
}

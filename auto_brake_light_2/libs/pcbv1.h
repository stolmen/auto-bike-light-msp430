/*
  pcbv1.h

  Bike light PCB: mappings for version 1 of the PCB. 
  
*/

#include <stdint.h>
//#include <msp430.h>


#ifndef PCBV1_H
#define PCBV1_H

/*
LED definitions
	- Numbering: with PCB orientation upwards. (see diagram)
	This numbering does NOT correspond with the PCB numbering, 
	or the LED signal numbering on the Altium schematic.
	
		2		1

	
		3		4
		
	- Due to the use of the pre-biased 'digital' transistor 
	ICs, two of the LED signals are 'active-high', and the
	other two are 'active-low'. These will also be defined below.

	- Mapping with Altium schematic and designator: 
	
		LED#	Signal#		Designator	
		1		LED0 P1.1	D301		Active-low
		2		LED3 P1.4	D303		Active-high
		3		LED1 P1.2	D302		Active-low
		4		LED2 P1.3	D300		Active-high
	
	Okay, now here are the #defines. Again, the names here 
	are NOT the same as the signal# defined above, but the LED#.
*/

// LED pins
#define LED1_PIN BIT1
#define LED2_PIN BIT4
#define LED3_PIN BIT2
#define LED4_PIN BIT3

// IIC comms pins
#define SCL_PIN BIT6
#define SDA_PIN BIT7
#define ACCEL_INT BIT0

// some macros to make life a little easier.
// To reduce the chance of unexpected behaviour, 
// do not use these macros on the same line as other code.
// Of course, set-up these outputs correctly first.
#define SWITCH_LED1_ON		P1OUT &= ~(LED1_PIN)
#define SWITCH_LED1_OFF		P1OUT |= LED1_PIN
#define SWITCH_LED2_ON		P1OUT |= LED2_PIN
#define SWITCH_LED2_OFF		P1OUT &= ~(LED2_PIN)
#define SWITCH_LED3_ON		P1OUT &= ~(LED3_PIN)
#define SWITCH_LED3_OFF		P1OUT |= LED3_PIN
#define SWITCH_LED4_ON		P1OUT |= LED4_PIN
#define SWITCH_LED4_OFF		P1OUT &= ~(LED4_PIN)
#define ALL_LEDS_OFF 		P1OUT = LED1_PIN + LED3_PIN;
#define ALL_LEDS_ON			P1OUT = LED2_PIN + LED4_PIN;

#endif

#include <msp430.h> 
#include <pcbv1.h>
#include <iic.h>
#include <mpu6050.h>
//#include <signal.h> 	// for GCC compiler. Used for ISR calls.



// FIR filter coefficient
#define K_FACTOR 0.1f

// State machine coefficients
#define THRESH ((int) 6000)


static void allLEDOff();
static void allLEDOn();


/*
 * main.c
 * Auto brake light project
 * Edward Ong
 * September 2015
 * First re-write attempt.
 * Heavy use of msp430g2x21_usi_12.c - code example provided by TI for
 * 	the implementation of the i2c state machine.
 */

int main(void) {
	_disable_interrupts();
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

    // *** Setup of registers and so on

    // DCO setup
    DCOCTL = 0;                               // Select lowest DCOx and MODx settings
    BCSCTL1 = CALBC1_1MHZ;                    // Set DCO
    DCOCTL = CALDCO_1MHZ;

    // Set all GPIOs to output by default.
    P1DIR = 0xFF;
	P2DIR = 0XFF;

	allLEDOff();

	// Set up accel interrupt GPIO pin
	P1DIR &= ~ACCEL_INT; 	// P1.5 input
	P1IES &= ~ACCEL_INT; 	// active high interrupt
	P1IE &= ~ACCEL_INT; 	// Make sure that no GPIO interrupts occur on this pin..

	// Select primary mode for both ports
	P1SEL = 0;
	P2SEL = 0;

	// Set address to the MPU6050.
	// This is the only slave device.
	slave_i2c_address = MPU6050_I2C_ADDRESS << 1;

	allLEDOn();

	/////// Initial configuration of MPU6050
	iicWrite(MPU6050_I2C_MST_CTRL, 0x00);

	// configure and enabled interrupts on data ready
	iicWrite(MPU6050_INT_PIN_CFG, MPU6050_LATCH_INT_EN);
	iicWrite(MPU6050_INT_ENABLE, MPU6050_DATA_RDY_EN);
	// wake from sleep, set sample and sleep mode
	iicWrite(MPU6050_PWR_MGMT_1, MPU6050_CYCLE);
	iicWrite(MPU6050_PWR_MGMT_2, MPU6050_LP_WAKE_CTRL_2 + MPU6050_STBY_XG + MPU6050_STBY_YG + MPU6050_STBY_ZG);

	allLEDOff();

	char state = 1;

	char z0 = 0;	// data bytes
	char z1 = 0;
	char y0 = 0;
	char y1 = 0;
	char x0 = 0;
	char x1 = 0;

	int x = 0;
	int y = 0;
	int z = 0;

	// Kill all interrupts.
	_BIC_SR(GIE);

	P1IFG = 0;			// clear P1 IFG.

	iicRead(MPU6050_INT_STATUS);
	_BIC_SR(GIE);
	P1IE |= ACCEL_INT;		// enable interrupts

	// Begin primary function state machine.
	while(1){
		_BIS_SR(LPM3_bits + GIE);

		// Read and construct 16-bit data
		z0 = iicRead(MPU6050_ACCEL_ZOUT_L);
		z1 = iicRead(MPU6050_ACCEL_ZOUT_H);
		z = z0 + (z1 << 8);

		y0 = iicRead(MPU6050_ACCEL_YOUT_L);
		y1 = iicRead(MPU6050_ACCEL_YOUT_H);
		y = y0 + (y1 << 8);

		x0 = iicRead(MPU6050_ACCEL_XOUT_L);
		x1 = iicRead(MPU6050_ACCEL_XOUT_H);
		x = x0 + (x1 << 8);

		// Kill all interrupts.
		_BIC_SR(GIE);

		// set new state.
		// Hysteresis!
		if (z > 0){
			state = 1;
		} else {
			state = 2;
		}

		// Only wakes up when interrupt is received from PORT 1.
		switch(state){
		case 1:
			// All LEDs on!
			P1OUT |= LED2_PIN + LED4_PIN;
			P1OUT &= ~(LED1_PIN + LED3_PIN);
			break;
		case 2:
			// Turn all LEDs off!
			P1OUT |= LED1_PIN + LED3_PIN;
			P1OUT &= ~(LED2_PIN + LED4_PIN);
			break;
		}

		// clear latch on MPU-6050, and re-enable interrupt for reception of more data.
		// GIE cleared to make sure that interrupt is not tripped before going into LPM.

		iicRead(MPU6050_INT_STATUS);
		_BIC_SR(GIE);
		P1IE |= ACCEL_INT;
	}
}

static void allLEDOff(){
	P1OUT |= LED1_PIN + LED3_PIN;
	P1OUT &= ~(LED2_PIN + LED4_PIN);
}
static void allLEDOn(){
	P1OUT |= LED2_PIN + LED4_PIN;
	P1OUT &= ~(LED1_PIN + LED3_PIN);
}


// interrupts from GPIO
#pragma vector=PORT1_VECTOR
__interrupt void PORT1 (void){
	// Disable interrupt until we're done processing the current data.
	P1IFG &= ~ACCEL_INT;
	P1IE &= ~ACCEL_INT;
	_BIC_SR(LPM3_EXIT); // wake up from low power mode
}

#include <msp430.h> 
#include <pcbv1.h>
#include <iic.h>
#include <mpu6050.h>
//#include <signal.h> 	// for GCC compiler. Used for ISR calls.

// FIR filter coefficient
#define K_FACTOR 0.1f

// State machine coefficients
#define THRESH 2000

#define ACCEL_SENS_F 16384.0f		// ACCELEROMETER SENSITIVITY FLOAT

// Initial accelerations!
// Divide by 16
typedef struct accel_data_struct{
	int x;
	int y;
	int z;
} accel_data;

static void allLEDOff();
static void allLEDOn();
static void readAccel(accel_data *data);

char update_pitch;

/*
 * main.c
 * Auto brake light project
 * Edward Ong
 * September 2015
 * First re-write attempt.
 *
 */

int main(void) {
	_disable_interrupts();
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

    // *** Setup of registers and so on

    // Allocate some space in RAM stack for some acceleration data.
	accel_data initial_accel;
    accel_data current_accel;

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

	// Set address to the MPU6050 for IIC.
	// This is the only slave device.
	slave_i2c_address = MPU6050_I2C_ADDRESS << 1;

	allLEDOn();

	/////// Initial configuration of MPU6050
	iicWrite(MPU6050_I2C_MST_CTRL, 0x00);
	// read initial orientation. initial_accel will not be written into at all any more.
	readAccel(&initial_accel);
	// configure and enabled interrupts on data ready
	iicWrite(MPU6050_INT_PIN_CFG, MPU6050_LATCH_INT_EN);
	iicWrite(MPU6050_INT_ENABLE, MPU6050_DATA_RDY_EN);
	// wake from sleep, set sample and sleep mode
	iicWrite(MPU6050_PWR_MGMT_1, MPU6050_CYCLE);
	iicWrite(MPU6050_PWR_MGMT_2, MPU6050_LP_WAKE_CTRL_2 + MPU6050_STBY_XG + MPU6050_STBY_YG + MPU6050_STBY_ZG);

	allLEDOff();

	// Kill all interrupts.
	_BIC_SR(GIE);

	// Clear P1IFG and release MPU6050 int_status latch
	P1IFG = 0;
	iicRead(MPU6050_INT_STATUS);

	// Declare some vars
	char state = 1;
	int comp_z;
	int cur_z = 0;
	update_pitch=0;

	// Timer setup
	CCTL0 = CCIE;                             // CCR0 interrupt enabled
	CCR0 = 50000;
	TACTL = TASSEL_2 + MC_2 + ID_3;                  // SMCLK, contmode, /8

	// Disable all maskable interrupts, THEN un-mask interrupts on ACCEL_INT.
	// This prevents jumping into the ISR immediately after un-masking.
	// This behaviour results in the ISR being serviced BEFORE entering LPM3!! :(
	_BIC_SR(GIE);
	P1IE |= ACCEL_INT;		// enable interrupts

	// Begin primary function state machine.
	/*
	 * The flow:
	 * 1. Update accelerometer reading variables
	 * 2. Perform pitch compensation on z (this only happens every now and then)
	 * 3. Parse compensated z into state machine
	 * 4. Quick LPF
	 * 4. LEDs will light up depending on the state
	 * 5. Wait for more accelerometer data in LPM3. Then repeat!
	 */
	while(1){
		_BIS_SR(LPM3_bits + GIE);

		// Kill all interrupts.
		_BIC_SR(GIE);

		readAccel(&current_accel);

		if (update_pitch){
			/*
			 * Periodic pitch compensation.
			 * 1. Calculate current compensation amount from z accel reading.
			 * 2. Update compensation amount through averaging.
			 * 3. Reset update_pitch.
			 */

			comp_z = (comp_z/16*15) + (current_accel.z/16);

			update_pitch = 0;
		}

		current_accel.z = current_accel.z - comp_z;
		cur_z = cur_z/8*7 + current_accel.z/8;

		// set new state.
		// Hysteresis!
		if (abs(cur_z) > THRESH){
			state = 1;
		} else {
			state = 2;
		}

		// Only wakes up when interrupt is received from PORT 1.
		switch(state){
		case 1:
			allLEDOn();
			break;
		case 2:
			allLEDOff();
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
	//P1OUT |= LED1_PIN + LED3_PIN;
	P1OUT &= ~(LED2_PIN + LED4_PIN);
}
static void allLEDOn(){
	P1OUT |= LED2_PIN + LED4_PIN;
	//P1OUT &= ~(LED1_PIN + LED3_PIN);
}

/*
 * readAccel
 * Updates 'data' struct with new accel readings.
 * Flow (for each of x,y,z):
 * 1. Read 2 bytes into memory
 * 2. Construct 2 byte word and store into struct
 */
static void readAccel(accel_data *data){
	char x0,x1,y0,y1,z0,z1;

	z0 = iicRead(MPU6050_ACCEL_ZOUT_L);
	z1 = iicRead(MPU6050_ACCEL_ZOUT_H);
	data->z = z0 + (z1 << 8);
	/*
	y0 = iicRead(MPU6050_ACCEL_YOUT_L);
	y1 = iicRead(MPU6050_ACCEL_YOUT_H);
	data->y = y0 + (y1 << 8);

	x0 = iicRead(MPU6050_ACCEL_XOUT_L);
	x1 = iicRead(MPU6050_ACCEL_XOUT_H);
	data->x = x0 + (x1 << 8);*/
}

/*
 * Port 1 ISR
 * Waking up from this will get the accelerometer readings updated!
 * Flow:
 * 1. Mask all P1 interrupts and clear interrupt flag
 * 2. Reset the update_pitch flag to signal accelerometer reading!
 * 3. (Accelerometer readings are updated in the state machine)
 */
#pragma vector=PORT1_VECTOR
__interrupt void PORT1 (void){
	// Disable interrupt until we're done processing the current data.
	P1IFG &= ~ACCEL_INT;
	P1IE &= ~ACCEL_INT;
	_BIC_SR(LPM3_EXIT); // wake up from low power mode
}

/*
 * Timer A0 ISR
 * Waking up from this will get the pitch updated!
 * Flow:
 * 1. Set the flag
 * 2. Reset timer
 * 4. (Pitch will get updated in state machine next time it wakes up.)
 */
#pragma vector=TIMERA0_VECTOR
__interrupt void TIMERA0(void){
	update_pitch = 1;
	P1OUT ^= LED1_PIN;
	_BIC_SR(LPM3_EXIT); // wake up from low power mode
}

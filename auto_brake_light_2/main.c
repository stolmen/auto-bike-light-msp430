#include <msp430.h> 
#include <pcbv1.h>
#include <iic.h>
#include <mpu6050.h>
// FIR filter coefficient
#define K_FACTOR 0.1f

// State machine coefficients
#define THRESH ((int) 0)

#define GYROSCOPE_SENSITIVITY 65.536f		// in LSB/(rad/s)
#define UPDATE_INTERVAL 40.0f							// update interval of data in milliseconds

// math!
#define M_PI 3.14159265359f

// Initial accelerations!
// Divide by 16
typedef struct accel_data_struct{
	int x;
	int y;
	int z;
} accel_data;

typedef struct gyro_data_struct{
	int x;
	int y;
	int z;
} gyro_data;

typedef struct three_axis_read_buffer_struct{
	char x0;
	char x1;
	char y0;
	char y1;
	char z0;
	char z1;
} three_axis_read_buffer;

static void allLEDOff();
static void allLEDOn();
static void readAccel(accel_data *data, three_axis_read_buffer *buffer);
static void readGyro(gyro_data *data, three_axis_read_buffer *buffer);
static void update_z_comp(accel_data *adata, gyro_data *gdata, float *pitch);

static float estimate_tan(float angle);
static float estimate_arctan(float ratio);

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

    // Allocate some space in RAM stack for some data
	accel_data initial_accel;
    accel_data current_accel;
    gyro_data current_gyro; 	// this is used in the pitch-compensation algorithm
    three_axis_read_buffer buffer;
    float pitch;			// pitch in radians
    float comp_z;			// compensated z

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
	// read initial orientation, and then initialise the compensation amount
	// note that the pitch is stored as a floating point number, in radians. (THink of a better way!)
	readAccel(&initial_accel, &buffer);
	pitch = estimate_arctan(((float) initial_accel.z) / ((float)initial_accel.x));	// theta = arctan(z/x)
	// configure and enabled interrupts on data ready
	iicWrite(MPU6050_INT_PIN_CFG, MPU6050_LATCH_INT_EN);
	iicWrite(MPU6050_INT_ENABLE, MPU6050_DATA_RDY_EN);
	// wake from sleep, set sample and sleep mode
	iicWrite(MPU6050_PWR_MGMT_1, MPU6050_CYCLE);
	iicWrite(MPU6050_PWR_MGMT_2, MPU6050_LP_WAKE_CTRL_2);

	allLEDOff();

	char state = 1;

	// Kill all interrupts.
	_BIC_SR(GIE);

	// Clear P1IFG and release MPU6050 int_status latch
	P1IFG = 0;
	iicRead(MPU6050_INT_STATUS);

	// Disable all maskable interrupts, THEN un-mask interrupts on ACCEL_INT.
	// This prevents jumping into the ISR immediately after un-masking.
	// This behaviour results in the ISR being serviced BEFORE entering LPM3!! :(
	_BIC_SR(GIE);
	P1IE |= ACCEL_INT;		// enable interrupts

	// Begin primary function state machine.
	while(1){
		_BIS_SR(LPM3_bits + GIE);

		// Signal flow:
		/*
		 * 1. Read accel and gyro data into current_gyro and current_accel
		 * 2. Using the complementary filter described in http://www.pieter-jan.com/node/11, calculate new pitch
		 * 3. Compensate for pitch by adding/subtracting from z axis acceleration
		 * 4. Using compensated acceleration, determine the next state
		 *
		 *
		 * Steps 2 and 3 are 'calculated' using the small angle approximations tan(x) = x and equivalently arctan(x) = x. (x in radians)
		 *
		 *
		 */

		readAccel(&current_accel, &buffer);
		readGyro(&current_gyro, &buffer);
		update_z_comp(&current_accel, &current_gyro, &pitch);
		comp_z = ((float)current_accel.z) - estimate_tan(pitch);

		// Kill all interrupts.
		_BIC_SR(GIE);

		// set new state.
		// Hysteresis!
		if (comp_z > 0.0f){
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

// Update curr_z_comp using data from accel and gyro, and the current compensation amount.
// Algorithm courtesy of http://www.pieter-jan.com/node/11
static void update_z_comp(accel_data *adata, gyro_data *gdata, float *pitch){
	float pitchAcc;

	*pitch += ((float)gdata->z / GYROSCOPE_SENSITIVITY) * UPDATE_INTERVAL; // Angle around the X-axis
	int forceMagnitudeApprox = abs(adata->x) + abs(adata->y) + abs(adata->z);

	if ((forceMagnitudeApprox > 8192) && (forceMagnitudeApprox < 32768)){
		pitchAcc = estimate_arctan((float)adata->z / (float)adata->x);
		*pitch = *pitch * 0.98f + pitchAcc * 0.02f;
	}
}


// // // Some estimation functions (turns out that they just convert between degrees and radians.

// Input: floating point number in degrees
// Output: a floating point number
static float estimate_tan(float angle){
	return(angle * M_PI / ((float) 180));
}

// Input: a floating point number
// Output: floating point number in degrees
static float estimate_arctan(float ratio){
	return(ratio * ((float) 180) / M_PI);
}


static void allLEDOff(){
	P1OUT |= LED1_PIN + LED3_PIN;
	P1OUT &= ~(LED2_PIN + LED4_PIN);
}
static void allLEDOn(){
	P1OUT |= LED2_PIN + LED4_PIN;
	P1OUT &= ~(LED1_PIN + LED3_PIN);
}

static void readAccel(accel_data *data, three_axis_read_buffer *buffer){
	buffer->z0 = iicRead(MPU6050_ACCEL_ZOUT_L);
	buffer->z1 = iicRead(MPU6050_ACCEL_ZOUT_H);
	data->z = buffer->z0 + (buffer->z1 << 8);

	buffer->y0 = iicRead(MPU6050_ACCEL_YOUT_L);
	buffer->y1 = iicRead(MPU6050_ACCEL_YOUT_H);
	data->y = buffer->y0 + (buffer->y1 << 8);

	buffer->x0 = iicRead(MPU6050_ACCEL_XOUT_L);
	buffer->x1 = iicRead(MPU6050_ACCEL_XOUT_H);
	data->x = buffer->x0 + (buffer->x1 << 8);
}

static void readGyro(gyro_data *data, three_axis_read_buffer *buffer){
	// Read and construct 16-bit data
	buffer->z0 = iicRead(MPU6050_GYRO_ZOUT_L);
	buffer->z1 = iicRead(MPU6050_GYRO_ZOUT_H);
	data->z = buffer->z0 + (buffer->z1 << 8);

	buffer->y0 = iicRead(MPU6050_GYRO_YOUT_L);
	buffer->y1 = iicRead(MPU6050_GYRO_YOUT_H);
	data->y = buffer->y0 + (buffer->y1 << 8);

	buffer->x0 = iicRead(MPU6050_GYRO_XOUT_L);
	buffer->x1 = iicRead(MPU6050_GYRO_XOUT_H);
	data->x = buffer->x0 + (buffer->x1 << 8);
}

// interrupts from GPIO
#pragma vector=PORT1_VECTOR
__interrupt void PORT1 (void){
	// Disable interrupt until we're done processing the current data.
	P1IFG &= ~ACCEL_INT;
	P1IE &= ~ACCEL_INT;
	_BIC_SR(LPM3_EXIT); // wake up from low power mode
}

#include <msp430.h> 
#include <pcbv1.h>
#include <iic.h>
#include <mpu6050.h>
//#include <signal.h> 	// for GCC compiler. Used for ISR calls.

// filter coefficients
#define ACCEL_COEFF 8
#define COMP_COEFF 16

#define DETECTION_THRESHOLD 2000

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
static int smoothFilter(int prev, int curr, int coeff);

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
	int comp_z = 0; 
    int comp_x = 0;
	int cur_z = 0;
	update_pitch=0;

	// Timer setup
    // TODO: set these up properly. 
	CCTL0 = CCIE;                             // CCR0 interrupt enabled
    CCTL1 = CCIE;                             // CCR1 interrupt enabled
	CCR0 = 50000;
    CCR1 = 2000;
	TACTL = TASSEL_2 + MC_2 + ID_2;                  // SMCLK, contmode, /4
    

	// Disable all maskable interrupts, THEN un-mask interrupts on ACCEL_INT.
	// This prevents jumping into the ISR immediately after un-masking.
	// This behaviour results in the ISR being serviced BEFORE entering LPM3!! :(
	_BIC_SR(GIE);
	P1IE |= ACCEL_INT;		// enable interrupts

	// Begin primary function state machine.
	/*
	 * The flow:
	 * 1. Update accelerometer reading variables
     * 2. (Periodically) update pitch compensation amount
	 * 2. Pitch compensation
     * 3. Bump compensation
	 * 4. Smoothing (two-sample weighted average)
	 * 5. Parse z into the state machine
	 * 6. LEDs will light up depending on the state
	 * 7. Wait for more accelerometer data in LPM3. Then repeat!
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
             * 1a. Also update comp_x for fast compensation.
			 * 2. Update smoothed compensation amount (two-sample weighted average).
			 * 3. Reset update_pitch flag.
			 */
            comp_x = smoothFilter(comp_x, current_accel.x, COMP_COEFF);
            comp_z = smoothFilter(comp_z, current_accel.z, COMP_COEFF);
            
			update_pitch = 0;
		}

		current_accel.z -= comp_z;
        current_accel.z -= comp_z / comp_x * current_accel.z;    // z_n = tan(theta) * z = g_z/g_x*z
        cur_z = smoothFilter(cur_z, current_accel.z, ACCEL_COEFF);

		// set new state.
		// Hysteresis!
		if (abs(cur_z) > DETECTION_THRESHOLD){
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

static int smoothFilter(int prev, int curr, int coeff){
	return ((prev/coeff*(coeff-1)) + (curr/coeff));
}


/*
 * readAccel
 * Updates 'data' struct with new accel readings.
 * Flow (for each of x,y,z):
 * 1. Read 2 bytes into memory
 * 2. Construct 2 byte word and store into struct
 */
static void readAccel(accel_data *data){
	char x0,x1;
	// char y0,y1;
	char z0,z1;

	z0 = iicRead(MPU6050_ACCEL_ZOUT_L);
	z1 = iicRead(MPU6050_ACCEL_ZOUT_H);
	data->z = z0 + (z1 << 8);

    /*
	y0 = iicRead(MPU6050_ACCEL_YOUT_L);
	y1 = iicRead(MPU6050_ACCEL_YOUT_H);
	data->y = y0 + (y1 << 8);
    */
    
	x0 = iicRead(MPU6050_ACCEL_XOUT_L);
	x1 = iicRead(MPU6050_ACCEL_XOUT_H);
	data->x = x0 + (x1 << 8);
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
    static count;   // Counts number of timer interrupts
    
    // TODO: move all this to main somehow!
    
    // Check where the interrupt came from
    if (TAIV & CCR0_BIT){
        // Interrupt came from capture-compare 0
        // So increment or decrement the duty cycle!
        if (TACCR0 >= CCR_MAX){
            TACCR0 -= 1;    // decrease duty cycle
        } else if (TACCR0 <= CCR_MIN){ 
            TACCR0 += 1;    // increase duty cycle
        }
        
        // Turn the LEDs on 
        turn_all_leds_on();
        
        
    } else if ((TAIV & CCR1_BIT) && (count++ > DATA_INTERVAL_COUNT)) {
        // It's time to update the pitch.
        update_pitch = TRUE;
        count = 0;
        _BIC_SR(LPM3_EXIT); // wake up from low power mode to read from accel
    } else { 
        turn_all_leds_off(); 
    }

	
}

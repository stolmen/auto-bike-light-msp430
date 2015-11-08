/*
 * iic.c
 *
 *  Created on: 7 Nov 2015
 *      Author: ed
 *
 *  Uses USI_VECTOR
 *   Heavy use of msp430g2x21_usi_12.c - code example provided by TI for
 * 	the implementation of the i2c state machine.
 *
 */

#include <iic.h>

#include <msp430.h>

// I2C COMMS
void Master_Transmit(void);
char Master_Recieve(void);

void Setup_USI_Master_TX(void);
void Setup_USI_Master_RX(void);

void Data_TX (void);
void Data_RX (void);

// State variables
char I2C_State, Bytecount, Transmit = 0;
char curr_data = 0;			// data to send
char curr_reg_address = 0; // target of transmission! (address)
char slave_address_sent = 0;	// flag, used when reading a register.
char curr_output = 0;		// this is data received from the slave upon reading a register.

#define number_of_bytes 2  // change this later..


#pragma vector = USI_VECTOR
__interrupt void USI_TXRX (void){


	switch(I2C_State){
		case 0: // Generate Start Condition & send address to slave
			Bytecount = 0;
			USISRL = 0x00;                // Generate Start Condition...
			USICTL0 |= USIGE+USIOE;
			USICTL0 &= ~USIGE;
			USISRL = slave_i2c_address;		// Send slave address + write first!
			USICNT = 8;
			// USICNT = (USICNT & 0xE0) + 0x08; // Bit counter = 8, TX Address
			I2C_State = 2;              	  // next state: rcv address (N)Ack
			break;

		case 2: // Receive Address Ack/Nack bit
			USICTL0 &= ~USIOE;            // SDA = input
			USICNT |= 0x01;               // Bit counter=1, receive (N)Ack bit
			I2C_State = 4;                // Go to next state: check (N)Ack
			break;

		case 4: // Process Address Ack/Nack & handle data TX

			if (USISRL & 0x01)            // If Nack received...
			{ // Send stop...
				USICTL0 |= USIOE;             // SDA = output
				USISRL = 0x00;
				USICNT |=  0x01;            // Bit counter=1, SCL high, SDA low
				I2C_State = 14;             // Go to next state: generate Stop
			}
			else
			{ // Ack received, TX adress to slave...
				if (slave_address_sent == 1){
					// Now, the slave will start sending data.
					// This should really only happen if
					// transmit == 0..
					Data_RX();
				} else{
					// Send the slave address across.
					USICTL0 |= USIOE;             // SDA = output
					USISRL = curr_reg_address;            // Load data byte
					USICNT |=  0x08;              // Bit counter = 8, start TX
					Bytecount++;
					I2C_State = 10;               // next state: receive data (N)Ac
				}


			}
			break;

		case 5: // Generate repeated start condition

			USISRL = 0x00;                // While clock is high, force SDA low.
			USICTL0 |= USIGE+USIOE;
			USICTL0 &= ~USIGE;

			USICTL0 |= USIOE;	// make sure that the output is enabled.

			// Now send slave address.
			USISRL = slave_i2c_address + 1;		// Send slave address + read.
			USICNT =  8; // Bit counter = 8, TX Address

			slave_address_sent = 1; // Set flag, so that data is sent in state 4.
			I2C_State = 2;              	  // next state: rcv address (N)Ack

			break;

		case 6: // Send Data Ack/Nack bit
			USICTL0 |= USIOE;             // SDA = output
			curr_output = USISRL;		// grab output


			if (Bytecount <= number_of_bytes-2)
			{                             // If this is not the last byte
				USISRL = 0x00;              // Send Ack
				I2C_State = 4;              // Go to next state: data/rcv again
				Bytecount++;
			}

			else //last byte: send NACK
			{
				USISRL = 0xFF;              // Send NAck
				I2C_State = 8;              // stop condition
			}

			USICNT |= 0x01;               // Bit counter = 1, send (N)Ack bit
			break;

		case 8: // Prep Stop Condition
			USICTL0 |= USIOE;             // SDA = output
			USISRL = 0x00;
			USICNT |=  0x01;              // Bit counter= 1, SCL high, SDA low
			I2C_State = 14;               // Go to next state: generate Stop
			break;

		case 10: // Receive Data Ack/Nack bit
			USICTL0 &= ~USIOE;            // SDA = input
			USICNT |= 0x01;               // Bit counter = 1, receive (N)Ack bit
			I2C_State = 12;               // Go to next state: check (N)Ack
			break;

		case 12: // Process Data Ack/Nack & send Stop
			USICTL0 |= USIOE;

			if (Transmit == 1){
				if (Bytecount == number_of_bytes){// If last byte
					USISRL = 0x00;
					I2C_State = 14;               // Go to next state: generate Stop
					USICNT |=  0x01;             // set count=1 to trigger next state
					Bytecount++;
				}else{
					Data_TX();                  // TX byte
				}
			} else {
				// prepare a repeated start transmission.

				USICTL0 |= USIOE;
				USISRL = 0xFF;
				USICNT = 1;

				I2C_State = 5;
			}

			break;

		case 14:// Generate Stop Condition
			USISRL = 0x0FF;               // USISRL = 1 to release SDA
			USICTL0 |= USIGE;             // Transparent latch enabled
			USICTL0 &= ~(USIGE+USIOE);    // Latch/SDA output disabled
			I2C_State = 0;                // Reset state machine for next xmt
			slave_address_sent = 0;		// Reset flag
			LPM0_EXIT;                    // Exit active for next transfer
			break;
		}

  USICTL1 &= ~USIIFG;                       // Clear pending flag
}


void Data_TX (void){
	USISRL = curr_data;          // Load data byte
	USICNT = (USICNT & 0xE0) + 8;              // Bit counter = 8, start TX
	I2C_State = 10;               // next state: receive data (N)Ack
	Bytecount++;
}

void Data_RX (void){
	USICTL0 &= ~USIOE;                  // SDA = input --> redundant
	USICNT  = (USICNT & 0xE0) + 8;                    // Bit counter = 8, RX data
	I2C_State = 6;                      // Next state: Test data and (N)Ack
}

void Setup_USI_Master_TX (void)
{
	_disable_interrupts();
	Bytecount = 0;
	Transmit = 1;
	USICTL0 = USIPE6+USIPE7+USIMST+USISWRST;  // Port & USI mode setup
	USICTL1 = USII2C+USIIE;                   // Enable I2C mode & USI interrupt
	USICKCTL = USIDIV_7+USISSEL_2+USICKPL;    // USI clk: SCL = SMCLK/128
	USICNT |= USIIFGCC;                       // Disable automatic clear control
	USICTL0 &= ~USISWRST;                     // Enable USI
	USICTL1 &= ~USIIFG;                       // Clear pending flag
	_enable_interrupts();
}

void Setup_USI_Master_RX (void)
{
	_disable_interrupts();
	Bytecount = 0;
	Transmit = 0;
	USICTL0 = USIPE6+USIPE7+USIMST+USISWRST;  // Port & USI mode setup
	USICTL1 = USII2C+USIIE;                   // Enable I2C mode & USI interrupt
	USICKCTL = USIDIV_7+USISSEL_2+USICKPL;    // USI clks: SCL = SMCLK/128
	USICNT |= USIIFGCC;                       // Disable automatic clear control
	USICTL0 &= ~USISWRST;                     // Enable USI
	USICTL1 &= ~USIIFG;                       // Clear pending flag
	_enable_interrupts();

}

void Master_Transmit(void){
	Setup_USI_Master_TX();
    USICTL1 |= USIIFG;                      // Set flag and start communication
    LPM0;                                   // CPU off, await USI interrupt
    __delay_cycles(10000);                  // Delay between comm cycles
}
char Master_Recieve(void){
  Setup_USI_Master_RX();
  USICTL1 |= USIIFG;                        // Set flag and start communication
  LPM0;                                     // CPU off, await USI interrupt
  __delay_cycles(10000);                    // Delay between comm cycles
  return curr_output;
}

// Wrappers for ADXL state machine
void iicWrite(char reg, char data){
	curr_data = data;
	curr_reg_address = reg;
	Master_Transmit();
}

char iicRead(char reg){
	slave_address_sent = 0;
	curr_reg_address = reg;
	return Master_Recieve();
}

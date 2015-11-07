/*
 * iic.h
 *
 *  Created on: 7 Nov 2015
 *      Author: ed
 *
 *  Before attempting to send things,
 *  be sure to write to 'slave_i2c_address'.
 *  Set this to the 7 bit address plus a leading 0 (LSB).
 */

#ifndef IIC_H_
#define IIC_H_

// I2C COMMS
void Master_Transmit(void);
char Master_Recieve(void);

void Setup_USI_Master_TX(void);
void Setup_USI_Master_RX(void);

void Data_TX (void);
void Data_RX (void);

// WRAPPERS FOR I2C COMMS FOR ADXL
void iicWrite(char reg, char data);
char iicRead(char reg);

char slave_i2c_address; 	// set this before starting transmission!



#endif /* IIC_H_ */

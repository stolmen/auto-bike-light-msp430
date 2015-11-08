/*
 * iic.h
 *
 *  Created on: 7 Nov 2015
 *      Author: stolmen
 *
 *      Based off MSP430 example code provided by TI.
 *
 *  Before attempting to send things,
 *  be sure to write to 'slave_i2c_address'.
 *  Set this to the 7 bit address plus a leading 0 (LSB).
 *
 *  At the moment only supports single byte reads and writes.
 */

#ifndef IIC_H_
#define IIC_H_

// WRAPPERS FOR I2C COMMS FOR ADXL
void iicWrite(char reg, char data);
char iicRead(char reg);

char slave_i2c_address; 	// be sure to set this to the 7 bit address shifted left by 1!

#endif /* IIC_H_ */

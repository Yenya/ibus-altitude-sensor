#ifndef BMP280_H_
#define BMP280_H_
/*
 * Library for Bosch BMP280 pressure sensor for AVR MCUs in plain old C.
 *
 * This library uses the I2C layer by Peter Fleury, and is loosely modelled
 * after the bmp085 library by Davide Gironi.
 * 
 * The library is distributable under the terms of the GNU General
 * Public License, version 2 only.
 *
 * Written by Jan "Yenya" Kasprzak, https://www.fi.muni.cz/~kas/
 *
 * Configuration can be done by editing the top of the bmp280.c file.
 */

uint8_t bmp280_init(void);		// call this first
uint8_t bmp280_get_status(void);	// read the status register
void bmp280_set_config(uint8_t t_sb, uint8_t filter, uint8_t spi3w_en);
	// set the configuration register
void bmp280_set_ctrl(uint8_t osrs_t, uint8_t osrs_p, uint8_t mode);
	// set the oversampling and mode (this starts the conversion)
void bmp280_measure(void);		// do a measurement

// the following functions return the result of the last measurement
#define bmp280_getpressure()	(_bmp280_pres)
#define bmp280_gettemperature()	(_bmp280_temp)
double bmp280_getaltitude(void);

// do not use directly, call the macros above
extern int32_t _bmp280_temp;
extern uint32_t _bmp280_pres;

#endif /* BMP280_H_ */

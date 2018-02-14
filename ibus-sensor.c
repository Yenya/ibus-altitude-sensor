
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#include "bmp085/bmp085.h"

#define N_IBUS_CHANNELS 6	// How many sensors there will be

volatile uint8_t sens_addr[N_IBUS_CHANNELS]; // sensor addresses

// Sensor IDs taken from qba667 FlySky FS-i6 firmware mod. See the
// ibustelemetry.h in his source code.
#define IBUS_SENS_INTV		0x00 // Internal Voltage
#define IBUS_SENS_TEMP		0x01 // Temperature
#define IBUS_SENS_RPM		0x02 // RPM
#define IBUS_SENS_EXTV		0x03 // External Voltage
#define IBUS_SENS_CLIMB		0x09 // Clibm rate m/s *100
#define IBUS_SENS_PRES		0x41 // Pressure
#define IBUS_SENS_GPS_ALT	0x82 // 4 bytes signed GPS alt m*100
#define IBUS_SENS_ALT		0x83 // 4 bytes signed Alt m*100
#define IBUS_SENS_ALT_MAX	0x84 // 4 bytes signed Alt m*100

// Which sensors we have
const uint8_t sens_type[N_IBUS_CHANNELS] = {
	IBUS_SENS_TEMP,
	IBUS_SENS_ALT,        // absolute height, 4 bytes signed m*100
	IBUS_SENS_GPS_ALT,    // relative height, 4 bytes signed m*100
	IBUS_SENS_ALT_MAX,    // max relative height, 4 bytes signed m*100
	IBUS_SENS_CLIMB,      // climb rate m/s * 100, 2 bytes
	IBUS_SENS_EXTV,
};
volatile uint32_t sens_val[N_IBUS_CHANNELS];

// function prototypes
static void handle_rx_packet(void);

/* ------------ on-board LEDs ------------- */
static void led_init(void)
{
	DDRB |= _BV(PB0); // Rx LED
	DDRD |= _BV(PD5); // Tx LED
}

static void led1_on(void)
{
	PORTB &= ~_BV(PB0);
}

static void led1_off(void)
{
	PORTB |= _BV(PB0);
}

static void led2_on(void)
{
	PORTD &= ~_BV(PD5);
}

static void led2_off(void)
{
	PORTD |= _BV(PD5);
}

/* ----------------- USART ----------------- */

// I-Bus uses 115200n8
#define UART_BAUD       115200
#define UBRR_VAL        ((F_CPU + 8UL * UART_BAUD) / (16UL*UART_BAUD) - 1)

#define BUFLEN 8
static volatile uint8_t buffer[BUFLEN];
static volatile uint8_t buf_offset;

static void serial_init(void)
{
	UBRR1 = UBRR_VAL;

	UCSR1A = 0;
	// UCSR0B = _BV(RXCIE1)|_BV(RXEN1)|_BV(TXEN1);
	UCSR1B = _BV(RXEN1) | _BV(RXCIE1) | _BV(UDRIE1);
        UCSR1C = _BV(UCSZ11)|_BV(UCSZ10);
}

static void recv_restart(void)
{
	led2_off();

	buf_offset = 0;
	UCSR1B &= ~_BV(TXEN1);
	UCSR1B |= _BV(RXEN1) | _BV(RXCIE1);
}

static void tx_start(void)
{
	buf_offset = 0;

	UCSR1B &= ~_BV(RXEN1);
	UCSR1B |= _BV(TXEN1) | _BV(UDRIE1);
}

// USART receive interrupt
ISR(USART1_RX_vect)
{
	uint8_t val = UDR1;

	// a shorthand - for now, we accept 4-byte packets only
	if (buf_offset == 0 && val != 4)
		return;
	
	buffer[buf_offset++] = val;

	if (buf_offset == buffer[0]) {
		handle_rx_packet();
		buf_offset = 0;
	}
}

// Next Tx byte wanted
ISR(USART1_UDRE_vect)
{
	if (buf_offset < buffer[0])
		UDR1 = buffer[buf_offset++];

	if (buf_offset >= buffer[0]) { // finished
		UCSR1B &= ~_BV(UDRIE1);
		UCSR1B |= _BV(TXCIE1);
	}
}

// Tx finished
ISR(USART1_TX_vect)
{
	UCSR1B &= ~_BV(TXCIE1);
	recv_restart();
}

/* ---- A/D converter for battery voltage ---- */
static void adc_init(void)
{
	DIDR0 |= _BV(ADC4D); // disable digital input on ADC4
	ADCSRA = _BV(ADEN)   // enable ADC
		| _BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2); // CLK/128 = 125 kHz
	ADMUX = _BV(REFS1) | _BV(REFS0) // internal 2.56V reference
		| _BV(MUX2); // ADC4 on pin PF4
}

// TODO: use the ADC interrupt instead
static uint16_t read_adc_sync(void)
{
	uint16_t retval;

	ADCSRA |= _BV(ADSC); // start the conversion

	// wait for the result
	while ((ADCSRA & _BV(ADIF)) == 0)
		;

	retval = ADCW;
	ADCSRA |= _BV(ADIF); // clear the interrupt flag

	return retval;
}

#define EXT_V_DIVIDER	((390+15)/15)	// 390k and 15k resistors
#define adc_to_10mv(x) ((x) * EXT_V_DIVIDER * 0.25)

/* ----------------- iBus ------------------ */
static void ibus_init(void)
{
	uint8_t i;

	for (i = 0 ; i < N_IBUS_CHANNELS; i++)
		sens_addr[i] = 0;

	recv_restart();
}

static void send_buffer(void)
{
	uint8_t i;
	uint16_t csum = 0xFFFF;

	led2_on(); // off after the frame is sent

	// compute the I-Bus checksum
	for (i = 0; i < buffer[0] - 2; i++)
		csum -= buffer[i];

	buffer[i++] = csum & 0xFF;
	buffer[i++] = csum >> 8;

	tx_start();
}

static void handle_rx_packet(void)
{
	uint16_t csum = 0xFFFF;
	uint8_t i, cmd, dev;

	for (i = 0; i < buf_offset-2; i++)
		csum -= (uint16_t)buffer[i];

	if ((buffer[buf_offset-2] != (csum & 0xFF))
		|| (buffer[buf_offset-1] != (csum >> 8))) { // invalid csum
		buf_offset = 0; // start over
		return;
	}

	cmd = buffer[1] & 0xF0;
	dev = buffer[1] & 0x0F;

	switch (cmd) {
	case 0x80: // discovery/assign address
		for (i = 0; i < N_IBUS_CHANNELS; i++) {
			if (sens_addr[i] == 0 || sens_addr[i] == dev) {
				sens_addr[i] = dev;
				send_buffer();
				return;
			}
		}
		break;
	case 0x90: // telemetry type request
		for (i = 0; i < N_IBUS_CHANNELS; i++) {
			if (sens_addr[i] == dev) {
				buffer[0] = 0x06; // len
				buffer[2] = sens_type[i];
				// Sensor IDs >= 0x80 are four-byte
				buffer[3] = sens_type[i] < 0x80 ? 0x02 : 0x04;
				send_buffer();
				return;
			}
		}
		break;
	case 0xA0: // get measurement request
		for (i = 0; i < N_IBUS_CHANNELS; i++) {
			if (sens_addr[i] == dev) {
				buffer[2] = sens_val[i] & 0xFF;
				buffer[3] = (sens_val[i] >> 8) & 0xFF;
				// two-byte or four-byte sensor?
				if (sens_type[i] < 0x80) {
					buffer[0] = 0x06;
				} else {
					buffer[0] = 0x08;
					buffer[4] = (sens_val[i] >> 16) & 0xFF;
					buffer[5] = (sens_val[i] >> 24) & 0xFF;
				}
				send_buffer();
				return;
			}
		}
		break;
	}
}

// Basic command interpreter for controlling port pins
#define ALTITUDE_SHIFT	5
#define CLIMB_SHIFT 4

int main(void)
{
	int32_t alt, base_alt, alt_measured;
	int32_t max_alt = 0, climb = 0, prev_alt = 0, climb_sum = 0;
	uint8_t base_alt_measurements = 20, climb_measurements = 0;

	bmp085_init();
	adc_init();
	led_init();

	alt = ((int32_t)bmp085_getaltitude() * 100) << ALTITUDE_SHIFT;
	base_alt = alt >> ALTITUDE_SHIFT;

	serial_init();
	ibus_init();

	sei();

	while (1) {
		int32_t tmp;
		uint8_t sens = 0;

		led1_on();

		// temperature
		sens_val[sens++] = 400 + 10*bmp085_gettemperature();

		alt_measured = bmp085_getaltitude() * 100;

		// absolute altitude running average
		alt -= alt >> ALTITUDE_SHIFT;
		alt += alt_measured;

		tmp = alt >> ALTITUDE_SHIFT;
		// absolute altitude
		sens_val[sens++] = tmp;

		if (base_alt_measurements) {
			base_alt_measurements--;
			max_alt = 0;
			base_alt = tmp;
			tmp = 0;
			prev_alt = 0;
		} else {
			// convert to relative
			tmp -= base_alt;

			// maximum relative
			if (tmp > max_alt) {
				max_alt = tmp;
			}
		}
		// relative altitude
		sens_val[sens++] = tmp;

		// maximum altitude
		sens_val[sens++] = max_alt;

		climb_sum += alt_measured;

		if (++climb_measurements >= (1 << CLIMB_SHIFT)) {
			climb_measurements = 0;
			climb_sum >>= CLIMB_SHIFT;
			climb = (climb_sum - prev_alt)*16/(1 << CLIMB_SHIFT);
			prev_alt = climb_sum;
			climb_sum = 0;
		}

		// climb rate (RoC)
		sens_val[sens++] = climb;

		// ext_voltage
		tmp = read_adc_sync();
		sens_val[sens++] = adc_to_10mv(tmp);

		led1_off();
                _delay_ms(60);
	}
}



#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#ifdef SENSOR_BMP085
#include "bmp085/bmp085.h"
#endif

#ifdef SENSOR_BMP280
#include "bmp280/bmp280.h"
#endif

#define N_IBUS_CHANNELS 6	// How many sensors there will be
#define MAIN_LOOP_MS	50	// Used for RoC -> m/s conversion

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
#ifdef __AVR_ATmega328P__
static void led_init(void)
{
	DDRB |= _BV(PB5); // Rx LED
}

static void led1_on(void)
{
	PORTB |= _BV(PB5);
}

static void led1_off(void)
{
	PORTB &= ~_BV(PB5);
}

static void led2_on(void) { }
static void led2_off(void) { }
#endif

#ifdef __AVR_ATmega32U4__
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
#endif

/* ----------------- USART ----------------- */

// I-Bus uses 115200n8
#define UART_BAUD       115200
#define UBRR_VAL        ((F_CPU + 8UL * UART_BAUD) / (16UL*UART_BAUD) - 1)

#define BUFLEN 8
static volatile uint8_t buffer[BUFLEN];
static volatile uint8_t buf_offset;

#ifdef __AVR_ATmega328P__
static void serial_init(void)
{
	UBRR0 = UBRR_VAL;

	UCSR0A = 0;
	UCSR0B = _BV(RXEN0) | _BV(RXCIE0) | _BV(UDRIE0);
        UCSR0C = _BV(UCSZ01)|_BV(UCSZ00);
}

static void serial_enable_rx(void)
{
	UCSR0B &= ~(_BV(TXEN0) | _BV(TXCIE0));
	UCSR0B |= _BV(RXEN0) | _BV(RXCIE0);
}

static void serial_enable_tx(void)
{
	UCSR0B &= ~_BV(RXEN0);
	UCSR0B |= _BV(TXEN0) | _BV(UDRIE0);
}

static void serial_notify_tx_end(void)
{
	UCSR0B &= ~_BV(UDRIE0);
	UCSR0B |= _BV(TXCIE0);
}

#define serial_rx_vect USART_RX_vect
#define serial_tx_vect USART_TX_vect
#define serial_udre_vect USART_UDRE_vect

#define serial_data UDR0

#endif

#ifdef __AVR_ATmega32U4__
static void serial_init(void)
{
	UBRR1 = UBRR_VAL;

	UCSR1A = 0;
	UCSR1B = _BV(RXEN1) | _BV(RXCIE1) | _BV(UDRIE1);
        UCSR1C = _BV(UCSZ11)|_BV(UCSZ10);
}

static void serial_enable_rx(void)
{
	UCSR1B &= ~(_BV(TXEN1) | _BV(TXCIE1));
	UCSR1B |= _BV(RXEN1) | _BV(RXCIE1);
}

static void serial_enable_tx(void)
{
	UCSR1B &= ~_BV(RXEN1);
	UCSR1B |= _BV(TXEN1) | _BV(UDRIE1);
}

static void serial_notify_tx_end(void)
{
	UCSR1B &= ~_BV(UDRIE1);
	UCSR1B |= _BV(TXCIE1);
}

#define serial_rx_vect USART1_RX_vect
#define serial_tx_vect USART1_TX_vect
#define serial_udre_vect USART1_UDRE_vect

#define serial_data UDR1

#endif

static void recv_restart(void)
{
	// led2_on();

	buf_offset = 0;
	serial_enable_rx();
}

static void tx_start(void)
{
	buf_offset = 0;
	serial_enable_tx();
}

// USART receive interrupt
ISR(serial_rx_vect)
{
	uint8_t val = serial_data;
	static uint8_t ledstate;

	// a shorthand - for now, we accept 4-byte packets only
	if (buf_offset == 0 && val != 4)
		return;
	
	buffer[buf_offset++] = val;
		if (ledstate) {
			led2_on();
			ledstate = 0;
		} else {
			led2_off();
			ledstate = 1;
		}

	if (buf_offset == buffer[0]) {
		handle_rx_packet();
		buf_offset = 0;
	}
}

// Next Tx byte wanted
ISR(serial_udre_vect)
{
	if (buf_offset < buffer[0])
		serial_data = buffer[buf_offset++];

	if (buf_offset >= buffer[0]) // finished
		serial_notify_tx_end();
}

// Tx finished
ISR(serial_tx_vect)
{
	recv_restart();
}

/* ---- A/D converter for battery voltage ---- */

#define EXT_V_DIVIDER	((390.0+15)/15)	// 390k and 15k resistors

#ifdef __AVR_ATmega328P__
static void adc_init(void)
{
	ADCSRA = _BV(ADEN)   // enable ADC
		| _BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2); // CLK/128 = 125 kHz
	ADMUX = _BV(REFS1) | _BV(REFS0) // internal 1.1V reference
		| _BV(MUX2) | _BV(MUX1) | _BV(MUX0); // ADC7
}

#define adc_to_10mv(x) ((x) * EXT_V_DIVIDER * 0.1074)

#endif

#ifdef __AVR_ATmega32U4__
static void adc_init(void)
{
	DIDR0 |= _BV(ADC4D); // disable digital input on ADC4
	ADCSRA = _BV(ADEN)   // enable ADC
		| _BV(ADPS0) | _BV(ADPS1) | _BV(ADPS2); // CLK/128 = 125 kHz
	ADMUX = _BV(REFS1) | _BV(REFS0) // internal 2.56V reference
		| _BV(MUX2); // ADC4 on pin PF4
}

#define adc_to_10mv(x) ((x) * EXT_V_DIVIDER * 0.25)

#endif

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

	// led2_on(); // off after the frame is sent

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
#ifdef SENSOR_BMP085
#define ALTITUDE_SHIFT	5
#define CLIMB_SHIFT	5
#else	// BMP280 does a running average by itself
#define ALTITUDE_SHIFT	0
#define CLIMB_SHIFT	3
#endif
#define VOLTAGE_SHIFT	5 // 10-bit ADC, so it has to be 6 or less

int main(void)
{
	int32_t alt, base_alt, alt_measured;
	int32_t max_alt = 0, climb = 0, prev_alt = 0, climb_sum = 0;
	uint8_t base_alt_measurements = 20, climb_measurements = 0;
	uint16_t voltage_raw;

#ifdef SENSOR_BMP085
	bmp085_init();
#endif
#ifdef SENSOR_BMP280
	bmp280_init();
	bmp280_set_config(0, 3, 0); // 0.5 ms delay, 8x filter, no 3-wire SPI
#endif
	adc_init();
	led_init();
	led1_off();

#ifdef SENSOR_BMP085
	alt = ((int32_t)bmp085_getaltitude() * 100) << ALTITUDE_SHIFT;
#endif
#ifdef SENSOR_BMP280
	bmp280_measure();
	alt = ((int32_t)bmp280_getaltitude() * 100) << ALTITUDE_SHIFT;
#endif
	base_alt = alt >> ALTITUDE_SHIFT;

	voltage_raw = read_adc_sync() << VOLTAGE_SHIFT;

	serial_init();
	ibus_init();

	sei();

	while (1) {
		int32_t tmp;
		uint8_t sens = 0;

		led1_on();

		// temperature
#ifdef SENSOR_BMP085
		sens_val[sens++] = 400 + 10*bmp085_gettemperature();

		alt_measured = bmp085_getaltitude() * 100;
#endif

#ifdef SENSOR_BMP280
		bmp280_measure();

		sens_val[sens++] = 400 + bmp280_gettemperature()/10;

		alt_measured = bmp280_getaltitude() * 100;
#endif

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

		/*
		 * RoC: we do not want to calculate it every time,
		 * because it is very imprecise. So we average the
		 * altitude (1 << CLIMB_SHIFT) times, and then compute
		 * the climb rate based of this.
		 *
		 * This is not very precise, it is better to use the raw
		 * altitude in Tx instead.
		 */
		climb_sum += alt_measured;

		if (++climb_measurements >= (1 << CLIMB_SHIFT)) {
			climb_measurements = 0;
			climb_sum >>= CLIMB_SHIFT;
			climb = (climb_sum - prev_alt)*1000/MAIN_LOOP_MS;
			prev_alt = climb_sum;
			climb_sum = 0;
		}

		// climb rate (RoC)
		sens_val[sens++] = climb;

		// ext_voltage
		voltage_raw -= voltage_raw >> VOLTAGE_SHIFT;
		voltage_raw += read_adc_sync();
		tmp = adc_to_10mv(voltage_raw >> VOLTAGE_SHIFT);
		if (tmp < 100) { // when unconnected, don't send the noise
			tmp = 0;
		}
		sens_val[sens++] = tmp;

		led1_off();
                _delay_ms(MAIN_LOOP_MS);
	}
}


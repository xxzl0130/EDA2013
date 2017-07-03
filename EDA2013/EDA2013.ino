#include <SPI.h>
#include <stdint.h>
#include "CircularBuffer.h"
#include <avr/wdt.h>

// pins from power monitor module
#define CLKPIN  2
#define SDOPIN  3     // should be on PORTD. Will need to change the ClockISR if different

#define BUFSIZE 20    // grab last 20 bytes of message

#define VOLTAGE_RANGE   0.250   // full scale V channel voltage
#define CURRENT_RANGE    0.050   // full scale I channel voltage (PGA 50x instead of 10x)
#define VOLTAGE_DIVIDER  450220.0/220.0    // input voltage channel divider (R1+R2/R2)
#define CURRENT_SHUNT    620      // empirically obtained multiplier to scale Vshunt drop to I    
#define FLOAT24       16777216.0  // 2^24 (converts to float24)
#define POWER_MULTIPLIER    1 / 512.0   // Energy->Power divider; not sure why, but seems correct. Datasheet not clear about this.

#define VOLTAGE_MULTIPLIER   (float)  (1 / FLOAT24 * VOLTAGE_RANGE * VOLTAGE_DIVIDER)
#define CURRENT_MULTIPLIER   (float)  (1 / FLOAT24 * CURRENT_RANGE * CURRENT_SHUNT)

CircularBuffer <uint8_t, BUFSIZE> cbuf;
volatile uint8_t buf = 0;
volatile uint8_t count = 0;
volatile bool syncpulse = false;

#define TIMER_RESET   1     // T2 Counter reset value. 
// Use 1 for Fosc = 16MHz and 100 for Fosc=8Mhz, goal to achieve about 10-30ms timer interval

union {
	uint8_t bytearray[4];
	uint32_t uint32;
	int32_t  int32;
} array2int;

void setup() {

	pinMode(SDOPIN, INPUT);
	pinMode(CLKPIN, INPUT_PULLUP);
	Serial.begin(9600);

	// setup Timer2 to filter for long pulses to reset sync
	// prescaler = /1024, ~16ms per overflow @ Fosc 16MHz
	TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);

	//wait for first long clock pulse
	while (pulseIn(CLKPIN, HIGH, 60000) < 30000);

	//setup watchdog timer to 8 seconds
	wdt_enable(WDTO_8S);
}

// SPI interrupt routine
void ClockISR(void)
{
	TCNT2 = TIMER_RESET; // reset T2 counter  
						 // grab byte from spi, stuff into circular buffer
	buf |= bitRead(PIND, SDOPIN) << (7 - count);
	count++;
	if (count == 8) {
		cbuf.push(buf);
		buf = 0;
		count = 0;
	}
}

ISR(TIMER2_OVF_vect)
{
	// went ~16ms without a clock pulse, probably in an inter-frame period; reset sync
	syncpulse = true;
}

void loop() {
	uint8_t i;

	// sniff spi from CS5460A

	buf = 0;
	count = 0;
	syncpulse = false;
	TIFR2 = _BV(TOV2); // clear T2 interrupts 
	TCNT2 = TIMER_RESET; // reset T2 counter
	TIMSK2 = _BV(TOIE2); // enable T2 overflow interrupt
						 // setup interrupt to clock in data until we hit the sync pulse
	attachInterrupt(digitalPinToInterrupt(CLKPIN), ClockISR, RISING);
	// stop when we we hit the long pulse
	while (!syncpulse);
	detachInterrupt(digitalPinToInterrupt(CLKPIN));
	TIMSK2 = 0; // disable T2 interrupt

	// parse result
	/*
	We'll only care about the last 20 bytes of the previous frame before the long clock pulse
	This contains the last status register where DRDY=1, followed by registers:
	Vrms, Irms, E (true power)
	*/

	bool result_good = false;
	cbuf.rp_front();

	// get status register
	for (i = 0; i < 4; i++)
		// read backwards, little-endian?
		array2int.bytearray[3 - i] = cbuf.pop();

	// check Status register is has conversion ready ( DRDY=1, ID=15 )
	// if this doesnt match expected result, probably means the buffer has junk
	if (array2int.uint32 == 0x009003C1)
		result_good = true;

	// discard next 4 bytes
	uint8_t b;
	for (i = 0; i < 4; i++)
		b = cbuf.pop();

	if (result_good) {

		// read Vrms register
		for (i = 0; i < 4; i++)
			// read backwards, little-endian?
			array2int.bytearray[3 - i] = cbuf.pop();
		uint32_t voltageraw = array2int.uint32;
		float voltage = voltageraw * VOLTAGE_MULTIPLIER;


		// read Irms register
		for (i = 0; i < 4; i++)
			// read backwards, little-endian?
			array2int.bytearray[3 - i] = cbuf.pop();
		uint32_t currentraw = array2int.uint32;
		float current = currentraw * CURRENT_MULTIPLIER;


		// read E (energy) register
		for (i = 0; i < 4; i++)
			// read backwards, little-endian?
			array2int.bytearray[3 - i] = cbuf.pop();

		if (array2int.bytearray[2] >> 7) {
			// must sign extend int24 -> int32LE
			array2int.bytearray[3] = 0xFF;
		}
		int32_t energyraw = array2int.int32;
		float true_power = energyraw * POWER_MULTIPLIER;

		float apparent_power = voltage * current;

		float power_factor = true_power / apparent_power;

		Serial.print("voltage: ");
		Serial.print(voltage);

		Serial.print(", current: ");
		Serial.print(current, 4);

		Serial.print(", true power: ");
		Serial.print(true_power, 1);

		Serial.print(", app. power: ");
		Serial.print(apparent_power, 1);

		Serial.print(", PF: ");
		Serial.print(power_factor);

		Serial.println();
	}

	wdt_reset(); // service watchdog timer
}


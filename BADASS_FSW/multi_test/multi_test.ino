// Includes

// Include sensor libraries

#include <Wire.h>

// include communication libraries
#include <XBee.h>
#include "ccsds_xbee.h"
#include "BADASS_Interface.h"

// Start things here

boolean status; // so that things only get sent once.

uint8_t data[200]; // send this.

void setup() {
	Serial.begin(250000);
	Serial3.begin(9600);

	for (int i = 0; i < 200; i++) {
		data[i] = (uint8_t)i;
	}

	status = true;
}

void loop() {
	if (status) {
		status = false;
		sendTlmMsg((uint16_t)2, data, 200);
	}
}

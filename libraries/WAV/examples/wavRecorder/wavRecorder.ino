/*!
* @file wavPlayer.ino
* @brief demostrate how to implement recording a wav file to sd card
*     The wav file have 24khz sample rate
* 
* @author ouki.wang(ouki.wang@dfrobot.com)
* @version  V1.0
* @date  2015-12-11
* @detail first version
*/

#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <WAV.h>

// set up variables using the SD utility library functions:
Sd2Card card;

const int chipSelect = 30;  ///< assign sd card chipselect pin

void setup()
{
	// Open serial communications and wait for port to open:
	Serial.begin(115200);
	while (!Serial) {
		; // wait for serial port to connect. Needed for Leonardo only
	}

    while(1){
		Serial.print("\nInitializing SD card...");

		// we'll use the initialization code from the utility libraries
		// since we're just testing if the card is working!
		if (!card.init(SPI_HALF_SPEED, chipSelect)) {
			Serial.println("initialization failed. Things to check:");
			Serial.println("* is a card inserted?");
			Serial.println("* is your wiring correct?");
			Serial.println("* did you change the chipSelect pin to match your shield or module?");
			delay(2000);
			continue;
		} else {
			Serial.println("Wiring is correct and a card is present.");
			if (!SD.begin(chipSelect)) {
				Serial.println("Card failed, or not present");
				delay(2000);
				continue;
			}
			break;
		}
	}
}

void loop() 
{
	int ret;
	ret = wav.record("rec.wav");
	unsigned int start = millis();
	while(ret == WAV_ENCODING){
		ret = wav.encode();
		if(millis() - start > 5000){
			wav.stop();
			ret = WAV_ENCODE_END;
		}
	};
	while(1);
}

/******************************************************************************
  Copyright (C) <2016>  <ouki.wang>
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
  Contact: ouki.wang@dfrobot.com
 ******************************************************************************/
 
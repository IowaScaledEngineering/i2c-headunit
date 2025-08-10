/*************************************************************************
Title:    I2C-LCD (4x20 LCD Controller via I2C)
Authors:  Michael Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     $Id: $
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2025 Michael Petersen & Nathan Holmes

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

*************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <avr/eeprom.h>

#include <avr/sleep.h>
#include <stdbool.h>
#include <stdint.h>

#include "i2c-rx.h"
#include "debouncer.h"
#include "lcd.h"

#define DEFAULT_TWI_ADDRESS 0x72
#define DEFAULT_BACKLIGHT 29
#define DEFAULT_LINES 4
#define DEFAULT_WIDTH 20
#define  DEFAULT_DISPLAY_SYSTEM_MESSAGES  true

//Internal EEPROM locations for the user settings
#define LOCATION_TWI_ADDRESS             0
#define LOCATION_BACKLIGHT_BRIGHTNESS    1
#define LOCATION_CONTRAST                2 //8 bit
#define LOCATION_DISPLAY_SYSTEM_MESSAGES 3 //8 bit

#define MIN(a,b) ((a)<(b)?(a):(b))
#define MAX(a,b) ((a)>(b)?(a):(b))

#define settingLCDwidth 20
#define settingLCDlines 4

volatile uint8_t i2c_txBuf[2];
const uint8_t i2c_txBufSize = sizeof(i2c_txBuf)/sizeof(i2c_txBuf[0]);

uint8_t characterCount = 0;
uint8_t currentFrame[settingLCDwidth * settingLCDlines]; //Max of 4 x 20 LCD

bool settingDisplaySystemMessages=false; //User can turn on/off the messages that are displayed when setting (like contrast) is changed

uint8_t customCharData[8]; //Records incoming custom character data
uint8_t customCharSpot = 0 ; //Keeps track of where we are in custCharData array
uint8_t customCharNumber = 0; //LCDs can store 8 custom chars, this keeps track

//New variables for Set RGB command
uint8_t rgbSpot = 0 ; //Keeps track of where we are in rgbData array

void updateDisplay(uint8_t incoming);

volatile uint32_t millis = 0;

uint32_t getMillis()
{
	uint32_t retmillis;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
	{
		retmillis = millis;
	}
	return retmillis;
}

volatile uint8_t backlightPWM = 0;

ISR(TIMER0_COMPA_vect) 
{
	static uint8_t subMillisCounter = 0;
	static uint8_t pwmPhase = 0;

	if (backlightPWM > pwmPhase)
		PORTD |= _BV(PD4);
	else 
		PORTD &= ~_BV(PD4);

	if (++pwmPhase >= 29)
		pwmPhase = 0;

	// Now do all the counter incrementing and such
	if (++subMillisCounter >= 4)
	{
		subMillisCounter = 0;
		millis++;
	}
}

void initializeTimer()
{
	TIMSK0 = 0;           // Timer interrupts OFF

	// Set up Timer/Counter0 for 100Hz clock
	TCCR0A = 0b00001010;  // CTC Mode
	                      // CS01 - 1:8 prescaler
	OCR0A = 250;          // 8MHz / 8 / 250 = 4kHz
	TIMSK0 = _BV(OCIE0A);
}

#define BACKLIGHT_OFF    0
#define BACKLIGHT_FULL  29

void lcd_backlightOn(uint8_t pwm)
{
	backlightPWM = pwm;
}


typedef enum 
{
	MODE_NORMAL, //No mode, just print
	MODE_COMMAND, //Used to indicate if a command byte has been received
	MODE_SETTING, //Used to indicate if a setting byte has been received
	MODE_CONTRAST, //First setting mode, then contrast change mode, then the value to change to
	MODE_TWI, //First setting mode, then I2C mode, then change I2C address
	MODE_RECORD_CUSTOM_CHAR, //First setting mode, then custom char mode, then record 8 bytes
	MODE_SET_RGB //First setting mode, then RGB mode, then get 3 bytes
} DisplayMode;


#define INPUT_UPDATE_MS 10

int main(void)
{
	// Deal with watchdog first thing
	MCUSR = 0;              // Clear reset status
	wdt_reset();            // Reset the WDT, just in case it's still enabled over reset
	wdt_enable(WDTO_1S);    // Enable it at a 1S timeout.

	uint32_t lastReadTime=0, currentTime=0;
	DebounceState8_t buttons;
	DebounceState8_t gpio;

	// PORT A
	//  PA7 - n/a
	//  PA6 - n/a
	//  PA5 - n/a
	//  PA4 - n/a
	//  PA3 - Input  - OUT1
	//  PA2 - Input  - OUT4
	//  PA1 - Input  - JP2
	//  PA0 - Input  - n/c

	// PORT B
	//  PB7 - Input  - OUT3
	//  PB6 - Input  - OUT2
	//  PB5 - Input  - n/c (ICSP)
	//  PB4 - Input  - n/c (ICSP
	//  PB3 - Input  - n/c (ICSP
	//  PB2 - Input  - Button 1
	//  PB1 - Input  - Button 2
	//  PB0 - Input  - Button 3

	// PORT C
	//  PC7 - Input  - n/c 
	//  PC6 - n/a    - /RESET (not I/O pin)
	//  PC5 - n/a    - SCL
	//  PC4 - n/a    - SDA
	//  PC3 - Output - LCD EN
	//  PC2 - Output - LCD RW
	//  PC1 - Output - LCD RS
	//  PC0 - Input  - JP1

	// PORT D
	//  PD7 - Input  - Button 4
	//  PD6 - Input  - OUT6
	//  PD5 - Input  - OUT5
	//  PD4 - Output - Backlight Enable
	//  PD3 - Output - LCD Data 3
	//  PD2 - Output - LCD Data 2
	//  PD1 - Output - LCD Data 1
	//  PD0 - Output - LCD Data 0 

	PORTA = 0b00001111;
	DDRA  = 0b00000000;

	PORTB = 0b11111111;
	DDRB  = 0b00000000;

	PORTC = 0b00110001;
	DDRC  = 0b00001110;

	PORTD = 0b11100000;
	DDRD  = 0b00011111;

	initializeTimer();

	initDebounceState8(&buttons, 0);
	initDebounceState8(&gpio, 0x3F);

	i2cSlaveInitialize(0x72, false);

	lcd_backlightOn(BACKLIGHT_FULL);
	lcd_init(LCD_DISP_ON);
	lcd_clrscr();
	lcd_gotoxy(0,0);

	sei();
	wdt_reset();

	while(1)
	{
		wdt_reset();

		// If there's a byte waiting to be handled, go get it
		if (ringBufferDepth(&i2cRx))
			updateDisplay(ringBufferPopNonBlocking(&i2cRx));

		currentTime = getMillis();
		if (((uint32_t)currentTime - lastReadTime) > INPUT_UPDATE_MS)
		{
			lastReadTime = currentTime;

	//  PB2 - Input  - Button 1
	//  PB1 - Input  - Button 2
	//  PB0 - Input  - Button 3
	//  PD7 - Input  - Button 4
			uint8_t state = ((PINB & _BV(2))?0:0x01) | ((PINB & _BV(1))?0:0x02) | ((PINB & _BV(0))?0:0x04) | ((PIND & _BV(7))?0:0x08);
			debounce8(state , &buttons);
			
			state = ((PINA & _BV(3))?0x01:0) | ((PINB & _BV(6))?0x02:0) | ((PINB & _BV(7))?0x04:0) | ((PINA & _BV(2))?0x08:0) | ((PIND & _BV(5))?0x10:0) | ((PIND & _BV(6))?0x20:0);
			debounce8(state , &gpio);
		
			i2c_txBuf[0] = getDebouncedState(&buttons);
			i2c_txBuf[1] = getDebouncedState(&gpio);
		}
	}
}

/*
Command cheat sheet:
 ASCII / DEC / HEX
 '|'    / 124 / 0x7C - Put into setting mode
 Ctrl+y / 25 / 0x19 - Change the TWI address. Follow Ctrl+x with number 0 to 255. 114 (0x72) is default.
 '-'    / 45 / 0x2D - Clear display. Move cursor to home position.
        / 128-157 / 0x80-0x9D - Set the primary backlight brightness. 128 = Off, 157 = 100%.

 '+'    / 43 / 0x2B - Set Backlight to RGB value, follow + by 3 numbers 0 to 255, for the r, g and b values.
         For example, to change the backlight to yellow send + followed by 255, 255 and 0.

Commands not implemented because they don't make any sense for this application

 Ctrl+c / 3 / 0x03 - Change width to 20
 Ctrl+d / 4 / 0x04 - Change width to 16
 Ctrl+e / 5 / 0x05 - Change lines to 4
 Ctrl+f / 6 / 0x06 - Change lines to 2
 Ctrl+g / 7 / 0x07 - Change lines to 1
 Ctrl+h / 8 / 0x08 - Software reset of the system
 Ctrl+i / 9 / 0x09 - Enable/disable splash screen
 Ctrl+j / 10 / 0x0A - Save currently displayed text as splash
 Ctrl+k / 11 / 0x0B - Change baud to 2400bps
 Ctrl+l / 12 / 0x0C - Change baud to 4800bps
 Ctrl+m / 13 / 0x0D - Change baud to 9600bps
 Ctrl+n / 14 / 0x0E - Change baud to 14400bps
 Ctrl+o / 15 / 0x0F - Change baud to 19200bps
 Ctrl+p / 16 / 0x10 - Change baud to 38400bps
 Ctrl+q / 17 / 0x11 - Change baud to 57600bps
 Ctrl+r / 18 / 0x12 - Change baud to 115200bps
 Ctrl+s / 19 / 0x13 - Change baud to 230400bps
 Ctrl+t / 20 / 0x14 - Change baud to 460800bps
 Ctrl+u / 21 / 0x15 - Change baud to 921600bps
 Ctrl+v / 22 / 0x16 - Change baud to 1000000bps
 Ctrl+w / 23 / 0x17 - Change baud to 1200bps
 Ctrl+x / 24 / 0x18 - Change the contrast. Follow Ctrl+x with number 0 to 255. 120 is default.
 Ctrl+z / 26 / 0x1A - Enable/disable ignore RX pin on startup (ignore emergency reset)
        / 158-187 / 0x9E-0xBB - Set the green backlight brightness. 158 = Off, 187 = 100%.
        / 188-217 / 0xBC-0xD9 - Set the blue backlight brightness. 188 = Off, 217 = 100%.

*/
#define SPECIAL_COMMAND 254 //0xFE: The command to do special HD77480 commands
#define SPECIAL_SETTING '|' //124, 0x7C, the pipe character: The command to do special settings: baud, lines, width, backlight, splash, etc
#define SPECIAL_RED_MIN 128 //Command minimum for red/white backlight brightness
const uint8_t BUFFER_SIZE = 128; //Number of characters we can hold in the incoming buffer
const uint8_t DISPLAY_BUFFER_SIZE = 4*20; //4x20 the max number of characters we will display at one time

#define SYSTEM_MESSAGE_DELAY 500 //Amount of time (ms) we spend displaying splash and system messages

//Delays for a specified period that is pet safe
void wdtSafeDelay(int delayAmount)
{
	uint32_t startTime = getMillis();

	while ((getMillis() - startTime) <= delayAmount)
		wdt_reset();
}

//Display the LCD buffer and return the cursor to where it was before the system message
void displayFrameBuffer(void)
{
	lcd_clrscr();
	lcd_gotoxy(0,0);
	for (uint8_t i = 0; i < (settingLCDlines * settingLCDwidth); i++)
		lcd_putc(currentFrame[i]);

	//Return the cursor to its original position
	lcd_gotoxy(characterCount % settingLCDwidth, characterCount / settingLCDwidth);
}


//Turn on messages like 'Contrast: 5' when user changes setting
void enableDisplaySystemMessages()
{
	settingDisplaySystemMessages = true;
	eeprom_write_byte((uint8_t*)LOCATION_DISPLAY_SYSTEM_MESSAGES, settingDisplaySystemMessages?1:0);

	//Display new setting to the user
	lcd_clrscr();
	lcd_gotoxy(0,0);
	lcd_puts_P("Messages ON");
	wdtSafeDelay(SYSTEM_MESSAGE_DELAY);
  displayFrameBuffer(); //Return the contents of the display*/
  
}

//Turn off system messsages
void disableDisplaySystemMessages()
{
	// FIXME
	settingDisplaySystemMessages = false;
	eeprom_write_byte((uint8_t*)LOCATION_DISPLAY_SYSTEM_MESSAGES, settingDisplaySystemMessages?1:0);
}


//Change the I2C or TWI address
void changeTWIAddress(uint8_t newAddress)
{
	// FIXME
/*  //Record the new address
  EEPROM.update(LOCATION_TWI_ADDRESS, newAddress);

  setupTWI(); //Leverage the regular startup function

  if (settingDisplaySystemMessages == true)
  {
    //Display the new TWI address
    SerLCD.clear();
    SerLCD.setCursor(0, 0); //First position, 1st row

    SerLCD.print("New TWI: 0x");
    SerLCD.print(newAddress, HEX);

    petSafeDelay(SYSTEM_MESSAGE_DELAY);

    displayFrameBuffer(); //Display what was there before
  }*/
}


//Display the current firmware version for a set amount of time
void displayFirmwareVersion()
{
	// FIXME

/*  SerLCD.clear();
  SerLCD.setCursor(0, 0);

  SerLCD.print(F("Firmware v"));
  SerLCD.print(firmwareVersionMajor);
  SerLCD.print(F("."));
  SerLCD.print(firmwareVersionMinor);

  petSafeDelay(SYSTEM_MESSAGE_DELAY);

  displayFrameBuffer(); //Return the contents of the display*/
}


//Flushes all characters from the frame buffer
void clearFrameBuffer()
{
	//Clear the frame buffer
	characterCount = 0;
	memset(currentFrame, ' ', settingLCDwidth * settingLCDlines);
}



void updateDisplay(uint8_t incoming)
{
	static DisplayMode mode = MODE_NORMAL;

	// Credit where due.  Much of this is directly borrowed from Nathan Seidle's OpenLCD project for maximum
	//  compatibility.  The project can be found here:  https://github.com/sparkfun/OpenLCD
	//  And per the beerware license, if you're ever down around Colorado Springs, or we're up around Boulder,
	//  you name the brewery and your beer of choice and it's on us.

	//If the last byte received wasn't special
	if (mode == MODE_NORMAL)
	{
		//Check to see if the incoming byte is special
		if (SPECIAL_SETTING == incoming) //SPECIAL_SETTING is 127
		{
			mode = MODE_SETTING;
		}
		else if (incoming == SPECIAL_COMMAND) //SPECIAL_COMMAND is 254
		{
			mode = MODE_COMMAND;
		}
		else if (incoming == 8) //Backspace
		{
			if (characterCount == 0) 
				characterCount = settingLCDwidth * settingLCDlines; //Special edge case
			characterCount--; //Back up
			currentFrame[characterCount] = ' '; //Erase this spot from the buffer
			displayFrameBuffer(); //Display what we've got
		}
		else //Simply display this character to the screen
		{
			lcd_putc(incoming);
			currentFrame[characterCount++] = incoming; //Record this character to the display buffer
			if (characterCount == settingLCDwidth * settingLCDlines)
				characterCount = 0; //Wrap condition
		}
	}
	else if (mode == MODE_SETTING)
	{
		mode = MODE_NORMAL; //We assume we will be returning to normal

		//Software reset
		if (incoming == 8) //Ctrl+h
		{
			while (1); //Hang out and let the watchdog punish us
		}
		//Set TWI address
		else if (incoming == 25) //Ctrl+y
		{
			mode = MODE_TWI; //Go to new mode
			//We now grab the next character on the next loop and use it to change the TWI address
		}
		//Record custom characters
		else if (incoming >= 27 && incoming <= 34)
		{
			//User can record up to 8 custom chars
			customCharNumber = incoming - 27; //Get the custom char spot to record to
			mode = MODE_RECORD_CUSTOM_CHAR; //Change to this special mode
		}

		//Display custom characters, 8 characters allowed, 35 to 42 inclusive
		else if (incoming >= 35 && incoming <= 42)
		{
			lcd_putc(incoming - 35); //You write location zero to display customer char 0
		}
		//Set Backlight RGB in one command to eliminate flicker
		else if (incoming == 43) //+ character
		{
			// Not relevant to us, no RG backlight
			mode = MODE_SET_RGB; //Go to new mode
		}
		//Display current firmware version
		else if (incoming == 44) //, character
		{
			displayFirmwareVersion();
		}
		//Clear screen and buffer
		else if (incoming == 45) //- character
		{
			lcd_clrscr();
			lcd_gotoxy(0,0);
			clearFrameBuffer(); //Get rid of all characters in our buffer
		}
		//Enable the displaying of system messages
		else if (incoming == 46) //. character
		{
			enableDisplaySystemMessages();
		}
		//Disable the displaying of system messages
		else if (incoming == 47) // / character
		{
			disableDisplaySystemMessages();
		}

		//If we get a second special setting character, then write it to the display
		//This allows us to print a pipe by escaping it as a double
		else if (SPECIAL_SETTING == incoming) //| character
		{
			lcd_putc(SPECIAL_SETTING);
			currentFrame[characterCount++] = incoming; //Record this character to the display buffer
			if (characterCount == settingLCDwidth * settingLCDlines)
				characterCount = 0; //Wrap condition
		}

		//The following commands start at integer value 128
		//Backlight Red or standard white
		else if (incoming >= SPECIAL_RED_MIN && incoming <= (SPECIAL_RED_MIN + 29))
		{
			lcd_backlightOn(incoming - SPECIAL_RED_MIN);
		}
	}
	else if (mode == MODE_TWI)
	{
		//Custom TWI address
		changeTWIAddress(incoming);
		mode = MODE_NORMAL; //Return to normal operation
	}
	else if (mode == MODE_COMMAND) //Deal with lower level commands
	{
		mode = MODE_NORMAL; //In general, return to normal mode
		if (incoming >> 7 == 1) //This is a cursor position command
		{
			incoming &= 0x7F; //Get rid of the leading 1

			uint8_t line = 0;
			uint8_t spot = 0;
			if (incoming >= 0 && incoming <= 19)
			{
				spot = incoming;
				line = 0;
			}
			else if (incoming >= 64 && incoming <= 83)
			{
				spot = incoming - 64;
				line = 1;
			}
			else if (incoming >= 20 && incoming <= 39)
			{
				spot = incoming - 20;
				line = 2;
			}
			else if (incoming >= 84 && incoming <= 103)
			{
				spot = incoming - 84;
				line = 3;
			}
			lcd_gotoxy(spot, line);
		}
		else if (incoming >> 6 == 1) //This is Set CGRAM address command
		{
			//User is trying to create custom character
			incoming &= 0b10111111; //Clear the ACG bit

			//User can record up to 8 custom chars
			customCharNumber = incoming - 27; //Get the custom char spot to record to

			mode = MODE_RECORD_CUSTOM_CHAR; //modeRecordCustomChar = true; //Change to this special mode
		}
		else if (incoming >> 4 == 1) //This is a scroll/shift command
		{
			/*See page 24/25 of the datasheet: https://www.sparkfun.com/datasheets/LCD/HD44780.pdf
				Bit 3: (S/C) 1 = Display shift, 0 = cursor move
				Bit 2: (R/L) 1 = Shift to right, 0 = shift left
			*/

			//Check for display shift or cursor shift
			if (incoming & 1 << 3) //Display shift
			{
				// FIXME
/*				if (incoming & 1 << 2) 
					SerLCD.scrollDisplayRight(); //Go right
				else
					SerLCD.scrollDisplayLeft(); //Go left*/
			}
			else //Cursor move
			{
				//Check for right/left cursor move
				if (incoming & 1 << 2) //Right shift
				{
					characterCount++; //Move cursor right
					if (characterCount == settingLCDwidth * settingLCDlines)
						characterCount = 0; //Wrap condition
				}
				else
				{
					if (characterCount == 0)
						characterCount = settingLCDwidth * settingLCDlines; //Special edge case
					characterCount--; //Move cursor left
				}
				lcd_gotoxy(characterCount % settingLCDwidth, characterCount / settingLCDwidth);  // Move the cursor
			}
		}
		else if (incoming >> 3 == 1) //This is a cursor or display on/off control command
		{
			/*See page 24 of the datasheet: https://www.sparkfun.com/datasheets/LCD/HD44780.pdf
				Bit 3: Always 1 (1<<3)
				Bit 2: 1 = Display on, 0 = display off
				Bit 1: 1 = Cursor displayed (an underline), 0 = cursor not displayed
				Bit 0: 1 = Blinking box displayed, 0 = blinking box not displayed
				You can combine bits 1 and 2 to turn on the underline and then blink a box. */

			lcd_command(incoming);
		}
		else if (incoming >> 4 != 0b00000011) //If not the data length (DL) command then send it to LCD
		{
			//We ignore the command that could set LCD to 8bit mode
			//But otherwise give the user the ability to pass commands directly
			//into the LCD.
			lcd_command(incoming);
		}
	}
	else if (mode == MODE_RECORD_CUSTOM_CHAR)
	{
		//We get into this mode if the user has sent the correct setting or system command

		customCharData[customCharSpot] = incoming; //Record this byte to the array

		customCharSpot++;
		if (customCharSpot > 7)
		{
			//Once we have 8 bytes, stop listening
			customCharSpot = 0; //Wrap variable at max of 7

			// FIXME
			//SerLCD.createChar(customCharNumber, customCharData); //Record the array to CGRAM

			//Record this custom char to EEPROM
// FIXME
//			for (uint8_t charSpot = 0 ; charSpot < 8 ; charSpot++)
//				EEPROM.update(LOCATION_CUSTOM_CHARACTERS + (customCharNumber * 8) + charSpot, customCharData[charSpot]); //addr, val

			//For some reason you need to re-init the LCD after a custom char is created
			// FIXME
			// SerLCD.begin(settingLCDwidth, settingLCDlines);
			mode = MODE_NORMAL; //Exit this mode
		}
	}
	else if (mode == MODE_CONTRAST)
	{
		//We get into this mode if the user has sent the ctrl+x (24) command to change contast
		// Do nothing
		//changeContrast(incoming);
		mode = MODE_NORMAL; //Exit this mode
	}
	else if (mode == MODE_SET_RGB)
	{
		//We get into this mode if the user has sent the + (43) command to set the backlight rgb values
		if (++rgbSpot > 2)
		{
			//Once we have 3 bytes, stop listening
			rgbSpot = 0;
			mode = MODE_NORMAL; //Exit this mode
		} 
	} 
}





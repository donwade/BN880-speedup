/*******************************************************************************************************
  Programs for Arduino - Copyright of the author Stuart Robinson - 28/08/18

  This program is supplied as is, it is up to the user of the program to decide if the program is
  suitable for the intended purpose and free from errors.
*******************************************************************************************************/

/*******************************************************************************************************
  Program Operation - This is a simple program to configure a Ublox GPS. 

  At startup GPS characters are read and sent to the IDE serial monitor for 2 seconds. 

  The GPS configuration is then cleared and a further 2 seconds of characters are sent. 

  Then most GPS sentences are turned off, leaving only GPRMC and GPGGA that are those needed for location
  and speed data etc. The refresh rate is then changed to 10hz.

  The GPS characters are again copied to serial monitor using the new configuration.
  
  GPS baud rate set at 9600 baud, Serial monitor set at 115200 baud. If the data displayed on the serial
  terminal appears to be random text with odd symbols its very likely you have the GPS serial baud rate
  set incorrectly for the GPS.

  Note that not all pins on all Arduinos will work with software serial, see here;

  https://www.arduino.cc/en/Reference/softwareSerial

  Serial monitor baud rate is set at 115200.

*******************************************************************************************************/

/*
A Class is a grouping of messages which are related to each other. The following table gives the short names,
description and Class ID Definitions.
NameClassDescription
NAV
RXM
INF
ACK
CFG
MON
AID
TIM
0x01
0x02
0x04
0x05
0x06
0x0A
0x0B
0x0D
Navigation Results: Position, Speed, Time, Acc, Heading, DOP, SVs used
Receiver Manager Messages: Satellite Status, RTC Status
Information Messages: Printf-Style Messages, with IDs such as Error, Warning, Notice
Ack/Nack Messages: as replies to CFG Input Messages
Configuration Input Messages: Set Dynamic Model, Set DOP Mask, Set Baud Rate, etc.
Monitoring Messages: Comunication Status, CPU Load, Stack Usage, Task Status
AssistNow Aiding Messages: Ephemeris, Almanac, other A-GPS data input
Timing Messages: Timepulse Output, Timemark Results

*/

#include <M5Unified.h>
#include <FastLED.h>                                                                                                                                                                    

#define LEDS_PIN 25
#define LEDS_NUM 10
CRGB ledsBuff[LEDS_NUM];


// 4 bytes of serial silence
 const uint32_t SILENCE_MS = 1000 * (float (8 * 4) /float(9600));

//                                      SYNC  SYNC  CLASS ID    LNLO  LNHI  PAYLOAD ------> CHK1 CHK2                                                                     
const PROGMEM  uint8_t ClearConfig[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x01, 0x19, 0x98};
const PROGMEM  uint8_t GPGLLOff[] =    {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B};
const PROGMEM  uint8_t GPGSVOff[] =    {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39};
const PROGMEM  uint8_t GPVTGOff[] =    {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47};
const PROGMEM  uint8_t GPGSAOff[] =    {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32};
const PROGMEM  uint8_t GPGGAOff[] =    {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24};
const PROGMEM  uint8_t GPRMCOff[] =    {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40};


// 31.17.2 Navigation/Measurement Rate Settings
//                                      SYNC  SYNC  CLASS ID    LNLO  LNHI  PAYLOAD ------> CHK1 CHK2                                                                     
const PROGMEM  uint8_t Navrate10hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};
//                                                                          0x64, 0x00 = 100 (times per second)    
//                                                                                      0x01, 0x00 = 1 measurement cycle                                           
//                                                                                                  0x01, 0x00 = 1 use GPS time 
const PROGMEM uint8_t Special[] =
{
	0xB5, 0x62, 0x06, 0x3E, 0x3C, 0x00, 0x00, 0x00, 0x20, 0x07, 0x00, 0x08, 0x10, 0x00, 0x01,
	0x00, 0x01, 0x01, 0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01, 0x02, 0x04, 0x08, 0x00,
	0x00, 0x00, 0x01, 0x01, 0x03, 0x08, 0x10, 0x00, 0x00, 0x00, 0x01, 0x01, 0x04, 0x00, 0x08,
	0x00, 0x00, 0x00, 0x01, 0x01, 0x05, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x01, 0x06, 0x08,
	0x0E, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2D, 0x79
};

const PROGMEM uint8_t test[] =
{
	0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x01, 0xFB, 0x10
};

const PROGMEM uint8_t many[] =
{
	0xB5, 0x62, 0x06, 0x3E, 0x34, 0x00, 0x00, 0x00, 0x2A, 0x06, 0x00, 0x08, 0x10, 0x00, 0x01,
	0x00, 0x01, 0x01, 0x01, 0x03, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01, 0x02, 0x08, 0x0C, 0x00,
	0x01, 0x00, 0x01, 0x01, 0x03, 0x02, 0x05, 0x00, 0x00, 0x00, 0x01, 0x01, 0x05, 0x03, 0x04,
	0x00, 0x00, 0x00, 0x05, 0x01, 0x06, 0x08, 0x0C, 0x00, 0x01, 0x00, 0x01, 0x01, 0x21, 0xDE
};


typedef enum { SILENCE, COLLECT, WAIT4_FIRST_CHAR} STATE;
STATE state = SILENCE;

uint32_t exitTime;

//------------------------------------------------------

uint16_t csum(const uint8_t *from, unsigned int N)
{
	uint8_t CK_A = 0, CK_B = 0;
	int I;

	Serial.print("CSUM = ");
	
	for(I=0;I<N;I++)
	{
		Serial.printf("%02X ", from[I]);
		CK_A = CK_A + from[I];
		CK_B = CK_B + CK_A;
	}
	Serial.println();
	
	Serial.printf("CKA=%02X CKB=%02X \n", CK_A, CK_B);
	
	return CK_A + CK_B << 8;
}

//------------------------------------------------------

void wait4Silence()
{
	// wait for silence
	Serial.println('+');
	
	uint32_t shortTime = millis();
	do 
	{
		// wait for silence
		while (Serial2.available())
		{
			Serial2.read(); //pitch it.
			shortTime = millis();
		}
	} while (millis() < (shortTime + SILENCE_MS));
	
	Serial.println('-');
	state = WAIT4_FIRST_CHAR;
}	
//------------------------------------------------------

void wait4Char()
{
	do 
	{
		if (Serial2.available())
		{
			state = COLLECT;
			return;
		}
	} while (millis() < exitTime );
}

void getData()
{
	uint32_t shortTime = millis();
	static uint32_t period;
	do 
	{
		while (Serial2.available())
		{
		
			uint8_t ok;
			ok = Serial2.read();
			Serial.printf("%02X ", ok);
			shortTime = millis();
		}

	// keep reading until data drop out
	} while (millis() < (shortTime + SILENCE_MS));
	
	Serial.printf("%d time \n", millis() - period);
	period = millis();
	
	state = WAIT4_FIRST_CHAR;
}

//------------------------------------------------------

void monitor(unsigned waitMs)
{
	exitTime = millis() + waitMs;

	do 
	{
		switch (state)
		{
			case SILENCE:
				wait4Silence();
			break;

			case COLLECT:
				getData();
			break;

			case WAIT4_FIRST_CHAR:
				wait4Char();
			break;
		}
	} while (millis() < exitTime);
	
	Serial.println();
}

//------------------------------------------------------
void colourBar(uint8_t R,uint8_t G, uint8_t B) 
{
	for (int i = 0; i < LEDS_NUM; i++) {
		ledsBuff[i].setRGB(R, G, B);
	}
	FastLED.show();
}	

void loop()
{


  Serial.println();
  Serial.println();
  Serial.flush();

  Serial.print("ClearConfig ");
  GPS_SendConfig(ClearConfig, 21);

  Serial.println();
  Serial.println("----------------------------------------------");
  Serial.flush();

  //now lets see 2 seconds of GPS characters at cleared GPS configuration

  Serial.println("gps reset");
  Serial.println("show 4 seconds of 1 second reports:");

  monitor(4000);  
  Serial.println();
  Serial.flush();

  
  //now turn off most of the GPS sentences and set the refresh rate to 10hz

  Serial.print("GPGLLOff ");
  GPS_SendConfig(GPGLLOff, 16);

  Serial.print("GPGSVOff ");
  GPS_SendConfig(GPGSVOff, 16);

  Serial.print("GPVTGOff ");
  GPS_SendConfig(GPVTGOff, 16);

  Serial.print("GPGSAOff ");
  GPS_SendConfig(GPGSAOff, 16);

  Serial.print("Navrate10hz ");
  GPS_SendConfig(Navrate10hz, 14);

  GPS_SendConfig(Special, sizeof(Special));
  GPS_SendConfig(test, sizeof(test));

  while (true)
  {
  	bool flip;
	flip ? colourBar(20,0,0) : colourBar(0, 20, 0);
	flip = !flip;
  	GPS_SendConfig(many, sizeof(many));
	delay(500);
	
  }

  Serial.println();
  Serial.println();
  Serial.flush();


  monitor(30000);

  Serial.printf("snoozed ****\n"); delay(-1); ///////////////////////
}


//------------------------------------------------------


void GPS_SendConfig(const uint8_t *Progmem_ptr, uint8_t arraysize)
{
	uint8_t byteread, index;
	const uint8_t *restore = Progmem_ptr;

	Serial.print(F("GPSSend  "));
	Serial.printf("\nSync1=%02X Sync2=%02X Class=%02X ID=%02X LEN=%d\n",
	  Progmem_ptr[0], // sync1
	  Progmem_ptr[1], // sync2
	  Progmem_ptr[2], // class
	  Progmem_ptr[3], // id1
	  Progmem_ptr[4] | Progmem_ptr[5] << 8
	  );


	for (index = 0; index < arraysize; index++)
	{
		byteread = *Progmem_ptr++;
		if (byteread < 0x10)
		{
		  Serial.print(F("0"));
		}
		Serial.print(byteread, HEX);
		Serial.print(F(" "));
	}
	Serial.println();

	Progmem_ptr = restore;

	for (index = 0; index < arraysize; index++)
	{
		byteread = *Progmem_ptr++;
		Serial2.write(byteread);
		delay(1);
	}

	delay(100);
	csum( &restore[2],arraysize - 4); // payload size only
	Serial.println();
	
	
}


void setup()
{

  M5.begin();
  Serial.begin(115200);
  
  FastLED.addLeds<SK6812, LEDS_PIN>(ledsBuff, LEDS_NUM);

  Serial.println();
  Serial.println();
  Serial.printf("90_UBlox_GPS_Configuration Starting %d", SILENCE_MS);
  delay(10000);
  Serial.println();
  Serial.println();
  Serial2.begin(9600);
}

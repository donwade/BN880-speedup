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

void SendPacket(char *explain, const uint8_t *pPacket, uint8_t packetSize, bool bDumpMsg = false);
uint8_t getMessage(uint8_t ID1, uint8_t ID2, uint8_t *packet, uint16_t packetSize, bool bDumpPacket = false);

#define LEDS_PIN 25
#define LEDS_NUM 10
CRGB ledsBuff[LEDS_NUM];

// given a message packet, where is the payload?
#define OFFSET2_PAYLOAD 6

// 4 bytes of serial silence
 const uint32_t SILENCE_MS = 1000 * (float (8 * 3) /float(9600));

//                                      SYNC  SYNC  CLASS ID    LNLO  LNHI  PAYLOAD ------> CHK1 CHK2                                                                     
const PROGMEM  uint8_t ClearConfig[] = {0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x01, 0x19, 0x98};
const PROGMEM  uint8_t GPGLLOff[] =    {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B};
const PROGMEM  uint8_t GPGSVOff[] =    {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39};
const PROGMEM  uint8_t GPVTGOff[] =    {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47};
const PROGMEM  uint8_t GPGSAOff[] =    {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32};
const PROGMEM  uint8_t GPGGAOff[] =    {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24};
const PROGMEM  uint8_t GPRMCOff[] =    {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x40};

/*
const PROGMEM  uint8_t GPGLLOff[] =    {0xB5, 0x62, 
										0x06, 0x01, 
										0x08, 0x00,
										0xF0, 
										0x01, 
										0x00, 0x00, 0x00, 0x00, 0x00, 0x01, // time is 6 bytes.
										0x01, 0x2B};
*/

// 31.17.2 Navigation/Measurement Rate Settings
//                                      SYNC  SYNC  CLASS ID      LNLO  LNHI  PAYLOAD ------> CHK1 CHK2
const PROGMEM  uint8_t Navrate10hz[]   = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};
//                                                                            0x64, 0x00 = 100 (times per second)    
//                                                                                      0x01, 0x00 = 1 measurement cycle                                           
//                                                                                                  0x01, 0x00 = 1 use GPS time 

const PROGMEM  uint8_t NavrateSlowhz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12};


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

//----------------------------------------------
// 19 NMEA Messages Overview
// 31.9 CFG-MSG (0x06 0x01)

typedef struct 
{
	uint8_t aclass;
	uint8_t id;
	const char *txt;
} CFG_MSG_ITEM;


const CFG_MSG_ITEM CFG_MSG_LIST[] =
{
	{ 0xF1 ,0x00	,"Get/Set Lat/Long Position Data" },
	{ 0xF1 ,0x03	,"Get/Set Satellite Status" },
	{ 0xF1 ,0x04	,"Get/Set Time of Day and Clock Information" },
	{ 0xF1 ,0x05	,"Get/Set Lat/Long Position Data" },
	{ 0xF1 ,0x06	,"Get/Set Lat/Long Position Data" },
	{ 0xF1 ,0x40	,"Set NMEA message output rate" },
	{ 0xF1 ,0x41	,"Set Protocols and Baudrate"},
	{ 0xF0 ,0x00	,"Global positioning system fix data" },
	{ 0xF0 ,0x01	,"Latitude and longitude, with time of position fix and status" },
	{ 0xF0 ,0x02 	,"GNSS DOP and Active Satellites" },
	{ 0xF0 ,0x03 	,"GNSS Satellites in View" },
	{ 0xF0 ,0x04 	,"Recommended Minimum data" },
	{ 0xF0 ,0x05	,"Course over ground and Ground speed" },
	{ 0xF0 ,0x06 	,"GNSS Range Residuals" },
	{ 0xF0 ,0x07 	,"GNSS Pseudo Range Error Statistics" },
	{ 0xF0 ,0x08	,"Time and Date" },
	{ 0xF0 ,0x09	,"GNSS Satellite Fault Detection" },
	{ 0xF0 ,0x0A	,"Datum Reference" },
	{ 0xF0 ,0x40	,"Poll message" },
	{ 0xF0 ,0x0E	,"True Heading and Status" },
	{ 0xF0 ,0x41	,"Text Transmission" }
};

const char *findConfigMsg (uint8_t aclass, uint8_t id)
{
	int i;
	int k = sizeof(CFG_MSG_LIST)/sizeof(CFG_MSG_ITEM);

	for (i =0; i < k; i++)
	{
		if ( aclass != CFG_MSG_LIST[i].aclass) continue;
		if ( id != CFG_MSG_LIST[i].id) continue;
		break;
	}

	if (i == k)
	{
		Serial.printf("%s missed class=%02X id=%02X\n", __FUNCTION__, aclass, id);
		return "UNKNOWN !!!!";
	}
	
	return CFG_MSG_LIST[i].txt;
}
//-------------------------------------------------
// see 31.9 CFG-MSG (0x06 0x01)

typedef struct 
{
	uint8_t foo[11];	
} DISABLE_ITEM;

const DISABLE_ITEM DISABLE_LIST[] =
{
//                                          |<--area-->     TICKS     
	{ 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00,    0xF0, 0x0A,    0x00,     0x04, 0x23 },
	{ 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00,    0xF0, 0x09,    0x00,     0x03, 0x21 },
	{ 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00,    0xF0, 0x00,    0x00,     0xFA, 0x0F },
	{ 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00,    0xF0, 0x01,    0x00,     0xFB, 0x11 },
	{ 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00,    0xF0, 0x0D,    0x00,     0x07, 0x29 },
	{ 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00,    0xF0, 0x06,    0x00,     0x00, 0x1B },
	{ 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00,    0xF0, 0x02,    0x00,     0xFC, 0x13 },
	{ 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00,    0xF0, 0x07,    0x00,     0x01, 0x1D },
	{ 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00,    0xF0, 0x03,    0x00,     0xFD, 0x15 },
	{ 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00,    0xF0, 0x0F,    0x00,     0x09, 0x2D },
	{ 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00,    0xF0, 0x04,    0x00,     0xFE, 0x17 },
	{ 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00,    0xF0, 0x05,    0x00,     0xFF, 0x19 },
	{ 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00,    0xF0, 0x08,    0x00,     0x02, 0x1F },
	{ 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00,    0xF1, 0x00,    0x00,     0xFB, 0x12 },
	{ 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00,    0xF1, 0x01,    0x00,     0xFC, 0x14 },
	{ 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00,    0xF1, 0x03,    0x00,     0xFE, 0x18 },
	{ 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00,    0xF1, 0x04,    0x00,     0xFF, 0x1A },
	{ 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00,    0xF1, 0x05,    0x00,     0x00, 0x1C },
	{ 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00,    0xF1, 0x06,    0x00,     0x01, 0x1E },


// report rate is "TICKS" times 'Navrate10hz[]' setting
// 'my changes area'
//											|<--area--> 	TICKS	 
	{ 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00,    0xF0, 0x01,    0x01,     0xFB, 0x11 },
	{ 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00,    0xF1, 0x06,    0x01,     0x01, 0x1E },
	{ 0xB5, 0x62, 0x06, 0x01, 0x03, 0x00,    0xF0, 0x03,    0x01,     0xFD, 0x15 },

};
//----------------------------------------------

typedef enum { SILENCE, COLLECT, WAIT4_FIRST_CHAR} STATE;
STATE state    = SILENCE;

uint32_t exitTime;

void GPS_SendConfig(const uint8_t *Progmem_ptr, uint8_t arraysize);

//------------------------------------------------------

uint16_t calc_csum(const uint8_t *from, unsigned int N)
{
	uint8_t CK_A = 0, CK_B = 0;
	int I;

	for(I=0;I<N;I++)
	{
		//Serial.printf(!(I%8) ? "%02X " : "%02X-", from[I]);
		CK_A = CK_A + from[I];
		CK_B = CK_B + CK_A;
	}
	
	//Serial.printf("calc_csum: CKA=%02X CKB=%02X \n", CK_A, CK_B);
	
	return CK_B | (CK_A << 8);
}

//------------------------------------------------------
void dumpPayload(uint8_t *from, uint32_t len)
{
	Serial.println("dump payload");
	for (int i = 0; i < len; i++)
	{
		printf("[%02d] 0x%02x \n", i, from[i]);		
	}
}
//------------------------------------------------------
void dumpPacket(const uint8_t *from, uint32_t len)
{
	Serial.println("dump packet");
	for (int i = 0; i < len; i++)
	{
		if(i == OFFSET2_PAYLOAD) Serial.println(); 	//payload
		if(i == 4 ) Serial.println(); 				// len
		if(i == len - 2 ) Serial.println(); 		// crc
		if(i >= OFFSET2_PAYLOAD && i < len -2)
			printf("      [%02d] 0x%02x \n", i - OFFSET2_PAYLOAD, from[i]);
		else
			printf("  [%02d] 0x%02x \n", i, from[i]);
			
	}
	Serial.println();
}
//------------------------------------------------------

// return size of PAYLOAD if valid

uint8_t getMessage(uint8_t ID1, uint8_t ID2, uint8_t *packet, uint16_t packetSize, bool bDumpPacket)
{

	uint32_t exit = millis() + 200; // wait 10ms for response
	
	uint8_t preamble[4];
	preamble[0] = 0xB5;
	preamble[1] = 0x62;
	preamble[2] = ID1;
	preamble[3] = ID2;

	memset(packet, 0x55, packetSize);
	memcpy(packet, preamble, sizeof(preamble));
	
	int i;

	// look for header
	for ( i = 0; i < sizeof(preamble); i++)
	{
	    rescan:
		while (!Serial2.available() && ( millis() < exit));
		if (millis() >= exit) break;

		if ( Serial2.read() != preamble[i]) 
		{
			i = 0;  // start rescan.
			//Serial.printf("rescan\n");
			goto rescan;
		}
		// missed. what was the char?
		//Serial.printf("rx = 0x%X\n", preamble[i]);
	}

	if ( i != sizeof(preamble))
	{
		Serial.printf("%s fail. exit early\n", __FUNCTION__);
		return 0;
	}


	// get length
	while (!Serial2.available() && ( millis() < exit));
	if (millis() >= exit) return 0;
	packet[4] = Serial2.read();
	
	//Serial.printf("lnL = 0x%X\n", packet[4]);

	while (!Serial2.available() && ( millis() < exit));
	if (millis() >= exit) return 0;
	packet[5] = Serial2.read();
	
	//Serial.printf("lnH = 0x%X\n", packet[5]);

	uint16_t payload_len = packet[5] << 8 | packet[4];
	Serial.printf("the packet size is ... %d\n", payload_len);


	// copy over packet + crc (2bytes)
	for ( i = 0; i < payload_len + 2; i++)
	{
		while (!Serial2.available() && ( millis() < exit));
		if (millis() >= exit) break;

		packet[ OFFSET2_PAYLOAD + i ] = Serial2.read();
	}

	if ( i != payload_len + 2)  
	{
		Serial.printf("packet + crc copy failed\n");
		return 0;
	}

	if (bDumpPacket) dumpPacket(packet, payload_len + 8);

	//Serial.printf("local CRCA=%02X CRCB=%02X\n", packet[ i+4], packet[i+5]);

	//verify
	calc_csum( &packet[2], payload_len + 4); // 2len+2ID1ID2=4
	
	return payload_len;
}
//------------------------------------------------------

void burnRxBuffers(void)
{
	while (Serial2.available()) Serial2.read();
}
//------------------------------------------------------
void makeMessage(char *explain, uint8_t ID1, uint8_t ID2, uint8_t *payload, uint16_t payloadSize)
{
	uint8_t *msg = (uint8_t *) alloca ( payloadSize + 8);
	msg[0] = 0xB5;
	msg[1] = 0x62;
	msg[2] = ID1;
	msg[3] = ID2;
	msg[4] = payloadSize & 0xFF;
	msg[5] = payloadSize >> 8;

	if (payloadSize)
	{
		assert(payload);
		memcpy(&msg[6], payload, payloadSize);
	}

	// +4  sum across ID1,ID2, lenLo, lenHi, payload
	uint16_t crc = calc_csum (&msg[2], payloadSize + 4);
	
	msg[OFFSET2_PAYLOAD + payloadSize + 0] = crc >> 8;	
	msg[OFFSET2_PAYLOAD + payloadSize + 1] = crc & 0xFF;	

	// packet size is header+OPCMD+len +payload +crc = 8 + payload
	SendPacket(explain, msg, payloadSize + 8);
}
//------------------------------------------------------
void getConfig(void) // 31.10 CFG-NAV5 (0x06 0x24)
{
	makeMessage("31.10 CFG-NAV5", 0x06, 0x24, NULL, 0);

	uint8_t result[50];
	getMessage(0x06, 0x24, result, sizeof(result));
}
//------------------------------------------------------
void getNEMA(void) // 31.12 CFG-NMEA (0x06 0x17)
{
	makeMessage("31.12 CFG-NMEA",0x06, 0x17, NULL, 0);

	uint8_t result[50];
	getMessage(0x06, 0x17, result, sizeof(result));
}

//------------------------------------------------------
void getSetUart(uint32_t baudrate = 0) // 31.16.2 Polls the configuration for one I/O Port
{
	uint8_t aPacket[50];

again:
	uint8_t portID[] = {1}; // 1=UART 3=USB 4=SPI
	makeMessage("31.16.2 CFG-UART (get baud)", 0x06, 0x0, portID, sizeof(portID));

	// ask h/w for current baudrate.
	uint8_t payload_len = getMessage(0x06, 0x0, aPacket, sizeof(aPacket));
	
	if (!payload_len)
	{
		// 2) we could be talking at 9600 but h/w alread up at 115200
		//	  therefore we would get not repsonse it this scenario.
		//	  Change arduino to 115200 and ask for baud rate again
		
		Serial.printf("%s: timeout occurred\n", __FUNCTION__);
		Serial.printf("h/w could already be at 115200, retrying cmd at 115200 baud\n");
		Serial2.begin(115200);
		goto again;
	}

	// got a response. 
	// we could be 9600 or 115200 on both ends.
	// Don't care. ALWAYS issue a 115200 config request.

	// baud rate is 8 bytes into the payload
	uint32_t *baud = (uint32_t *) &aPacket[OFFSET2_PAYLOAD + 8];
	
	Serial.printf("gps baud rate (h/w) is %d\n", *baud);

	if (!baudrate) return;  // 0 = query only

	// config for 115200
	// good thing its a little endian message order

	*baud = baudrate;
	
	//Serial.printf("dump after change\n");
	//dumpPacket(aPacket, payload_len + 8);

	SendPacket("31.16.2 CFG-UART (set to 115200)", aPacket, payload_len + 2 + 2 + 2 + 2);
	getSetUart(0); // did it stick?
	
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
#if 0
			//Serial.printf("%02X%c ", ok, ok);
#else
			Serial.printf("%c", ok);
#endif
			shortTime = millis();
		}

	// keep reading until data drop out
	} while (millis() < (shortTime + SILENCE_MS));
	
	Serial.printf("%d time \n", millis() - period);
	period = millis();
	
	state = WAIT4_FIRST_CHAR;
}

//------------------------------------------------------

void monitor(unsigned long waitMs)
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
// make entire LED bar one colour
void colourBar(uint8_t R,uint8_t G, uint8_t B) 
{
	for (int i = 0; i < LEDS_NUM; i++) {
		ledsBuff[i].setRGB(R, G, B);
	}
	FastLED.show();
}	

// make range of LEDs one color
void colourNleds(uint8_t who, uint8_t width, uint8_t R,uint8_t G, uint8_t B) 
{
	assert(who < LEDS_NUM);
	assert(who + width < LEDS_NUM);
	
	for (int i = who; i < who + width; i++) {
		ledsBuff[i].setRGB(R, G, B);
	}
	FastLED.show();
}	



void loop()
{


  Serial.println();
  Serial.println();
  Serial.flush();

  colourBar(5, 0, 0);
  
  Serial.println();
  Serial.println("----------------------------------------------");
  Serial.flush();

  //now lets see 2 seconds of GPS characters at cleared GPS configuration

  Serial.println("show X seconds of 1 second reports:");

  SendPacket("ClearConfig", ClearConfig, sizeof(ClearConfig));

  for (int i = 0; i < sizeof(DISABLE_LIST)/sizeof(DISABLE_ITEM); i++)
  {
  	char who[30];
	const char *what;
	sprintf(who, "DISABLE #%d", i);
  	SendPacket(who,DISABLE_LIST[i].foo, sizeof(DISABLE_ITEM), false);
	what = findConfigMsg(DISABLE_LIST[i].foo[6], DISABLE_LIST[i].foo[7]);
	
	Serial.printf(">>> feature %s ... send at rate = %d \n", what, DISABLE_LIST[i].foo[8]);
  }
	
  getConfig(); // 31.10 CFG-NAV5 (0x06 0x24)
  getNEMA();   // 31.12 CFG-NMEA (0x06 0x17)
  //getSetUart();   // 31.16.2 Polls the configuration for one I/O Port



  monitor(5000);  
  Serial.println();
  Serial.flush();

  
  //now turn off most of the GPS sentences and set the refresh rate to 10hz

#if 0
  //Serial.print("ClearConfig ");
  //SendPacket(ClearConfig, 21);


  Serial.println("GPGLLOff ");
  SendPacket(GPGLLOff, 16);

  Serial.println("GPGSVOff ");
  SendPacket(GPGSVOff, 16);

  Serial.println("GPVTGOff ");
  SendPacket(GPVTGOff, 16);

  Serial.println("GPGSAOff ");
  SendPacket(GPGSAOff, 16);

  Serial.println("Navrate10hz ");
  SendPacket(Navrate10hz, 14);

  Serial.println("Special ");
  SendPacket(Special, sizeof(Special));

  Serial.println("test");
  SendPacket(test, sizeof(test));
#endif

#if 0
  Serial.println("reset chip");
  uint8_t payload[] = { 0xFF, 0xFF, 0x02, 0x00, 00};
  makeMessage(0x06, 0x04, &payload[0], sizeof(payload));
#endif


#if 0
  while (true)
  {
  	bool flip;
	flip ? colourBar(10,0,0) : colourBar(0, 0, 10);
	flip = !flip;
  	SendPacket(many, sizeof(many));
	delay(500);
	
  }
#endif

  Serial.println();
  Serial.println();
  Serial.flush();


  getConfig(); // 31.10 CFG-NAV5 (0x06 0x24)
  getNEMA();   // 31.12 CFG-NMEA (0x06 0x17)

  Serial.print("Navrate10hz ");
  SendPacket("Navrate to 10hz", Navrate10hz, sizeof(Navrate10hz), true);

  monitor(300000);

  #if 0
  Serial.printf("RELAY ENGAGED *\n");
  colourBar(0, 0, 0); // dark


    while (true)
	{
		uint8_t c;
		bool one;
		bool two;
		
		if (Serial1.available())
		{
			c = Serial1.read();
			Serial2.write(c);
			one ? colourNleds(0, 2, 2, 2, 0) : colourNleds(0, 2, 0, 2, 2);
			one = !one;
		}

		if (Serial2.available())
		{
			c= Serial2.read();
			Serial1.write(c);
			two ? colourNleds(3, 2, 2, 0, 2) : colourNleds(3, 2, 0, 2, 0);
			two = !two;
		}
	}
	#endif
 }

//------------------------------------------------------
// user can provide a packet message that has a bad crc value.
// This can occur when a RMW operation on the packet has occured
// and the user shouldn't have to know about crc stuff.
//
// The crc will be corrected as needed.

void SendPacket(char *explain, const uint8_t *pPacket, uint8_t packetSize, bool bDumpMsg)
{
	uint8_t byteread, index;
	const uint8_t *restore = pPacket;

	Serial.printf("----------------------\n%s\nSync1=%02X Sync2=%02X Class=%02X ID=%02X LEN=%d\n",
	  explain,
	  pPacket[0], // sync1
	  pPacket[1], // sync2
	  pPacket[2], // class
	  pPacket[3], // id1
	  pPacket[4] | pPacket[5] << 8
	  );

	if (bDumpMsg) dumpPacket(pPacket,packetSize);
#if 0
	for (index = 0; index < packetSize; index++)
	{
		byteread = *pPacket++;
		if (byteread < 0x10)
		{
		  Serial.print(F("0"));
		}
		Serial.print(byteread, HEX);
		Serial.print(F(" "));
	}
#endif

	Serial.println();

	burnRxBuffers();

retry:
	pPacket = restore;

	for (index = 0; index < packetSize; index++)
	{
		byteread = *pPacket++;
		Serial2.write(byteread);
		delay(1);
	}

	delay(10);
	pPacket = restore;

	// csum sits as the last 2 bytes of the supplied packet
	
	if (bDumpMsg) Serial.printf("csum passed in %02X %02X\n", pPacket[packetSize -2 ], pPacket[packetSize -1 ]);
	
	uint16_t usr_sum = (pPacket[packetSize -2 ] << 8) | (pPacket[packetSize -1]);
	
	// recalc the checksum. this could be a RMW type buffer.
	
	uint16_t c_sum = calc_csum( &restore[2],packetSize - 4); // payload size only
	
	if (bDumpMsg) Serial.printf("calc recalc is %02X %02X\n", c_sum >> 8, c_sum & 0xFF);

	if (usr_sum != c_sum)
	{
		Serial.printf ("%s correcting csum (0x%04X vs 0x%04X) ------\n", __FUNCTION__, usr_sum, c_sum);
		
		uint8_t clone[packetSize];
		memcpy(clone, pPacket, packetSize);
		
		clone[packetSize -2 ] = c_sum >> 8; 
		clone[packetSize -1 ] = c_sum & 0xFF;
		SendPacket("adjusted CSUM", clone, packetSize, true);
	}
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


  /*
     the gps chip could be still on 9600.
     tell it to move to 115200
     if it's already sitting no 11500, this 9600
     will be garbage and thus rejected.
  */
  
  Serial2.begin(9600);
  getSetUart(115200); // 31.16.2 Polls and set the configuration for one I/O Port

  // fine. The gps chip was already at 115200 or moved to it.
  Serial2.begin(115200);
}

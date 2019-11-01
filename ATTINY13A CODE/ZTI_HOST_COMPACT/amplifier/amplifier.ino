#include "Arduino.h"

//---ZUPPA TWI INTERFACE(ZTI)---
// RATED SPEED ATMEGA328P 100kbps
// OMIT CHECKSUM FOR SMALLER CODE SIZE

#define SDA_PIN 0x10
#define SCL_PIN 0x20

#define SDA_PIN_NEG 0xEF
#define SCL_PIN_NEG 0xDF
#define SDA_OUTPUT_MASK 0b00010000
#define SDA_INPUT_MASK  0b11101111
//------------------------FOR QUECTEL START FROM HERE-----------------------
// ZTI PROTOCOL SPECS
#define HEADER_1 0x7A
#define HEADER_2 0x62

enum FrameId
{
	WRITE_DATA_FRAME=0,
	READ_DATA_FRAME,
	ENTER_SLEEP_MODE,
	RESET_DEVICE_FRAME
};

enum FrameField
{
	H1=0,
	H2,
	DEVICE_ADDRESS,
	MASTER_COMMAND,
	PACKET_LENGTH_1,
	PACKET_LENGTH_2,
	PACKET_LENGTH_3,
	PACKET_LENGTH_4,
	PAYLOAD,
};

uint8_t chk1,chk2;

//----------CONVERSION------------
union LongToWordConv
{
	uint16_t dataWord[2];
	uint32_t dataLong;
	}ltoWConv;

//--------CROSS-PLATFORM_PORTABLE_CODES---------
void assignDataPinDirection(int direction,int pullUp)
{
	(pullUp==1)?PORTC|=SDA_PIN:PORTC&=SDA_PIN_NEG;
	(direction==1)?DDRC|=SDA_PIN:DDRC&=SDA_PIN_NEG;
}

void assignClockPinDirection(int direction,int pullUp)
{
	(pullUp==1)?PORTC|=SCL_PIN:PORTC&=SCL_PIN_NEG;
	(direction==1)?DDRC|=SCL_PIN:DDRC&=SCL_PIN_NEG;
}

int setDataPinState(int state)
{
	(state==1)?PORTC|=SDA_PIN:PORTC&=SDA_PIN_NEG;
	return 1;
}

void setClockPinState(int state)
{
	(state==1)?PORTC|=SCL_PIN:PORTC&=SCL_PIN_NEG;
	return 1;
}

int getDataPinState()
{
	return ((PINC & SDA_PIN)?1:0);
}

void minorDelay()
{
	delay(1);
}

void internalDelayMs(uint16_t dly)
{
	delay(dly);
}



//----------------------------------------------
void initializeZTIInterface()
{
	 assignDataPinDirection(1,1);
	 assignClockPinDirection(1,1);
}

void writeAByte(uint8_t data)
{
	volatile uint8_t i;
	chk1+=data;
	chk2+=chk1;
	assignDataPinDirection(1,1);
	assignClockPinDirection(1,1);
	setDataPinState(0);
	minorDelay();
    setDataPinState(1);
	minorDelay();
	setDataPinState(0);
	minorDelay();
	setDataPinState(1);
	minorDelay();
	setDataPinState(0);
	minorDelay();
	setClockPinState(0);
	minorDelay();
	//Serial.println("SEND");
    for(i=0;i<8;i++)
	{
		setDataPinState((data & (1<<i))?1:0);
		minorDelay();
		setClockPinState(1);
		minorDelay();
		setClockPinState(0);
		minorDelay();
	}
	setClockPinState(1);
	setDataPinState(1);
}

void writeFrameShort(uint8_t deviceAddr,uint8_t masterCommand,uint8_t *payload,uint8_t payloadSize)
{
	volatile uint8_t i;
	writeAByte(HEADER_1);
	writeAByte(HEADER_2);
	chk1=chk2=0;
	/*writeAByte(deviceAddr);
	writeAByte(masterCommand);
	writeAByte(payloadSize);
	writeAByte(0x00);
	writeAByte(0x00);
	writeAByte(0x00);*/ // DISABLE FRAME INFOMATION PROTOCOL
	/*payload[0]=1;
	payload[1]=2;
	payload[2]=3;
	payload[3]=4;*/
	for(i=0;i<4;i++)
	writeAByte(payload[i]);
	//writeAByte(chk1);
	//writeAByte(chk2);
}

uint32_t readDATAFromBus(uint8_t command,uint16_t sleepTime)
{
	uint32_t ret=0,temp=0;
	char c;
	uint8_t payload[4];
	payload[0]=command;
	payload[1]=sleepTime & 0x00FF;
	payload[2]=(sleepTime & 0xFF00)>>8;
	payload[3]=0;
	writeFrameShort(0x01,READ_DATA_FRAME,payload,4);
	internalDelayMs(45);
	assignDataPinDirection(0,0);
	setClockPinState(0);
	for(int i=0;i<32;i++)
	{
		minorDelay();
		setClockPinState(1);
		internalDelayMs(12);
		temp=getDataPinState();
		ret|=temp<<i;
		minorDelay();
		setClockPinState(0);
		minorDelay();
	}
	assignDataPinDirection(1,1);
	return ret;
}

/////////////////////////ARDUINO SECTION !!!!!!!!!!!!! -- DONT USE FOR QUECTEL----------------------------
void setup()
{
	Serial.begin(115200);
	Serial.println("START");
	initializeZTIInterface();
	/* add setup code here */

}

void loop()
{
uint16_t cntr=20,wait=0;
  while(1)
  {
	  if(Serial.available()>0)
	  {
		  char c=Serial.read();
		  if(c=='s')
		  {
			  wait=1;
			  Serial.println("GIVING sleep COMMAND");
			  ltoWConv.dataLong=readDATAFromBus(ENTER_SLEEP_MODE,cntr);
		      Serial.print("RESP: ");
			  Serial.print(ltoWConv.dataWord[0] & 0xFFFF);
			  Serial.write(',');
			  Serial.println(ltoWConv.dataWord[1] & 0x7FFF);
			  delay(1000);
		  }
		  else if(c=='t')
		  {
			  cntr=Serial.parseInt();
			  Serial.print("GIVING SLEEP TIME: ");
			  Serial.println(cntr);
			  delay(1000);
		  }
		  else if(c=='w')
		  {
			  Serial.println("CLR WAIT!");
			  wait=0;
		  }
	  }
	  if(wait==0)
	  {
	   ltoWConv.dataLong=readDATAFromBus(READ_DATA_FRAME,cntr);
	   Serial.print("RESP: ");
	   Serial.print(ltoWConv.dataWord[0] & 0xFFFF);
	   Serial.write(',');
	   Serial.println(ltoWConv.dataWord[1] & 0x7FFF);
	  }
  }
  /* add main program code here */

}
/*while(1)
{
if(Serial.available()>0)
{
c=Serial.read();
if(c=='b')
break;
else if(c=='o')
{
PORTC|=SCL_PIN;
Serial.println("ON");
}
else if(c=='f')
{
PORTC&=SCL_PIN_NEG;
Serial.println("OFF");
}
}
Serial.println(PINC,BIN);
delay(1000);
}*/
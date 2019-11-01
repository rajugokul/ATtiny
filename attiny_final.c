/*
 * attiny_quc_twi.c
 *
 * Created: 25-10-2019 11:35:36
 * Author : vj_gokul
 */ 

#define F_CPU 1200000 // 1.2MHZ
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

//-------------DEFINES----------------

#define ADC_MUX_OPTION_0 0x01//mux0 mux1//pb2
#define ADC_MUX_OPTION_1 0x02//mux1 mux1//pb4
//////////////////////////////////////////////////////////////

#define MODULE_RESET_TIME 9000 // 9 second reset
#define COMMON_DELAY 4500
#define DEFAULT_SLEEP_TIME 120 // 2mins

#define MY_DEVICE_ADDRESS 0x01

#define SDA_PIN  0x01//PB0 - PINNAME RI
#define SCL_PIN 0x02// PB1 - PINNAME DCD

#define SDA_PIN_NEG 0xFE//-PB0
#define SCL_PIN_NEG 0xFD//-PB1


#define PWR_KEY_PIN 0x08 // - pwrkey of quectel//-----PB0
#define PWR_KEY_PIN_NEG 0xF7   //0xFE

//  PROGRAM SECTIONS
#define ENABLE_ZTI_BLOCK
#define ENABLE_ADC_BLOCK
#define ENABLE_PWR_KEY_BLOCK
#define ENABLE_RESET_BLOCK


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

//------------VARIABLES---------------
struct ZTIProtocolClass
{
	uint8_t dataPointer;
	uint16_t heartBeatTimer;
	//--------------------
	uint8_t payload[4];
}ztiProtocol;

#define BUFF_SIZE 6
volatile uint8_t  inputBuffer[BUFF_SIZE];
volatile uint16_t outputBuffer[2];
volatile static uint8_t clk,dat,sda_stat,pReg,reg,dataPointer=0,sleepCommandGiven=0,slaveSend=0;
volatile static int8_t  ctr_data=0,ctr_clk=0;
int16_t sleepTime=DEFAULT_SLEEP_TIME;
//------------HELPER_FUNCTIONS--------

//------------INTERRUPT---------------
ISR(WDT_vect)
{
	if(sleepCommandGiven==1)
	{
		sleepTime-=8;
		ztiProtocol.heartBeatTimer=slaveSend=0;
		if(sleepTime>0)
		wdt_disable();
		else
		{
		sleepCommandGiven=0;
		asm volatile("rjmp 0 \n\t");
		}
	}
}
#ifdef ENABLE_ZTI_BLOCK
ISR(PCINT0_vect)
{
	//cli()
	sda_stat=((PINB ^ pReg) & SDA_PIN);
	(sda_stat)?ctr_data++:1;
	
	clk=((PINB ^ pReg) & SCL_PIN)?1:0;
	((PINB & SCL_PIN)&&(clk==1))?(dat=1):(dat=2);
	pReg=PINB;
	
	(clk==1)?ctr_data=0:1;
	if(slaveSend==0){
	if(ctr_data>=3)
	{
		ctr_data=0;
		ctr_clk=0;
		reg=0;
	}
	else if(dat==1)
	{
		reg|=((PINB & SDA_PIN)?1:0)<<ctr_clk;
		ctr_clk++;
		if(ctr_clk==8)
		{
			inputBuffer[dataPointer]=reg;
			ctr_clk=reg=0;
			dataPointer++;
		}
	}
	}
	else
	{
		if(dat==1)
		{
			(outputBuffer[(ctr_clk>=16)?1:0] & (1<<(ctr_clk%16)))?(PORTB|=SDA_PIN):(PORTB&=SDA_PIN_NEG);
			ctr_clk++;
			if(ctr_clk>=32){ slaveSend=0; ctr_clk=0; DDRB&=SDA_PIN_NEG; }
		}
	}
	clk=dat=0;
}
#endif

//------------MAIN_CODE---------------
int main(void)
{
	uint8_t adcTog=0;
	char c;
	// Interrupt Pins for Input
	#ifdef ENABLE_ZTI_BLOCK
	PCMSK=0b00000011;//select interrupt pins
	GIMSK=0b00100000;//set interrupt is enable
	#endif
	// GPIO OUTPUT FOR PWR KEY
	#ifdef ENABLE_PWR_KEY_BLOCK
	DDRB|=PWR_KEY_PIN;
	PORTB&=PWR_KEY_PIN_NEG;
	_delay_ms(COMMON_DELAY);
	PORTB|=PWR_KEY_PIN;
	#else
	_delay_ms(1000);
	#endif
	wdt_enable(WDTO_8S); // set prescaler to 0.5s and enable Watchdog Timer
	WDTCR |= _BV(WDTIE); 
	sei(); // enable global interrupts
	/* loop */
	while(1)
	{
	wdt_reset();
	#ifdef ENABLE_ZTI_BLOCK
	while(dataPointer>0)
	{
		dataPointer--;
		c=inputBuffer[dataPointer];
		if(ztiProtocol.dataPointer>=DEVICE_ADDRESS)
		{
			ztiProtocol.payload[ztiProtocol.dataPointer-DEVICE_ADDRESS]=c;
			ztiProtocol.dataPointer++;
			if(ztiProtocol.dataPointer>=6){
				ztiProtocol.heartBeatTimer=ztiProtocol.dataPointer=0;				
				(ztiProtocol.payload[0]==ENTER_SLEEP_MODE)?(sleepCommandGiven=1):1;
				sleepTime=(((uint16_t)ztiProtocol.payload[2])<<8)|((uint16_t)ztiProtocol.payload[1]);
				DDRB|=SDA_PIN;
				PORTB&=SDA_PIN_NEG;
				_delay_ms(27);
				ctr_clk=0;
				slaveSend=1;
				
			}
		}
		(c==HEADER_2)?((ztiProtocol.dataPointer==1)?(ztiProtocol.dataPointer=2):(ztiProtocol.dataPointer=0)):1;
		((c==HEADER_1)&&(ztiProtocol.dataPointer==0))?(ztiProtocol.dataPointer=1):1;
	}
	#endif
	
	#ifdef ENABLE_RESET_BLOCK
	ztiProtocol.heartBeatTimer++;
	if(ztiProtocol.heartBeatTimer>MODULE_RESET_TIME)
	{
		ztiProtocol.heartBeatTimer=0;
		asm volatile("rjmp 0 \n\t");
	}
	#endif
	#ifdef ENABLE_ADC_BLOCK
	 (adcTog)?(outputBuffer[0]=ADC):(outputBuffer[1]=ADC);
	 adcTog=!adcTog;
	 ADMUX=0;
         (adcTog)?(ADMUX|=ADC_MUX_OPTION_1):(ADMUX|=ADC_MUX_OPTION_0);
	 //ADMUX  |= (1 << ((adcTog)?(MUX0):(MUX1)));
	 ADCSRA |= (1 << ADEN) | (1 << ADSC);
	#endif
	_delay_ms(1);
	//-- KERNEL BLOCK
	if((slaveSend==0)&&(sleepCommandGiven==1)){
		wdt_enable(WDTO_8S);
		WDTCR |= _BV(WDTIE);
		sei(); // enable global interrupts
		ADCSRA=0x00; //Disable ADC
		DDRB=PORTB=0;
		ACSR = (1<<ACD); //Disable the analog comparator
		DIDR0 = 0x3F; //Disable digital input buffers on all ADC0-ADC5 pins.
		set_sleep_mode(SLEEP_MODE_PWR_DOWN);
		sleep_mode();
	}
	}
	
}

//////////////////////////////////////////////////////////////////////////
//
// MIDI HUB WITH BEAT CLOCK METRONOME
// Jason Hotchkiss
// 
// FOR PIC16F1825
// SOURCEBOOST C
//
// Rev 1.00:    Feb 2013 - port of original PIC16F688 code
// Rev 1.01: 29 Mar 2013 - fix issue with input buffer position
//
//////////////////////////////////////////////////////////////////////////
#include <system.h>

// CONFIG OPTIONS 
// - RESET INPUT DISABLED
// - WATCHDOG TIMER OFF
// - INTERNAL OSC
#pragma DATA _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _MCLRE_OFF &_CLKOUTEN_OFF
#pragma DATA _CONFIG2, _WRT_OFF & _PLLEN_OFF & _STVREN_ON & _BORV_19 & _LVP_OFF
#pragma CLOCK_FREQ 8000000

// inputs
#define P_RUN		portc.3
#define P_DEC		porta.4
#define P_INC		porta.5

// outputs
#define P_LED0		portc.0 
#define P_LED1		portc.1 
#define P_LED2		portc.2 
#define P_LED3		porta.1 
#define P_LED4		porta.0
#define P_LED5		porta.2 

// bit masks for button tracking
#define M_BUTTON_RUN 0x01
#define M_BUTTON_INC 0x02
#define M_BUTTON_DEC 0x04
#define M_AUTO_REPEAT 0x80

// MIDI beat clock messages
#define MIDI_SYNCH_TICK     0xf8
#define MIDI_SYNCH_START    0xfa
#define MIDI_SYNCH_CONTINUE 0xfb
#define MIDI_SYNCH_STOP     0xfc

// Auto repeat delays (ms)
#define AUTO_REPEAT_INTERVAL	30
#define AUTO_REPEAT_DELAY		500

// Key debounce period (ms)
#define DEBOUNCE_PERIOD			50

// midi TICK is sent every 1/24th of a beat
// so we need to divide the bpm into this number
// to get the tick interval
#define MS_PER_TICK 2500

// Timer 1 used to maintain a millisecond counter
// 8,000,000 fosc
// 2mhz fosc/4
// prescale/8 = 250khz
// 250 = 1ms
// 65536 - 250 = 65286
#define TIMER_INIT_SCALAR 0xff06
#define INIT_TIMER1 { tmr1l=(TIMER_INIT_SCALAR & 0xff); tmr1h=(TIMER_INIT_SCALAR>>8); }
volatile unsigned long millis = 0;

// define the buffer used to receive MIDI input
#define SZ_RXBUFFER 20
typedef unsigned char byte;
byte rxBuffer[SZ_RXBUFFER];
byte rxHead = 0;
byte rxTail = 0;

////////////////////////////////////////////////////////////
// INTERRUPT HANDLER CALLED WHEN CHARACTER RECEIVED AT 
// SERIAL PORT OR WHEN TIMER 1 OVERLOWS
void interrupt( void )
{
	// timer 1 rollover
	if(pir1.0)
	{
		INIT_TIMER1;
		++millis;
		pir1.0 = 0;
	}
	
	// serial rx interrupt
	if(pir1.5)
	{	
		// get the byte
		byte b = rcreg;
		
		// calculate next buffer head
		byte nextHead = (rxHead + 1);
		if(nextHead >= SZ_RXBUFFER) 
		{
			nextHead -= SZ_RXBUFFER;
		}
		
		// if buffer is not full
		if(nextHead != rxTail)
		{
			// store the byte
			rxBuffer[rxHead] = b;
			rxHead = nextHead;
		}		
	}
}


////////////////////////////////////////////////////////////
// INITIALISE SERIAL PORT FOR MIDI
void init_usart()
{
	pir1.1 = 1;		//TXIF 		
	pir1.5 = 0;		//RCIF
	
	pie1.1 = 0;		//TXIE 		no interrupts
	pie1.5 = 1;		//RCIE 		enable
	
	baudcon.4 = 0;	// SCKP		synchronous bit polarity 
	baudcon.3 = 1;	// BRG16	enable 16 bit brg
	baudcon.1 = 0;	// WUE		wake up enable off
	baudcon.0 = 0;	// ABDEN	auto baud detect
		
	txsta.6 = 0;	// TX9		8 bit transmission
	txsta.5 = 1;	// TXEN		transmit enable
	txsta.4 = 0;	// SYNC		async mode
	txsta.3 = 0;	// SEDNB	break character
	txsta.2 = 0;	// BRGH		high baudrate 
	txsta.0 = 0;	// TX9D		bit 9

	rcsta.7 = 1;	// SPEN 	serial port enable
	rcsta.6 = 0;	// RX9 		8 bit operation
	rcsta.5 = 1;	// SREN 	enable receiver
	rcsta.4 = 1;	// CREN 	continuous receive enable
		
	spbrgh = 0;		// brg high byte
	spbrg = 15;		// brg low byte (31250)	
	
}

////////////////////////////////////////////////////////////
// SEND A BYTE ON SERIAL PORT
void send(byte c)
{
	txreg = c;
	while(!txsta.1);
}

////////////////////////////////////////////////////////////
// RUN MIDI THRU
void midiThru()
{
	// loop until there is no more data or
	// we receive a full message
	for(;;)
	{
		// buffer overrun error?
		if(rcsta.1)
		{
			rcsta.4 = 0;
			rcsta.4 = 1;
		}
		// any data in the buffer?
		if(rxHead == rxTail)
		{
			// no data ready
			return;
		}
		
		// read the character out of buffer
		byte q = rxBuffer[rxTail];
		if(++rxTail >= SZ_RXBUFFER) 
			rxTail -= SZ_RXBUFFER;
				
		// send it to output
		P_LED2 = 1;
		P_LED3 = 1;
		send(q);
		P_LED2 = 0;
		P_LED3 = 0;
	}	
}

////////////////////////////////////////////////////////////
// UPDATE LEDS
void setLeds(byte d)
{
	P_LED0 = !!(d & 0x01);
	P_LED1 = !!(d & 0x02);
	P_LED2 = !!(d & 0x04);
	P_LED3 = !!(d & 0x08);
	P_LED4 = !!(d & 0x10);
	P_LED5 = !!(d & 0x20);
}

////////////////////////////////////////////////////////////
// MAIN
void main()
{ 
	// osc control / 8MHz / internal
	osccon = 0b01110010;
	
	// configure io
	trisa = 0b00110000;              	
    trisc = 0b00111000;              
	ansela = 0b00000000;
	anselc = 0b00000000;
	porta=0;
	portc=0;

	// initialise MIDI comms
	init_usart();

	intcon.7 = 1; //GIE
	intcon.6 = 1; //PEIE
	
	tmr1l = 0;
	tmr1h = 0;
	t1con.5 = 1; // prescaler mode
	t1con.4 = 1; // prescaler mode
	t1con.1 = 0; // internal
	t1con.0 = 1; // enabled
	pie1.0 = 1; // timer 1 interrupt enable
	
	// initialise app variables
	INIT_TIMER1;
	unsigned long nextTick = 0;
	unsigned long nextKeyPoll = 0;
	int bpm = 120;
	byte running = 0;
	byte tickCount = 0;
	byte lastButtonStatus = 0;
	unsigned long autoRepeatBegin = 0;
	unsigned long nextAutoRepeat = 0;
	unsigned long debouncePeriodEnd = 0;
	unsigned long tickInterval = MS_PER_TICK/bpm;
	
	for(;;)
	{
		// run midi thru
		midiThru();
	
		// run beat clock
		if(millis > nextTick)
		{
			nextTick = millis + tickInterval;
			tickCount = (tickCount+1)%24;
			if(running)
			{
				send(MIDI_SYNCH_TICK);
				setLeds(1<<(tickCount>>2));
			}
			else if(!tickCount)
			{
				setLeds(0b110011);
			}
			else 
			{
				setLeds(0);
			}
		}
		
		// keyboard handling
		if(millis >= debouncePeriodEnd)
		{
			// gather up the button statuses into a single byte
			byte thisButtonStatus = 
				(!P_RUN ? M_BUTTON_RUN : 0) |
				(!P_DEC ? M_BUTTON_DEC : 0) |
				(!P_INC ? M_BUTTON_INC : 0);
				
				
			// see if anything has changed since last poll
			byte buttonActivity = thisButtonStatus ^ lastButtonStatus;			
			lastButtonStatus = thisButtonStatus;
			byte buttonsPressed = 0;
			if(!buttonActivity)
			{				
				// do we need to autorepeat?
				if(thisButtonStatus && millis > autoRepeatBegin && millis > nextAutoRepeat)
				{
					// flag auto repeat
					thisButtonStatus = thisButtonStatus|M_AUTO_REPEAT;
					buttonsPressed = thisButtonStatus;
					nextAutoRepeat = millis + AUTO_REPEAT_INTERVAL;
				}
			}
			else
			{
				// mask out just buttons that have been pressed
				buttonsPressed = buttonActivity & thisButtonStatus;
				if(buttonsPressed)
				{
					// prepare debounce and auto repeat
					autoRepeatBegin = millis + AUTO_REPEAT_DELAY;
					debouncePeriodEnd = millis + DEBOUNCE_PERIOD;
					nextAutoRepeat = 0;
				}
			}
			
			// any new button presses or auto repeats?
			if(buttonsPressed)
			{
				// execute the command
				switch(thisButtonStatus)
				{
					// RUN 
					case M_BUTTON_RUN:
						running = !running;
						if(running)
							send(MIDI_SYNCH_START);
						else
							send(MIDI_SYNCH_STOP);			
						break;
						
					// DEC
					case M_BUTTON_DEC:
					case M_AUTO_REPEAT|M_BUTTON_DEC:
						if(bpm>30)
						{
							--bpm;
							tickInterval = MS_PER_TICK/bpm;
						}
						break;
						
					// INC
					case M_BUTTON_INC:
					case M_AUTO_REPEAT|M_BUTTON_INC:
						if(bpm<250)
						{
							++bpm;
							tickInterval = MS_PER_TICK/bpm;
						}
						break;
				
					// INC+DEC
					case M_BUTTON_INC|M_BUTTON_DEC:
						bpm = 120;
						tickInterval = MS_PER_TICK/bpm;
						break;
				}				
			}
		}
	}
}
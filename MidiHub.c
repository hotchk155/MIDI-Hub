//////////////////////////////////////////////////////////////////////////
//
// MIDI HUB WITH BEAT CLOCK METRONOME
// Jason Hotchkiss
// 
// FOR PIC16F1825
// SOURCEBOOST C
//
// Rev H0:    Feb 2013 - port of original PIC16F688 code
// Rev H1: 29 Mar 2013 - fix issue with input buffer position
// Rev H2: 17 May 2013 - increase debounce period
// Rev H3: 12 Jul 2013 - use 16MHz clock, increase BPM accuracy, LED PWM
//
//////////////////////////////////////////////////////////////////////////
#include <system.h>

// CONFIG OPTIONS 
// - RESET INPUT DISABLED
// - WATCHDOG TIMER OFF
// - INTERNAL OSC
#pragma DATA _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _MCLRE_OFF &_CLKOUTEN_OFF
#pragma DATA _CONFIG2, _WRT_OFF & _PLLEN_OFF & _STVREN_ON & _BORV_19 & _LVP_OFF
#pragma CLOCK_FREQ 16000000

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

// Auto repeat delays 
#define AUTO_REPEAT_INTERVAL	20
#define AUTO_REPEAT_DELAY	 	250

// Key debounce period 
#define DEBOUNCE_PERIOD			50

// PWM fade stuff
#define FADE_PERIOD				2
#define PWM_MAX 				20

// typedefs
typedef unsigned char byte;

// timer stuff
volatile byte tick_flag = 0;
volatile unsigned int timer_init_scalar = 0;
volatile unsigned long systemTicks = 0;

// define the buffer used to receive MIDI input
#define SZ_RXBUFFER 20
volatile byte rxBuffer[SZ_RXBUFFER];
volatile byte rxHead = 0;
volatile byte rxTail = 0;

// led and PWM stuff
byte led0 = 0;
byte led1 = 0;
byte led2 = 0;
byte led3 = 0;
byte led4 = 0;
byte led5 = 0;
byte pwm = 0;

////////////////////////////////////////////////////////////
// INTERRUPT HANDLER CALLED WHEN CHARACTER RECEIVED AT 
// SERIAL PORT OR WHEN TIMER 1 OVERLOWS
void interrupt( void )
{
	// timer 0 rollover ISR. Maintains the count of 
	// "system ticks" that we use for key debounce etc
	if(intcon.2)
	{
		systemTicks++;
		intcon.2 = 0;
	}

	// timer 1 rollover ISR. Responsible for timing
	// the tempo of the MIDI clock
	if(pir1.0)
	{
		tmr1l=(timer_init_scalar & 0xff); 
		tmr1h=(timer_init_scalar>>8); 
		tick_flag = 1;
		pir1.0 = 0;
	}
		
	// serial rx ISR
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
	spbrg = 30;		// brg low byte (31250)	
	
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
// SETUP THE TIMER FOR A SPECIFIC BPM
void setTimerForBPM(int bpm)
{
/*
	beats per second = bpm / 60 
	midi ticks per second = 24 * (bpm / 60)	
	timer counts per MIDI tick = (timer counts per second)/(midi ticks per second)
		= (timer counts per second)/(24 * (bpm / 60))
		= (timer counts per second/24)/(bpm / 60)
		= 60 * (timer counts per second/24)/bpm
	timer init scalar = 65536 - timer counts per MIDI tick	
*/
	#define TIMER_COUNTS_PER_SECOND (unsigned long)500000
	unsigned long x = (60 * TIMER_COUNTS_PER_SECOND)/24;
	x = x / bpm;
	timer_init_scalar = 65535 - x;
}

////////////////////////////////////////////////////////////
// MAIN
void main()
{ 
	// initialise app variables
	unsigned long nextKeyPoll = 0;
	byte running = 0;
	byte tickCount = 0;
	byte lastButtonStatus = 0;
	unsigned long autoRepeatBegin = 0;
	unsigned long nextAutoRepeat = 0;
	unsigned long nextFade = 0;
	unsigned long debouncePeriodEnd = 0;
	int bpm = 120;	

	// osc control / 16MHz / internal
	osccon = 0b01111010;
	
	// configure io
	trisa = 0b00110000;              	
    trisc = 0b00111000;              
	ansela = 0b00000000;
	anselc = 0b00000000;
	porta=0;
	portc=0;

	// initialise MIDI comms
	init_usart();

	// setup default BPM
	setTimerForBPM(bpm);

	
	// Configure timer 1
	// Input 4MHz
	// Prescaled to 500KHz
	tmr1l = 0;	 // reset timer count register
	tmr1h = 0;
	t1con.7 = 0; // } Fosc/4 rate
	t1con.6 = 0; // }
	t1con.5 = 1; // } 1:8 prescale
	t1con.4 = 1; // }
	t1con.0 = 1; // timer 1 on
	pie1.0 = 1;  // timer 1 interrupt enable
	

	// Configure timer 0
	// 	timer 0 runs at 4MHz
	// 	prescaled 1/64 = 62500Hz
	// 	rollover at 256 = 244Hz
	// 	~4ms per rollover	
	option_reg.5 = 0; // timer 0 driven from instruction cycle clock
	option_reg.3 = 0; // timer 0 is prescaled
	option_reg.2 = 1; // }
	option_reg.1 = 0; // } 1/64 prescaler
	option_reg.0 = 1; // }
	intcon.5 = 1; 	  // enabled timer 0 interrrupt
	intcon.2 = 0;     // clear interrupt fired flag
	
	// enable interrupts	
	intcon.7 = 1; //GIE
	intcon.6 = 1; //PEIE
	
	// App loop
	for(;;)
	{	
				
		// run midi thru
		midiThru();
	
		// run beat clock
		if(tick_flag)
		{
			tick_flag = 0;
			tickCount = (tickCount + 1)%24;
			if(running)
			{
				// send MIDI clock
				send(MIDI_SYNCH_TICK);
				
				// update the LEDs
				switch(tickCount>>2)
				{
					case 0: led0=PWM_MAX; break;
					case 1: led1=PWM_MAX; break;
					case 2: led2=PWM_MAX; break;
					case 3: led3=PWM_MAX; break;
					case 4: led4=PWM_MAX; break;
					case 5: led5=PWM_MAX; break;
				}
			}
			else if(!tickCount)
			{
				// blink leds 0,1,4,5
				led0 = led1 = led4 = led5 = PWM_MAX;
			}
		}	
		
		// keyboard handling
		if(systemTicks >= debouncePeriodEnd)
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
				if(thisButtonStatus && systemTicks > autoRepeatBegin && systemTicks > nextAutoRepeat)
				{
					// flag auto repeat
					thisButtonStatus = thisButtonStatus|M_AUTO_REPEAT;
					buttonsPressed = thisButtonStatus;
					nextAutoRepeat = systemTicks + AUTO_REPEAT_INTERVAL;
				}
			}
			else
			{
				// mask out just buttons that have been pressed
				buttonsPressed = buttonActivity & thisButtonStatus;
				if(buttonsPressed)
				{
					// prepare debounce and auto repeat
					autoRepeatBegin = systemTicks + AUTO_REPEAT_DELAY;
					debouncePeriodEnd = systemTicks + DEBOUNCE_PERIOD;
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
							bpm--; 
							setTimerForBPM(bpm);												
						}
						break;
						
					// INC
					case M_BUTTON_INC:
					case M_AUTO_REPEAT|M_BUTTON_INC:
						if(bpm<250)
						{
							bpm++;
							setTimerForBPM(bpm);						
						}
						break;
				
					// INC+DEC
					case M_BUTTON_INC|M_BUTTON_DEC:
						bpm=120;
						setTimerForBPM(bpm);
						break;
				}				
			}
		}	
		
		// Deal with PWM
		if(led0 == pwm) P_LED0 = 0;
		if(led1 == pwm) P_LED1 = 0;
		if(led2 == pwm) P_LED2 = 0;
		if(led3 == pwm) P_LED3 = 0;
		if(led4 == pwm) P_LED4 = 0;
		if(led5 == pwm) P_LED5 = 0;		
		if(++pwm==PWM_MAX)
		{
			P_LED0 = !!led0;
			P_LED1 = !!led1;
			P_LED2 = !!led2;
			P_LED3 = !!led3;
			P_LED4 = !!led4;
			P_LED5 = !!led5;
			pwm=0;		
		}
	
		// Deal with fade
		if(systemTicks >= nextFade)
		{
			nextFade = systemTicks + FADE_PERIOD;
			if(!!led0) led0--;
			if(!!led1) led1--;
			if(!!led2) led2--;
			if(!!led3) led3--;
			if(!!led4) led4--;
			if(!!led5) led5--;
		}	
	}
}
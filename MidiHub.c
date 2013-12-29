//////////////////////////////////////////////////////////////////////////
//
//                                                             //     //
//             //        //  //  //                  //        //     //
//                       //      //                  //      ////// //////
// ////////    //    //////  //  //////    //    //  //////    //     //
// //  //  //  //  //    //  //  //    //  //    //  //    //  //     //
// //  //  //  //  //    //  //  //    //  //    //  //    //  
// //      //  //  //    //  //  //    //  //    //  //    //  
// //      //  //    //////  //  //    //    //////  //////    
//
// MIDI HUB WITH BEAT CLOCK METRONOME
// By Jason Hotchkiss
// FOR PIC16F1825 SOURCEBOOST C
//
// This work is licensed under the Creative Commons 
// Attribution-NonCommercial 3.0 Unported License. 
// To view a copy of this license, please visit:
// http://creativecommons.org/licenses/by-nc/3.0/
//
// Please contact me directly if you'd like a CC 
// license allowing use for commercial purposes:
// jason_hotchkiss<at>hotmail.com
//
// Full repository with hardware information:
// https://github.com/hotchk155/MIDI-Hub

// Rev H0:    Feb 2013 - port of original PIC16F688 code
// Rev H1: 29 Mar 2013 - fix issue with input buffer position
// Rev H2: 17 May 2013 - increase debounce period
// Rev H3: 12 Jul 2013 - use 16MHz clock, increase BPM accuracy, LED PWM
// Rev H4:  1 Sep 2013 - tap tempo mode, options menu
// Rev H5: 17 Nov 2013 - new options for explicit midi START/STOP/CONTINUE
// Rev H6: 29 Dec 2013 - tighter baudrate definition, added version display
//
#define FIRMWARE_VERSION 6
//////////////////////////////////////////////////////////////////////////


//
// HEADER FILES
//
#include <system.h>
#include <rand.h>
#include <eeprom.h>

// PIC CONFIG BITS
// - RESET INPUT DISABLED
// - WATCHDOG TIMER OFF
// - INTERNAL OSC
#pragma DATA _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _MCLRE_OFF &_CLKOUTEN_OFF
#pragma DATA _CONFIG2, _WRT_OFF & _PLLEN_OFF & _STVREN_ON & _BORV_19 & _LVP_OFF
#pragma CLOCK_FREQ 16000000

//
// TYPE DEFS
//
typedef unsigned char byte;

//
// MACRO DEFS
//

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
#define M_BUTTON_RUN 			0x01
#define M_BUTTON_INC 			0x02
#define M_BUTTON_DEC 			0x04
#define M_LONG_PRESS			0x40
#define M_AUTO_REPEAT 			0x80

// MIDI beat clock messages
#define MIDI_SYNCH_TICK     	0xf8
#define MIDI_SYNCH_START    	0xfa
#define MIDI_SYNCH_CONTINUE 	0xfb
#define MIDI_SYNCH_STOP     	0xfc

// Auto repeat delays 
#define AUTO_REPEAT_INTERVAL	80
#define AUTO_REPEAT_DELAY	 	500

// Key debounce period 
#define DEBOUNCE_PERIOD			100

// PWM fade stuff
#define FADE_PERIOD				30
#define PWM_DIM 				5
#define PWM_MAX 				50
#define INITIAL_DUTY			10

#define TIMER_0_INIT_SCALAR		5	// Timer 0 is an 8 bit timer counting at 250kHz
									// using this init scalar means that rollover
									// interrupt fires once per ms

// Tempo defs
#define BPM_MIN					30
#define BPM_MAX					300
#define BPM_DEFAULT				120

// EEPROM usage
#define EEPROM_ADDR_MAGIC_COOKIE 9
#define EEPROM_ADDR_OPTIONS	10
#define EEPROM_MAGIC_COOKIE 0xA5

// Menu size
#define MENU_SIZE 6 

//
// GLOBAL DATA
//

// timer stuff
volatile byte tick_flag = 0;
volatile unsigned int timer_init_scalar = 0;
volatile unsigned long systemTicks = 0; // each system tick is 1ms

// define the buffer used to receive MIDI input
#define SZ_RXBUFFER 20
volatile byte rxBuffer[SZ_RXBUFFER];
volatile byte rxHead = 0;
volatile byte rxTail = 0;


// Configuration options
enum {
	OPTION_PASSREALTIMEMSG 	= 0x01,
	OPTION_PASSOTHERMSG 	= 0x02,
	OPTION_STARTSTOP 		= 0x04,	
	OPTION_THRUANIMATE 		= 0x08,
	OPTION_DISCREET 		= 0x10,
	OPTIONS_DEFAULT 		= OPTION_PASSOTHERMSG|OPTION_STARTSTOP|OPTION_THRUANIMATE
};
byte _options = OPTIONS_DEFAULT;

// Operating modes
enum {
	MODE_STEP,		// BEAT CLOCK ON, INC/DEC SET
	MODE_TAP,		// BEAT CLOCK ON, TAP SET
	MODE_NOCLOCK,	// BEAT CLOCK OFF
	MODE_MENU		// BEAT CLOCK OFF, OPTIONS MENU
};
byte _mode = MODE_STEP;

// Brightness settings
#define NUM_BRIGHTNESS_LEVELS 6
byte brightnessLevels[NUM_BRIGHTNESS_LEVELS];
byte _brightness = 0;

// LED duty buffer
byte duty[6];

// BPM setting
int _bpm = 0;

////////////////////////////////////////////////////////////
// INTERRUPT HANDLER CALLED WHEN CHARACTER RECEIVED AT 
// SERIAL PORT OR WHEN TIMER 1 OVERLOWS
void interrupt( void )
{
	// timer 0 rollover ISR. Maintains the count of 
	// "system ticks" that we use for key debounce etc
	if(intcon.2)
	{
		tmr0 = TIMER_0_INIT_SCALAR;
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
// SAVE OPTIONS TO EEPROM
void saveOptions()
{
	eeprom_write(EEPROM_ADDR_OPTIONS, _options);
	eeprom_write(EEPROM_ADDR_MAGIC_COOKIE, EEPROM_MAGIC_COOKIE);
}

////////////////////////////////////////////////////////////
// LOAD OPTIONS FROM EEPROM
void loadOptions()
{
	// "magic cookie" is a known value written to the EEPROM
	// with each valid save. Makes sure we can avoid reading
	// garbage from EEPROM when there is no previous save
	_options = eeprom_read(EEPROM_ADDR_OPTIONS);
	if(eeprom_read(EEPROM_ADDR_MAGIC_COOKIE) != EEPROM_MAGIC_COOKIE)
		_options = OPTIONS_DEFAULT; 
}


////////////////////////////////////////////////////////////
// INITIALISE SERIAL PORT FOR MIDI
void initUSART()
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
	spbrg = 31;		// brg low byte (31250)	
	
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

		// Check for MIDI realtime message (e.g. clock)
		if((q & 0xF8) == 0xF8)
		{
			// if we are not passing realtime messages then skip it
			if(!(_options & OPTION_PASSREALTIMEMSG))
				continue;
		}
		else
		{
			// if we are not passing non-realtime messages then skip it
			if(!(_options & OPTION_PASSOTHERMSG))
				continue;
		}
		
		// should we animate the LEDs based on thru traffic?
		if(MODE_NOCLOCK == _mode && (_options & OPTION_THRUANIMATE))
		{
			// animate and send
			duty[q%6] = q%INITIAL_DUTY;
			send(q);
		}
		// should we indicate thru traffic with flickering LEDs?
		else 
		{					
			// flicker and send
			P_LED2 = 1;
			P_LED3 = 1;
			send(q);
			P_LED2 = 0;
			P_LED3 = 0;
		}
	}		
}

////////////////////////////////////////////////////////////
// SETUP THE TIMER FOR A SPECIFIC BPM
void setBPM(int b)
{
	if(b < BPM_MIN)
		b = BPM_MIN;
	else if(b > BPM_MAX)
		b = BPM_MAX;
	_bpm = b;
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
	x = x / _bpm;
	timer_init_scalar = 65535 - x;
}

////////////////////////////////////////////////////////////
// SHOW VERSION
void showVersion()
{
	P_LED0 = !!(FIRMWARE_VERSION&0x20);
	P_LED1 = !!(FIRMWARE_VERSION&0x10);
	P_LED2 = !!(FIRMWARE_VERSION&0x08);
	P_LED3 = !!(FIRMWARE_VERSION&0x04);
	P_LED4 = !!(FIRMWARE_VERSION&0x02);
	P_LED5 = !!(FIRMWARE_VERSION&0x01);
	delay_s(5);
}

////////////////////////////////////////////////////////////
// MAIN
void main()
{ 
	// initialise app variables
	byte running = 0;
	byte tickCount = 0;
	byte lastButtonStatus = 0;
	unsigned long autoRepeatBegin = 0;
	unsigned long nextAutoRepeat = 0;
	unsigned long nextFade = 0;
	unsigned long debouncePeriodEnd = 0;
	unsigned long lastTapSystemTicks = 0;
	unsigned long firstTapSystemTicks = 0;
	unsigned long tapPeriodAccumulator = 0;
	unsigned long menuLoopCount = 0;
	byte tapCount = 0;
	byte menuOption = 0;
	byte runLock = 0;
	byte midiRestart = 0;
	
	// initialise brightness levels
	brightnessLevels[0] = 50;
	brightnessLevels[1] = 20;
	brightnessLevels[2] = 10;
	brightnessLevels[3] = 5;
	brightnessLevels[4] = 2;
	brightnessLevels[5] = 1;
	byte maxDuty = brightnessLevels[0];
	byte pwm=0;	
	duty[0] = duty[1] = duty[2] = duty[3] = duty[4] = duty[5] = 0;
	
	// osc control / 16MHz / internal
	osccon = 0b01111010;
	
	// configure io
	trisa = 0b00110000;              	
    trisc = 0b00111000;              
	ansela = 0b00000000;
	anselc = 0b00000000;
	porta=0;
	portc=0;

	// Version display
	if(!P_RUN)
		showVersion();
		
	// initialise MIDI comms
	initUSART();

	// setup default BPM
	setBPM(BPM_DEFAULT);

	// Configure timer 1 (controls tempo)
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
	
	// Configure timer 0 (controls systemticks)
	// 	timer 0 runs at 4MHz
	// 	prescaled 1/16 = 250kHz
	// 	rollover at 250 = 1kHz
	// 	1ms per rollover	
	option_reg.5 = 0; // timer 0 driven from instruction cycle clock
	option_reg.3 = 0; // timer 0 is prescaled
	option_reg.2 = 0; // }
	option_reg.1 = 1; // } 1/16 prescaler
	option_reg.0 = 1; // }
	intcon.5 = 1; 	  // enabled timer 0 interrrupt
	intcon.2 = 0;     // clear interrupt fired flag
	
	// enable interrupts	
	intcon.7 = 1; //GIE
	intcon.6 = 1; //PEIE

	// default operating mode is clock/step control
	_mode = MODE_STEP;

	// load options
	loadOptions();
	
	// App loop
	for(;;)
	{	
				
		// run midi thru
		midiThru();
		
		// RUNNING MENU
		if(MODE_MENU == _mode)
		{
			menuLoopCount++;
			byte flash = ((menuLoopCount & 0xF00) == 0x100);
			duty[0] = (flash && (menuOption == 0)) ? PWM_MAX : ((_options & (1<<0)) ? PWM_DIM : 0);
			duty[1] = (flash && (menuOption == 1)) ? PWM_MAX : ((_options & (1<<1)) ? PWM_DIM : 0);
			duty[2] = (flash && (menuOption == 2)) ? PWM_MAX : ((_options & (1<<2)) ? PWM_DIM : 0);
			duty[3] = (flash && (menuOption == 3)) ? PWM_MAX : ((_options & (1<<3)) ? PWM_DIM : 0);
			duty[4] = (flash && (menuOption == 4)) ? PWM_MAX : ((_options & (1<<4)) ? PWM_DIM : 0);
			duty[5] = maxDuty;
		}
		else 
		// SPLIT-ONLY MODE
		if(MODE_NOCLOCK == _mode)
		{
			// animation is driven from MIDI thru function,
			// but we'll fade the LEDs in this main loop
			if(systemTicks > nextFade)
			{
				for(byte i=0;i<6;++i) 
					if(duty[i]) 
						--duty[i];
				nextFade = systemTicks + FADE_PERIOD;
			}			
		}
		else
		// STEP/TAP MODE
		if(tick_flag)
		{
			tick_flag = 0;	
			if(++tickCount > 23)
			{
				if(midiRestart)
				{
					send(MIDI_SYNCH_START);
					midiRestart = 0;
				}
				tickCount = 0;
			}
			if(running)
				send(MIDI_SYNCH_TICK);
		
			// mid-tap entry?
			if(tapCount)
			{
				// illuminate LEDs for tap tempo entry
				duty[0] = PWM_MAX;
				duty[1] = (tapCount > 1) ? maxDuty : 0;
				duty[2] = (tapCount > 2) ? maxDuty : 0;
				duty[3] = (tapCount > 3) ? maxDuty : 0;
				duty[4] = (tapCount > 4) ? maxDuty : 0;
				duty[5] = (tapCount > 5) ? maxDuty : 0;
				
				// exit tap temp entry if it has been more than 
				// 1 second since the last valid tap
				if(systemTicks - lastTapSystemTicks > 1000)
				{
					lastTapSystemTicks = 0;
					firstTapSystemTicks = 0;
					tapCount = 0;
				}
			}			
			else if(running)
			{
				if(_options & OPTION_DISCREET)
				{
					duty[0] = maxDuty;
					duty[1] = 0;
					duty[2] = 0;
					duty[3] = 0;
					duty[4] = 0;
					duty[5] = (tickCount == 1)? maxDuty : 0;
				}
				else
				{				
					// Cycling "running" animation
					byte whichLED = tickCount/4;				
					duty[0] = (whichLED == 0)? maxDuty : 0;
					duty[1] = (whichLED == 1)? maxDuty : 0;
					duty[2] = (whichLED == 2)? maxDuty : 0;
					duty[3] = (whichLED == 3)? maxDuty : 0;
					duty[4] = (whichLED == 4)? maxDuty : 0;
					duty[5] = (whichLED == 5)? maxDuty : 0;
				}
			}
			else 
			{
				if(_options & OPTION_DISCREET)
				{
					duty[0] = 0;
					duty[1] = 0;
					duty[2] = 0;
					duty[3] = 0;
					duty[4] = 0;
					duty[5] = tickCount? 0 : maxDuty;
				}
				else
				{
					// Flashing "paused" indicator (leds 0,1,4,5)
					duty[0] = tickCount? 0 : maxDuty;
					duty[1] = tickCount? 0 : maxDuty;
					duty[2] = 0;
					duty[3] = 0;
					duty[4] = tickCount? 0 : maxDuty;
					duty[5] = tickCount? 0 : maxDuty;				
				}
			}			
		}	

		// PWM the LEDs
		P_LED0 = (duty[0]>pwm);
		P_LED1 = (duty[1]>pwm);
		P_LED2 = (duty[2]>pwm);
		P_LED3 = (duty[3]>pwm);
		P_LED4 = (duty[4]>pwm);
		P_LED5 = (duty[5]>pwm);
		if(++pwm > PWM_MAX) 
			pwm=0;

		// HANDLE USER INPUT
		if(systemTicks >= debouncePeriodEnd) // check not debouncing
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
				if(thisButtonStatus && systemTicks > autoRepeatBegin)
				{
					if(!nextAutoRepeat)
					{
						thisButtonStatus = thisButtonStatus|M_LONG_PRESS;
						buttonsPressed = thisButtonStatus;
						nextAutoRepeat = systemTicks + AUTO_REPEAT_INTERVAL;
					}
					else if(systemTicks > nextAutoRepeat)
					{
						thisButtonStatus = thisButtonStatus|M_AUTO_REPEAT;
						buttonsPressed = thisButtonStatus;
						nextAutoRepeat = systemTicks + AUTO_REPEAT_INTERVAL;
					}
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
					nextAutoRepeat = 0;
				}
				debouncePeriodEnd = systemTicks + DEBOUNCE_PERIOD;
			}
			
			// any new button presses or auto repeats?
			if(buttonsPressed)
			{
				// execute the command
				switch(thisButtonStatus)
				{
					////////////////////////////////////////////////////////////
					// ALL BUTTONS PRESSED TOGETHER (MENU MODE)
					case M_BUTTON_RUN|M_BUTTON_DEC|M_BUTTON_INC:
						menuOption = 0;
						_mode = MODE_MENU;
						break;
						
					////////////////////////////////////////////////////////////
					// DEC AND RUN PRESSED TOGETHER (TAP TEMPO MODE)
					case M_BUTTON_RUN|M_BUTTON_DEC:
						_mode = MODE_TAP;
						break;
						
					////////////////////////////////////////////////////////////
					// INC AND RUN PRESSED TOGETHER (SPLIT ONLY MODE)
					case M_BUTTON_RUN|M_BUTTON_INC:
						_mode = MODE_NOCLOCK;
						break;

					////////////////////////////////////////////////////////////
					// INC AND DEC PRESSED TOGETHER (DEFAULT BPM)
					case M_BUTTON_INC|M_BUTTON_DEC:
						if(MODE_STEP == _mode)
							setBPM(BPM_DEFAULT);
						break;
						
					////////////////////////////////////////////////////////////
					// RUN PRESSED
					case M_BUTTON_RUN:				
						if(MODE_TAP == _mode || MODE_STEP == _mode)
						{
							if(runLock)
							{
								midiRestart = 1;
							}
							else
							{
								// When clock is enabled we toggle run/paused
								running = !running;
								if(_options & OPTION_STARTSTOP)
								{
									if(running)
									{
										tickCount = 0;
										send(MIDI_SYNCH_START);
									}
									else
									{
										send(MIDI_SYNCH_STOP);			
									}
								}
							}
						}						
						else
						if(MODE_NOCLOCK == _mode) // SPLIT MODE
						{
							send(MIDI_SYNCH_START);
							running = 1;
						}
						else 
						if(MODE_MENU == _mode)
						{
							// Exits from menu mode
							_mode = MODE_STEP;
							running = 0;
						}
						break;

					case M_BUTTON_RUN|M_LONG_PRESS:										
						if(!runLock)
						{
							runLock = 1;
							running = 1;
						}
						else
						{
							runLock = 0;
						}
						break;
						
					////////////////////////////////////////////////////////////
					// DEC PRESSED
					case M_BUTTON_DEC:
						if(MODE_MENU == _mode)
						{							
							if(5 == menuOption)
							{
								_brightness = (_brightness + 1) % NUM_BRIGHTNESS_LEVELS;
								maxDuty = brightnessLevels[_brightness];
							}
							else
							{
								// In menu mode, toggles options on/off
								_options ^= (1<<menuOption);
							}							
							saveOptions();
							break;
						}
						else 
						if(MODE_NOCLOCK == _mode)
						{
							// Exits split-only mode
							_mode = MODE_STEP;
							break;
						}
						else 
						if(MODE_TAP == _mode)
						{						
							// In tap mode, counts a "tap"
							if(!tapCount)
							{
								tapCount = 1;
								firstTapSystemTicks = systemTicks;
							}
							else
							if(tapCount < 6 && systemTicks > firstTapSystemTicks)//rollover check
							{
								unsigned long period = systemTicks - firstTapSystemTicks;
								period = period / tapCount;
								setBPM(60000UL / period);
								tapCount++;
							}
							lastTapSystemTicks = systemTicks;
							break;
						}//fallthru
					case M_LONG_PRESS|M_BUTTON_DEC:
					case M_AUTO_REPEAT|M_BUTTON_DEC:
						if(MODE_STEP == _mode)
							setBPM(_bpm-1);												
						break;
						
					////////////////////////////////////////////////////////////
					// INC PRESSED
					case M_BUTTON_INC:
						if(MODE_MENU == _mode)
						{
							// menu mode - select menu option
							menuOption = (menuOption+1) % MENU_SIZE;
							break;
						}
						else 
						if(MODE_NOCLOCK == _mode) // SPLIT MODE
						{
							if(running)
							{
								send(MIDI_SYNCH_STOP);
								running = 0;
							}
							else
							{
								send(MIDI_SYNCH_CONTINUE);
								running = 1;
							}						
						}
						if(MODE_TAP == _mode)
						{
							// tap mode - exits tap mode
							_mode = MODE_STEP;
							break;
						}//fallthru
					case M_LONG_PRESS|M_BUTTON_INC:
					case M_AUTO_REPEAT|M_BUTTON_INC:
						if(MODE_STEP == _mode)
							setBPM(_bpm+1);												
						break;
				
				}				
			}
		}			
	}
}
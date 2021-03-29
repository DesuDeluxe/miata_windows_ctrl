/*
 * main.c
 *
 *  Created on: Jan 18, 2021
 *      Author: desu
 */
#define F_CPU 8000000UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

//8mhz internal
//avrdude -pm328p -cusbasp -U lfuse:w:0xe2:m -U hfuse:w:0xd9:m -U efuse:w:0xff:m

#define CNT_T_SEC 4

#define DEBOUNCE_TIME 40
// 

#define WINPINS_SHIFT 1
#define GNDPINS_SHIFT 2


#define WINDOW_SW1_UP_PIN  PIND1//window 1 switch up
#define WINDOW_SW1_DWN_PIN PIND2
#define WINDOW_SW2_UP_PIN PIND3
#define WINDOW_SW2_DWN_PIN PIND4


#define DSW1_PIN PINC2 //door switch to ground (door open)
#define DSW2_PIN PINC3

#define ASW_PIN PINC4//signal form alarm to close windows - 20s ground
#define TSW_PIN PINC5//soft top down switch


#define M1_CCW PORTB |= (1 << PORTB0); PORTD &= ~(1 << PORTD7)
#define M1_CW PORTB &= ~(1 << PORTB0); PORTD |= (1 << PORTD7)
#define M1_STOP PORTB &= ~(1 << PORTB0); PORTD &= ~(1 << PORTD7)


#define M2_CCW PORTD |= (1 << PORTD6); PORTD &= ~(1 << PORTD5)
#define M2_CW PORTD &= ~(1 << PORTD6); PORTD |= (1 << PORTD5)
#define M2_STOP PORTD &= ~(1 << PORTD6); PORTD &= ~(1 << PORTD5)

#define WINPINS ((1 << DDD1) | (1 << DDD2) | (1 << DDD3) | (1 << DDD4))
#define GNDSWPINS ((1 << DDC2) | (1 << DDC3) | (1 << DDC4) | (1 << DDC5))



typedef enum {
	SW_TAP_D = 1,
	DOOR_OC_D = 2,
	SW_HOLD_D = 4*CNT_T_SEC
} motor_delay_t;



typedef enum {
	W_OPEN = 0,
	W_CLOSED = 1,
	W_UNNKNOWN = 2
} window_state_t;

volatile union {
 uint8_t all;
 struct {
  uint8_t wsw1up :1;
  uint8_t wsw1dwn :1;
  uint8_t wsw2up :1;
  uint8_t wsw2dwn :1;
	uint8_t soft :1;
	uint8_t alarm :1;
	uint8_t door2 :1;
	uint8_t door1 :1;
 };
} INPUTS;

volatile union {
 uint8_t all;
 struct {
  uint8_t wsw1up :1;
  uint8_t wsw1dwn :1;
  uint8_t wsw2up :1;
  uint8_t wsw2dwn :1;
 };
} WSW_IN;


volatile uint8_t TIMER_COUNT = 0;
volatile uint8_t SW_STATE_CHANGED_TIME = 0;
volatile uint8_t SW_STATE_CHANGED = 0;
volatile uint8_t IN_STATE_CHANGED = 0;
uint8_t SW_DEBOUNCED_CNT = 0;

void set_motors();


ISR(PCINT1_vect)
{
	INPUTS.soft = (~PINC & (1 << TSW_PIN)) >> TSW_PIN;
	INPUTS.alarm = (~PINC & (1 << ASW_PIN)) >> ASW_PIN;
	INPUTS.door2 = (~PINC & (1 << DSW2_PIN)) >> DSW2_PIN;
	INPUTS.door1 = (~PINC & (1 << DSW1_PIN)) >> DSW1_PIN;

	SW_STATE_CHANGED_TIME = TIMER_COUNT;
	IN_STATE_CHANGED = 1;

}

ISR(PCINT2_vect)
{
	SW_STATE_CHANGED_TIME = TIMER_COUNT;
	SW_STATE_CHANGED += 1;
}

volatile uint8_t test = 0;

ISR(TIMER1_COMPA_vect)
{
	TIMER_COUNT++;
}


void initialize(){


	PORTB = 0x0;
	PORTC = 0x0;
	PORTD = 0x0;

	//inputs
	DDRD &= ~WINPINS;
	DDRC &= ~GNDSWPINS;

	//pin change interrupts
	PCICR |= (1 << PCIE1) | (1 << PCIE2);
	PCMSK2 |= ((1 << PCINT17) | (1 << PCINT18) | (1 << PCINT19) | (1 << PCINT20));
	PCMSK1 |= ((1 << PCINT10) | (1 << PCINT11) | (1 << PCINT12) | (1 << PCINT13));

	//outputs
	DDRB |= (1 << DDB0);
	DDRD |= ((1 << DDD5) | (1 << DDD6) | (1 << DDD7));

	//pull-ups
	PORTB = 0xFF & ~(1 << PORTB0);
	PORTC = 0xFF;
	PORTD = 0xFF & ~((1 << PORTD5) | (1 << PORTD6) | (1 << PORTD7));

	//timer
	TCCR1B |= (1<<WGM12);//CTC mode
	TCNT1 = 0;//initializing timer
	OCR1A = 7813;//preloading timer so that it can count with ~250ms
	//OCR1A = 15625;//preloading timer so that it can count with ~500ms
	//OCR1A = 31250;//preloading timer so that it can count with ~1 sec
	TIMSK1 |= (1<<OCIE1A);//Timer compare A Match interrupt
	TCCR1B |= (1<<CS12);//Starting timer and clock prescaler(256)

	M1_STOP;
	M2_STOP;

	sei();                     // turn on interrupts


}


void debounceSw()
{
	WSW_IN.wsw1up = (~PIND & (1 << WINDOW_SW1_UP_PIN)) >> WINDOW_SW1_UP_PIN;
	WSW_IN.wsw1dwn = (~PIND & (1 << WINDOW_SW1_DWN_PIN)) >> WINDOW_SW1_DWN_PIN;
	WSW_IN.wsw2up = (~PIND & (1 << WINDOW_SW2_UP_PIN)) >> WINDOW_SW2_UP_PIN;
	WSW_IN.wsw2dwn = (~PIND & (1 << WINDOW_SW2_DWN_PIN)) >> WINDOW_SW2_DWN_PIN;
	//if(!WSW_IN.all)
	//	return 0;
	_delay_ms(DEBOUNCE_TIME);
	WSW_IN.wsw1up = (~PIND & (1 << WINDOW_SW1_UP_PIN)) >> WINDOW_SW1_UP_PIN;
	WSW_IN.wsw1dwn = (~PIND & (1 << WINDOW_SW1_DWN_PIN)) >> WINDOW_SW1_DWN_PIN;
	WSW_IN.wsw2up = (~PIND & (1 << WINDOW_SW2_UP_PIN)) >> WINDOW_SW2_UP_PIN;
	WSW_IN.wsw2dwn = (~PIND & (1 << WINDOW_SW2_DWN_PIN)) >> WINDOW_SW2_DWN_PIN;
	SW_DEBOUNCED_CNT +=1;

	SW_STATE_CHANGED = 0;

}



uint8_t wsw1_start_time = 0;
uint8_t wsw2_start_time = 0;

uint8_t motor1_start_time = 0;
uint8_t motor2_start_time = 0;
uint8_t motor1_work_time = 0;
uint8_t motor2_work_time = 0;
uint8_t window1_door_state = W_CLOSED;
uint8_t window2_door_state = W_CLOSED;
uint8_t ticks =0;

void set_motors(){

	if(WSW_IN.wsw1up){
		wsw1_start_time = TIMER_COUNT;
		M1_CW;
		window1_door_state = W_UNNKNOWN;
		motor1_start_time = TIMER_COUNT;
		motor1_work_time = SW_TAP_D;
	}
	else if(WSW_IN.wsw1dwn){
		wsw1_start_time = TIMER_COUNT;
		M1_CCW;
		window1_door_state = W_UNNKNOWN;
		motor1_start_time = TIMER_COUNT;
		motor1_work_time = SW_TAP_D;
	}
	//else if(!WSW_IN.wsw1up || !WSW_IN.wsw1dwn){
	else{
		//uint8_t t = TIMER_COUNT
		if((TIMER_COUNT - wsw1_start_time) > 3)
			motor1_work_time = SW_HOLD_D;
	}


	if(WSW_IN.wsw2up){
		wsw2_start_time = TIMER_COUNT;
		M2_CW;
		window2_door_state = W_UNNKNOWN;
		motor2_start_time = TIMER_COUNT;
		motor2_work_time = SW_TAP_D;
	}
	else if(WSW_IN.wsw2dwn){
		wsw2_start_time = TIMER_COUNT;
		M2_CCW;
		window2_door_state = W_UNNKNOWN;
		motor2_start_time = TIMER_COUNT;
		motor2_work_time = SW_TAP_D;
	}
	//else if(!WSW_IN.wsw2up || !WSW_IN.wsw2dwn){
	else{
		//uint8_t t = TIMER_COUNT
		if((TIMER_COUNT - wsw2_start_time) > 3)
			motor2_work_time = SW_HOLD_D;
	}
}

int main(void) {

	initialize();
	INPUTS.all = 0x0;
	WSW_IN.all = 0x0;
	wsw1_start_time = 0;
	wsw2_start_time = 0;

	motor1_start_time = 0;
	motor2_start_time = 0;
	motor1_work_time = 0;
	motor2_work_time = 0;
	window1_door_state = W_CLOSED;
	window2_door_state = W_CLOSED;
	ticks = 0;
	uint8_t sw_prev_state = 0;

	(void)sw_prev_state;

	while (1){

		if(SW_STATE_CHANGED){
			debounceSw();
			set_motors();
			IN_STATE_CHANGED=0;
		}

		else if(IN_STATE_CHANGED){

			if(INPUTS.alarm && !INPUTS.soft && (window1_door_state != W_CLOSED)){
				M1_CW;
				window1_door_state = W_CLOSED;
				motor1_work_time = SW_HOLD_D;
				motor1_start_time = TIMER_COUNT;
			}

			if(INPUTS.alarm && !INPUTS.soft && (window2_door_state != W_CLOSED)){
				M2_CW;
				window2_door_state = W_CLOSED;
				motor2_work_time = SW_HOLD_D;
				motor2_start_time = TIMER_COUNT;
			}

			if(INPUTS.door1 && (window1_door_state == W_CLOSED)){
				M1_CCW;
				window1_door_state = W_OPEN;
				motor1_start_time = TIMER_COUNT;
				motor1_work_time = DOOR_OC_D;
			}
			else if((!INPUTS.door1) && (window1_door_state == W_OPEN)){
				M1_CW;
				window1_door_state = W_CLOSED;
				motor1_start_time = TIMER_COUNT;
				motor1_work_time = DOOR_OC_D;
			}

			if(INPUTS.door2 && (window2_door_state == W_CLOSED)){
				M2_CCW;
				window2_door_state = W_OPEN;
				motor2_start_time = TIMER_COUNT;
				motor2_work_time = DOOR_OC_D;
			}
			else if((!INPUTS.door2) && (window2_door_state == W_OPEN)){
				M2_CW;
				window2_door_state = W_CLOSED;
				motor2_start_time = TIMER_COUNT;
				motor2_work_time = DOOR_OC_D;
			}
		}



		//stop motors
		if( !(WSW_IN.wsw1up || WSW_IN.wsw1dwn) && ( (TIMER_COUNT - motor1_start_time) > motor1_work_time) ){
			M1_STOP;
		}
		if( !(WSW_IN.wsw2up || WSW_IN.wsw2dwn) && ( (TIMER_COUNT - motor2_start_time) > motor2_work_time) ){
			M2_STOP;
		}

		//go to sleep
		if(TIMER_COUNT - SW_STATE_CHANGED_TIME > 60*CNT_T_SEC){
			M1_STOP;
			M2_STOP;
			set_sleep_mode (SLEEP_MODE_PWR_DOWN);
			sleep_mode();
		}
		//_delay_ms(50);
		}
	return 0;
}

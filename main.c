#define F_CPU 16000000UL // 16MHz

//#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/eeprom.h>
//#include <math.h>

// define speaker
#define SPEAKER_P PB3
#define SPEAKER_DDR DDRB
#define SPEAKER_PORT PORTB
#define SPEAKER_PIN PINB

// define button
#define BUT_PORT PORTB
#define BUT_DDR DDRB
#define BUT_PIN PINB
#define BUT_START PB1
#define BUT_PAUSE PB2

// define uleds
#define ULEDS_PORT PORTB
#define ULEDS_DDR DDRB
#define ULEDS_P PB4
	
// define leds
#define LED_DDR DDRD
#define LED_PORT PORTD
#define LED_PIN PIND
#define LED_P PD6

// define disp
#define HC595_SHCP PD3
#define HC595_STCP PD4
#define HC595_DATA PD5
#define DISP_SHCP_high() PORTD |= 1<<HC595_SHCP;
#define DISP_SHCP_low() PORTD &= ~(1<<HC595_SHCP);
#define DISP_STCP_high() PORTD |= 1<<HC595_STCP;
#define DISP_STCP_low() PORTD &= ~(1<<HC595_STCP);
#define DISP_DATA_high() PORTD |= 1<<HC595_DATA;
#define DISP_DATA_low() PORTD &= ~(1<<HC595_DATA);

/*struct DISTANCE {
	uint8_t higher_two;
	uint16_t high_four;
	uint16_t low_four;
};

struct SPEED {
	uint8_t high;
	uint8_t low;
};
*/

void init(void);
void display_bits(uint32_t disp);
void display_digit(uint16_t digit, uint8_t dot);
void timer0_init(void);
void timer0_start(void);
void timer0_stop(void);
void timer1_init(void);
void timer1_start(void);
void timer1_stop(void);
void clear_button_flags(void);
uint16_t secs_to_clock(uint16_t secs);



// VARIABLES
//
uint8_t symbol[] = {
	0b11111100,		//0
	0b01100000,		//1
	0b11011010,		//2
	0b11110010,		//3
	0b01100110,		//4
	0b10110110,		//5
	0b10111110,		//6
	0b11100000,		//7
	0b11111110,		//8
	0b11110110,		//9
	0b00000000		// NONE
};

uint64_t text[7] = {
	/*0b100111001111110010011110100011100000000000000000, // DISP_COEF
	0b101101101100111010011110011110100001000001101110, // DISP_SPEd_H
	0b101101101100111010011110011110100001000000011100, // DISP_SPEd_L
	0b101101101100111000010000101101100110000010111100, // DISP_SP_SIG
	0b110011001110010011111100011110101001111000000000, // DISP_MOdE
	0b011110101001111010001110000111000001111000000000,	// DISP_dEFLt
	0b000100000001000001111010001110100010101010011110	// DISP_donE
	*/
};

/*uint16_t EEMEM e_speed_max = 5100;
uint16_t EEMEM e_beeper_speed_interval = 500;*/

uint16_t EEMEM e_time;

struct FLAG {
	unsigned timeout : 1;
	unsigned update : 1;
	unsigned but_start : 1;
	unsigned but_pause : 1;
} flag;



uint16_t time;


int main(void)
{
	init();

	// READ EEPROM
	time = eeprom_read_word(&e_time);

	// UPDATE DISPLAY
	display_digit(secs_to_clock(time), 2);

	while (1)
	{
		if (flag.update)
		{
			flag.update = 0;
			display_digit(secs_to_clock(time), 2);
			if (flag.timeout)
			{
				flag.timeout = 0;
				ULEDS_PORT &= ~(1<<ULEDS_P); // disable uleds
			}
		}

		if (flag.but_start)
		{
			clear_button_flags();
			ULEDS_PORT |= 1<<ULEDS_P; // enable uleds
			timer1_start();
		}

		if (flag.but_pause)
		{
			clear_button_flags();
			timer1_stop();
		}

		// DELAY
		_delay_ms(10);
	}

	return 0;
}


void clear_button_flags(void)
{
	//flag.but_mode = 0;
}


ISR(TIMER0_OVF_vect)
{
	timer0_stop();
	GIMSK = 1<<PCIE; // Enable Pin Change Interrupts
}

ISR(TIMER1_COMPA_vect)
{
	time--;
	flag.update = 1;

	if (~(time)) flag.timeout = 1;
}

ISR(PCINT_vect)
{
	uint8_t cur_buttons = BUT_PIN;
	GIMSK = 0<<PCIE; // Disable Pin Change Interrupts
	if ((cur_buttons & (1<<BUT_START | 1<<BUT_PAUSE)) != (1<<BUT_START | 1<<BUT_PAUSE))
	{
		if (cur_buttons & 1<<BUT_START)
		{
			flag.but_start = 1;
		}
		if (cur_buttons & 1<<BUT_PAUSE)
		{
			flag.but_pause = 1;
		}

		timer0_start();
	}
}


void init(void)
{
	DDRD = 0b1111<<3;
	DDRB = 0b11111<<3;
	PORTD = 0;
	PORTB = 0;

	timer0_init();
	timer1_init();

	// TIMER for LEDS and buttons interrupts delay
	//timer2_init();
	//timer2_start();

	// READ EEPROM
	/*
	speed_max = eeprom_read_word(&e_speed_max);
	speed_min = eeprom_read_word(&e_speed_min);
	beeper_speed_interval = eeprom_read_word(&e_beeper_speed_interval);
	koef5w = eeprom_read_word(&e_koef5w);
	mode_general = eeprom_read_byte(&e_mode);
	*/

	// external interrupts
	GIMSK = 1<<PCIE; // enable external interrupts

	// ANALOG COMPARATOR DISABLE
	ACSR = 1<<ACD;

	// PIN CHANGE INTERRUPTS INIT
	PCMSK = 0b111; // MASK

	//mode_set(mode_general);

	// ENABLE GLOBAL INTERRUPTS
	sei();
}

void display_bits(uint32_t disp)
{
	uint8_t i;
	DISP_SHCP_low();
	DISP_STCP_low();
	for (i=24; i>0; i--) {
		if (disp & 1<<0) DISP_DATA_high()
		else DISP_DATA_low();
		disp>>=1;
		DISP_SHCP_high();
		DISP_SHCP_low();
	}
	DISP_STCP_high();
	DISP_STCP_low();
}


void display_digit(uint16_t digit, uint8_t dot)
{
	uint8_t dig[3];
	dig[2] = digit / 100; digit %= 100;
	dig[1] = digit / 10; digit %= 10;
	dig[0] = digit; 

	/*uint8_t i;
	for (i = 2; i > 0; i--)
	{
		if (dig[i] == 0) dig[i] = 10;
		else break;
	}*/

	uint8_t i;
	for (i = 0; i < 3; i++)
		dig[i] = symbol[dig[i]];

	// add dot
	if (dot !=  255) dig[dot] = dig[dot] + 0b1;

	//rere = (uint64_t)dig_1<<40 | (uint64_t)dig_2<<32 | (uint64_t)dig_3<<24 | (uint64_t)dig_4<<16 | (uint64_t)dig_5<<8 | (uint64_t)dig_6;
	uint32_t rere = (uint32_t)dig[2]<<16|(uint32_t)dig[1]<<8|(uint32_t)dig[0];
	display_bits(rere);
}

// TIMER0
void timer0_init(void)
{
	TIMSK |= 1<<TOIE0; // output compare A match interrupt enable
	//TCCR0A = 1<<WGM11; // CTC mode
	//TCNT0 = 0;
}
void timer0_start(void)
{
	TCCR0B |= (1<<CS02 | 0<<CS01 | 1<<CS00); // prescaler = 1024
}
void timer0_stop(void)
{
	TCCR0B &= ~(1<<CS02 | 1<<CS01 | 1<<CS00);
}


// TIMER1
void timer1_init(void)
{
	TIMSK |= 1<<OCIE1A;	// output compare A match interrupt enable
	TCCR1A = 0;
	TCCR1B = 1<<WGM12;		// set CTC mode

	OCR1AH = (((15625)>>8) & 0xff); // prescaler for 1 second interrupt
	OCR1AL = (15625 & 0xff);
}
void timer1_start(void)
{
	TCCR1B |= (1<<CS12 | 0<<CS11 | 0<<CS10); // prescaler = 256
}
void timer1_stop(void)
{
	TCCR1B &= ~(1<<CS12 | 1<<CS11 | 1<<CS10);
}

uint16_t secs_to_clock(uint16_t secs)
{
	if (secs > (9*60+59)) return 0;
	uint8_t mins = secs / 60;
	secs %= 60;
	return mins * 100 + secs;
}

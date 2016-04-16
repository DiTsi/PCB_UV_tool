#define F_CPU 4000000UL // 16MHz

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
#define BUT_CONF PB0

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
void set_time(void);
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

uint32_t text[1] = {
	//0b100111001111110010011110100011100000000000000000, // DISP_COEF
	//0b101101101100111010011110011110100001000001101110, // DISP_SPEd_H
	//0b101101101100111010011110011110100001000000011100, // DISP_SPEd_L
	//0b101101101100111000010000101101100110000010111100, // DISP_SP_SIG
	//0b110011001110010011111100011110101001111000000000, // DISP_MOdE
	//0b011110101001111010001110000111000001111000000000,	// DISP_dEFLt
	0b101101101110111001111100	// DISP_SAV
};

/*uint16_t EEMEM e_speed_max = 5100;
uint16_t EEMEM e_beeper_speed_interval = 500;*/

uint16_t EEMEM e_time;

struct FLAG {
	unsigned timeout : 1;
	unsigned work : 1;
	unsigned conf : 1;
	unsigned update : 1;
	unsigned but_start : 1;
	unsigned but_pause : 1;
	unsigned but_conf : 1;
	unsigned pause : 1;
} flag;



uint16_t time;
uint8_t timer0_counter;


int main(void)
{
	init();

	// READ EEPROM
	set_time();

	// UPDATE DISPLAY
	display_bits(0b1);

	while (1)
	{
		if (flag.update)
		{
			flag.update = 0;
			display_digit(secs_to_clock(time), 2);
			if (flag.timeout)
			{
				flag.timeout = 0;
				timer1_stop();
				ULEDS_PORT &= ~(1<<ULEDS_P); // disable uleds

				SPEAKER_PORT |= 1<<SPEAKER_P;
				_delay_ms(40);
				SPEAKER_PORT &= ~(1<<SPEAKER_P);
				_delay_ms(100);
				SPEAKER_PORT |= 1<<SPEAKER_P;
				_delay_ms(40);
				SPEAKER_PORT &= ~(1<<SPEAKER_P);


				_delay_ms(1500);
				set_time();
				display_bits(0b1);

				flag.work = 0;
			}
		}


		if (flag.work ^ 0b1)
		{
			if (flag.but_conf)
			{
				clear_button_flags();

				if (flag.conf)
				{
					eeprom_update_word(&e_time, time);
					set_time();
					display_bits(text[0]);
					_delay_ms(1000);
					display_bits(0b1);
					flag.conf = 0;
				}
				else
				{
					flag.conf = 1;
					display_digit(secs_to_clock(time), 2);
				}
			}
		}

		if (flag.but_start)
		{
			clear_button_flags();
			if (flag.conf)
			{
				time -= 5;
				if (time < 30) time = 30;
				display_digit(secs_to_clock(time), 2);
			}
			else
			{
				display_digit(secs_to_clock(time), 2);
				flag.work = 1;
				ULEDS_PORT |= 1<<ULEDS_P; // enable uleds
				timer1_start();
	
				if (flag.pause) 
				{
					flag.pause = 0;
					LED_PORT &= ~(1<<LED_P);
				}
			}
		}

		if (flag.but_pause)
		{
			clear_button_flags();
			if (flag.conf)
			{
				time += 5;
				if (time > 600) time = 600;
				display_digit(secs_to_clock(time), 2);
			}
			else
			{
				if (flag.work)
				{
					if (flag.pause)
					{
						flag.pause = 0;
						LED_PORT &= ~(1<<LED_P);
						set_time();
						TCNT1 = 0;
						display_bits(0b1);
		
						flag.work = 0;
					}
					else
					{
						flag.pause = 1;
						timer1_stop();
						LED_PORT |= (1<<LED_P);
						ULEDS_PORT &= ~(1<<ULEDS_P); // disable uleds
					}
				}
			}
		}

		// DELAY
		_delay_ms(10);
	}

	return 0;
}


void clear_button_flags(void)
{
	flag.but_start = 0;
	flag.but_pause = 0;
	flag.but_conf = 0;
}


ISR(TIMER0_OVF_vect)
{
	timer0_counter--;
	if (timer0_counter == 0)
	{
		timer0_stop();
		GIMSK = 1<<PCIE; // Enable Pin Change Interrupts
	}
}

ISR(TIMER1_COMPA_vect)
{
	time--;
	flag.update = 1;

	if (time == 0) flag.timeout = 1;
}

ISR(PCINT_vect)
{
	GIMSK = 0<<PCIE; // Disable Pin Change Interrupts
	timer0_start();
	uint8_t cur_buttons = BUT_PIN;
	if ((cur_buttons & (1<<BUT_START | 1<<BUT_PAUSE | 1<<BUT_CONF)) != (1<<BUT_START | 1<<BUT_PAUSE | 1<<BUT_CONF))
	{

		if ((cur_buttons & 1<<BUT_CONF) != (1<<BUT_CONF))
		{
			flag.but_conf = 1;
		}
		if ((cur_buttons & 1<<BUT_START) != (1<<BUT_START))
		{
			flag.but_start = 1;
		}
		if ((cur_buttons & 1<<BUT_PAUSE) != (1<<BUT_PAUSE))
		{
			flag.but_pause = 1;
		}
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

	// external interrupts
	GIMSK = 1<<PCIE; // enable external interrupts

	// ANALOG COMPARATOR DISABLE
	ACSR = 1<<ACD;

	// PIN CHANGE INTERRUPTS INIT
	PCMSK = 1<<BUT_START | 1<<BUT_PAUSE | 1<<BUT_CONF; // MASK

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
	timer0_counter = 2;
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

void set_time(void)
{
	time = eeprom_read_word(&e_time);
	if ((time>600) || (time<30)) time = 71; // "111" on disp - eeprom info error
}

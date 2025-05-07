#line 1 "C:\\Users\\buechel\\HiDrive\\privat\\Arduino\\Artisan_Roaster_Arduino\\TriacDimmer.cpp"
#include "TriacDimmer.h"
#include <Arduino.h>
#include <assert.h>
#include <util/atomic.h>

uint16_t TriacDimmer::detail::pulse_length;
uint16_t TriacDimmer::detail::min_trigger;
float TriacDimmer::detail::on_thresh;
float TriacDimmer::detail::off_thresh;

volatile bool TriacDimmer::detail::ch_A_en;
volatile uint16_t TriacDimmer::detail::ch_A_up;
volatile uint16_t TriacDimmer::detail::ch_A_dn;
volatile uint16_t ch_A_dn_buf;

volatile bool TriacDimmer::detail::ch_B_en;
volatile uint16_t TriacDimmer::detail::ch_B_up;
volatile uint16_t TriacDimmer::detail::ch_B_dn;
volatile uint16_t ch_B_dn_buf;

uint16_t last_icr = 0;
volatile uint16_t TriacDimmer::detail::period = 20000;

volatile int8_t curr;		// current value in N:M sequence
volatile bool newN = true;	// signals that output level has changed
volatile int8_t duty_2 = 0; // in %

uint16_t phase_delay[101] = { // 50Hz values based on linearizing power output
							  /* 0      1      2      3      4      5      6     7      8       9 */
	/* 00 */ 20000, 17680, 17061, 16621, 16265, 15961, 15692, 15448, 15225, 15017,
	/* 10 */ 14822, 14638, 14462, 14295, 14134, 13979, 13830, 13685, 13544, 13407,
	/* 20 */ 13274, 13143, 13016, 12891, 12768, 12647, 12529, 12412, 12297, 12184,
	/* 30 */ 12072, 11961, 11851, 11743, 11636, 11529, 11423, 11319, 11215, 11111,
	/* 40 */ 11008, 10906, 10804, 10703, 10602, 10501, 10401, 10300, 10200, 10100,
	/* 50 */ 10000, 9900, 9800, 9700, 9599, 9499, 9398, 9297, 9196, 9094,
	/* 60 */ 8992, 8889, 8785, 8681, 8577, 8471, 8364, 8257, 8149, 8039,
	/* 70 */ 7928, 7816, 7703, 7588, 7471, 7353, 7232, 7109, 6984, 6857,
	/* 80 */ 6726, 6593, 6456, 6315, 6170, 6021, 5866, 5705, 5538, 5362,
	/* 90 */ 5178, 4983, 4775, 4552, 4308, 4039, 3735, 3379, 2939, 2320,
	/* 100 */ 0};

void TriacDimmer::begin(uint16_t pulse_length, uint16_t min_trigger, float on_thresh, float off_thresh)
{
	// Don't need atomic writes since the ISRs haven't started yet.
	TriacDimmer::detail::pulse_length = pulse_length;
	TriacDimmer::detail::min_trigger = min_trigger;
	TriacDimmer::detail::on_thresh = on_thresh;
	TriacDimmer::detail::off_thresh = off_thresh;

	TCCR1A = 0;
	TCCR1B = _BV(ICNC1)	  // input capture noise cancel
			 | _BV(CS11); // /8 prescaler
						  //			| _BV(ICES1) //positive edge --> we get a negative edge on the zero crossing

	pinMode(8, INPUT);

	TIFR1 = _BV(ICF1);	 // clear IC interrupt flag
	TIMSK1 = _BV(ICIE1); // enable input capture interrupt
}

void TriacDimmer::end()
{
	TIMSK1 = 0;	  // disable the interrupts first!
	TIFR1 = 0xFF; // clear all flags
	TCCR1A = 0;	  // clear to reset state
	TCCR1B = 0;
}

void TriacDimmer::setICC(uint8_t value)
{
	duty_2 = constrain(value, 0, 100);
	newN = true;
	pinMode(ICC_PIN, OUTPUT); // only set to output once configured.
}

void TriacDimmer::setDuty(uint8_t pin, uint8_t value)
{
float temp_f;
uint8_t temp_on,temp_off;
	assert(pin == 9 || pin == 10);
    value = constrain(value,0,100);
    temp_f   = TriacDimmer::detail::on_thresh * 100;
	temp_on  = temp_f;
    temp_f   = TriacDimmer::detail::off_thresh * 100;
	temp_off = temp_f;
	if (value > temp_on)
	{
		digitalWrite(pin, HIGH);
		TriacDimmer::disable(pin);
	}
	else if (value < temp_off)
	{
		digitalWrite(pin, LOW);
		TriacDimmer::disable(pin);
	}
	else
	{
		if ((pin & 0x01) == 0x01)
		{ // if (pin == 9){
			TriacDimmer::detail::setChannelA_direct(phase_delay[value]+ZC_LEAD);
			TriacDimmer::detail::ch_A_en = true;
		}
		else
		{ // if (pin == 10){
			TriacDimmer::detail::setChannelB_direct(phase_delay[value]+ZC_LEAD);
			TriacDimmer::detail::ch_B_en = true;
		}
	}
	pinMode(pin, OUTPUT); // only set to output once configured.
}



void TriacDimmer::setBrightness(uint8_t pin, float value)
{
	assert(pin == 9 || pin == 10);

	if (value > TriacDimmer::detail::on_thresh)
	{
		digitalWrite(pin, HIGH);
		TriacDimmer::disable(pin);
	}
	else if (value < TriacDimmer::detail::off_thresh)
	{
		digitalWrite(pin, LOW);
		TriacDimmer::disable(pin);
	}
	else
	{
		if ((pin & 0x01) == 0x01)
		{ // if (pin == 9){
			TriacDimmer::detail::setChannelA(1 - value);
			TriacDimmer::detail::ch_A_en = true;
		}
		else
		{ // if (pin == 10){
			TriacDimmer::detail::setChannelB(1 - value);
			TriacDimmer::detail::ch_B_en = true;
		}
	}
	pinMode(pin, OUTPUT); // only set to output once configured.
}

void TriacDimmer::disable(uint8_t pin)
{
	assert(pin == 9 || pin == 10);

	if ((pin & 0x01) == 0x01)
	{ // if (pin == 9){
		TriacDimmer::detail::disableChannelA();
	}
	else
	{ // if (pin == 10){
		TriacDimmer::detail::disableChannelB();
	}
}

float TriacDimmer::getCurrentBrightness(uint8_t pin)
{
	assert(pin == 9 || pin == 10);

	if ((pin & 0x01) == 0x01)
	{ // if (pin == 9){
		return 1 - TriacDimmer::detail::getChannelA();
	}
	else
	{ // if (pin == 10){
		return 1 - TriacDimmer::detail::getChannelB();
	}
}

uint16_t TriacDimmer::getPeriod()
{
	return TriacDimmer::detail::period;
}

void TriacDimmer::detail::setChannelA(float value)
{
	uint16_t p, u, d;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		// Don't need atomic read for pulse_length or min_trigger since they're not updated from an ISR.
		p = TriacDimmer::detail::period;
	}
	u = p * value;
	d = constrain(u + TriacDimmer::detail::pulse_length, TriacDimmer::detail::min_trigger, p);
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		TriacDimmer::detail::ch_A_up = u;
		TriacDimmer::detail::ch_A_dn = d;
	}
}

void TriacDimmer::detail::setChannelB(float value)
{
	uint16_t p, u, d;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		// Don't need atomic read for pulse_length or min_trigger since they're not updated from an ISR.
		p = TriacDimmer::detail::period;
	}
	u = p * value;
	d = constrain(u + TriacDimmer::detail::pulse_length, TriacDimmer::detail::min_trigger, p);
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		TriacDimmer::detail::ch_B_up = u;
		TriacDimmer::detail::ch_B_dn = d;
	}
}

void TriacDimmer::detail::setChannelA_direct(uint16_t value)
{
	uint16_t p, u, d;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		// Don't need atomic read for pulse_length or min_trigger since they're not updated from an ISR.
		p = TriacDimmer::detail::period;
	}
	u = value;
	d = constrain(u + TriacDimmer::detail::pulse_length, TriacDimmer::detail::min_trigger, p);
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		TriacDimmer::detail::ch_A_up = u;
		TriacDimmer::detail::ch_A_dn = d;
	}
}

void TriacDimmer::detail::setChannelB_direct(uint16_t value)
{
	uint16_t p, u, d;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		// Don't need atomic read for pulse_length or min_trigger since they're not updated from an ISR.
		p = TriacDimmer::detail::period;
	}
	u = value;
	d = constrain(u + TriacDimmer::detail::pulse_length, TriacDimmer::detail::min_trigger, p);
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		TriacDimmer::detail::ch_B_up = u;
		TriacDimmer::detail::ch_B_dn = d;
	}
}

float TriacDimmer::detail::getChannelA()
{
	uint16_t p, u;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		p = TriacDimmer::detail::period;
		u = TriacDimmer::detail::ch_A_up;
	}
	return (float)p / u;
}

float TriacDimmer::detail::getChannelB()
{
	uint16_t p, u;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		p = TriacDimmer::detail::period;
		u = TriacDimmer::detail::ch_B_up;
	}
	return (float)p / u;
}

void TriacDimmer::detail::disableChannelA()
{
	TriacDimmer::detail::ch_A_en = false;
	TCCR1A &= ~_BV(COM1A0) | _BV(COM1A1);
}

void TriacDimmer::detail::disableChannelB()
{
	TriacDimmer::detail::ch_B_en = false;
	TCCR1A &= ~_BV(COM1B0) | _BV(COM1B1);
}

ISR(TIMER1_CAPT_vect)
{
	TIMSK1 &= ~(_BV(OCIE1A) | _BV(OCIE1B)); // clear interrupts, in case they haven't run yet
	TCCR1A &= ~(_BV(COM1A0) | _BV(COM1B0));
	TCCR1C = _BV(FOC1A) | _BV(FOC1B); // ensure outputs are properly cleared

	OCR1A = ICR1 + TriacDimmer::detail::ch_A_up;
	OCR1B = ICR1 + TriacDimmer::detail::ch_B_up;
	ch_A_dn_buf = TriacDimmer::detail::ch_A_dn;
	ch_B_dn_buf = TriacDimmer::detail::ch_B_dn;

	TCCR1A |=
		(TriacDimmer::detail::ch_A_en ? _BV(COM1A0) | _BV(COM1A1) : 0) |
		(TriacDimmer::detail::ch_B_en ? _BV(COM1B0) | _BV(COM1B1) : 0); // set OC1x on compare match, only if enabled
	TIFR1 = _BV(OCF1A) | _BV(OCF1B);									// clear compare match flags
	TIMSK1 |= _BV(OCIE1A) | _BV(OCIE1B);								// enable input capture and compare match interrupts

	TriacDimmer::detail::period = ICR1 - last_icr;
	last_icr = ICR1;

	if ((signed)(TCNT1 - OCR1A) >= 0)
	{
		TCCR1C = _BV(FOC1A); // interrupt ran late, trigger match manually
	}
	if ((signed)(TCNT1 - OCR1B) >= 0)
	{
		TCCR1C = _BV(FOC1B); // interrupt ran late, trigger match manually
	}

	// now do ICC
	if (newN)
	{
		curr = RATIO_M;
		newN = false;
	}
	curr -= 1;
	if (curr < 1) // curr == 0 ?
	{
		curr = RATIO_M;
	}
	if (curr > duty_2)
	{
		digitalWrite(ICC_PIN, LOW); // start pulse
        //PORTD &= ~(1<<PD7);
	}
	else
	{
		digitalWrite(ICC_PIN, HIGH); // start pulse
	    //PORTD |= (1<<PD7);
	}
}

ISR(TIMER1_COMPA_vect)
{
	TIMSK1 &= ~_BV(OCIE1A); // disable match interrupt
	TCCR1A &= ~_BV(COM1A0); // clear OC1x on compare match

	OCR1A = last_icr + ch_A_dn_buf;

	if ((signed)(TCNT1 - OCR1A) >= 0)
	{
		TCCR1C = _BV(FOC1A); // interrupt ran late, trigger match manually
	}
}

ISR(TIMER1_COMPB_vect)
{
	TIMSK1 &= ~_BV(OCIE1B); // disable match interrupt
	TCCR1A &= ~_BV(COM1B0); // clear OC1x on compare match

	OCR1B = last_icr + ch_B_dn_buf;

	if ((signed)(TCNT1 - OCR1B) >= 0)
	{
		TCCR1C = _BV(FOC1B); // interrupt ran late, trigger match manually
	}
}

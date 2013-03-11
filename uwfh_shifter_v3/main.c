#include <msp430x22x2.h>
//#include "can.h"
//#include "can_data.h"
#include "main.h"
#include <string.h>
#include "spi.h"
#include "can.h"
#include "can_data.h"
//https://github.com/davidebids/uwfh_shifter.git

//
//State Declarations
//
#define STATE_IDLE							0
#define STATE_UPSHIFT						1
#define STATE_DOWNSHIFT						2
#define STATE_MANUAL_IDLE					3
#define STATE_GEAR_CHECK					4
#define STATE_NEUTRAL_FROM_FIRST			5
#define STATE_NEUTRAL_FROM_SECOND			6
#define STATE_NEUTRAL						7
#define STATE_NEUTRAL_TO_FIRST				8
#define STATE_NEUTRAL_TO_SECOND				9
#define STATE_CLUTCH_PADDLE					10

//
//Variable Declarations
//
float shift_posn, temp;
float up_posn = 20; //Position in fraction of volts
float down_posn = 430; //Position in fraction of volts
float rest_posn = 220; //Position in fraction of volts
float neutral_posn = 110; //Position in fraction of volts - for upshifting into neutral
float shift_half = 330; //Position in fraction of volts - for downshifting into neutral
float paddle_rest_posn = 335; //Position in fraction of volts
unsigned char clutch_state, shift_state, prev_state, ign_cut, gear_status, in_neutral;
unsigned int gear_num;

float clutch_posn, in_sample, controller_output, paddle_val;
float upper_limit = 800;
float lower_limit = 462;
float ref_posn, m, b;
float prev_in_sample, out_sample, prev_out_sample; //rads, volts, volts

unsigned int gear_stk1[1*1];

//sample CAN
unsigned int cv_stk2[3*10];

//
// Function Declarations
//
void timer_init(void);
void actuate_clutch(void);

//
// Initialize port pins
//
void initPortPins(void)
{
	//Set Initial Values
	P1OUT = 0;
	P2OUT = 0;
	P3OUT = 0;
	P4OUT = 0;

	//Digital Outputs
	P2DIR = PIN3 + PIN4 + PIN5;
	P3DIR = PIN0;
	P4DIR = PIN7;
}

void pwm_config (void)
{
	  P2DIR |= BIT3;
	  P2SEL |= BIT3;

	  TACCR0 = 512-1;                             // PWM Period/2
	  TACCTL1 = OUTMOD_7;                       // TACCR1 toggle/set
	  TACCR1 = 0;                              // TACCR1 PWM duty cycle
	  TACTL = TASSEL_2 + MC_1;                  // SMCLK, up mode, 12MHz
}

void clutch_pwm (void)
{
	if (ref_posn > upper_limit)
	{
		ref_posn = upper_limit;
	}
	else if (ref_posn < lower_limit)
	{
		ref_posn = lower_limit;
	}

	in_sample = (ref_posn - clutch_posn)/217.3; //rads

	out_sample = 0.985766901310270*prev_out_sample + 7.882180794557980*in_sample - 7.735594428311607*prev_in_sample; //volts

	if (out_sample > 0)
	{
		P2OUT |= BIT5; //DIR ccw
		controller_output = out_sample;
	}
	else if (out_sample < 0)
	{
		P2OUT &= ~BIT5; //DIR
		controller_output = out_sample*-1;
	}

	if (controller_output > 12)
	{
		controller_output = 12;
	}
	else if (controller_output < -12)
	{
		controller_output = -12;
	}

	prev_out_sample = out_sample;
	prev_in_sample = in_sample;

	__delay_cycles(12000);
}

float readADC(int channel)
{
	float adc_val;

	ADC10CTL0 &= ~ENC;
	ADC10CTL0 = ADC10ON + ADC10SR + ADC10SHT_0 + SREF_0;

	if (channel == 14) { //shifter
		ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_14;
	}
	else if (channel == 15) { //clutch
		ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_15;
	}
	else if (channel == 13) { //paddle
		ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_13;
	}
	else if (channel == 0) { //wheel_spd1
		ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_0;
	}
	else if (channel == 1) { //wheel_spd2
		ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_1;
	}
	else if (channel == 2) { //wheel_spd3
		ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_2;
	}

	ADC10AE0 = 0x4;
	ADC10AE1 = 0xC0;
	ADC10CTL0 |= ENC;

	// ADC Start Conversion - Software trigger
	ADC10CTL0 |= ADC10SC;

	// Loop until ADC10IFG is set indicating ADC conversion complete
	while ((ADC10CTL0 & ADC10IFG) == 0);

	// Read ADC conversion result from ADC10MEM
	adc_val = ADC10MEM;

	return adc_val;
}

void initActuators(void)
{
	clutch_state = 0;
	actuate_clutch();

	shift_posn = readADC(14);

	if (shift_posn < rest_posn) {
		while (shift_posn < rest_posn) {
			P2OUT |= PIN4; //DIR
			P3OUT |= PIN0; //PWMH

			shift_posn = readADC(14);
		}
		P3OUT &= ~PIN0;
	}
	else if (shift_posn > rest_posn) {
		while (shift_posn > rest_posn) {
			P2OUT &= ~PIN4; //DIR
			P3OUT |= PIN0; //PWMH

			shift_posn = readADC(14);
		}
		P3OUT &= ~PIN0;
	}
}

void clock_init (void)
{
	DCOCTL = CALDCO_12MHZ;
	BCSCTL1 = CALBC1_12MHZ;
}

//Clutch actuation method, clutch_state = 1 - full disengage, clutch_state = 0 - full engage
void actuate_clutch(void)
{
	if (clutch_state == 1) {
		ref_posn = upper_limit;

		for (;;)
		{
			clutch_posn = readADC(15);
			clutch_pwm();
			TACCR1 = (controller_output/12)*512;

			if (clutch_posn < ref_posn - 20)
			{
				break;
			}
		}
	}
	else if (clutch_state == 0) {
		ref_posn = lower_limit;

		for (;;)
		{
			clutch_posn = readADC(15);
			clutch_pwm();
			TACCR1 = (controller_output/12)*512;

			if (clutch_posn < ref_posn + 20)
			{
				break;
			}
		}
	}
}

void ignition_cut (void) //P2.1
{
	if (ign_cut == 1) {
		P4OUT |= PIN7;			//start spark cut signal
	}
	else if (ign_cut == 0) {
		P4OUT &= ~PIN7;			//end spark cut signal
	}
}

void gear_indication (void)
{
	if (gear_num == 1 && shift_state != STATE_NEUTRAL) {
		//can_write_gear (SCU_GEAR_S1, gear_stk1);
	}
	else if (gear_num == 2 && shift_state != STATE_NEUTRAL) {
		//can_write_gear (SCU_GEAR_S1, gear_stk1);
	}
	else if (gear_num == 3 && shift_state != STATE_NEUTRAL) {
		//can_write_gear (SCU_GEAR_S1, gear_stk1);
	}
	else if (gear_num == 4 && shift_state != STATE_NEUTRAL) {
		//can_write_gear (SCU_GEAR_S1, gear_stk1);
	}
	else if (gear_num == 5 && shift_state != STATE_NEUTRAL) {
		//can_write_gear (SCU_GEAR_S1, gear_stk1);
	}
	else if (shift_state == STATE_NEUTRAL) {
		//can_write_gear (SCU_GEAR_S1, gear_stk1);
	}
	else {
		//can_write_gear (SCU_GEAR_S1, gear_stk1);
	}
}

void shift_gear (void)
{
	//gear_status = 1 (upshift), gear_status = 2 (downshift), gear_status = 3 (neutral - half shift)

	shift_posn = readADC(14);

	if (gear_status == 1) {
		if (in_neutral == 1) {
			while (shift_posn > neutral_posn) {
				P2OUT &= ~PIN4; //DIR
				P3OUT |= PIN0; //PWMH

				shift_posn = readADC(14);
			}

			P3OUT &= ~PIN0;

			while (shift_posn < rest_posn) {
				P2OUT |= PIN4; //DIR
				P3OUT |= PIN0; //PWMH

				shift_posn = readADC(14);
			}

			P3OUT &= ~PIN0;
			in_neutral = 0;
		}
		else if (in_neutral == 0)
		{
			if (in_neutral != 1) {
				ign_cut = 1;
				ignition_cut();
			}

			while (shift_posn > up_posn) {
				P2OUT &= ~PIN4; //DIR
				P3OUT |= PIN0; //PWMH

				shift_posn = readADC(14);
			}

			P3OUT &= ~PIN0;

			if (in_neutral != 1) {
				ign_cut = 0;
				ignition_cut();
			}

			while (shift_posn < rest_posn) {
				P2OUT |= PIN4; //DIR
				P3OUT |= PIN0; //PWMH

				shift_posn = readADC(14);
			}

			P3OUT &= ~PIN0;
		}
	}
	else if (gear_status == 2) {
		clutch_state = 1;
		actuate_clutch();

		while (shift_posn < down_posn)
		{
			clutch_posn = readADC(15);
			clutch_pwm();
			TACCR1 = (controller_output/12)*512;

			P2OUT |= PIN4; //DIR
			P3OUT |= PIN0; //PWMH
			shift_posn = readADC(14);
		}

		P3OUT &= ~PIN0;

		while (shift_posn > rest_posn)
		{
			clutch_posn = readADC(15);
			clutch_pwm();
			TACCR1 = (controller_output/12)*512;

			P2OUT &= ~PIN4; //DIR
			P3OUT |= PIN0; //PWMH
			shift_posn = readADC(14);
		}

		P3OUT &= ~PIN0;

		clutch_state = 0;
		actuate_clutch();
	}
	else if (gear_status == 3) {
		shift_posn = readADC(14);

		if (gear_num == 1 && in_neutral != 1)
		{
			while (shift_posn > neutral_posn) {
				P2OUT &= ~PIN4; //DIR
				P3OUT |= PIN0; //PWMH

				shift_posn = readADC(14);
			}

			P3OUT &= ~PIN0;

			while (shift_posn < rest_posn) {
				P2OUT |= PIN4; //DIR
				P3OUT |= PIN0; //PWMH

				shift_posn = readADC(14);
			}

			P3OUT &= ~PIN0;
		}
		else if (gear_num == 2 && in_neutral != 1)
		{
			clutch_state = 1;
			actuate_clutch();

			while (shift_posn < shift_half) {
				clutch_posn = readADC(15);
				clutch_pwm();
				TACCR1 = (controller_output/12)*512;

				P2OUT |= PIN4; //DIR
				P3OUT |= PIN0; //PWMH

				shift_posn = readADC(14);
			}

			P3OUT &= ~PIN0;

			while (shift_posn > rest_posn) {
				clutch_posn = readADC(15);
				clutch_pwm();
				TACCR1 = (controller_output/12)*512;

				P2OUT &= ~PIN4; //DIR
				P3OUT |= PIN0; //PWMH

				shift_posn = readADC(14);
			}

			P3OUT &= ~PIN0;

			clutch_state = 0;
			actuate_clutch();
		}

		in_neutral = 1;
	}
}

void main(void)
{
	unsigned int i;
	gear_num = 1;
	controller_output = 0;
	shift_state = STATE_IDLE;
	prev_state = STATE_IDLE;

	WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

	//Delay to allow 3.3lV rail to fully rise, essential for CAN part
	//because MSP430 will turn on at 1.8V or less
	//Especially if 3.3V voltage supervisor is not installed!
	for(i=0; i<65000; i++)
		asm("nop");

	clock_init();
	__enable_interrupt();                     // Enable interrupts

	initPortPins();                           // Initialize port pins
	pwm_config();
	initActuators();
	//timer_init();
	gear_indication();
	TACCR1 = (controller_output/12)*512;	  // Initial PWM value of 0
	ign_cut = 0;

	//spi_init();
	//can_init(CAN_BITRATE_250);
	//spi_set_mode (UCCKPH, 0, 5);			//need this????

  for(;;) //1 kHz loop
  {
	  __bis_SR_register(GIE); //enable general interrupts

    /*
     * Out of sleep mode.
     */

    //Send a test can message
    //spi_set_mode ( UCCKPH, 0, 5 );
	//can_write_vcell (BATT_S1, cv_stk2);

	//Read in first value for clutch paddle
	paddle_val = readADC(13);

    if (shift_state == STATE_IDLE)
    {
    	if (((P1IN & PIN1) != PIN1) && gear_num < 5) //upshift button pressed
        {
    		shift_state = STATE_UPSHIFT;
        }
        else if (((P1IN & PIN2) != PIN2) && gear_num >= 1) //downshift button pressed
        {
        	shift_state = STATE_DOWNSHIFT;
        }
        else if (((P1IN & PIN2) != PIN2) && prev_state == STATE_NEUTRAL)
        {
        	shift_state = STATE_DOWNSHIFT;
        }
    	else if (((P1IN & PIN3) != PIN3) && prev_state != STATE_NEUTRAL) //neutral button pressed
    	{
    		shift_state = STATE_GEAR_CHECK;
    	}
    	else if (paddle_val > paddle_rest_posn)
    	{
			shift_state = STATE_CLUTCH_PADDLE;
    	}

    	TACCR1 = 0;
    	prev_state = STATE_IDLE;
    }
    else if (shift_state == STATE_UPSHIFT)
    {
    	while ((P1IN & PIN1) != PIN1);

    	gear_indication();
    	gear_status = 1;
		shift_gear();

		gear_num++;
		gear_indication();

		prev_state = STATE_UPSHIFT;
		shift_state = STATE_IDLE;
    }
    else if (shift_state == STATE_DOWNSHIFT)
    {
		while ((P1IN & PIN2) != PIN2);

    	gear_indication();
		in_neutral = 0;
		gear_status = 2;
		shift_gear();

		if (gear_num <= 1) {
			gear_num = 1;
			gear_indication();
		}
		else {
			gear_num--;
			gear_indication();
		}

		shift_state = STATE_IDLE;
    }
    else if (shift_state == STATE_GEAR_CHECK)
    {
		while ((P1IN & PIN3) != PIN3);

    	if (gear_num == 1) {
			shift_state = STATE_NEUTRAL_FROM_FIRST;
		}
		else if (gear_num == 2) {
			shift_state = STATE_NEUTRAL_FROM_SECOND;
		}
		else {
			shift_state = STATE_IDLE;
		}
    }
    else if (shift_state == STATE_NEUTRAL_FROM_FIRST)
    {
    	gear_status = 3;
    	shift_gear();
    	shift_state = STATE_NEUTRAL;
    }
    else if (shift_state == STATE_NEUTRAL_FROM_SECOND)
    {
    	gear_status = 3;
    	shift_gear();
    	gear_num--;
    	shift_state = STATE_NEUTRAL;
    }
    else if (shift_state == STATE_NEUTRAL)
    {
    	in_neutral = 1;
    	gear_indication();
    	prev_state = STATE_NEUTRAL;
		shift_state = STATE_IDLE;
    }
    else if (shift_state == STATE_CLUTCH_PADDLE)
    {
    	m = (upper_limit - lower_limit)/146;
    	b = lower_limit - (m*327);

    	for (;;)
    	{
    		paddle_val = readADC(13);

			ref_posn = m*paddle_val + b;

			TACCR1 = (controller_output/12)*512;
			clutch_posn = readADC(15);
			clutch_pwm();

			if (paddle_val < paddle_rest_posn && clutch_posn < lower_limit + 20)
			{
				break;
			}
    	}

    	shift_state = STATE_IDLE;
    }
  }
}

// Timer Interrupt Service Routine
#pragma vector=TIMERB0_VECTOR
__interrupt void Timer_B (void)
{
  LPM0_EXIT;
}

// Initialize TimerB to wake up processor at 2kHz
void timer_init(void)
{
  CCR0 = 1483;
  TBCTL = TBSSEL_2 + ID_3 + MC_1;                  // SMCLK, upmode
  CCTL0 = CCIE;                             // CCR0 interrupt enabled
}

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
float shift_posn, clutch_posn, paddle_val, temp;
float up_posn = 952; //Position in fraction of volts
float down_posn = 69; //Position in fraction of volts
float rest_posn = 532; //Position in fraction of volts
float neutral_posn = 740; //Position in fraction of volts
float shift_half = 230; //Position in fraction of volts
float clutch_retract = 385; //Position in fraction of volts
float clutch_half = 193; //Position in fraction of volts
float clutch_extend = 0; //Position in fraction of volts
unsigned char clutch_state, shift_state, prev_state, ign_cut, gear_status, in_neutral;
unsigned int gear_num;

unsigned int gear_stk1[1*1];

//sample CAN
unsigned int cv_stk2[3*10];

//
// Function Declarations
//
void timer_init(void);

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

//Clutch actuation method
void actuate_clutch(void)
{
	clutch_posn = readADC(15);

	if (clutch_state == 1) {
		while (clutch_posn < clutch_half) {
			P2OUT &= ~PIN3; //DIR
			P2OUT |= PIN5; //PWMH

			clutch_posn = readADC(15);
		}

		P2OUT &= ~PIN5;
	}
	else if (clutch_state == 0) {
		while (clutch_posn > clutch_extend) {
			P2OUT |= PIN3; //DIR
			P2OUT |= PIN5; //PWMH

			clutch_posn = readADC(15);
		}

		P2OUT |= PIN3; //DIR
		P2OUT |= PIN5; //PWMH
	}
}

void ignition_cut (void) //P2.1
{
	if (ign_cut == 1) {
		P4OUT |= PIN7;			//turn on LED
	}
	else if (ign_cut == 0) {
		P4OUT &= ~PIN7;			//turn off LED
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
			while (shift_posn < neutral_posn) {
				P2OUT |= PIN4; //DIR
				P3OUT |= PIN0; //PWMH

				shift_posn = readADC(14);
			}

			P3OUT &= ~PIN0;

			while (shift_posn > rest_posn) {
				P2OUT &= ~PIN4; //DIR
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

			while (shift_posn < up_posn) {
				P2OUT |= PIN4; //DIR
				P3OUT |= PIN0; //PWMH

				shift_posn = readADC(14);
			}

			P3OUT &= ~PIN0;

			if (in_neutral != 1) {
				ign_cut = 0;
				ignition_cut();
			}

			while (shift_posn > rest_posn) {
				P2OUT &= ~PIN4; //DIR
				P3OUT |= PIN0; //PWMH

				shift_posn = readADC(14);
			}

			P3OUT &= ~PIN0;
		}
	}
	else if (gear_status == 2) {
		clutch_state = 1;
		actuate_clutch();

		while (shift_posn > down_posn) {
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

		clutch_state = 0;
		actuate_clutch();
	}
	else if (gear_status == 3) {
		shift_posn = readADC(14);

		if (gear_num == 1 && in_neutral != 1)
		{
			while (shift_posn < neutral_posn) {
				P2OUT |= PIN4; //DIR
				P3OUT |= PIN0; //PWMH

				shift_posn = readADC(14);
			}

			P3OUT &= ~PIN0;

			while (shift_posn > rest_posn) {
				P2OUT &= ~PIN4; //DIR
				P3OUT |= PIN0; //PWMH

				shift_posn = readADC(14);
			}

			P3OUT &= ~PIN0;
		}
		else if (gear_num == 2 && in_neutral != 1)
		{
			clutch_state = 1;
			actuate_clutch();

			while (shift_posn > shift_half) {
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
	shift_state = STATE_IDLE;
	prev_state = STATE_IDLE;

	WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

	//Delay to allow 3.3V rail to fully rise, essential for CAN part
	//because MSP430 will turn on at 1.8V or less
	//Especially if 3.3V voltage supervisor is not installed!
	for(i=0; i<65000; i++)
		asm("nop");

	clock_init();
	__enable_interrupt();                     // Enable interrupts

	initPortPins();                           // Initialize port pins
	initActuators();
	timer_init();
	gear_indication();
	ign_cut = 0;

	spi_init();
	can_init(CAN_BITRATE_250);
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
    	else if (paddle_val > 375)
    	{
			shift_state = STATE_CLUTCH_PADDLE;
    	}

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
    	temp = ((paddle_val - 375) / (495 - 375)) * 385;

		while (clutch_posn != temp) {
			if (temp >= clutch_posn) {
				P2OUT &= ~PIN3; //DIR
			}
			else if (temp < clutch_posn)
			{
				P2OUT |= PIN3; //DIR
			}

			P2OUT |= PIN5; //PWMH

			clutch_posn = readADC(15);
			paddle_val = readADC(13);

			temp = ((paddle_val - 375) / (495 - 375)) * 385;

			if (temp + 5 > clutch_posn && temp - 5 < clutch_posn) {
				while (temp + 115 > clutch_posn && temp - 115 < clutch_posn) {
					P2OUT &= ~PIN5;

					clutch_posn = readADC(15);
					paddle_val = readADC(13);

					temp = ((paddle_val - 375) / (495 - 375)) * 385;
				}
			}

			if (paddle_val < 375) {
				P2OUT |= PIN5; //PWMH
				break;
			}

		}

    	shift_state = STATE_IDLE;
    }
  //else - what - throw exception, reset? -- display N/A as gear indication, cause driver to downshift a bunch of times to reset to first gear -- used for when the car shuts off eg. from BRB
  }
}

// Timer Interrupt Service Routine
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A (void)
{
  LPM0_EXIT;
}

// Initialize TimerA to wake up processor at 2kHz
void timer_init(void)
{
  CCR0 = 1483;
  TACTL = TASSEL_2 + ID_3 + MC_1;                  // SMCLK, upmode
  CCTL0 = CCIE;                             // CCR0 interrupt enabled
}

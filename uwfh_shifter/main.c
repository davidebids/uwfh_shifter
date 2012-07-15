#include <msp430x22x2.h>
//#include "can.h"
//#include "can_data.h"
#include "main.h"
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
	P2DIR = PIN1;
	P3DIR = PIN1 + PIN2 + PIN3 + PIN4 + PIN5;
	P4DIR = PIN2 + PIN7;
}

void initActuators(void)
{
	ADC10CTL0 &= ~ENC;
	ADC10CTL0 = ADC10ON + ADC10SR + ADC10SHT_0 + SREF_0;
	ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_14; //channel 14
	ADC10AE0 = 0x4;
	ADC10AE1 = 0xC0;
	ADC10CTL0 |= ENC;

	// ADC Start Conversion - Software trigger
	ADC10CTL0 |= ADC10SC;

	// Loop until ADC10IFG is set indicating ADC conversion complete
	while ((ADC10CTL0 & ADC10IFG) == 0);

	// Read ADC conversion result from ADC10MEM
	shift_posn = ADC10MEM;

	if (shift_posn < rest_posn) {
		while (shift_posn < rest_posn) {
			P3OUT |= PIN3; //DIR
			P4OUT |= PIN2; //PWMH

			ADC10CTL0 &= ~ENC;
			ADC10CTL0 = ADC10ON + ADC10SR + ADC10SHT_0 + SREF_0;
			ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_14; //channel 14
			ADC10AE0 = 0x4;
			ADC10AE1 = 0xC0;
			ADC10CTL0 |= ENC;

			// ADC Start Conversion - Software trigger
			ADC10CTL0 |= ADC10SC;

			// Loop until ADC10IFG is set indicating ADC conversion complete
			while ((ADC10CTL0 & ADC10IFG) == 0);

			// Read ADC conversion result from ADC10MEM
			shift_posn = ADC10MEM;
		}
		P4OUT &= ~PIN2;
	}
	else if (shift_posn > rest_posn) {
		while (shift_posn > rest_posn) {
			P3OUT &= ~PIN3; //DIR
			P4OUT |= PIN2; //PWMH

			ADC10CTL0 &= ~ENC;
			ADC10CTL0 = ADC10ON + ADC10SR + ADC10SHT_0 + SREF_0;
			ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_14; //channel 14
			ADC10AE0 = 0x4;
			ADC10AE1 = 0xC0;
			ADC10CTL0 |= ENC;

			// ADC Start Conversion - Software trigger
			ADC10CTL0 |= ADC10SC;

			// Loop until ADC10IFG is set indicating ADC conversion complete
			while ((ADC10CTL0 & ADC10IFG) == 0);

			// Read ADC conversion result from ADC10MEM
			shift_posn = ADC10MEM;
		}
		P4OUT &= ~PIN2;
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
	ADC10CTL0 &= ~ENC;
	ADC10CTL0 = ADC10ON + ADC10SR + ADC10SHT_0 + SREF_0;
	ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_15;
	ADC10AE0 = 0x4;
	ADC10AE1 = 0xC0;
	ADC10CTL0 |= ENC;

	// ADC Start Conversion - Software trigger
	ADC10CTL0 |= ADC10SC;

	// Loop until ADC10IFG is set indicating ADC conversion complete
	while ((ADC10CTL0 & ADC10IFG) == 0);

	// Read ADC conversion result from ADC10MEM
	clutch_posn = ADC10MEM;

	if (clutch_state == 1) {
		while (clutch_posn < clutch_half) {
			P3OUT &= ~PIN2; //DIR
			P3OUT |= PIN1; //PWMH

			ADC10CTL0 &= ~ENC;
			ADC10CTL0 = ADC10ON + ADC10SR + ADC10SHT_0 + SREF_0;
			ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_15;
			ADC10AE0 = 0x4;
			ADC10AE1 = 0xC0;
			ADC10CTL0 |= ENC;

			// ADC Start Conversion - Software trigger
			ADC10CTL0 |= ADC10SC;

			// Loop until ADC10IFG is set indicating ADC conversion complete
			while ((ADC10CTL0 & ADC10IFG) == 0);

			// Read ADC conversion result from ADC10MEM
			clutch_posn = ADC10MEM;
		}

		P3OUT &= ~PIN1;
	}
	else if (clutch_state == 0) {
		while (clutch_posn > clutch_extend) {
			P3OUT |= PIN2; //DIR
			P3OUT |= PIN1; //PWMH

			ADC10CTL0 &= ~ENC;
			ADC10CTL0 = ADC10ON + ADC10SR + ADC10SHT_0 + SREF_0;
			ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_15;
			ADC10AE0 = 0x4;
			ADC10AE1 = 0xC0;
			ADC10CTL0 |= ENC;

			// ADC Start Conversion - Software trigger
			ADC10CTL0 |= ADC10SC;

			// Loop until ADC10IFG is set indicating ADC conversion complete
			while ((ADC10CTL0 & ADC10IFG) == 0);

			// Read ADC conversion result from ADC10MEM
			clutch_posn = ADC10MEM;
		}

		P3OUT |= PIN2; //DIR
		P3OUT |= PIN1; //PWMH
	}
}

void ignition_cut (void) //P2.1
{
	if (ign_cut == 1) {
		P2OUT |= PIN1;			//turn on LED
	}
	else if (ign_cut == 0) {
		P2OUT &= ~PIN1;			//turn off LED
	}
}

void gear_indication (void) //P3.4, 3.5, 4.7
{
	if (gear_num == 1 && shift_state != STATE_NEUTRAL) {
		P4OUT |= PIN7;
		P3OUT &= ~(PIN4 + PIN5);
	}
	else if (gear_num == 2 && shift_state != STATE_NEUTRAL) {
		P3OUT |= PIN4;
		P3OUT &= ~PIN5;
		P4OUT &= ~PIN7;
	}
	else if (gear_num == 3 && shift_state != STATE_NEUTRAL) {
		P3OUT |= PIN4;
		P4OUT |= PIN7;
		P3OUT &= ~PIN5;
	}
	else if (gear_num == 4 && shift_state != STATE_NEUTRAL) {
		P3OUT |= PIN5;
		P3OUT &= ~PIN4;
		P4OUT &= ~PIN7;
	}
	else if (gear_num == 5 && shift_state != STATE_NEUTRAL) {
		P3OUT |= PIN5;
		P4OUT |= PIN7;
		P3OUT &= ~PIN4;
	}
	else if (shift_state == STATE_NEUTRAL) {
		P3OUT |= PIN4 + PIN5;
		P4OUT |= PIN7;
	}
	else {
		P3OUT &= ~(PIN4 + PIN5);
		P4OUT &= ~PIN7;
	}
}

void shift_gear (void)
{
	//gear_status = 1 (upshift), gear_status = 2 (downshift), gear_status = 3 (neutral - half shift)

	ADC10CTL0 &= ~ENC;
	ADC10CTL0 = ADC10ON + ADC10SR + ADC10SHT_0 + SREF_0;
	ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_14; //channel 14
	ADC10AE0 = 0x4;
	ADC10AE1 = 0xC0;
	ADC10CTL0 |= ENC;

	// ADC Start Conversion - Software trigger
	ADC10CTL0 |= ADC10SC;

	// Loop until ADC10IFG is set indicating ADC conversion complete
	while ((ADC10CTL0 & ADC10IFG) == 0);

	// Read ADC conversion result from ADC10MEM
	shift_posn = ADC10MEM;

	if (gear_status == 1) {
		if (in_neutral == 1) {
			while (shift_posn < neutral_posn) {
				P3OUT |= PIN3; //DIR
				P4OUT |= PIN2; //PWMH

				ADC10CTL0 &= ~ENC;
				ADC10CTL0 = ADC10ON + ADC10SR + ADC10SHT_0 + SREF_0;
				ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_14; //channel 14
				ADC10AE0 = 0x4;
				ADC10AE1 = 0xC0;
				ADC10CTL0 |= ENC;

				// ADC Start Conversion - Software trigger
				ADC10CTL0 |= ADC10SC;

				// Loop until ADC10IFG is set indicating ADC conversion complete
				while ((ADC10CTL0 & ADC10IFG) == 0);

				// Read ADC conversion result from ADC10MEM
				shift_posn = ADC10MEM;
			}

			P4OUT &= ~PIN2;

			while (shift_posn > rest_posn) {
				P3OUT &= ~PIN3; //DIR
				P4OUT |= PIN2; //PWMH

				ADC10CTL0 &= ~ENC;
				ADC10CTL0 = ADC10ON + ADC10SR + ADC10SHT_0 + SREF_0;
				ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_14; //channel 14
				ADC10AE0 = 0x4;
				ADC10AE1 = 0xC0;
				ADC10CTL0 |= ENC;

				// ADC Start Conversion - Software trigger
				ADC10CTL0 |= ADC10SC;

				// Loop until ADC10IFG is set indicating ADC conversion complete
				while ((ADC10CTL0 & ADC10IFG) == 0);

				// Read ADC conversion result from ADC10MEM
				shift_posn = ADC10MEM;
			}

			P4OUT &= ~PIN2;
			in_neutral = 0;
		}
		else if (in_neutral == 0)
		{
			if (in_neutral != 1) {
				ign_cut = 1;
				ignition_cut();
			}

			while (shift_posn < up_posn) {
				P3OUT |= PIN3; //DIR
				P4OUT |= PIN2; //PWMH

				ADC10CTL0 &= ~ENC;
				ADC10CTL0 = ADC10ON + ADC10SR + ADC10SHT_0 + SREF_0;
				ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_14; //channel 14
				ADC10AE0 = 0x4;
				ADC10AE1 = 0xC0;
				ADC10CTL0 |= ENC;

				// ADC Start Conversion - Software trigger
				ADC10CTL0 |= ADC10SC;

				// Loop until ADC10IFG is set indicating ADC conversion complete
				while ((ADC10CTL0 & ADC10IFG) == 0);

				// Read ADC conversion result from ADC10MEM
				shift_posn = ADC10MEM;
			}

			P4OUT &= ~PIN2;

			if (in_neutral != 1) {
				ign_cut = 0;
				ignition_cut();
			}

			while (shift_posn > rest_posn) {
				P3OUT &= ~PIN3; //DIR
				P4OUT |= PIN2; //PWMH

				ADC10CTL0 &= ~ENC;
				ADC10CTL0 = ADC10ON + ADC10SR + ADC10SHT_0 + SREF_0;
				ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_14; //channel 14
				ADC10AE0 = 0x4;
				ADC10AE1 = 0xC0;
				ADC10CTL0 |= ENC;

				// ADC Start Conversion - Software trigger
				ADC10CTL0 |= ADC10SC;

				// Loop until ADC10IFG is set indicating ADC conversion complete
				while ((ADC10CTL0 & ADC10IFG) == 0);

				// Read ADC conversion result from ADC10MEM
				shift_posn = ADC10MEM;
			}

			P4OUT &= ~PIN2;
		}
	}
	else if (gear_status == 2) {
		clutch_state = 1;
		actuate_clutch();

		while (shift_posn > down_posn) {
			P3OUT &= ~PIN3; //DIR
			P4OUT |= PIN2; //PWMH

			ADC10CTL0 &= ~ENC;
			ADC10CTL0 = ADC10ON + ADC10SR + ADC10SHT_0 + SREF_0;
			ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_14; //channel 14
			ADC10AE0 = 0x4;
			ADC10AE1 = 0xC0;
			ADC10CTL0 |= ENC;

			// ADC Start Conversion - Software trigger
			ADC10CTL0 |= ADC10SC;

			// Loop until ADC10IFG is set indicating ADC conversion complete
			while ((ADC10CTL0 & ADC10IFG) == 0);

			// Read ADC conversion result from ADC10MEM
			shift_posn = ADC10MEM;
		}

		P4OUT &= ~PIN2;

		while (shift_posn < rest_posn) {
			P3OUT |= PIN3; //DIR
			P4OUT |= PIN2; //PWMH

			ADC10CTL0 &= ~ENC;
			ADC10CTL0 = ADC10ON + ADC10SR + ADC10SHT_0 + SREF_0;
			ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_14; //channel 14
			ADC10AE0 = 0x4;
			ADC10AE1 = 0xC0;
			ADC10CTL0 |= ENC;

			// ADC Start Conversion - Software trigger
			ADC10CTL0 |= ADC10SC;

			// Loop until ADC10IFG is set indicating ADC conversion complete
			while ((ADC10CTL0 & ADC10IFG) == 0);

			// Read ADC conversion result from ADC10MEM
			shift_posn = ADC10MEM;
		}

		P4OUT &= ~PIN2;

		clutch_state = 0;
		actuate_clutch();
	}
	else if (gear_status == 3) {
		ADC10CTL0 &= ~ENC;
		ADC10CTL0 = ADC10ON + ADC10SR + ADC10SHT_0 + SREF_0;
		ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_14; //channel 14
		ADC10AE0 = 0x4;
		ADC10AE1 = 0xC0;
		ADC10CTL0 |= ENC;

		// ADC Start Conversion - Software trigger
		ADC10CTL0 |= ADC10SC;

		// Loop until ADC10IFG is set indicating ADC conversion complete
		while ((ADC10CTL0 & ADC10IFG) == 0);

		// Read ADC conversion result from ADC10MEM
		shift_posn = ADC10MEM;

		if (gear_num == 1 && in_neutral != 1)
		{
			while (shift_posn < neutral_posn) {
				P3OUT |= PIN3; //DIR
				P4OUT |= PIN2; //PWMH

				ADC10CTL0 &= ~ENC;
				ADC10CTL0 = ADC10ON + ADC10SR + ADC10SHT_0 + SREF_0;
				ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_14; //channel 14
				ADC10AE0 = 0x4;
				ADC10AE1 = 0xC0;
				ADC10CTL0 |= ENC;

				// ADC Start Conversion - Software trigger
				ADC10CTL0 |= ADC10SC;

				// Loop until ADC10IFG is set indicating ADC conversion complete
				while ((ADC10CTL0 & ADC10IFG) == 0);

				// Read ADC conversion result from ADC10MEM
				shift_posn = ADC10MEM;
			}

			P4OUT &= ~PIN2;

			while (shift_posn > rest_posn) {
				P3OUT &= ~PIN3; //DIR
				P4OUT |= PIN2; //PWMH

				ADC10CTL0 &= ~ENC;
				ADC10CTL0 = ADC10ON + ADC10SR + ADC10SHT_0 + SREF_0;
				ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_14; //channel 14
				ADC10AE0 = 0x4;
				ADC10AE1 = 0xC0;
				ADC10CTL0 |= ENC;

				// ADC Start Conversion - Software trigger
				ADC10CTL0 |= ADC10SC;

				// Loop until ADC10IFG is set indicating ADC conversion complete
				while ((ADC10CTL0 & ADC10IFG) == 0);

				// Read ADC conversion result from ADC10MEM
				shift_posn = ADC10MEM;
			}

			P4OUT &= ~PIN2;
			//in_neutral = 1;
		}
		else if (gear_num == 2 && in_neutral != 1)
		{
			clutch_state = 1;
			actuate_clutch();

			while (shift_posn > shift_half) {
				P3OUT &= ~PIN3; //DIR
				P4OUT |= PIN2; //PWMH

				ADC10CTL0 &= ~ENC;
				ADC10CTL0 = ADC10ON + ADC10SR + ADC10SHT_0 + SREF_0;
				ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_14; //channel 14
				ADC10AE0 = 0x4;
				ADC10AE1 = 0xC0;
				ADC10CTL0 |= ENC;

				// ADC Start Conversion - Software trigger
				ADC10CTL0 |= ADC10SC;

				// Loop until ADC10IFG is set indicating ADC conversion complete
				while ((ADC10CTL0 & ADC10IFG) == 0);

				// Read ADC conversion result from ADC10MEM
				shift_posn = ADC10MEM;
			}

			P4OUT &= ~PIN2;

			while (shift_posn < rest_posn) {
				P3OUT |= PIN3; //DIR
				P4OUT |= PIN2; //PWMH

				ADC10CTL0 &= ~ENC;
				ADC10CTL0 = ADC10ON + ADC10SR + ADC10SHT_0 + SREF_0;
				ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_14; //channel 14
				ADC10AE0 = 0x4;
				ADC10AE1 = 0xC0;
				ADC10CTL0 |= ENC;

				// ADC Start Conversion - Software trigger
				ADC10CTL0 |= ADC10SC;

				// Loop until ADC10IFG is set indicating ADC conversion complete
				while ((ADC10CTL0 & ADC10IFG) == 0);

				// Read ADC conversion result from ADC10MEM
				shift_posn = ADC10MEM;
			}

			P4OUT &= ~PIN2;

			clutch_state = 0;
			actuate_clutch();

			//in_neutral = 1;
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

  for(;;) //1 kHz loop
  {
	  __bis_SR_register(GIE); //enable general interrupts

    /*
     * Out of sleep mode.
     */

	//Read in first value for clutch paddle
	ADC10CTL0 &= ~ENC;
	ADC10CTL0 = ADC10ON + ADC10SR + ADC10SHT_0 + SREF_0;
	ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_2; //channel 14
	ADC10AE0 = 0x4;
	ADC10AE1 = 0xC0;
	ADC10CTL0 |= ENC;

	// ADC Start Conversion - Software trigger
	ADC10CTL0 |= ADC10SC;

	// Loop until ADC10IFG is set indicating ADC conversion complete
	while ((ADC10CTL0 & ADC10IFG) == 0);

	// Read ADC conversion result from ADC10MEM
	paddle_val = ADC10MEM;

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
    	//in_neutral = 0;
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
				P3OUT &= ~PIN2; //DIR
			}
			else if (temp < clutch_posn)
			{
				P3OUT |= PIN2; //DIR
			}

			P3OUT |= PIN1; //PWMH

			ADC10CTL0 &= ~ENC;
			ADC10CTL0 = ADC10ON + ADC10SR + ADC10SHT_0 + SREF_0;
			ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_15;
			ADC10AE0 = 0x4;
			ADC10AE1 = 0xC0;
			ADC10CTL0 |= ENC;

			// ADC Start Conversion - Software trigger
			ADC10CTL0 |= ADC10SC;

			// Loop until ADC10IFG is set indicating ADC conversion complete
			while ((ADC10CTL0 & ADC10IFG) == 0);

			// Read ADC conversion result from ADC10MEM
			clutch_posn = ADC10MEM;

			ADC10CTL0 &= ~ENC;
			ADC10CTL0 = ADC10ON + ADC10SR + ADC10SHT_0 + SREF_0;
			ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_2;
			ADC10AE0 = 0x4;
			ADC10AE1 = 0xC0;
			ADC10CTL0 |= ENC;

			// ADC Start Conversion - Software trigger
			ADC10CTL0 |= ADC10SC;

			// Loop until ADC10IFG is set indicating ADC conversion complete
			while ((ADC10CTL0 & ADC10IFG) == 0);

			// Read ADC conversion result from ADC10MEM
			paddle_val = ADC10MEM;

			temp = ((paddle_val - 375) / (495 - 375)) * 385;

			if (temp + 5 > clutch_posn && temp - 5 < clutch_posn) {
				while (temp + 115 > clutch_posn && temp - 115 < clutch_posn) {
					P3OUT &= ~PIN1;

					ADC10CTL0 &= ~ENC;
					ADC10CTL0 = ADC10ON + ADC10SR + ADC10SHT_0 + SREF_0;
					ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_15;
					ADC10AE0 = 0x4;
					ADC10AE1 = 0xC0;
					ADC10CTL0 |= ENC;

					// ADC Start Conversion - Software trigger
					ADC10CTL0 |= ADC10SC;

					// Loop until ADC10IFG is set indicating ADC conversion complete
					while ((ADC10CTL0 & ADC10IFG) == 0);

					// Read ADC conversion result from ADC10MEM
					clutch_posn = ADC10MEM;

					ADC10CTL0 &= ~ENC;
					ADC10CTL0 = ADC10ON + ADC10SR + ADC10SHT_0 + SREF_0;
					ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_2;
					ADC10AE0 = 0x4;
					ADC10AE1 = 0xC0;
					ADC10CTL0 |= ENC;

					// ADC Start Conversion - Software trigger
					ADC10CTL0 |= ADC10SC;

					// Loop until ADC10IFG is set indicating ADC conversion complete
					while ((ADC10CTL0 & ADC10IFG) == 0);

					// Read ADC conversion result from ADC10MEM
					paddle_val = ADC10MEM;

					temp = ((paddle_val - 375) / (495 - 375)) * 385;
				}
			}

			if (paddle_val < 375) {
				P3OUT |= PIN1; //PWMH
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

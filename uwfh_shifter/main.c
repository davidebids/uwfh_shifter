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

#define DIR_REVERSE	0
#define DIR_FORWARD	1

//
//Variable Declarations
//
long double shift_posn, clutch_posn;
long double up_posn = 5; //Position in Volts
long double down_posn = 3; //Position in Volts
long double rest_posn = 1; //Position in Volts
long double neutral_posn = 2; //Position in Volts
long double clutch_engaged = 5; //Position in Volts
unsigned char clutch_state, shift_state, prev_state, ign_cut, gear_status, in_neutral;
unsigned int gear_num;
unsigned char man_dir, man_mode;

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

	//Analog Inputs
	P4DIR = ~(PIN5 + PIN6);
	P2DIR = ~PIN2;

	//Enable Interrupt for P2.2 Clutch Paddle
	P2IES |= PIN2;
	P2IFG &= ~PIN2;
	P2IE |= PIN2;
}

void clock_init (void)
{
	DCOCTL = CALDCO_12MHZ;
	BCSCTL1 = CALBC1_12MHZ;
	//BCSCTL2 = SELM_0 + DIVM_0 + DIVS_0;
}

//Clutch actuation method
void actuate_clutch(void)
{
	if (clutch_state == 1) {
		while (clutch_posn < clutch_engaged) { //update position with legitimate value - only move clutch as much as needed for the shift, smallest amount for quickest shift
			TBCCR1 = 512;
			//SOME_PIN = DIR_FORWARD;
		}
	}
	else if (clutch_state == 0) {
		while (clutch_posn > clutch_engaged) {
			TBCCR1 = 512;
			//SOME PIN = DIR_REVERSE - disengage clutch
		}
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

	if (gear_status == 1) {

		if (in_neutral != 1) {
			ign_cut = 1;
			ignition_cut();
		}

		while (shift_posn < up_posn) { //update value corresponding to actuator position, PWM DIR_FORWARD
			TBCCR1 = 512; //feedback from pot
		}

		if (in_neutral != 1) {
			ign_cut = 0;
			ignition_cut();
		}

		while (shift_posn > rest_posn) { //update value corresponding to actuator position, PWM DIR_REVERSE
			TBCCR1 = 512; //feedback from pot
		}
	}
	else if (gear_status == 2) {
		clutch_state = 1;
		actuate_clutch();

		while (shift_posn > down_posn) { //update value corresponding to actuator position, PWM DIR_REVERSE
			TBCCR1 = 512;
		}

		clutch_state = 0;
		actuate_clutch();

		while (shift_posn < rest_posn) { //update value corresponding to actuator position, PWM DIR_FORWARD - back to rest position
			TBCCR1 = 512;
		}
	}
	else if (gear_status == 3) {
		if (gear_num == 1)
		{
			while (shift_posn < neutral_posn) { //update value corresponding to position, PWM DIR_FORWARD
				TBCCR1 = 512;
			}
		}
		else if (gear_num == 2)
		{
	    	clutch_state = 1;
	    	actuate_clutch();

			while (shift_posn > neutral_posn) { //update value corresponding to position, PWM DIR_REVERSE
				TBCCR1 = 512;
			}

	    	clutch_state = 0;
	    	actuate_clutch();

			while (shift_posn < rest_posn) { //update value corresponding to actuator position, PWM DIR_FORWARD
				TBCCR1 = 512;
			}
		}
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
	timer_init();
	gear_indication();
	ign_cut = 0;

//  spi_init();
//  can_init(CAN_BITRATE_250);

  for(;;) //1 kHz loop
  {
    _BIS_SR(LPM0_bits + GIE);               // LPM0, enable interrupts

    /*
     * Out of sleep mode.
     */

    if (shift_state == STATE_IDLE)
    {
    	if (((P1IN & PIN1) != PIN1) && gear_num < 5) //upshift button pressed -- steering wheel button press code goes here
        {
    		shift_state = STATE_UPSHIFT;
        }
        else if (((P1IN & PIN2) != PIN2) && gear_num >= 1) //downshift button pressed --change this to just > 1?
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

    	prev_state = STATE_IDLE;
    }
    else if (shift_state == STATE_UPSHIFT)
    {
	   /* gear_status = 1;
		shift_gear();*/
		//in_neutral = 0;

    	gear_indication();

    	while ((P1IN & PIN1) != PIN1);

		gear_num++;
		gear_indication();

		prev_state = STATE_UPSHIFT;
		shift_state = STATE_IDLE;
    }
    else if (shift_state == STATE_DOWNSHIFT)
    {
		/*gear_status = 2;
		shift_gear();*/
		//in_neutral = 0;

    	gear_indication();

		if (gear_num <= 1) {
			gear_num = 1;
			gear_indication();
		}
		else {
			gear_num--;
			gear_indication();
		}

		while ((P1IN & PIN2) != PIN2);
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
    	/*gear_status = 3;
    	shift_gear();*/
    	shift_state = STATE_NEUTRAL;
    }
    else if (shift_state == STATE_NEUTRAL_FROM_SECOND)
    {
    	/*gear_status = 3;
    	shift_gear();*/
    	gear_num--;
    	shift_state = STATE_NEUTRAL;
    }
    else if (shift_state == STATE_NEUTRAL)
    {
    	gear_indication();
    	prev_state = STATE_NEUTRAL;
		shift_state = STATE_IDLE;
		//in_neutral = 1;
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

#pragma vector=TIMERB0_VECTOR
__interrupt void Timer_B (void)
{
  //P4OUT ^=  PIN4;
}

#pragma vector = PORT2_VECTOR //Paddle is P2.2
__interrupt void clutch_control_ISR (void)
{
	switch(P2IFG & PIN2) {
		case PIN2:
			P2IFG &= ~PIN2;
			//insert scaling for paddle potentiometer to clutch actuator here - full control of clutch
			return;
		default:
			P2IFG = 0;
			return;
	}
}

// Initialize TimerA to wake up processor at 2kHz
void timer_init(void) //remove??
{

  CCR0 = 1483;
  TACTL = TASSEL_2 + ID_3 + MC_1;                  // SMCLK, upmode

  TBCCR0 = 512 - 1;                         // PWM Period
  TBCCTL1 = OUTMOD_7;                       // TBCCR1 reset/set
  TBCCR1 = 384;                             // TBCCR1 PWM duty cycle //fraction of 512 (75% duty cycle)
  TBCCTL2 = OUTMOD_7;
  TBCCR2 = 128;								//fraction of 512 (25% duty cycle)
  TBCTL = TBSSEL_2 + MC_1;                  // SMCLK, up mode

  CCTL0 = CCIE;                             // CCR0 interrupt enabled
  TBCCTL0 = CCIE;                             // CCR0 interrupt enabled
}

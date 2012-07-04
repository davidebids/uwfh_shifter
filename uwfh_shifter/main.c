//#include "msp430.h"
#include <msp430x22x2.h>
//#include "can.h"
//#include "can_data.h"
//https://github.com/davidebids/uwfh_shifter.git

#include "main.h"


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
unsigned char clutch_state, shift_state, prev_state, ign_cut, gear_status;
int gear_num;
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
  P1DIR = 0xFF;								// no inputs needed
  P2DIR = ~(PIN2+PIN1);                		// Set P2.2,1 as input
  P3DIR = ~(PIN5+PIN2);							// Set P3.5,2 as an input

  P4DIR = (char)~(0x82); //~(PIN7+PIN1);						// Set P4.7,1 as input
  P4SEL = PIN4 + PIN5;

  P3SEL = PIN1 + PIN2 + PIN3;
  P3OUT = 0x00;
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
			TBCCR1 = 512; //DIR_FORWARD
		}
	}
	else if (clutch_state == 0) {
		while (clutch_posn > clutch_engaged) {
			TBCCR1 = 512; //DIR_REVERSE - disengage clutch
		}
	}
}

void ignition_cut (void) //required to upshift to neutral?
{
	if (ign_cut == 1) {
		//turn on LED
	}
	else if (ign_cut == 0) {
		//turn off LED
	}
}

void shift_gear (void)
{
	//gear_status = 1 (upshift), gear_status = 2 (downshift), gear_status = 3 (neutral - half shift)

	if (gear_status == 1) {

		ign_cut = 1;
		ignition_cut();

		while (shift_posn < up_posn) { //update value corresponding to actuator position, PWM DIR_FORWARD
			TBCCR1 = 512; //feedback from pot
		}

		ign_cut = 0;
		ignition_cut();

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
	gear_num = 0;
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
    	if (IO_SW0) //upshift button pressed
        {
            shift_state = STATE_UPSHIFT;
        }
        else if (IO_SW0) //downshift button pressed
        {
        	shift_state = STATE_DOWNSHIFT;
        }
        else if (IO_SW0) //clutch paddle engaged -- replace this with ISR?
        {
        	clutch_state = 1;
        	actuate_clutch();
        }
		else if (IO_SW0) //clutch paddle disengaged -- replace this with ISR?
		{
			clutch_state = 0;
			actuate_clutch();
        }
    	else if (IO_SW0 && prev_state != STATE_NEUTRAL) //neutral button pressed
    	{
    		shift_state = STATE_GEAR_CHECK;
    	}

    	prev_state = STATE_IDLE;
    }
    else if (shift_state == STATE_UPSHIFT)
    {
    	if (prev_state == STATE_IDLE)
    	{
            gear_status = 1;
            shift_gear();
        	gear_num++;
        	prev_state = STATE_UPSHIFT;
        	shift_state = STATE_IDLE;
    	}
    	else if (prev_state == STATE_MANUAL_IDLE)
    	{
    		//add code
    	}

    }
    else if (shift_state == STATE_DOWNSHIFT)
    {
    	if (prev_state == STATE_IDLE)
    		{
    			gear_status = 2;
    			shift_gear();

    			if (gear_num <= 1) {
    				gear_num = 1;
    			}
    			else {
        			gear_num--;
    			}

    			shift_state = STATE_IDLE;
    		}
    		else if (prev_state == STATE_MANUAL_IDLE)
    		{
    			//add code
    		}
    }
    else if (shift_state == STATE_MANUAL_IDLE) //add code
    {
    	/*prev_state = STATE_MANUAL_IDLE;
    	shift_state = STATE_UPSHIFT;
    	shift_state = STATE_DOWNSHIFT;*/
    }
    else if (shift_state == STATE_GEAR_CHECK)
    {
		if (gear_num != 1 || gear_num != 2) {
			shift_state = STATE_IDLE;
		}
		else if (gear_num == 1) {
			shift_state = STATE_NEUTRAL_FROM_FIRST;
		}
		else if (gear_num == 2) {
			shift_state = STATE_NEUTRAL_FROM_SECOND;
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
		prev_state = STATE_NEUTRAL;
		shift_state = STATE_IDLE;
    }
    //else - what - throw exception, reset? -- display N/A as gear indication, cause driver to downshift a bunch of times to reset to first gear -- used for when the car shuts off eg. from BRB
  }
}


// Timer Interrupt Service Routine
#pragma vector=TIMERA0_VECTOR
__interrupt void Timer_A (void)
{
  //P2OUT ^= PIN2+PIN1;                       // Toggle P2.2,1

  LPM0_EXIT;
}

#pragma vector=TIMERB0_VECTOR
__interrupt void Timer_B (void)
{
  //P4OUT ^=  PIN4;
}

//#pragma vector=SOME_PIN_VECTOR
__interrupt void clutch_control (void)
{
	//insert scaling for paddle potentiometer to clutch actuator here - full control of clutch
	//add vector enable interrupt - eg. TBIE in main()
}

// Initialize TimerA to wake up processor at 2kHz
void timer_init(void)
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

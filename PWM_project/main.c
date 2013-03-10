#include <msp430.h>

float clutch_posn, in_sample, controller_output, paddle_val;
float ref_posn;// = 560;
float prev_in_sample, out_sample, prev_out_sample; //rads, volts, volts

void clock_init (void)
{
	DCOCTL = CALDCO_12MHZ;
	BCSCTL1 = CALBC1_12MHZ;
}

float readADC(int channel)
{
	float adc_val;

	ADC10CTL0 &= ~ENC;
	ADC10CTL0 = ADC10ON + ADC10SR + ADC10SHT_0 + SREF_0;

	if (channel == 15) { //clutch
		ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_15;
	}
	else if (channel == 13) { //paddle
		ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_13;
	}
	else if (channel == 14) { //shifter
		ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_14;
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

void pwm_config (void)
{
	  P2DIR |= BIT3;
	  P2SEL |= BIT3;

	  TACCR0 = 512-1;                             // PWM Period/2
	  TACCTL1 = OUTMOD_7;                       // TACCR1 toggle/set
	  TACCR1 = 256;                              // TACCR1 PWM duty cycle
	  TACTL = TASSEL_2 + MC_1;                  // SMCLK, up mode
}

void clutch_pwm (void)
{
	if (ref_posn > 560)
	{
		ref_posn = 560;
	}
	else if (ref_posn < 225)
	{
		ref_posn = 225;
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

int main(void)
{
	P2OUT = 0;
	P2DIR = BIT5;

	WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT
	clock_init();

    pwm_config();
    controller_output = 0;

    for (;;)
    {
    	if ((P1IN & BIT1) != BIT1) //upshift button
    	{
    		ref_posn = 560;
    	}
    	else if ((P1IN & BIT2) != BIT2) //downshift button
    	{
    		ref_posn = 225;
    	}
    	//paddle_val = readADC(13);

    	//ref_posn = 2.249*paddle_val - 549.72;

    	TACCR1 = (controller_output/12)*512;
    	clutch_posn = readADC(15);
    	clutch_pwm();
    }
}

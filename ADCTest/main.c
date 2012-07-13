#include <msp430x22x2.h>

float temp;

void main(void)
{
    P2OUT = 0;
	P2DIR = BIT1;

	//Watchdog Off
	WDTCTL = WDTPW + WDTHOLD;

	//Clock Init
	DCOCTL = CALDCO_12MHZ;
	BCSCTL1 = CALBC1_12MHZ;

	//System init
	__bis_SR_register(GIE); //enable general interrupts

	int flag = 0;
	float tempA15, tempA14, tempA2;

	for (;;) {
		if (flag == 0) {
			ADC10CTL0 &= ~ENC;
			ADC10CTL0 = ADC10ON + ADC10SR + ADC10SHT_0 + SREF_0;
			ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_15; //channel 15
			ADC10AE0 = 0x4;
			ADC10AE1 = 0xC0;
			ADC10CTL0 |= ENC;

			// ADC Start Conversion - Software trigger
			ADC10CTL0 |= ADC10SC;

			// Loop until ADC10IFG is set indicating ADC conversion complete
			while ((ADC10CTL0 & ADC10IFG) == 0);

			// Read ADC conversion result from ADC10MEM
			tempA15 = ADC10MEM;
			//need to set ADC10MEM to 0 after reading value?

			flag = 1;
		}
		else if (flag == 1) {
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
			tempA14 = ADC10MEM;

			flag = 2;
		}
		if (flag == 2) {
			ADC10CTL0 &= ~ENC;
			ADC10CTL0 = ADC10ON + ADC10SR + ADC10SHT_0 + SREF_0;
			ADC10CTL1 = CONSEQ_0 + ADC10SSEL_0 + ADC10DIV_0 + SHS_0 + INCH_2; //channel 2
			ADC10AE0 = 0x4;
			ADC10AE1 = 0xC0;
			ADC10CTL0 |= ENC;

			// ADC Start Conversion - Software trigger
			ADC10CTL0 |= ADC10SC;

			// Loop until ADC10IFG is set indicating ADC conversion complete
			while ((ADC10CTL0 & ADC10IFG) == 0);

			// Read ADC conversion result from ADC10MEM
			tempA2 = ADC10MEM;
			//need to set ADC10MEM to 0 after reading value?

			flag = 0;
		}

		if (tempA14 > 0x320 && tempA15 > 0x320 && tempA2 > 0x320) {
			P2OUT |= BIT1;
		}
		else {
			P2OUT &= ~BIT1;
		}
	}

	//-------------------------------
	/*for (;;) {
	    // ADC Start Conversion - Software trigger
	    ADC10CTL0 |= ADC10SC;

	    // Loop until ADC10IFG is set indicating ADC conversion complete
	    while ((ADC10CTL0 & ADC10IFG) == 0);

	    // Read ADC conversion result from ADC10MEM
	    temp = ADC10MEM;

	    if (temp > 0x1F4) {
	        P2OUT |= BIT1;
	    }
	    else {
	    	P2OUT &= ~BIT1;
	    }
	}*/
}

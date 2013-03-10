#include <msp430.h>

int main (void)
 {
	 P2DIR |= BIT3;
	 P2SEL |= BIT3;

	 WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

	 TACCR0 = 512-1;                             // PWM Period/2
	 TACCTL1 = OUTMOD_7;                       // TACCR1 toggle/set
	 TACCR1 = 256;                              // TACCR1 PWM duty cycle
	 TACTL = TASSEL_2 + MC_1;                  // ACLK, up mode*/ - 12kHz

	 __bis_SR_register(LPM0_bits);
 }

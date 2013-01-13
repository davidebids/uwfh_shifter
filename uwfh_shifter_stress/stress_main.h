#ifndef STRESS_MAIN_H_
#define STRESS_MAIN_H_

#include "bits.h"

void clock_init();
void initPortPins();                           // Initialize port pins
void initActuators();
void timer_init();
void gear_indication();
void shift_gear();

#endif /*STRESS_MAIN_H_*/

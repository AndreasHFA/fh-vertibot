#ifndef HELPERFUNCTIONS_H
#define HELPERFUNCTIONS_H

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <stm32f30x.h>
#include <math.h>

#define INIT_DELAY_TIME 150000

void Delay(volatile uint32_t delayCount);

float min(float a, float b);
float max(float a, float b);


#endif /* HELPERFUNCTIONS_H */


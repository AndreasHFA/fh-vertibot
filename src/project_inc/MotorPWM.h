#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H

#include <stm32f30x.h>


/***************Function Prototypes***********************/
void PWM_Motor(uint8_t Motor, uint16_t Value);
void Enable_Motor();
void Disable_Motor();


#endif /*MOTOR_PWM_H */

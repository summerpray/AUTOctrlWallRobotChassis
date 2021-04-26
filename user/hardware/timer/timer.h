#ifndef TIMER_H
#define TIMER_H
#include "main.h"

#define PWM1  TIM4->CCR3     //Ä¦²ÁÂÖ,PD14
#define PWM2  TIM4->CCR4     //Ä¦²ÁÂÖ,PD15

extern void TIM1_Init(uint16_t arr, uint16_t psc);
extern void TIM3_Init(uint16_t arr, uint16_t psc);
extern void TIM6_Init(uint16_t arr, uint16_t psc);
extern void TIM12_Init(uint16_t arr, uint16_t psc);
extern void TIM8_Init(void);
void TIM4_FrictionPwmOutp(int16_t pwm1,int16_t pwm2);//8,9
void TIM4_Init(void);
#endif

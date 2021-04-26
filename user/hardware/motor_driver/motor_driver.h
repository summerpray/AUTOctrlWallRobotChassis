#ifndef __MOTOR_DRIVER_H
#define __MOTOR_DRIVER_H


#include "main.h"


void Motor_driver_Init(void);

//#define HIGH 1
//#define LOW  0

//#define IN1(a)  if(a) \
//	      GPIO_ResetBits(GPIOC,GPIO_Pin_1);\
//      else \
//				GPIO_SetBits(GPIOC,GPIO_Pin_1)
//			

//#define IN2(a)  if(a) \
//	      GPIO_ResetBits(GPIOC,GPIO_Pin_5);\
//      else \
//				GPIO_SetBits(GPIOC,GPIO_Pin_5)
   
			
void motor_1_Run_back(void);
void motor_1_Run_front(void);
void motor_1_Stop(void);
			
			
#endif

#include "motor_driver.h"
#include "stm32f4xx.h"

void Motor_driver_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure); 
    
	  GPIO_SetBits(GPIOA,GPIO_Pin_5);
	 //GPIO_SetBits(GPIOA,GPIO_Pin_4);
}

void motor_1_Stop(void)
{
	
				GPIO_ResetBits(GPIOA,GPIO_Pin_4);
				GPIO_ResetBits(GPIOA,GPIO_Pin_5);
	
}

void motor_1_Run_front(void)
{
	
				GPIO_SetBits(GPIOA,GPIO_Pin_4);
				GPIO_ResetBits(GPIOA,GPIO_Pin_5);
	
}


void motor_1_Run_back(void)
{
	
				GPIO_ResetBits(GPIOA,GPIO_Pin_4);
				GPIO_SetBits(GPIOA,GPIO_Pin_5);
	
}

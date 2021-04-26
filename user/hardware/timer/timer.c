#include "timer.h"
#include "stm32f4xx.h"

void TIM1_Init(uint16_t arr, uint16_t psc)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource13, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource14, GPIO_AF_TIM1);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_Init(GPIOE, &GPIO_InitStructure);

    RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_TIM1, DISABLE);

    TIM_TimeBaseInitStructure.TIM_Period = arr - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc - 1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_Pulse = 1000;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);

    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM1, ENABLE);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);

    TIM_Cmd(TIM1, ENABLE);
}

void TIM3_Init(uint16_t arr, uint16_t psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource5, GPIO_AF_TIM3);

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM3, DISABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM3_NVIC;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_TimeBaseInitStructure.TIM_Period = arr - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc - 1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

    /* TIM3 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC2Init(TIM3, &TIM_OCInitStructure);

    TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM3, ENABLE);

    TIM_Cmd(TIM3, ENABLE);
}

void TIM6_Init(uint16_t arr, uint16_t psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM6, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM6, DISABLE);

    TIM_DeInit(TIM6);
    TIM_TimeBaseInitStructure.TIM_Period = arr - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc - 1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM6, &TIM_TimeBaseInitStructure);
    TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = TIM6_NVIC;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM6, ENABLE); //ʹ�ܶ�ʱ��6
}

void TIM12_Init(uint16_t arr, uint16_t psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM12, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM12, DISABLE);

    TIM_TimeBaseInitStructure.TIM_Period = arr - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc - 1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

    TIM_TimeBaseInit(TIM12, &TIM_TimeBaseInitStructure);

    TIM_Cmd(TIM12, ENABLE);
}

/**
  * @brief  TIM8,舵机(弹仓)初始化
  * @param  void
  * @retval void
  * @attention TIM8->CCR3,PI7
  */
void TIM8_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;

	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI,ENABLE);
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);	
	
	  GPIO_InitStructure.GPIO_Pin =GPIO_Pin_7;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	  GPIO_Init(GPIOI,&GPIO_InitStructure);
	
  	GPIO_PinAFConfig(GPIOI,GPIO_PinSource7, GPIO_AF_TIM8); 

    TIM_TimeBaseInitStructure.TIM_Prescaler     = 1800-1;
	  TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up;	 //向上计数
	  TIM_TimeBaseInitStructure.TIM_Period        = 2000;                 //20MS	计数周期
	  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;		     //时钟分割,不为1则*2
	  TIM_TimeBaseInit(TIM8,&TIM_TimeBaseInitStructure);	
	
	  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;		             //PWM1模式
	  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		//输出比较使能
	  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputState_Disable;	//互补输出比较失能
	  TIM_OCInitStructure.TIM_Pulse = 0;		                      //捕获脉冲值
	  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;		//输出极性高
	  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;		//互补输出极性低
	  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;		
	  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;	
		
		TIM_OC3Init(TIM8,&TIM_OCInitStructure);		//通道3
		
		TIM_OC3PreloadConfig(TIM8,TIM_OCPreload_Enable);
				 
	  TIM_ARRPreloadConfig(TIM8,ENABLE);
	
	  TIM_CtrlPWMOutputs(TIM8,ENABLE);
	
	  TIM_Cmd(TIM8,ENABLE);
}

/**
  * @brief  摩擦轮电机配置(初始化+解锁)
  * @param  void
  * @retval void
  * @attention PWM1\2直接定义为CCR寄存器,PD14->CH3,PD15->CH4
  */
void TIM4_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource15, GPIO_AF_TIM4);    
	
	
	
	TIM_TimeBaseInitStructure.TIM_Prescaler = 90-1;//1MHz
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;		
	TIM_TimeBaseInitStructure.TIM_Period =2500-1;      //  1MHz/2500Hz  400Hz
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;		
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;		
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;		
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputState_Disable;	
	TIM_OCInitStructure.TIM_Pulse = 0;		
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;    		
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;		
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;		
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Set;		
	TIM_OC3Init(TIM4,&TIM_OCInitStructure);		
	TIM_OC3PreloadConfig(TIM4,TIM_OCPreload_Enable);
	
	TIM_OC4Init(TIM4,&TIM_OCInitStructure);
	TIM_OC4PreloadConfig(TIM4,TIM_OCPreload_Enable);
				 
	TIM_ARRPreloadConfig(TIM4,ENABLE);
	TIM_Cmd(TIM4,ENABLE);
	TIM_CtrlPWMOutputs(TIM4,ENABLE);
	
	
	PWM1 = 1000;		//解锁摩擦轮,大于1000
	PWM2 = 1000;    //
}

/**
  * @brief  摩擦轮输出函数
  * @param  void
  * @retval pwm1  pwm2
  * @attention 左右都是正,以后接线的时候注意三条线的接法
  */
void TIM4_FrictionPwmOutp(int16_t pwm1,int16_t pwm2)//8,9
{
	PWM1 = pwm1+1000;	//持续输出时要大于1ms
	PWM2 = pwm2+1000;
}

#ifndef __USART7_H
#define __USART7_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "sys.h" 
#include "stm32f4xx.h"

#define EN_USART7_RX 			1		//使能（1）/禁止（0）串口1接收
	
#define VISION_BUFFER_LEN   	100    //定义了裁判系统数据接收长度



void usart7_init(u32 bound);
void UART7_SendChar(uint8_t cData);//发送一个字节的数据
#endif

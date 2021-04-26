#include "rc.h"
#include "stm32f4xx.h"

void RC_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
        /* -------------- Enable Module Clock Source ----------------------------*/
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA2, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

        RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, ENABLE);
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, DISABLE);

        GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1); //PB7  usart1 rx
                                                                  /* -------------- Configure GPIO ---------------------------------------*/
        {
                GPIO_InitTypeDef GPIO_InitStructure;
                USART_InitTypeDef USART_InitStructure;
                GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
                GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
                GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
                GPIO_Init(GPIOB, &GPIO_InitStructure);

                USART_DeInit(USART1);

                USART_InitStructure.USART_BaudRate = 100000;
                USART_InitStructure.USART_WordLength = USART_WordLength_8b;
                USART_InitStructure.USART_StopBits = USART_StopBits_1;
                USART_InitStructure.USART_Parity = USART_Parity_Even;
                USART_InitStructure.USART_Mode = USART_Mode_Rx;
                USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
                USART_Init(USART1, &USART_InitStructure);

                USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);  //DMA串口接收请求中断使能

                USART_ClearFlag(USART1, USART_FLAG_IDLE);  //清除空闲线路中断标志位
                USART_ITConfig(USART1, USART_IT_IDLE, ENABLE); //使能串口空闲中断

                USART_Cmd(USART1, ENABLE);   //使能串口
        }

        /* -------------- Configure NVIC ---------------------------------------*/
        {                                                                              //配置中断函数就要配置优先级
                NVIC_InitTypeDef NVIC_InitStructure;
                NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
                NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = RC_NVIC;                        
                NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
                NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
                NVIC_Init(&NVIC_InitStructure);
        }

        //DMA2 stream5 ch4  or (DMA2 stream2 ch4)    !!!!!!! P206 of the datasheet
        /* -------------- Configure DMA -----------------------------------------*/
        {
                DMA_InitTypeDef DMA_InitStructure;
                DMA_DeInit(DMA2_Stream2);

                DMA_InitStructure.DMA_Channel = DMA_Channel_4;                              //DMA通道4，在8个通道中选择一个
                DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);       //外设地址
                DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rx1_buf;                  //存储器0地址，双缓存模式还要使用M1AR
                DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                     //外设到存储器模式
                DMA_InitStructure.DMA_BufferSize = dma_buf_num;                             //数据传输量，以外设数据项为单位
                DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;            //外设地址保持不变
                DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                     //存储器地址递增
                DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;     //外设数据位宽8位
                DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;             //存储器数据位宽8位
                DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                             //循环模式
                DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;                     //高优先级
                DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                      //禁止FIFO模式
                DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
                DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                 //单次传输
                DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;         //单次传输
                DMA_Init(DMA2_Stream2, &DMA_InitStructure);
                DMA_DoubleBufferModeConfig(DMA2_Stream2, (uint32_t)rx2_buf, DMA_Memory_0);  //双缓冲模式，buffer0先被传输
                DMA_DoubleBufferModeCmd(DMA2_Stream2, ENABLE);                              //使能双缓冲模式
                DMA_Cmd(DMA2_Stream2, DISABLE); //Add a disable
                DMA_Cmd(DMA2_Stream2, ENABLE);
        }
}
void RC_unable(void)
{
        USART_Cmd(USART1, DISABLE);
}
void RC_restart(uint16_t dma_buf_num)
{
        USART_Cmd(USART1, DISABLE);
        DMA_Cmd(DMA2_Stream2, DISABLE);
        DMA_SetCurrDataCounter(DMA2_Stream2, dma_buf_num);

        USART_ClearFlag(USART1, USART_FLAG_IDLE);

        DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
        DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
        DMA_Cmd(DMA2_Stream2, ENABLE);
        USART_Cmd(USART1, ENABLE);
}

#ifndef _CAP_TASK_H
#define _CAP_TASK_H

#include "main.h"
#include "stdbool.h"
#include "CAN_receive.h"


void Super_cap(void);
void Cap_UpdateTarget_Power(int16_t power );
void Cap_Init(void);
void Cap_75(void);
void Super_cap(void);
void CAN1_Cap_Send(uint16_t temPower);


#endif

#ifndef _MAGAZINE_H
#define _MAGAZINE_H

#include "main.h"

//���ֿ��ؽǶȶ�Ӧ��PWMֵ

#define Magazine_Close_Angle 1849
#define Magazine_Open_Angle 1800

typedef enum
{
    MAG_OPEN = 0,
    MAG_CLOSE = 1,

} MagState;

void Magazine_Ctrl(void);
void Magazine_Servo(int16_t pwm);
void Magazine_Key_Ctrl(void);
bool Magezine_Rc_Switch(void);
/*******************���ָ�������*************************/
bool Magazine_IfOpen(void);
bool Magazine_IfWait(void);

#endif

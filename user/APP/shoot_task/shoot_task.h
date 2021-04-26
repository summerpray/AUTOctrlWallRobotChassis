#ifndef SHOOT_TASK__H
#define SHOOT_TASK__H

#include "main.h"
#include  <math.h>
#include <stdlib.h>

#define 	REVOL_SPEED_GRID      8			  //拨盘格数
#define   AN_BULLET         (24576.0f)		//单个子弹电机位置增加值
#define   REVOL_SPEED_RATIO   2160        //电机轴一秒转一圈,2160转子转速,60*36,乘射频再除以拨盘格数就可得相应射频下的转速


/* 打符摄像头预编译 */
#define    BUFF_CAM_CHAS	0
#define    BUFF_CAM_GIMB	1
#define    BUFF_CAM_TYPE	BUFF_CAM_GIMB

/******拨盘,控制逻辑与云台类似*********/

//拨盘电机模式,位置环与速度环
//#define    REVOL_LOOP_POSI     0
//#define    REVOL_LOOP_SPEED    1
//uint16_t   Revolver_mode;//拨盘模式选择
//拨盘电机模式,位置环与速度环
typedef enum
{
	REVOL_POSI_MODE  = 0,
	REVOL_SPEED_MODE = 1,
}eRevolverCtrlMode;


//遥控模式下的一些标志位
#define    FRIC_STEP0    0
#define    FRIC_STEP1    1
#define    FRIC_STEP2    2


//速度选择
#define FRI_OFF  	0
#define FRI_LOW  	1		//低速
#define FRI_MID  	2		//中速	
#define FRI_HIGH 	3		//高速
#define FRI_MAD  	4		//打符射速
#define FRI_SENTRY  5		//哨兵射速


typedef enum
{
	SHOOT_NORMAL       =  0,//射击模式选择,默认不动
	SHOOT_SINGLE       =  1,//单发
	SHOOT_TRIPLE       =  2,//三连发
	SHOOT_HIGHTF_LOWS  =  3,//高射频低射速
	SHOOT_MIDF_HIGHTS  =  4,//中射频高射速
	SHOOT_BUFF         =  5,//打符模式
	SHOOT_AUTO         =  6,//自瞄自动射击
}eShootAction;

//摩擦轮电机ID
typedef enum
{
	Fric_Left =  0,  
	Fric_Right = 1,  
	
}Fric_Motor_t;

void shoot_task(void *pvParameters); //任务函数
void REVOLVER_Rest(void);    //拨盘重启
void REVOLVER_InitArgument(void);  //拨盘参数初始化
void REVOLVER_Rc_Ctrl(void);  //遥控器控制
void REVOLVER_Key_Ctrl(void);  //键盘控制
bool REVOLVER_Rc_Switch(void); //拨盘遥控打弹
void Revolver_Angle_Rest(void); //拨盘角度清零
void REVOL_PositStuck(void);//位置环卡弹处理
void REVOL_SpeedStuck(void);//速度环卡弹处理
void REVOL_PositionLoop(void); //位置环PID
void REVOL_SpeedLoop(void); //速度环PID
void Fric_SpeedLoop(Fric_Motor_t Fric_Motor);//摩擦轮速度环PID
void Fric_Speed_PID_Calculate(void);//摩擦轮速度环PID计算
void REVOL_UpdateMotorAngleSum(void);//统计转过角度总和
void REVOLVER_UpdateMotorSpeed(int16_t speed);//中断读转速
void REVOLVER_UpdateMotorAngle(int16_t angle);//中断读角度
void Reset_Fric(void);//复位摩擦轮
void ReFric(void);//用于摩擦轮清空枪膛
void Fric_Power_Change(void);//改变摩擦轮速度
void REVOLVER_CANbusCtrlMotor(void);//can发送函数


void Fric_Key_Ctrl(void); //键盘模式摩擦轮的控制
void SHOOT_NORMAL_Ctrl(void);  //键盘模式下的普通模式
void SHOOT_SINGLE_Ctrl(void); //左键单发模式
void SHOOT_TRIPLE_Ctrl(void); //连发控制
void SHOOT_HIGHTF_LOWS_Ctrl(void);//高射频低射速控制
void SHOOT_MIDF_HIGHTS_Ctrl(void);//中射频高射速控制
void SHOOT_AUTO_Ctrl(void);//自瞄射击控制
void SHOOT_BUFF_Ctrl(void);//打符射击控制,仅用于摄像头位于底盘
void SHOOT_BUFF_Ctrl_Gimbal(void);//打符射击控制，摄像头在云台
void REVOLVER_KeyPosiCtrl(void);//键盘模式拨盘位置环控制
void REVOLVER_KeySpeedCtrl(void);//键盘模式拨盘速度环控制
void Fric_RC_Ctrl(void);

void REVOLVER_SHAKE(void);//测试平台用
bool FRIC_RcSwitch(void);
void Friction_Ramp(void);
void Fric_mode(uint16_t speed);
#endif 

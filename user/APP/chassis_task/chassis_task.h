/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       chassis.c/h
  * @brief      完成底盘控制任务。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef CHASSISTASK_H
#define CHASSISTASK_H
#include "main.h"
#include "CAN_Receive.h"
#include "Gimbal_Task.h"
#include "pid.h"
#include "Remote_Control.h"
#include "user_lib.h"
//任务开始空闲一段时间
#define CHASSIS_TASK_INIT_TIME 357



extern void chassis_task(void *pvParameters);


//底盘电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 10000.0f
#define M3505_MOTOR_SPEED_PID_KI 0.01f
#define M3505_MOTOR_SPEED_PID_KD 3.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

#define M3505_MOTOR_SPEED_KEY_PID_KP 10000.0f//6000
#define M3505_MOTOR_SPEED_KEY_PID_KI 0.01f
#define M3505_MOTOR_SPEED_KEY_PID_KD 3.0f
#define M3505_MOTOR_SPEED_KEY_PID_MAX_OUT 16000.0f
#define M3505_MOTOR_SPEED_KEY_PID_MAX_IOUT 2000.0f

//底盘旋转跟随PID 键盘模式
#define CHASSIS_FOLLOW_GIMBAL_PID_KEY_KP 0.3f  //0.15
#define CHASSIS_FOLLOW_GIMBAL_PID_KEY_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KEY_KD 0.01f
#define CHASSIS_FOLLOW_GIMBAL_PID_KEY_MAX_OUT 3.0f //3.0
#define CHASSIS_FOLLOW_GIMBAL_PID_KEY_MAX_IOUT 0.1f

//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 0.15f  //0.15
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.01f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 3.0f //3.0
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.1f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f    //一阶低通滤波参数
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

//底盘任务控制间隔 0.002s
#define CHASSIS_CONTROL_TIME 0.002

//键盘模式底盘功率切换控制
#define LOW 0
#define HIGH 1

//底盘电机最大速度
#define MAX_WHEEL_SPEED 2.0f  

//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 2.0f
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 1.0f
//底盘运动过程最大旋转速度
#define NORMAL_MAX_CHASSIS_SPEED_Z 1.0f


//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.004f
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.004f

#define CHASSIS_RC_DEADLINE 10

#define HIGH_RATE 1.6f
#define LOW_RATE 1.9f

//摇摆原地不动摇摆最大角度(rad)
#define SWING_NO_MOVE_ANGLE 0.5f
//摇摆过程底盘运动最大角度(rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f

//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 0.001f


//m3508转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机需要更换比例
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

void chassis_feedback_update(void);   //电机数据更新
/*--------------------myself-------------*/
/*----------------------------------------------------------对轮子转速的限制-----------------------------------------------------------------------------------------*/

#define 	  Omni_Speed_Max              6000     //底盘水平移动速度限幅,防止键盘模式下速度超过这个值
#define		  STANDARD_MAX_NORMAL         9000     //平地开车最快速度，防止摇杆比例*660超过这个值
#define     Omni_SupCap_Max             10000    //按住shift加速时候的速度，后续测试再更改

/*---------------------------------------------------------------------------------------------------------------------------------------------------------------*/

/*----------------------------------------------------------底盘扭头的相关参数--------------------------------------------------------------------------------------*/

#define     theta_error                 0
#define		  REVOLVE_MAX_NORMAL          9000     //平地扭头最快速度
#define     REVOLVE_KD                 (350.f)
#define     REVOLVE_ANGLE               35
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------*/

/*----------------------------------------------不同模式下,斜坡函数对应的时间值,一般用普通速度就行-----------------------------------------------------------------*/

#define     TIME_INC_NORMAL             10	       //键盘斜坡,越大增加速度越快,完成时间越短
#define     TIME_DEC_NORMAL             500        //键盘斜坡,越大减小速度越快(一般要比INC大一点,这样松开键盘能更快为0,太大则会造成底盘停下来的时候跳跃)

#define     TIME_INC_SLOW               1		       //补弹模式下速度变化快慢
#define     TIME_INC_UPUP               3		       //手动爬坡模式下速度变化快慢

#define     TIME_INC_SALTATION          1          //突然变速情况下速度变化快慢

#define     REVOLVE_SLOPE_NORMAL        80         //底盘普通模式斜坡,越大越快,完成时间越短
#define     REVOLVE_SLOPE_CORGI         50         //底盘扭屁股模式斜坡,越大越快,完成时间越短
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------*/

#define     LIMIT_CHASSIS_MAX        9000       //功率限制情况下底盘单个电机最大输出
  
/*----------------------------------------------不同模式下的最高速度限制--------------------------------------------------------------------------------------*/

#define    REVOLVE_MAX_CORGI         9000       //底盘扭屁股最快速度,太大会让角度过大

#define    STANDARD_MAX_SLOW         1500	      //补弹模式下水平移动速度
#define    REVOLVE_MAX_SLOW          2000       //补弹模式下扭头速度
 
#define    STANDARD_MAX_UPUP       3000       //手动爬坡模式下水平移动速度
#define    REVOLVE_MAX_UPUP        9000       //手动爬坡模式下扭头速度

#define    LIMIT_CHASSIS_MAX         9000       //功率限制情况下底盘单个电机最大输出
#define    CHAS_CURRENT_LIMIT        36000     //四个轮子的速度总和最大值,单个输出*4,限功率调比例可用

/*---------------------------------------------------------------------------------------------------------------------------------------------------------------*/
//底盘电流限幅
#define     iTermChassis_Max            3000     //微分限幅

/*----------------------------------------------底盘规格相关参数修改--------------------------------------------------------------------------------------*/

#define RADIUS     76      //麦轮半径
#define PERIMETER  478     //麦轮周长
#define WHEELTRACK 512     //左右轮距
#define WHEELBASE  378     //前后轴距

#define GIMBAL_Y_OFFSET 0 //云台靠后30

#define CHASSIS_DECELE_RATIO (1.0f/19.0f)  //电机减数比

#define RADIAN_COEF 57.3f  
/*---------------------------------------------------------------------------------------------------------------------------------------------------------------*/
typedef enum
{
	
	CHASSIS_MECH_MODE = 0,//机械
	CHASSIS_GYRO_MODE = 1,//陀螺仪,底盘跟随云台
	CHASSIS_SHAKE_MODE = 2, //摇摆模式
	CHASSIS_TOP_MODE =3,
	
} ChassisCtrlMode;

typedef enum	
{
	CHASSIS_NORMAL   = 0,//普通模式，正常前进
	CHASSIS_SHAKE    = 1,//扭屁股模式
	CHASSIS_REVOLVE  = 2,//小陀螺模式
	CHASSIS_UP       = 3,//爬坡模式
	CHASSIS_MISS     = 4,//自动闪避模式
	CHASSIS_SLOW     = 5, //低俗补弹模式
	CHASSIS_BUFF     = 6,//打符模式，底盘为机械模式
 } ChassisActionMode;

//底盘电机ID
typedef enum
{
	
	LEFT_FRON_201 = 0,  // 左前
	RIGH_FRON_202 = 1,  // 右前
	LEFT_BACK_203 = 2,  // 左后
	RIGH_BACK_204 = 3,  // 右后
	
} ChassisWheel;


void Chassis_Motor_Speed_PID(void) ;
void Chassis_Motor_Key_PID( ChassisWheel Wheel ) ; //按键模式时候的PID
void Chassis_MotorOutput(void);
void CHASSIS_REST(void);
void Chassis_Set_Mode(void); //遥控器设置模式
void Chassis_Rc_Control(void);//遥控器数据引入为 Chassis_Move_X  Chassis_Move_Y
void Chassis_Set_Contorl(void);//不同模式不同处理方式
void Chassis_Set_key_Contorl(void); //键盘模式不同模式不同处理
void Chassis_Omni_Move_Calculate(void);
void Chassis_RC_Ctrl(void);
void Chassis_Key_Ctrl(void);
void Chassis_Power_Change(void); //换挡
void Chassis_NORMAL_Mode_Ctrl(void);
void CHASSIS_CANSend(void);
void CHASSIS_UpdateMotorAngle( ChassisWheel Wheel, int16_t angle );
void CHASSIS_UpdateMotorSpeed( ChassisWheel Wheel, int16_t speed );
void CHASSIS_UpdateMotorCur( ChassisWheel Wheel, int16_t current );
void Chassis_Init(void);
void Angle_error(void);
float Chassis_SpeedZ_PID(int16_t ErrorReal, float kp);
void Chassis_Motor_Speed_PID_KEY(void);
float Chassis_Z_Corgi(float get, float set);
void CHASSIS_CORGI_Mode_Ctrl_Time(int16_t sRevolMax, int16_t sRevolRamp);
void CHASSIS_CORGI_Mode_Ctrl(int16_t sRevolMax, int16_t sRevolRamp);
void CHASSIS_UPUP_Mode_Ctrl(void);
void CHASSIS_MISS_Mode_Ctrl(void);
void Chassis_Keyboard_Move_Calculate( int16_t sMoveMax, int16_t sMoveRamp );
void Chassis_Keyboard_Move_TOP_Calculate(int16_t sMoveMax, int16_t sMoveRamp);
float Chassis_Key_MoveRamp( uint8_t status, int16_t *time, int16_t inc, int16_t dec );
void Chassis_Mouse_Move_Calculate( int16_t sRevolMax );
bool CHASSIS_IfActiveMode(void);//获取底盘移动模式
bool Chassis_IfSHAKE(void);//是否在扭屁股

#endif

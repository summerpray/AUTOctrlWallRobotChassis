/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       chassis.c/h
  * @brief      完成底盘控制任务。
  * @note        键盘少写一个shif加速功能 ，要结合超级电容
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
#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "Gimbal_Task.h"
#include "start_task.h"
#include "rc.h"
#include "main.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "shoot_task.h"
#include "led.h"
#include "super_cap.h"
#include "arm_math.h"

#include "CAN_Receive.h"

#include "pid.h"
#include "stdio.h"
#include <stdlib.h>
#include "Remote_Control.h"
#include "usart6.h"

#include "kalman.h"
#include "judge.h"
#include "magazine.h"

#define rc_deadline_limit(input, output, dealine)        \
	{                                                    \
		if ((input) > (dealine) || (input) < -(dealine)) \
		{                                                \
			(output) = (input);                          \
		}                                                \
		else                                             \
		{                                                \
			(output) = 0;                                \
		}                                                \
	}

extern RC_ctrl_t rc_ctrl;
ChassisCtrlMode Chassis_Mode;	  //机械模式 陀螺仪模式
ChassisActionMode Chassis_Action; //底盘动作形态
extKalman_t Chassis_Error_Kalman; //定义一个kalman指针

PidTypeDef motor_pid[4];
PidTypeDef motor_key_pid[4];
PidTypeDef chassis_angle_pid;
PidTypeDef chassis_angle_key_pid;

first_order_filter_type_t chassis_cmd_slow_set_vx;
first_order_filter_type_t chassis_cmd_slow_set_vy;

//小陀螺
bool Chass_Switch_F = 1;
u8 Chass_Key_F_Change = 0;

//自动闪避
#define MISS_MAX_TIME 1000	 //自动闪避最大归位时间,单位2*ms
uint32_t Miss_Mode_Time = 0; //自动闪避已过时长

//主任务
uint8_t remote_change = TRUE;

//底盘功率更改变量
uint16_t Chassis_Power_Level = 0;

float temp[2] = {0,0};

void chassis_task(void *pvParameters)
{
	//    //空闲一段时间
	vTaskDelay(CHASSIS_TASK_INIT_TIME); //时间为357

	for (;;)
	{
		if (SYSTEM_GetSystemState() == SYSTEM_STARTING) //系统状态为开始
		{
			Chassis_Init();
			Cap_Init();
		}
		else
		{
			chassis_feedback_update();
			if (IF_RC_SW2_UP) //键盘模式
			{			
				//Chassis_Power_Change();    //底盘功率换挡
				Chassis_Key_Ctrl();		   //底盘按键功能	
			}
			else //遥控器模式
			{
				Chassis_Set_Mode();		   //切换模式
				Chassis_Rc_Control();

				/*切换键盘模式时候的变量初始化*/
				Chassis_Action = CHASSIS_NORMAL;
				Chass_Switch_F = 1;		//重置扭腰
				Chass_Key_F_Change = 0; //重置扭腰
				remote_change = TRUE;	//标记切回了遥控模式
			}
		}
		
		Chassis_Set_Contorl(); //底盘移动计算
		Chassis_Omni_Move_Calculate(); //底盘全向运动分析
		if (IF_RC_SW2_UP)
		{
			//Chassis_MotorOutput();
			Chassis_Motor_Speed_PID_KEY(); //PID计算
		}
		else
		{
			Chassis_Motor_Speed_PID(); //PID计算
		}

		CHASSIS_CANSend(); //发送电流
		vTaskDelay(TIME_STAMP_2MS);
	}
}

/*----------------------myself-----------------------*/

/*----------------------------------------------***定义底盘全向移动变量***-------------------------------------------------*/

float Chassis_Move_X; //前后
float Chassis_Move_Y; //左右平移
float Chassis_Move_Z; //左右旋转

fp32 angle_swing_set; //摇摆角度

//陀螺仪模式下底盘偏差(相对YAW的中心分离机械角度)
float Chassis_Gyro_Error;

fp32 motor_chassis_speed[4];	 //定义四个轮子的速度，用于更新   m/s
fp32 motor_key_chassis_speed[4]; //定义四个轮子的速度，用于更新   m/s
/*----------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------***速度限幅***-------------------------------------------------------------*/

fp32 vx_max_speed; //前进方向最大速度 单位m/s
fp32 vx_min_speed; //前进方向最小速度 单位m/s
fp32 vy_max_speed; //左右方向最大速度 单位m/s
fp32 vy_min_speed; //左右方向最小速度 单位m/s
fp32 vz_max_speed; //左右方向最大速度 单位m/s
fp32 vz_min_speed; //左右方向最小速度 单位m/s

float theta;
float theta_error1;
float theta_error2;

//键盘模式 限幅
float Chassis_Standard_Move_Max;					//底盘前后左右平移限速
float Chassis_Revolve_Move_Max;						//底盘左右旋转限速,根据不同运动模式实时更改,所以不能用宏定义
float Chassis_Final_Output_Max = LIMIT_CHASSIS_MAX; //限制PID运算最终输出值,底盘功率限制,根据电机转速实时变化
//键盘模式 响应  初始化里要定义
float kKey_Mech_Chassis_Standard; //定义机械模式平移键盘响应
float kKey_Mech_Chassis_Revolve;  //定义机械模式旋转键盘响应
float kKey_Gyro_Chassis_Standard; //定义陀螺仪模式平移键盘响应
float kKey_Gyro_Chassis_Revolve;  //定义陀螺仪模式旋转键盘响应
/*----------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------***斜坡参数***-------------------------------------------------------------*/
float Slope_Chassis_Move_Z; //斜坡计算出的移动变量,这是目标输出量的实时斜坡值

uint16_t timeInc;									//斜坡增加变化时间
uint16_t timeInc_Saltation;							//前后方向突变下的斜坡增加量,比正常情况下要小
int16_t timeXFron, timeXBack, timeYLeft, timeYRigh; //键盘  s  w  d   a

//键盘模式下全向移动计算,斜坡量
float Slope_Chassis_Move_Fron, Slope_Chassis_Move_Back;
float Slope_Chassis_Move_Left, Slope_Chassis_Move_Righ;

//键盘模式下扭头斜坡,主要用在扭屁股模式中
float Slope_Chassis_Revolve_Move;

/*----------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------***定义PID参数***--------------------------------------------------------------*/

//测量云台角度来计算底盘的欧拉角
extern float Cloud_Angle_Measure[2][3]; //  pitch/yaw    mech/gyro

//底盘期望速度
float Chassis_Speed_Target[4]; //ID

//底盘速度误差
float Chassis_Speed_Error[4]; //ID

//底盘测量角度
int16_t Chassis_Angle_Measure[4];

//底盘测量速度
float Chassis_Speed_Measure[4];

//底盘测量速度
int16_t Chassis_Current_Measure[4];

//另一个pid

//底盘测量速度
int16_t Chassis_Current_Measure[4];

//底盘速度误差和
float Chassis_Speed_Error_Sum[4]; //ID
float Chassis_Speed_Error_NOW[4], Chassis_Speed_Error_LAST[4];

//单级PID参数
float Chassis_Speed_kpid[4][3]; //	motorID kp/ki/kd

float pTermChassis[4], iTermChassis[4], dTermChassis[4]; //ID
float pidTermChassis[4];								 //ID,计算输出量

/*----------------------------------------------------------------------------------------------------------------------*/

//底盘电机输出量
float Chassis_Final_Output[4];

/**
  * @brief  初始化电机参数
  * @param  void
  * @retval void
  * @attention  Chassis_Move_X Chassis_Move_Y Chassis_Move_Z 归0
  */
void CHASSIS_REST(void)
{
	Slope_Chassis_Move_Z = 0; //扭屁股实时输出斜坡
	Chassis_Move_X = 0;
	Chassis_Move_Y = 0;
	Chassis_Move_Z = 0;
}

/**
  * @brief  底盘初始化  主要是PID 限幅初始化
  * @param  void
  * @retval void
  * @attention  
  *              
  */
void Chassis_Init(void)
{
	//底盘速度环pid值
	const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};				   //初始化遥控器模式KP、KI、KD
	const static fp32 motor_speed_key_pid[3] = {M3505_MOTOR_SPEED_KEY_PID_KP, M3505_MOTOR_SPEED_KEY_PID_KI, M3505_MOTOR_SPEED_KEY_PID_KD}; //初始化键盘模式KP、KI、KD
	//底盘旋转环pid值
	const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};				   //初始化地盘旋转跟随KP、KI、KD
	const static fp32 chassis_yaw_key_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KEY_KP, CHASSIS_FOLLOW_GIMBAL_PID_KEY_KI, CHASSIS_FOLLOW_GIMBAL_PID_KEY_KD}; //初始化地盘旋转跟随KP、KI、KD 的键盘模式PID

	const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM}; //设置x方向一阶低通滤波参数
	const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM}; //设置y方向一阶低通滤波参数
	uint8_t i;															 //循环变量,只在后面循环用

	Chassis_Mode = CHASSIS_MECH_MODE; //初始化底盘模式为机械模式

	//初始化PID 运动
	for (i = 0; i < 4; i++)
	{

		PID_Init(&motor_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT); //第二个变量代表这里是位置环
		PID_Init(&motor_key_pid[i], PID_POSITION, motor_speed_key_pid, M3505_MOTOR_SPEED_KEY_PID_MAX_OUT, M3505_MOTOR_SPEED_KEY_PID_MAX_IOUT);
	}

	//初始化旋转PID
	PID_Init(&chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
	PID_Init(&chassis_angle_key_pid, PID_POSITION, chassis_yaw_key_pid, CHASSIS_FOLLOW_GIMBAL_PID_KEY_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_KEY_MAX_IOUT);
	//用一阶滤波代替斜波函数生成
	first_order_filter_init(&chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
	first_order_filter_init(&chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

	vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;	//底盘前进的最大速度
	vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X; //底盘后退的最大速度

	vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;	//底盘左平移的最大速度   //方向debug看
	vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y; //底盘右平移的最大速度

	vz_max_speed = NORMAL_MAX_CHASSIS_SPEED_Z;	//底盘顺时针旋转的最大速度
	vz_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Z; //底盘逆时针旋转的最大速度

	/**************键盘模式限幅参数*************************/
	Chassis_Standard_Move_Max = STANDARD_MAX_NORMAL; //键盘水平移动限幅  9000
	Chassis_Revolve_Move_Max = REVOLVE_MAX_NORMAL;	 //键盘左右扭头限幅,可以稍微大一点,否则云台扭太快会导致云台撞到限位,太大又会导致到目标位置后晃动 9000

	/**************键盘按键灵敏度*************************/
	kKey_Mech_Chassis_Revolve = 60;	 //键盘机械模式下扭头速度响应快慢,别太大,不然扭头太快
	kKey_Gyro_Chassis_Revolve = -10; //-8.1;//注意正负,键盘陀螺仪模式下底盘跟随云台转动的速度,别给太大,否则震荡会很严重

	/**************PID参数*************************/
	Chassis_Speed_kpid[LEFT_FRON_201][KP] = 18;
	Chassis_Speed_kpid[LEFT_FRON_201][KI] = 0.08; //0.08;
	Chassis_Speed_kpid[LEFT_FRON_201][KD] = 0;

	Chassis_Speed_kpid[RIGH_FRON_202][KP] = 18;
	Chassis_Speed_kpid[RIGH_FRON_202][KI] = 0.08; //0.08;
	Chassis_Speed_kpid[RIGH_FRON_202][KD] = 0;

	Chassis_Speed_kpid[LEFT_BACK_203][KP] = 18;
	Chassis_Speed_kpid[LEFT_BACK_203][KI] = 0.08; //0.08;
	Chassis_Speed_kpid[LEFT_BACK_203][KD] = 0;

	Chassis_Speed_kpid[RIGH_BACK_204][KP] = 18;
	Chassis_Speed_kpid[RIGH_BACK_204][KI] = 0.08; //0.08;
	Chassis_Speed_kpid[RIGH_BACK_204][KD] = 0;

	chassis_feedback_update();
}

/**
  * @brief  更新底盘电机数据   速度  与云台的叠加角度
  * @param  void
  * @retval void
  * @attention  
  *              
  */
void chassis_feedback_update(void)
{
	uint8_t i = 0;
	for (i = 0; i < 4; i++)
	{
		motor_chassis_speed[i] = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * Chassis_Speed_Measure[i];	 //转换成m/s的转速
		motor_key_chassis_speed[i] = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * Chassis_Speed_Measure[i]; //转换成m/s的转速
	}
	Chassis_Gyro_Error = GIMBAL_GetOffsetAngle();
}

/**
  * @brief  遥控器切换模式，并修改相应变量
  * @param  void
  * @retval void
  * @attention  
  *              
  */
void Chassis_Set_Mode(void)
{
	if (IF_RC_SW2_MID)
	{
		Chassis_Mode = CHASSIS_GYRO_MODE; //S2下 陀螺仪模式
		flow_led_on(3);
		flow_led_off(2);
	}
	else if (IF_RC_SW2_UP)
	{
		Chassis_Mode = CHASSIS_MECH_MODE; //其他机械模式
		angle_swing_set = 0;
		flow_led_on(2);
		flow_led_off(3);
	}
	else if (IF_RC_SW2_DOWN)
	{
		Reset_Fric();
		Chassis_Mode = CHASSIS_TOP_MODE; //其他机械模式
		angle_swing_set = 0;
		flow_led_on(2);
		flow_led_off(3);
	}
	//	if(IF_RC_SW1_DOWN)
	//	{
	//		Chassis_Mode = CHASSIS_SHAKE_MODE;
	//		flow_led_on(2);
	//		flow_led_on(3);
	//	}
}

/**
  * @brief  遥控器数据引入为 Chassis_Move_X  Chassis_Move_Y
  * @param  void
  * @retval void
  * @attention  
  *              
  */
void Chassis_Rc_Control(void)
{
	//遥控器原始通道值
	int16_t vx_channel, vy_channel;
	fp32 vx_set_channel, vy_set_channel;
	//死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0               当遥控器拨动值较小时输出为0，防止误触
	rc_deadline_limit(rc_ctrl.rc.ch[3], vy_channel, CHASSIS_RC_DEADLINE);
	rc_deadline_limit(rc_ctrl.rc.ch[2], vx_channel, CHASSIS_RC_DEADLINE);

	temp[0] = rc_ctrl.rc.ch[3];
	temp[1] = rc_ctrl.rc.ch[2];

	vx_set_channel = vx_channel * -CHASSIS_VX_RC_SEN; //将遥控器的值转化为机器人运动的速度
	vy_set_channel = vy_channel *  CHASSIS_VY_RC_SEN;

	//一阶低通滤波代替斜波作为底盘速度输入
	first_order_filter_cali(&chassis_cmd_slow_set_vx, vx_set_channel);
	first_order_filter_cali(&chassis_cmd_slow_set_vy, vy_set_channel);

	//停止信号，不需要缓慢加速，直接减速到零
	if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
	{
		chassis_cmd_slow_set_vx.out = 0.0f;
	}

	if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
	{
		chassis_cmd_slow_set_vy.out = 0.0f;
	}
	Chassis_Move_X = chassis_cmd_slow_set_vx.out; //遥控器输出X轴方向的值
	Chassis_Move_Y = chassis_cmd_slow_set_vy.out; //遥控器输出Y轴方向的值

}

/**
  * @brief  不同模式不同处理方式
  * @param  void
  * @retval void
  * @attention  
  *              
  */
void Chassis_Set_Contorl(void)
{
	float Chassis_Move_X1;
	float Chassis_Move_Y1;
	if (Chassis_Mode == CHASSIS_SHAKE_MODE) //扭腰模式
	{
		fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;

		float C_A_M_Y_M = 0;

		chassis_follow_gimbal_yaw_control(); //是否需要摇摆      在这里得到angle_swing_set

		//旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
		sin_yaw = arm_sin_f32(Cloud_Angle_Measure[YAW][MECH]);
		cos_yaw = arm_cos_f32(Cloud_Angle_Measure[YAW][MECH]);
#if YAW_POSITION == YAW_DOWN //轴上轴下读取的电机角度值是反的
		C_A_M_Y_M = Cloud_Angle_Measure[YAW][CHANGE];
#else
		C_A_M_Y_M = Cloud_Angle_Measure[YAW][MECH];
#endif

		Chassis_Move_X = cos_yaw * Chassis_Move_X + sin_yaw * Chassis_Move_Y; //当我们控制移动时需要用到
		Chassis_Move_Y = -sin_yaw * Chassis_Move_X + cos_yaw * Chassis_Move_Y;

		//计算旋转PID角速度
		Chassis_Move_Z = PID_Calc(&chassis_angle_pid, C_A_M_Y_M, rad_format(angle_swing_set));
		//速度限幅
		Chassis_Move_X = fp32_constrain(Chassis_Move_X, vx_min_speed, vx_max_speed);
		Chassis_Move_Y = fp32_constrain(Chassis_Move_Y, vy_min_speed, vy_max_speed);
	}

	else if (Chassis_Mode == CHASSIS_GYRO_MODE)
	{
		Angle_error();
		//计算旋转的角速度
		Chassis_Move_Z = -PID_Calc(&chassis_angle_pid, 0.0f, Chassis_Gyro_Error); //假设当前实际值为0，设定误差值为目标值；如果设定当前目标值为0，实际值为误差值，此时误差值应取反（或运动方向取反）
		//设置底盘运动的速度
		Chassis_Move_X = fp32_constrain(Chassis_Move_X, vx_min_speed, vx_max_speed);
		Chassis_Move_Y = fp32_constrain(Chassis_Move_Y, vy_min_speed, vy_max_speed);
	}
	else if (Chassis_Mode == CHASSIS_MECH_MODE)
	{
		Angle_error();
		Chassis_Move_Z = CHASSIS_WZ_RC_SEN * rc_ctrl.rc.ch[0];
		Chassis_Move_Z = fp32_constrain(Chassis_Move_Z, vz_min_speed, vz_max_speed);
		Chassis_Move_X = fp32_constrain(Chassis_Move_X, vx_min_speed, vx_max_speed);
		Chassis_Move_Y = fp32_constrain(Chassis_Move_Y, vy_min_speed, vy_max_speed);
	}
	else if (Chassis_Mode == CHASSIS_TOP_MODE)
	{
		if ((fabs(Chassis_Move_X) < 0.001) && (fabs(Chassis_Move_Y) < 0.001))
		{
			Chassis_Move_Z = 1.0f;
		}
		else
		{
			Chassis_Move_Z = 1.0f;

			Angle_error();
			Chassis_Move_X1 = (cos(theta) * Chassis_Move_X) + (-sin(theta) * Chassis_Move_Y);
			Chassis_Move_Y1 = (sin(theta) * Chassis_Move_X) + (cos(theta) * Chassis_Move_Y);
			Chassis_Move_X = fp32_constrain(Chassis_Move_X1, vx_min_speed, vx_max_speed);
			Chassis_Move_Y = fp32_constrain(Chassis_Move_Y1, vy_min_speed, vy_max_speed);
			Chassis_Move_Z = fp32_constrain(Chassis_Move_Z, vz_min_speed, vz_max_speed);
		}
	}
}

/**
  * @brief  换挡
  * @param  void
  * @retval void
  * @attention  
  *              
  */
void Chassis_Power_Change(void)
{
	if(IF_KEY_PRESSED_CTRL || IF_KEY_PRESSED_SHIFT)
	{ 
		if (IF_KEY_PRESSED_SHIFT)
		{
			Chassis_Power_Level ++;
			if (Chassis_Power_Level > 1)
			{
				Chassis_Power_Level = 1;
			}
		}
		else if (IF_KEY_PRESSED_CTRL)
		{
			Chassis_Power_Level = 0;
			
		}
	}
}


/**
  * @brief  不同模式不同处理方式
  * @param  void
  * @retval void
  * @attention  
  *              
  */
void Chassis_Set_key_Contorl(void)
{
	float Chassis_Move_X1;
	float Chassis_Move_Y1;
	if (Chassis_Mode == CHASSIS_SHAKE_MODE) //扭腰模式
	{
		fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;

		float C_A_M_Y_M = 0;

		chassis_follow_gimbal_yaw_control(); //是否需要摇摆      在这里得到angle_swing_set

		//旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
		sin_yaw = arm_sin_f32(Cloud_Angle_Measure[YAW][MECH]);
		cos_yaw = arm_cos_f32(Cloud_Angle_Measure[YAW][MECH]);
#if YAW_POSITION == YAW_DOWN //轴上轴下读取的电机角度值是反的
		C_A_M_Y_M = Cloud_Angle_Measure[YAW][CHANGE];
#else
		C_A_M_Y_M = Cloud_Angle_Measure[YAW][MECH];
#endif

		Chassis_Move_X = cos_yaw * Chassis_Move_X + sin_yaw * Chassis_Move_Y; //当我们控制移动时需要用到
		Chassis_Move_Y = -sin_yaw * Chassis_Move_X + cos_yaw * Chassis_Move_Y;

		//计算旋转PID角速度
		Chassis_Move_Z = PID_Calc(&chassis_angle_pid, C_A_M_Y_M, rad_format(angle_swing_set));
		//速度限幅
		Chassis_Move_X = fp32_constrain(Chassis_Move_X, vx_min_speed, vx_max_speed);
		Chassis_Move_Y = fp32_constrain(Chassis_Move_Y, vy_min_speed, vy_max_speed);
	}
	else if (Chassis_Mode == CHASSIS_GYRO_MODE)
	{
		Angle_error();
		//计算旋转的角速度
		Chassis_Move_Z = -PID_Calc(&chassis_angle_key_pid, 0.0f, Chassis_Gyro_Error); //假设当前实际值为0，设定误差值为目标值；如果设定当前目标值为0，实际值为误差值，此时误差值应取反（或运动方向取反）
		//设置底盘运动的速度
		Chassis_Move_X = fp32_constrain(Chassis_Move_X, vx_min_speed, vx_max_speed);
		Chassis_Move_Y = fp32_constrain(Chassis_Move_Y, vy_min_speed, vy_max_speed);
	}
	else if (Chassis_Mode == CHASSIS_MECH_MODE)
	{
		Angle_error();
		Chassis_Move_Z = CHASSIS_WZ_RC_SEN * rc_ctrl.rc.ch[0];
		Chassis_Move_Z = fp32_constrain(Chassis_Move_Z, vz_min_speed, vz_max_speed);
		Chassis_Move_X = fp32_constrain(Chassis_Move_X, vx_min_speed, vx_max_speed);
		Chassis_Move_Y = fp32_constrain(Chassis_Move_Y, vy_min_speed, vy_max_speed);
	}
	else if (Chassis_Mode == CHASSIS_TOP_MODE)
	{
		if ((fabs(Chassis_Move_X) < 0.001) && (fabs(Chassis_Move_Y) < 0.001))
		{
			Chassis_Move_Z = 1.5f;
		}
		else
		{
			Chassis_Move_Z = 0.5f;
			Angle_error();

			Chassis_Move_X1 = (cos(theta) * Chassis_Move_X) + (-sin(theta) * Chassis_Move_Y);
			Chassis_Move_Y1 = (sin(theta) * Chassis_Move_X) + (cos(theta) * Chassis_Move_Y);
			Chassis_Move_X = fp32_constrain(Chassis_Move_X1, vx_min_speed, vx_max_speed);
			Chassis_Move_Y = fp32_constrain(Chassis_Move_Y1, vy_min_speed, vy_max_speed);
			Chassis_Move_Z = fp32_constrain(Chassis_Move_Z, vz_min_speed, vz_max_speed);
			
		}
	}
}

/**
  * @brief  键盘控制，得出底盘全向运动速度
  * @param  void
  * @retval void
  * @attention 模式选择,进入某模式后记得写退出到普通模式的判断
  * 无按键按下会一直处于自动闪避模式,模式切换的按键按下则处于模式切换选择模式
  */
void Chassis_Key_Ctrl(void)
{
	if (remote_change == TRUE) //刚从遥控模式切过来,默认为陀螺仪模式
	{
		Chassis_Mode = CHASSIS_GYRO_MODE;
		remote_change = FALSE;
	}
	Chassis_NORMAL_Mode_Ctrl(); //在普通模式里执行模式切换，也是正常行驶的模式
	switch (Chassis_Action)		//枚举类型未列完整会警告
	{
	case CHASSIS_NORMAL:			//普通模式
		Chassis_NORMAL_Mode_Ctrl(); //在普通模式里执行模式切换，也是正常行驶的模式
		Chassis_Keyboard_Move_Calculate(STANDARD_MAX_NORMAL, TIME_INC_NORMAL);
		break;
		/*----------------------------小陀螺模式---------------------------	*/
	case CHASSIS_REVOLVE:
		if (!IF_KEY_PRESSED_F)
		{
			Chass_Switch_F = 1;
		}

		if (IF_KEY_PRESSED_F && Chass_Switch_F == 1)
		{
			Chass_Switch_F = 0;
			Chass_Key_F_Change++;
			Chass_Key_F_Change %= 2; //按基数次有效，偶数次无效，按一次开再按一次关
		}

		if (Chass_Key_F_Change)
		{
			Chassis_Mode = CHASSIS_TOP_MODE; //陀螺仪模式,底盘跟随云台动
			Chassis_Keyboard_Move_Calculate(STANDARD_MAX_NORMAL, TIME_INC_NORMAL);
			//这里要写一个小陀螺模式的函数
		}
		else
		{
			Chassis_Mode = CHASSIS_GYRO_MODE; 
			Chassis_Action = CHASSIS_NORMAL; //退出小陀螺模式，进入普通模式
		}
		break;
		/*----------------------------爬坡模式---------------------------	*/
	case CHASSIS_UP:
		CHASSIS_UPUP_Mode_Ctrl();
		break;

		//		case CHASSIS_MISS:    //自动闪避模式
		//			CHASSIS_MISS_Mode_Ctrl();     //可能不好用
		//		break;
		/*----------------------------低速补弹模式----------------------------*/
	case CHASSIS_SLOW:
		if (Magazine_IfOpen() != TRUE) //弹仓关闭
		{
			Chassis_Action = CHASSIS_NORMAL; //底盘退出补弹模式
		}
		else
		{
			Chassis_Mode = CHASSIS_MECH_MODE; //补弹时底盘进入机械模式

			Chassis_Keyboard_Move_Calculate(STANDARD_MAX_SLOW, TIME_INC_SLOW);
			Chassis_Mouse_Move_Calculate(REVOLVE_MAX_SLOW);
		}
		break;
		/*----------------------------打符模式----------------------------*/
	case CHASSIS_BUFF:
		if (GIMBAL_IfBuffHit() != TRUE)
		{
			Chassis_Action = CHASSIS_NORMAL;  //退出打符模式
			Chassis_Mode = CHASSIS_GYRO_MODE; //切回陀螺仪模式
		}
		else
		{
			Chassis_Mode = CHASSIS_MECH_MODE; //打符底盘进入机械模式
			CHASSIS_REST();					  //目标速度置0
		}
		break;
	}
}

/**
  * @brief  底盘键盘模式选择,按键响应
  * @param  void
  * @retval void
  * @attention 底盘键盘控制状态下的所有模式切换都在这
  */
void Chassis_NORMAL_Mode_Ctrl(void)
{
	/*----------------------------------------------****用来标记按键就按一下****--------------------------------------------------------*/
	if (!IF_KEY_PRESSED_F) //F松开
	{
		Chass_Switch_F = 1;
	}

	/*---------------------------------------------------------------------------------------------------------------------------------*/

	/*----------------------------------------------****F按一下则进入扭屁股模式****----------------------------------------------------*/
	//	if (IF_KEY_PRESSED_F && !IF_KEY_PRESSED_CTRL  && Chass_Switch_F == 1)
	//	{
	//		Chass_Switch_F = 0;
	//		Chass_Key_F_Change ++;
	//		Chass_Key_F_Change %= 2;
	//		Chassis_Action = CHASSIS_SHAKE;//记得写个能退出扭屁股模式的函数
	//	}

	/*---------------------------------------------------------------------------------------------------------------------------------*/

	/*----------------------------------------------****R按一下则进入小陀螺模式****----------------------------------------------------*/

	else if (IF_KEY_PRESSED_F && !IF_KEY_PRESSED_CTRL && Chass_Switch_F == 1)
	{
		Chass_Switch_F = 0;
		Chass_Key_F_Change++;
		Chass_Key_F_Change %= 2;
		Chassis_Action = CHASSIS_REVOLVE; //记得写个能退出小陀螺模式的函数
	}

	/*---------------------------------------------------------------------------------------------------------------------------------*/

	/*---------------------------------------------****W和Ctrl一起按则进入爬坡模式****----------------------------------------------------*/

	else if (IF_KEY_PRESSED_W && IF_KEY_PRESSED_CTRL)
	{
		Chassis_Action = CHASSIS_UP; //爬坡模式
	}

	/*---------------------------------------------------------------------------------------------------------------------------------*/

	/*---------------------------------------------****弹舱只要开启就进入低速补弹模式****----------------------------------------------------*/
	else if (Magazine_IfOpen() == TRUE) //弹仓开启,进入补弹模式
	{
		Chassis_Action = CHASSIS_SLOW;
	}

	/*---------------------------------------------------------------------------------------------------------------------------------*/

	/*---------------------------------------------****确认打符模式就进入打符模式****----------------------------------------------------*/
	//换挡
	else if (GIMBAL_IfBuffHit() == TRUE) //打符模式
	{
		Chassis_Action = CHASSIS_BUFF;
	}

	/*---------------------------------------------------------------------------------------------------------------------------------*/
		/* 	以下内容没有必要 */
		/*-----------------------------------------------****按住Ctrl则进入机械模式****----------------------------------------------------*/
		// if (IF_KEY_PRESSED_CTRL)
		// {
		// 	Chassis_Mode = CHASSIS_MECH_MODE;
		// }
		// else //松开CTRL进入陀螺仪模式
		// {
		// 	Chassis_Mode = CHASSIS_GYRO_MODE;
		// }
		// Chassis_Keyboard_Move_Calculate(STANDARD_MAX_NORMAL, TIME_INC_NORMAL); //设置速度最大值与斜坡时间
		// Chassis_Mouse_Move_Calculate(REVOLVE_MAX_NORMAL);
}

/*-------------------------------------------鼠标键盘控制计算Chassis_Move_X |Chassis_Move_Y |Chassis_Move_Z-------------------------------------------------*/
/**
  * @brief  鼠标控制底盘旋转,键盘QEC控制快速转圈
  * @param  速度最大输出量 
  * @retval void
  * @attention  鼠标控制左右旋转
  */

void Chassis_Mouse_Move_Calculate(int16_t sRevolMax)
{
	//	static int16_t sErrorPrev = 0;//上次偏离误差
	float sErrorReal = 0; //yaw偏离中心误差

	Chassis_Revolve_Move_Max = sRevolMax; //左右旋转限速

	if (Chassis_Mode == CHASSIS_GYRO_MODE) //陀螺仪模式
	{
		sErrorReal = GIMBAL_GetOffsetAngle(); //获取实时偏差,用于扭屁股下的底盘位置补偿
		Chassis_Move_Z = -PID_Calc(&chassis_angle_key_pid, 0.0f, sErrorReal);
		//Chassis_Move_Z = Chassis_SpeedZ_PID(sErrorReal, kKey_Gyro_Chassis_Revolve);
		//Chassis_Move_Z = PID_Calc(&chassis_angle_pid, 0.0f, Chassis_Gyro_Error);
	}
	else //机械模式
	{
		Chassis_Move_Z = constrain_float(MOUSE_X_MOVE_SPEED * kKey_Mech_Chassis_Revolve, -Chassis_Revolve_Move_Max, Chassis_Revolve_Move_Max);
	}
}

/**
  * @brief  键盘模式下底盘运动计算
  * @param  速度最大输出量    增加速度(最大293)
  * @retval void
  * @attention  键盘控制前后左右平移,平移无机械和陀螺仪模式之分
  *             需要获取时间来进行斜坡函数计算
  */
void Chassis_Keyboard_Move_Calculate(int16_t sMoveMax, int16_t sMoveRamp)
{
	static portTickType ulCurrentTime = 0;
	static uint32_t ulDelay = 0;
	float k_rc_z = 1; //根据Z速度调节前后左右平移移速比

	Chassis_Standard_Move_Max = sMoveMax; //调整速度限幅,水平移动
	timeInc = sMoveRamp;

	ulCurrentTime = xTaskGetTickCount(); //当前系统时间

	if (fabs(Chassis_Move_Z) > 800) //扭头速度越快,前后速度越慢,防止转弯半径过大
	{
		k_rc_z = ((Chassis_Revolve_Move_Max - fabs(Chassis_Move_Z) + 800) * (Chassis_Revolve_Move_Max - fabs(Chassis_Move_Z) + 800)) / (Chassis_Revolve_Move_Max * Chassis_Revolve_Move_Max);

		k_rc_z = constrain_float(k_rc_z, 0, 1);
	}
	else
	{
		k_rc_z = 1;
	}

	// if (ulCurrentTime >= ulDelay) //每10ms变化一次斜坡量
	// {
	// 	ulDelay = ulCurrentTime + TIME_STAMP_10MS;

	// 	if (Chassis_Action == CHASSIS_NORMAL && !KEY_PRESSED_OFFSET_SHIFT) //只有一般模式下才判断速度突变情况,防止打滑
	// 	{
	// 		if (IF_KEY_PRESSED_W) //等超级电容出来再测试电容放电时是否要全力加速
	// 		{
	// 			timeXBack = 0; //按下前进则后退斜坡归零,方便下次计算后退斜坡
	// 			//前进X是正数,可在此加入超级电容按键判断
	// 			if (Chassis_Move_X < sMoveMax / 2.5)		//转向突变,刚开始的一小段时间斜坡降低,防止轮子打滑浪费功率
	// 			{											//利用速度是否到最大速度的1/5来判断不知道是否合理
	// 				timeInc_Saltation = TIME_INC_SALTATION; //以爬坡模式处理速度方向突变
	// 			}
	// 			else //已经过了打滑时间且轮子有了一定的速度
	// 			{
	// 				timeInc_Saltation = sMoveRamp;
	// 			}
	// 		}

	// 		if (IF_KEY_PRESSED_S)
	// 		{
	// 			timeXFron = 0; //同理
	// 			//后退X是负
	// 			if (Chassis_Move_X > (-sMoveMax) / 2.5) //转向突变,刚开始的一小段时间斜坡降低,防止轮子打滑浪费功率
	// 			{
	// 				timeInc_Saltation = TIME_INC_SALTATION; //以爬坡模式处理速度方向突变
	// 			}
	// 			else //已经过了打滑时间且轮子有了一定的速度
	// 			{
	// 				timeInc_Saltation = sMoveRamp;
	// 			}
	// 		}

	// 		if (IF_KEY_PRESSED_D)
	// 		{
	// 			timeYRigh = 0;
	// 		}

	// 		if (IF_KEY_PRESSED_A)
	// 		{
	// 			timeYLeft = 0;
	// 		}

	// 		//键盘模式下全向移动,斜坡量计算,注意正负,最大输出量*斜坡比例得到缓慢增加的值,模拟摇杆
	// 		//前后的增加斜坡是变化的
	// 		Slope_Chassis_Move_Fron = (int16_t)(Chassis_Standard_Move_Max *
	// 											Chassis_Key_MoveRamp(IF_KEY_PRESSED_D, &timeXFron, timeInc / 1.5, TIME_DEC_NORMAL));

	// 		Slope_Chassis_Move_Back = (int16_t)(-Chassis_Standard_Move_Max *
	// 											Chassis_Key_MoveRamp(IF_KEY_PRESSED_A, &timeXBack, timeInc / 1.5, TIME_DEC_NORMAL));

	// 		//左右的增加斜坡跟前后不一样,别搞错
	// 		Slope_Chassis_Move_Left = (int16_t)(+Chassis_Standard_Move_Max *
	// 											Chassis_Key_MoveRamp(IF_KEY_PRESSED_S, &timeYRigh, timeInc_Saltation, TIME_DEC_NORMAL));

	// 		Slope_Chassis_Move_Righ = (int16_t)(Chassis_Standard_Move_Max *
	// 											Chassis_Key_MoveRamp(IF_KEY_PRESSED_W, &timeYLeft, timeInc_Saltation, TIME_DEC_NORMAL));

	// 		Chassis_Move_X = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) * k_rc_z / 4000; //将遥控器的值转化为机器人运动的速度
	// 		Chassis_Move_Y = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) * k_rc_z / 4000;
	// 	}
	//	else //其他模式不需要进行速度方向突变特殊处理
		
	if (IF_KEY_PRESSED_W)
	{
		timeXBack = 0; //按下前进则后退斜坡归零,方便下次计算后退斜坡
	}

	if (IF_KEY_PRESSED_S)
	{
		timeXFron = 0; //同理
	}
	if (IF_KEY_PRESSED_D)
	{
		timeYLeft = 0;
	}

	if (IF_KEY_PRESSED_A)
	{
		timeYRigh = 0;
	}
	int16_t vx_channel, vy_channel;
	fp32 vx_set_channel, vy_set_channel;
	Slope_Chassis_Move_Righ = (int16_t)(-1024 *
										Chassis_Key_MoveRamp(IF_KEY_PRESSED_D, &timeYRigh, timeInc / 1.5, TIME_DEC_NORMAL * 0.8));

	Slope_Chassis_Move_Left = (int16_t)(+1024 *
										Chassis_Key_MoveRamp(IF_KEY_PRESSED_A, &timeYLeft, timeInc / 1.5, TIME_DEC_NORMAL * 0.8));

	Slope_Chassis_Move_Back = (int16_t)(-1024 *
										Chassis_Key_MoveRamp(IF_KEY_PRESSED_S, &timeXBack, timeInc, TIME_DEC_NORMAL));

	Slope_Chassis_Move_Fron = (int16_t)(+1024 *
										Chassis_Key_MoveRamp(IF_KEY_PRESSED_W, &timeXFron, timeInc, TIME_DEC_NORMAL));

	vx_channel = (Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ) * k_rc_z;
	vy_channel = (Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron) * k_rc_z; //将遥控器的值转化为机器人运动的速度
		
	vx_set_channel = vx_channel *  CHASSIS_VX_RC_SEN; //将遥控器的值转化为机器人运动的速度
	vy_set_channel = vy_channel *  CHASSIS_VY_RC_SEN;

	//一阶低通滤波代替斜波作为底盘速度输入
	first_order_filter_cali(&chassis_cmd_slow_set_vx, vx_set_channel);
	first_order_filter_cali(&chassis_cmd_slow_set_vy, vy_set_channel);
	if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
	{
		chassis_cmd_slow_set_vx.out = 0.0f;
	}

	if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
	{
		chassis_cmd_slow_set_vy.out = 0.0f;
	}

	Chassis_Move_X = chassis_cmd_slow_set_vx.out; //遥控器输出X轴方向的值
	Chassis_Move_Y = chassis_cmd_slow_set_vy.out; //遥控器输出Y轴方向的值
		// Slope_Chassis_Move_Righ = (int16_t)(-Chassis_Standard_Move_Max *
		// 									Chassis_Key_MoveRamp(IF_KEY_PRESSED_D, &timeYRigh, timeInc, TIME_DEC_NORMAL * 0.8));

		// Slope_Chassis_Move_Left = (int16_t)(+Chassis_Standard_Move_Max *
		// 									Chassis_Key_MoveRamp(IF_KEY_PRESSED_A, &timeYLeft, timeInc, TIME_DEC_NORMAL * 0.8));

		// Slope_Chassis_Move_Back = (int16_t)(-Chassis_Standard_Move_Max *
		// 									Chassis_Key_MoveRamp(IF_KEY_PRESSED_S, &timeXBack, timeInc, TIME_DEC_NORMAL));

		// Slope_Chassis_Move_Fron = (int16_t)(+Chassis_Standard_Move_Max *
		// 									Chassis_Key_MoveRamp(IF_KEY_PRESSED_W, &timeXFron, timeInc, TIME_DEC_NORMAL));
		
		
	temp[0] = Chassis_Move_X;
	temp[1] = Chassis_Move_Y;
		
	
}
/*-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

/*-----------------------------------------------------------------键盘模式相关动作模式----------------------------------------------------------------------------------------*/
/*-------------------------------------------------扭腰，小陀螺（未写），手动爬坡，自动闪避，低速补弹（未写），打符模式（未写）----------------------------------------------*/
/**
  * @brief  扭屁股模式(位置不变版)
  * @param  速度最大输出量    增加到最大量所需时间
  * @retval void
  * @attention  不管时间，扭到位了就换向
  */
//扭屁股换向选择
#define CORGI_BEGIN 0
#define CORGI_LEFT 1
#define CORGI_RIGH 2

uint16_t stateCorgi = CORGI_BEGIN; //标记往哪扭,默认不扭
bool IfCorgiChange = FALSE;		   //是否扭到了另一边
float corgi_angle_target = 0;	   //左右目标角度
void CHASSIS_CORGI_Mode_Ctrl(int16_t sRevolMax, int16_t sRevolRamp)
{
	//	int16_t  sAngleError   = 0;
	//	float    vectorXBuffer = 0;
	//	float    vectorYBuffer = 0;
	//	float    angle         = 0;

	//	Chassis_Revolve_Move_Max = sRevolMax;//最大速度设置
	//	Slope_Chassis_Revolve_Move = sRevolRamp;//扭头斜坡设置

	//	sAngleError = GIMBAL_GetOffsetAngle();//计算yaw中心偏离,保证底盘扭屁股的时候也能跟随云台动

	//	//计算角度偏差,机械角度转换成欧拉角,用于前进速度补偿
	////	angle = -(float)sAngleError / (float)4096 * PI;
	//	angle = -(float)sAngleError;

	//	vectorXBuffer = Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron;//暂存实时X变化
	//	vectorYBuffer = Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ;

	//	Chassis_Move_X = vectorXBuffer * cos( angle ) - vectorYBuffer * sin( angle );
	//	Chassis_Move_Y = vectorXBuffer * sin( angle ) + vectorYBuffer * cos( angle );

	//秘技:反复横跳......
	//	switch (stateCorgi)
	//	{
	//		case CORGI_BEGIN:	//以后可以试试用个随机(标志位不停取反),来让开始扭头的方向随机
	//			corgi_angle_target = -900;//可改最大移动角度,自动闪避模式下被击打时的扭腰角度
	//			IfCorgiChange = FALSE;
	//			stateCorgi    = CORGI_LEFT;
	//		break;
	//
	//		case CORGI_LEFT:
	//			corgi_angle_target = -1024;//可改最大移动角度
	//			IfCorgiChange = FALSE;

	//			if (sAngleError < -700)//角度误差大于700
	//			{
	//					stateCorgi = CORGI_RIGH;
	//				  IfCorgiChange = TRUE;//标记可以换向
	//			}
	//		break;
	//
	//		case CORGI_RIGH:
	//			corgi_angle_target = 1024;
	//			IfCorgiChange = FALSE;

	//			if (sAngleError > 700)//角度误差大于700
	//			{
	//				stateCorgi = CORGI_LEFT;
	//				IfCorgiChange = TRUE;//标记可以换向
	//			}
	//		break;
	//	}
	//  corgi_angle_target = 0.3;

	//	Chassis_Move_Z = Chassis_SpeedZ_PID( (sAngleError - corgi_angle_target), kKey_Gyro_Chassis_Revolve);
	fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;

	float C_A_M_Y_M = 0;

	chassis_follow_gimbal_yaw_control(); //是否需要摇摆      在这里得到angle_swing_set

	//旋转控制底盘速度方向，保证前进方向是云台方向，有利于运动平稳
	sin_yaw = arm_sin_f32(Cloud_Angle_Measure[YAW][MECH]);
	cos_yaw = arm_cos_f32(Cloud_Angle_Measure[YAW][MECH]);
#if YAW_POSITION == YAW_DOWN //轴上轴下读取的电机角度值是反的
	C_A_M_Y_M = Cloud_Angle_Measure[YAW][CHANGE];
#else
	C_A_M_Y_M = Cloud_Angle_Measure[YAW][MECH];
#endif
	Chassis_Move_X = cos_yaw * Chassis_Move_X + sin_yaw * Chassis_Move_Y; //当我们控制移动时需要用到
	Chassis_Move_Y = -sin_yaw * Chassis_Move_X + cos_yaw * Chassis_Move_Y;

	//计算旋转PID角速度
	Chassis_Move_Z = PID_Calc(&chassis_angle_pid, C_A_M_Y_M, rad_format(angle_swing_set));
	//速度限幅
	Chassis_Move_X = fp32_constrain(Chassis_Move_X, vx_min_speed, vx_max_speed);
	Chassis_Move_Y = fp32_constrain(Chassis_Move_Y, vy_min_speed, vy_max_speed);
}

/**
  * @brief  扭屁股模式(时间不变版)
  * @param  速度最大输出量    增加到最大量所需时间
  * @retval void
  * @attention  不管扭没扭到位，时间到了就换向
  */
void CHASSIS_CORGI_Mode_Ctrl_Time(int16_t sRevolMax, int16_t sRevolRamp)
{
	static uint32_t twist_count;
	static float chass_yaw_set;
	float angle = 0;
	int16_t sAngleError = 0;
	float vectorXBuffer = 0;
	float vectorYBuffer = 0;

	static int16_t twist_period = 800; //500*2ms为一个扭腰周期
	static int16_t twist_angle = 40;   //左右最大分离欧拉角度设置

	twist_count++;

	Chassis_Revolve_Move_Max = sRevolMax;	 //最大速度设置
	Slope_Chassis_Revolve_Move = sRevolRamp; //扭头斜坡设置

	sAngleError = GIMBAL_GetOffsetAngle(); //计算yaw中心偏离,保证底盘扭屁股的时候也能跟随云台动

	//计算角度偏差,机械角度转换成欧拉角,用于前进速度补偿
	angle = -(float)sAngleError / (float)8192 * 360; //6.283f;

	vectorXBuffer = Slope_Chassis_Move_Back + Slope_Chassis_Move_Fron; //暂存实时X变化
	vectorYBuffer = Slope_Chassis_Move_Left + Slope_Chassis_Move_Righ;

	//正弦函数扭屁股
	Chassis_Move_X = vectorXBuffer * cos(angle / 57.3f) - vectorYBuffer * sin(angle / 57.3f);
	Chassis_Move_Y = vectorXBuffer * sin(angle / 57.3f) + vectorYBuffer * cos(angle / 57.3f);

	chass_yaw_set = twist_angle * sin(2 * PI / twist_period * twist_count); //计算底盘目标分离角度
	Chassis_Move_Z = -Chassis_Z_Corgi(angle, chass_yaw_set);
}

/**
  * @brief  扭屁股专用（时间不变版）
  * @param  当前云台偏离量，目标偏移量
  * @retval 旋转速度
  * @attention 
  */
float Chassis_Z_Corgi(float get, float set)
{
	float error[2];
	float z = 0;

	error[NOW] = set - get;

	z = 15 * ((error[NOW] * 8) + 10 * (error[NOW] - error[LAST])); //PD计算
	z = constrain_float(z, -5000, 5000);

	error[LAST] = error[NOW];

	return z;
}

/**
  * @brief  手动爬坡模式
  * @param  void
  * @retval void
  * @attention  
  */
void CHASSIS_UPUP_Mode_Ctrl(void)
{
	if (!IF_KEY_PRESSED_W || !IF_KEY_PRESSED_CTRL) //松开任意一个退出爬坡模式

	{
		Chassis_Action = CHASSIS_NORMAL; //底盘退出爬坡模式
	}
	else
	{
		Chassis_Mode = CHASSIS_GYRO_MODE; //陀螺仪模式

		Chassis_Keyboard_Move_Calculate(STANDARD_MAX_UPUP, TIME_INC_UPUP);
		Chassis_Mouse_Move_Calculate(REVOLVE_MAX_UPUP);
	}
}

/**
  * @brief  自动闪避模式
  * @param  void
  * @retval void
  * @attention  
  */
void CHASSIS_MISS_Mode_Ctrl(void)
{
	int16_t sAngleError = 0;
	sAngleError = GIMBAL_GetOffsetAngle(); //计算yaw中心偏离,保证底盘扭屁股的时候也能跟随云台动

	//有按键按下或者长时间没再受到攻击则退出自动闪避,保证操作跟手
	if (IF_KEY_PRESSED || Miss_Mode_Time > MISS_MAX_TIME)
	{
		Chassis_Action = CHASSIS_NORMAL; //底盘切换成正常模式
		Miss_Mode_Time = 0;
	}
	else
	{
		Chassis_Mode = CHASSIS_GYRO_MODE;

		if (JUDGE_IfArmorHurt() == TRUE //装甲板数据更新,即受到新的伤害
			|| IfCorgiChange == FALSE)	//屁股没有扭到旁边
		{
			//开启扭屁股一次
			CHASSIS_CORGI_Mode_Ctrl(REVOLVE_MAX_CORGI, REVOLVE_SLOPE_CORGI);
			Miss_Mode_Time = 0;
		}
		else
		{
			Slope_Chassis_Move_Z = 0; //扭屁股实时输出斜坡
			Chassis_Move_X = 0;
			Chassis_Move_Y = 0;
			Chassis_Move_Z = Chassis_SpeedZ_PID((sAngleError - corgi_angle_target), kKey_Gyro_Chassis_Revolve);
			Miss_Mode_Time++;
		}
	}
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
/**
  * @brief  底盘全向算法,计算各电机转速
  * @param  void
  * @retval void
  * @attention  (201/202/203/204 --> 左前/右前/左后/右后)
  *              	X前(+)后(-)     Y左(-)右(+)     Z扭头
  */
void Chassis_Omni_Move_Calculate(void)
{
	/* NEW */
	static float rotate_ratio_fl; //前左
	static float rotate_ratio_fr; //前右
	static float rotate_ratio_bl; //后左
	static float rotate_ratio_br; //后右
	static float wheel_rpm_ratio;

	rotate_ratio_fr = ((WHEELBASE + WHEELTRACK) / 2.0f - GIMBAL_Y_OFFSET) / RADIAN_COEF;
	rotate_ratio_fl = ((WHEELBASE + WHEELTRACK) / 2.0f - GIMBAL_Y_OFFSET) / RADIAN_COEF;
	rotate_ratio_bl = ((WHEELBASE + WHEELTRACK) / 2.0f + GIMBAL_Y_OFFSET) / RADIAN_COEF;
	rotate_ratio_br = ((WHEELBASE + WHEELTRACK) / 2.0f + GIMBAL_Y_OFFSET) / RADIAN_COEF;

	wheel_rpm_ratio = 60.0f / (PERIMETER * CHASSIS_DECELE_RATIO); //	60/周长*减数比

	//全向算法

	Chassis_Speed_Target[LEFT_FRON_201] = (-Chassis_Move_X + Chassis_Move_Y + Chassis_Move_Z * rotate_ratio_fr) * wheel_rpm_ratio;
	Chassis_Speed_Target[RIGH_FRON_202] = (-Chassis_Move_X - Chassis_Move_Y + Chassis_Move_Z * rotate_ratio_fl) * wheel_rpm_ratio;
	Chassis_Speed_Target[LEFT_BACK_203] = (+Chassis_Move_X + Chassis_Move_Y + Chassis_Move_Z * rotate_ratio_bl) * wheel_rpm_ratio;
	Chassis_Speed_Target[RIGH_BACK_204] = (+Chassis_Move_X - Chassis_Move_Y + Chassis_Move_Z * rotate_ratio_br) * wheel_rpm_ratio;
}

/**
  * @brief  底盘电机PID计算输出
  * @param  电机ID
  * @retval void
  * @attention  (201/202/203/204 --> 左前/右前/左后/右后)
  */
void Chassis_Motor_Speed_PID(void)
{
	fp32 max_vector = 0.0f, vector_rate = 0.0f;
	uint8_t i = 0;

	//计算轮子控制最大速度，并限制其最大速度
	for (i = 0; i < 4; i++)
	{
		if (max_vector < fabs(Chassis_Speed_Target[i]))
		{
			max_vector = fabs(Chassis_Speed_Target[i]);
		}
	}

	if (max_vector > MAX_WHEEL_SPEED)
	{
		vector_rate = MAX_WHEEL_SPEED / max_vector;
		for (i = 0; i < 4; i++)
		{
			Chassis_Speed_Target[i] *= vector_rate;
		}
	}

	//计算pid

	for (i = 0; i < 4; i++)
	{
		PID_Calc(&motor_pid[i], motor_chassis_speed[i], Chassis_Speed_Target[i]);
	}

	//赋值电流值
	for (i = 0; i < 4; i++)
	{
		Chassis_Final_Output[i] = motor_pid[i].out;
	}
}

/**
  * @brief  键盘模式底盘电机PID计算输出
  * @param  电机ID
  * @retval void
  * @attention  (201/202/203/204 --> 左前/右前/左后/右后)
  */
void Chassis_Motor_Speed_PID_KEY(void)
{
	fp32 max_vector = 0.0f, vector_rate = 0.0f;
	uint8_t i = 0;

	//计算轮子控制最大速度，并限制其最大速度
	for (i = 0; i < 4; i++)
	{
		if (max_vector < fabs(Chassis_Speed_Target[i]))
		{
			max_vector = fabs(Chassis_Speed_Target[i]);
		}
	}
	//底盘不同功率控制 
	if (max_vector > MAX_WHEEL_SPEED && Chassis_Power_Level == LOW)
	{
		vector_rate = MAX_WHEEL_SPEED / (max_vector);
		for (i = 0; i < 4; i++)
		{
			Chassis_Speed_Target[i] *= vector_rate / LOW_RATE;
		}
	}
	else if (max_vector > MAX_WHEEL_SPEED && Chassis_Power_Level == HIGH)
	{
		vector_rate = MAX_WHEEL_SPEED / (max_vector);
		for (i = 0; i < 4; i++)
		{
			Chassis_Speed_Target[i] *= vector_rate / HIGH_RATE;
		}
	}



	//计算pid

	for (i = 0; i < 4; i++)
	{
		PID_Calc(&motor_key_pid[i], motor_key_chassis_speed[i], Chassis_Speed_Target[i]);
	}

	//赋值电流值
	for (i = 0; i < 4; i++)
	{
		//Chassis_Final_Output[i] = constrain_float(motor_key_pid[i].out, -4000, 4000);
		Chassis_Final_Output[i] = motor_key_pid[i].out;
	}
}

/**
  * @brief  发送电机最终电流值
  * @param  void
  * @retval void
  * @attention  CAN1发送
  */
void CHASSIS_CANSend(void)
{
	CAN_CMD_CHASSIS(Chassis_Final_Output[0], Chassis_Final_Output[1], Chassis_Final_Output[2], Chassis_Final_Output[3]);
}

/**
  * @brief  底盘电机PID计算,单级
  * @param  电机ID
  * @retval void
  * @attention  (201/202/203/204 --> 左前/右前/左后/右后)
  */
void Chassis_Motor_Key_PID(ChassisWheel Wheel)
{

	//计算速度误差
	Chassis_Speed_Error[Wheel] = Chassis_Speed_Target[Wheel] - Chassis_Speed_Measure[Wheel];

	Chassis_Speed_Error_Sum[Wheel] += Chassis_Speed_Error[Wheel];

	pTermChassis[Wheel] = Chassis_Speed_Error[Wheel] * Chassis_Speed_kpid[Wheel][KP];
	iTermChassis[Wheel] = Chassis_Speed_Error_Sum[Wheel] * Chassis_Speed_kpid[Wheel][KI] * 0.002f;
	//积分限幅
	iTermChassis[Wheel] = constrain_float(iTermChassis[Wheel], -iTermChassis_Max, iTermChassis_Max);

	Chassis_Speed_Error_NOW[Wheel] = Chassis_Speed_Error[Wheel];
	dTermChassis[Wheel] = (Chassis_Speed_Error_NOW[Wheel] - Chassis_Speed_Error_LAST[Wheel]) * Chassis_Speed_kpid[Wheel][KD];
	Chassis_Speed_Error_LAST[Wheel] = Chassis_Speed_Error_NOW[Wheel];

	//积分项缓慢减小,防止误差为0时突然失力
	if (pTermChassis[Wheel] * iTermChassis[Wheel] < 0)
	{
		Chassis_Speed_Error_Sum[Wheel] = constrain_float(Chassis_Speed_Error_Sum[Wheel],
														 -(3000 / Chassis_Speed_kpid[Wheel][KI] / 5.f),
														 (3000 / Chassis_Speed_kpid[Wheel][KI] / 5.f));
	}

	pidTermChassis[Wheel] = pTermChassis[Wheel] + iTermChassis[Wheel] + dTermChassis[Wheel];

	pidTermChassis[Wheel] = constrain_float(pidTermChassis[Wheel], -Chassis_Final_Output_Max, Chassis_Final_Output_Max);

	//记录输出电流
	Chassis_Final_Output[Wheel] = pidTermChassis[Wheel];
}

/**
  * @brief  旋转速度PID计算
  * @param  当前云台偏离量，目标偏移量
  * @retval 旋转速度
  * @attention 
  */
float speed_z_pterm = 0;
float speed_z_iterm = 0;
float speed_z_dterm = 0;
float speed_z = 0;
float Chassis_SpeedZ_PID(int16_t ErrorReal, float kp)
{
	static int16_t ErrorPrev = 0; //上次偏离误差
	static int32_t ErrorSum = 0;  //上次偏离误差
	static int32_t ErrorPR = 0;
	static int32_t ErrorPR_KF = 0;

	ErrorPR_KF = KalmanFilter(&Chassis_Error_Kalman, ErrorReal);

	//P
	speed_z_pterm = ErrorReal * kp; //根据yaw偏离中心计算电流
	speed_z_pterm = constrain_float(speed_z_pterm, -REVOLVE_MAX_NORMAL, REVOLVE_MAX_NORMAL);

	//I
	ErrorSum -= ErrorPR_KF;
	speed_z_iterm = ErrorSum * 3 * 0.002f;
	if (abs(ErrorReal) <= 10) //10
	{
		ErrorSum = 0;
	}
	//积分限幅
	speed_z_iterm = constrain_float(speed_z_iterm, -5000, 5000);

	//D
	ErrorPR = ErrorPR_KF - ErrorPrev;

	if (abs(ErrorPR_KF) > REVOLVE_ANGLE) //35
	{
		speed_z_dterm = -(ErrorPR)*REVOLVE_KD; //600;//650;//125.f;
	}
	else
	{
		speed_z_dterm = 0;
	}
	//扭头最大速度限幅
	speed_z = speed_z_pterm + speed_z_dterm; // + speed_z_iterm;//// + ;
	speed_z = constrain(speed_z, -Chassis_Revolve_Move_Max, +Chassis_Revolve_Move_Max);

	ErrorPrev = ErrorPR_KF; //记录上次误差

	return speed_z;
}

/**
  * @brief  分别对4个电机做PID运算,计算最终输出电流(发送给电调的值)
  * @param  void
  * @retval void
  * @attention  (201/202/203/204 --> 左前/右前/左后/右后)
  */
void Chassis_MotorOutput(void)
{
	Chassis_Motor_Key_PID(LEFT_FRON_201);
	Chassis_Motor_Key_PID(RIGH_FRON_202);
	Chassis_Motor_Key_PID(LEFT_BACK_203);
	Chassis_Motor_Key_PID(RIGH_BACK_204);
}

/**
  * @brief  底盘键盘斜坡函数
  * @param  判断按键是否被按下, 时间量, 每次增加的量, 一共要减小的量
  * @retval 斜坡比例系数
  * @attention  0~1
  */
float Chassis_Key_MoveRamp(uint8_t status, int16_t *time, int16_t inc, int16_t dec)
{
	float factor = 0;

	factor = 0.15 * sqrt(0.15 * (*time)); //计算速度斜坡,time累加到296.3斜坡就完成  //0.15

	if (status == 1) //按键被按下
	{
		if (factor < 1) //防止time太大
		{
			*time += inc;
		}
	}
	else //按键松开
	{
		if (factor > 0)
		{
			*time -= dec;

			if (*time < 0)
			{
				*time = 0;
			}
		}
	}

	factor = constrain_float(factor, 0, 1); //注意一定是float类型限幅

	return factor; //注意方向
}

/**
  * @brief  获取底盘移动模式
  * @param  void
  * @retval TRUE:机械模式    false:陀螺仪模式
  * @attention  
  */
bool CHASSIS_IfActiveMode(void)
{
	if (Chassis_Mode == CHASSIS_MECH_MODE)
	{
		return TRUE; //机械
	}
	else
	{
		return FALSE; //陀螺仪
	}
}

/**
  * @brief  底盘是否处于扭屁股模式
  * @param  void
  * @retval TRUE FALSE
  * @attention  
  */
bool Chassis_IfSHAKE(void)
{
	if (Chassis_Action == CHASSIS_SHAKE)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  获取电机角度
  * @param  ID,CAN数据
  * @retval void
  * @attention  (201/202/203/204 --> 左前/右前/左后/右后),CAN1中调用
  */
void CHASSIS_UpdateMotorAngle(ChassisWheel Wheel, int16_t angle)
{
	Chassis_Angle_Measure[Wheel] = angle;
}

/**
  * @brief  获取电机转速
  * @param  ID,CAN数据
  * @retval void
  * @attention  (201/202/203/204 --> 左前/右前/左后/右后),CAN1中调用
  */

void CHASSIS_UpdateMotorSpeed(ChassisWheel Wheel, int16_t speed)
{
	Chassis_Speed_Measure[Wheel] = speed;
}

/**
  * @brief  获取电机转矩电流
  * @param  ID,CAN数据
  * @retval void
  * @attention  (201/202/203/204 --> 左前/右前/左后/右后),CAN1中调用
  */
void CHASSIS_UpdateMotorCur(ChassisWheel Wheel, int16_t current)
{
	Chassis_Current_Measure[Wheel] = current;
}

void Angle_error(void)
{
	theta = -Cloud_Angle_Measure[YAW][MECH]; //+ theta_error;
	theta_error1 = cos(theta);
	theta_error2 = sin(theta);
}

/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note       AV  DV左右调头 弹仓开启 云台不动模式 未明。   卡尔曼滤波还需要了解，自瞄，打符控制还需要了解。
  * @history    板子充电口朝我   陀螺仪排针超外   左正右负    电机充电口朝我  R字 左正右负    电流左正右负       电机反装的话  左负右正   电流左负右正
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */

#include "Gimbal_Task.h"
#include "kalman.h"
#include "kalman_filter.h"
#include "main.h"
#include "led.h"

#include "arm_math.h"

#include "user_lib.h"

#include "remote_control.h"
#include "CAN_Receive.h"

#include "pid.h"
#include "start_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "mpu6050.h"
#include "inv_mpu.h"
#include "vision.h"
#include "magazine.h"
#include "chassis_task.h"
#include "gimbal_key_control.h"


#define gimbal_total_pid_clear(void)              \
    {                                             \
        Gimbal_PID_clear(&Gimbal_Yaw_Mech_PID);   \
        Gimbal_PID_clear(&Gimbal_Yaw_Gyro_PID);   \
        PID_clear(&gimbal_yaw_motor_gyro_pid);    \
        PID_clear(&gimbal_yaw_motor_mech_pid);    \
			                                      \
        Gimbal_PID_clear(&Gimbal_Pitch_Mech_PID); \
        Gimbal_PID_clear(&Gimbal_Pitch_Gyro_PID); \
        PID_clear(&gimbal_pitch_motor_gyro_pid);  \
        PID_clear(&gimbal_pitch_motor_mech_pid);  \
    }


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
/*--------------------------------------myself-------------------------------*/
extern  float Chassis_Gyro_Error ;
extern  RC_ctrl_t rc_ctrl;    //定义遥控器结构体参数
GimbalCtrlMode  modeGimbal;   //定义云台控制模式    机械/陀螺仪
eGimbalAction  actGimbal;     //定义云台运动模式  调头 自瞄 打符等
Critical_t Yaw_Gyro_Angle;    
extKalman_t Vision_Distance_Kalman;                     //定义视觉距离卡尔曼滤波结构体
speed_calc_data_t Vision_Yaw_speed_Struct;              //定义视觉yaw速度测速结构体
speed_calc_data_t Vision_Pitch_speed_Struct;            //定义视觉pitch速度测速结构体
kalman_filter_t yaw_kalman_filter;                      //定义yaw卡尔曼滤波器结构体
kalman_filter_t pitch_kalman_filter;                    //定义pitch卡尔曼滤波器结构体

extern VisionRecvData_t VisionRecvData; //定义视觉接收的数据结构体

Gimbal_PID_t Gimbal_Yaw_Mech_PID;      //PID一系列结构体
Gimbal_PID_t Gimbal_Yaw_Gyro_PID;
Gimbal_PID_t Gimbal_Pitch_Mech_PID;
Gimbal_PID_t Gimbal_Pitch_Gyro_PID;
Gimbal_PID_t Gimbal_Pitch_key_PID;
Gimbal_PID_t Gimbal_yaw_key_PID;
PidTypeDef gimbal_yaw_motor_gyro_pid;
PidTypeDef gimbal_pitch_motor_gyro_pid;
PidTypeDef gimbal_yaw_motor_mech_pid;
PidTypeDef gimbal_pitch_motor_mech_pid;
PidTypeDef gumbal_angle_pid;
/*-----------------------------------------------------PID参数----------------------------------------------------------------------------*/

//陀螺仪参数
float angleMpuPitch,	angleMpuYaw,	angleMpuRoll;//陀螺仪角度值
short palstanceMpuPitch,	palstanceMpuYaw,	palstanceMpuRoll;//陀螺仪角速度值

//机械角度中间变量,从CAN中读取数据
int16_t  angleMotorPit,  angleMotorYaw; 
int16_t  speedMotorPit,  speedMotorYaw; 
int16_t  currentMotorPit,  currentMotorYaw; 
int16_t  out;
//期望角度
float Cloud_Angle_Target[2][3];//  pitch/yaw    mech/gyro
extern float Cloud_Angle_Target_GD[2][2];   //  pitch/yaw    mech/gyro  定义在key_control.c里

//测量角度
float Cloud_Angle_Measure[2][4];//  pitch/yaw    mech/gyro/change      change用在底盘跟随陀螺仪模式,防止多转180度

//测量电机转速
float Cloud_Speed_Measure[2][2];//  pitch/yaw    mech/gyro

//测量电机电流值
float Cloud_Current_Measure[2][2];//  pitch/yaw    mech/gyro

//测量角速度
float Cloud_Palstance_Measure[2][3];//  pitch/yaw    mech/gyro


float motor_gyro_set[2][3];  //PID计算外环结果，角速度设定值  pitch/yaw    mech/gyro
//float motor_gyro_set[2][2]; 

float current_set[2][3];      //PID计算内环结果，输出  pitch/yaw    mech/gyro
//float current_set[2][2];

float given_current[2][3];     //PID最终赋值变量  pitch/yaw    mech/gyro
//float given_current[2][2];

/*------------------------------------------------------------------------------------------------------------------------------------------*/

/*----------------------------------------------------------遥控器相关变量-------------------------------------------------------------------*/

//上电斜坡变量
float Slope_Begin_Pitch = 0.0020 ;  //刚上电时移动快慢
float Slope_Begin_Yaw = 0.0020 ;

fp32 rc_add_yaw, rc_add_pit;       //遥控器增量
int16_t yaw_channel, pitch_channel; //遥控器中间变量
fp32 bias_angle;      //当前控制误差角度
 
/*--------------------------------------------------------云台键盘模式下各小函数辅助变量--------------------------------------------------------------------*/
//调头模式角度目标
extern float TURNMode_Yaw_Back_Total;//按下C,yaw需要改变的角度值
extern float TURNMode_Yaw_Turn_Total;//按下QE,yaw需要改变的角度值,正负代表左右转

//键盘陀螺仪模式下鼠标统计yaw偏移量,此值会自己缓慢减小,防止甩头过快
float Mouse_Gyro_Yaw, Mouse_Gyro_Pitch;

/*自动打弹用的一些标志位*/
bool Mobility_Prediction_Yaw = FALSE;//预测是否开启标志位
bool Mobi_Pre_Yaw_Fire = FALSE;//默认预测没到位，禁止开枪

uint16_t mobpre_yaw_left_delay = 0;//向左预测延时判断可开火消抖
uint16_t mobpre_yaw_right_delay = 0;//向右预测延时判断可开火消抖
uint16_t mobpre_yaw_stop_delay = 0;//预测关闭延时判断可开火消抖

//上层自瞄预测用
float pitch_angle_raw;
float yaw_angle_raw;
float Auto_Error_Pitch[2];
float Auto_Error_Yaw[2];
float Auto_Distance;
uint32_t Gimbal_Vision_Time[2];

//下层自瞄预测用
float Auto_Distance;//预测距离
float vision_time_update_time;
float Vision_Angle_Speed_Yaw, Vision_Angle_Speed_Pitch;//卡尔曼滤波速度测量值
float *yaw_kf_result, *pitch_kf_result;//二阶卡尔曼滤波结果,0角度 1速度
float yaw_speed_k = 0;//yaw速度预测比例
float kf_yaw_angcon = 0;//yaw预测比例限幅
float pitch_speed_k = 0;//pitch速度预测比例
float kf_pitch_angcon = 0;//pitch预测比例限幅
float debug_kf_y_angle;//yaw预测暂存
float debug_kf_p_angle;//pitch预测暂存
float debug_kf_angle_temp;//预测角度斜坡暂存量
float debug_kf_angle_ramp = 20;//预测角度斜坡变化量
float kf_speed_yl = 0;//速度过低关闭预测
uint16_t Auto_KF_Delay = 0;//自瞄突然开启,卡尔曼滤波开启延时
float debug_y_sk;// = 38;//35;//30;//移动预测系数,越大预测越多
float debug_y_sb_sk;//哨兵预测系数
float debug_y_sb_brig_sk;//桥头哨兵
float debug_p_sk;//移动预测系数,越大预测越多
float debug_auto_err_y=120;// = 10;//15;//10;//15;//yaw角度过大关闭预测              具体值在程序中还需要修改               
float debug_auto_err_p;//pitch角度过大关闭预测
float debug_kf_delay=80;// = 150;//100;//200;//120;//150;//预测延时开启              具体值在程序中还需要修改
float debug_kf_speed_yl;//yaw速度过低关闭预测
float debug_kf_speed_yl_sb;//抬头打哨兵时减小最低可开预测量
float debug_kf_speed_yh;//yaw速度过高关闭预测
float debug_kf_speed_pl;//pitch速度过低关闭预测
float debug_kf_y_angcon;// = 130;//125;//115;//135;//yaw预测量限幅
float debug_kf_p_angcon;//pitch预测量限幅



int my_op = 0;
/*------------------------------------------------------------------------------------------------------------------------------------------*/



//每2ms执行一次任务函数
void GIMBAL_task(void *pvParameters)
{
	portTickType currentTime;	
	
	for(;;)
	{
		currentTime = xTaskGetTickCount();//当前系统时间
		
		/* 代码段 */
		if (SYSTEM_GetSystemState() == SYSTEM_STARTING)//初始化模式
		{
       		GIMBAL_InitCtrl();
		}
		else
		{
			if(IF_RC_SW2_UP)
			{
				if (IF_MOUSE_PRESSED_RIGH)
				{
					actGimbal = GIMBAL_AUTO;
				}
				KEY_Set_Mode();
			}
			else
			{
				RC_Set_Mode();
				GIMBAL_Set_Control();
			  	actGimbal = GIMBAL_NORMAL;
			}
		}
		//根据操作模式变换PID,每次都要变,很重要
		//GIMBAL_PositionLoop();
		if (IF_MOUSE_PRESSED_RIGH)
		{
			GIMBAL_PositionLoop_AUTO();
		}
		else
		{
			GIMBAL_PositionLoop();
		}
		GIMBAL_CanSend();
		
		//如果不是自瞄模式,对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度      ????????????????????????????????
		
		if(VisionRecvData.distance >= 999.0f)
		{
			led_red_on();
		}
		else 
			
		{
			led_red_off();
		}

		vTaskDelayUntil(&currentTime, TIME_STAMP_2MS);//绝对延时
	}
}


/**
  * @brief  云台初始化,主要是PID初始化
  * @param  void
  * @retval void
  * @attention 
  */
void GIMBAL_InitCtrl(void)
{
	
		static bool bAngleRecord  = FALSE;
	  static portTickType  ulTimeCurrent = 0;
	
	  if (xTaskGetTickCount( ) - ulTimeCurrent > TIME_STAMP_100MS)//保证不断电情况下下次可用
	   {
		   bAngleRecord = FALSE;
	   }
		 
		 ulTimeCurrent = xTaskGetTickCount( );
		 
    static const fp32 Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
    static const fp32 Yaw_speed_pid[3]   = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};
		
		modeGimbal = CLOUD_MECH_MODE;
    //初始化yaw电机pid		
		GIMBAL_PID_Init(&Gimbal_Yaw_Gyro_PID,YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD);
		GIMBAL_PID_Init(&Gimbal_Yaw_Mech_PID,YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT, YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD);
		GIMBAL_PID_Init(&Gimbal_yaw_key_PID,YAW_KEY_PID_MAX_OUT, YAW_KEY_PID_MAX_IOUT, YAW_KEY_PID_KP, YAW_KEY_PID_KI, YAW_KEY_PID_KD);
		PID_Init(&gimbal_yaw_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
		PID_Init(&gimbal_yaw_motor_mech_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);		
		
    //初始化pitch电机pid	
		GIMBAL_PID_Init(&Gimbal_Pitch_Gyro_PID,PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD);
		GIMBAL_PID_Init(&Gimbal_Pitch_Mech_PID,PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD);
		GIMBAL_PID_Init(&Gimbal_Pitch_key_PID,PITCH_KEY_PID_MAX_OUT, PITCH_KEY_PID_MAX_IOUT, PITCH_KEY_PID_KP, PITCH_KEY_PID_KI, PITCH_KEY_PID_KD);
		PID_Init(&gimbal_pitch_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);
		PID_Init(&gimbal_pitch_motor_mech_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);		
		
		gimbal_total_pid_clear();		
		
		//初始化云台跟随PID
		const static fp32 gimbal_yaw_pid[3] = {GIMBAL_FOLLOW_CHASSIS_PID_KP, GIMBAL_FOLLOW_CHASSIS_PID_KI, GIMBAL_FOLLOW_CHASSIS_PID_KD};          //初始化地盘旋转跟随KP、KI、KD 
    PID_Init(&gumbal_angle_pid, PID_POSITION, gimbal_yaw_pid, GIMBAL_FOLLOW_CHASSIS_PID_MAX_OUT, GIMBAL_FOLLOW_CHASSIS_PID_MAX_OUT);
		
		
		Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Measure[YAW][GYRO];
		Cloud_Angle_Target[PITCH][GYRO] = Cloud_Angle_Measure[PITCH][GYRO];
		
		Cloud_Angle_Target[YAW][TOP]=Cloud_Angle_Measure[YAW][GYRO];
		
			//记录上电时云台机械角度
	  if (bAngleRecord == FALSE)
	  {
	  	bAngleRecord = TRUE;
			
		  Cloud_Angle_Target[PITCH][MECH] = Cloud_Angle_Measure[PITCH][MECH];
		  Cloud_Angle_Target[YAW][MECH] = Cloud_Angle_Measure[YAW][MECH];
	  }
		
			//平缓地让云台移动到中间,防止刚上电狂甩
	  Cloud_Angle_Target[PITCH][MECH] = RAMP_float( mid_pitch_angle, Cloud_Angle_Target[PITCH][MECH], Slope_Begin_Pitch);
	  Cloud_Angle_Target[YAW][MECH]   = RAMP_float( mid_yaw_angle, Cloud_Angle_Target[YAW][MECH], Slope_Begin_Yaw);
}
/*-----------------------------------------------云台遥控器控制模式选择和遥控器目标值计算--------------------------------------------------*/
/**
  * @brief  云台遥控器控制模式
  * @param  void
  * @retval void
  * @attention 
  */
void RC_Set_Mode(void)
{
	if(IF_RC_SW2_MID)
	{
		modeGimbal = CLOUD_GYRO_MODE;
	}
	else  if(IF_RC_SW2_UP)
	{
		modeGimbal = CLOUD_GYRO_MODE;
	}
	else if(IF_RC_SW2_DOWN)
	{
		modeGimbal = CLOUD_TOP_MODE;
	}
}

/**
  * @brief  计算云台的目标值
  * @param  void
  * @retval void
  * @attention 
  */

void GIMBAL_Set_Control(void)
{
	  //将遥控器的数据处理死区 int16_t yaw_channel,pitch_channel
    rc_deadline_limit(RC_CH0_RLR_OFFSET, yaw_channel, RC_deadband);
    rc_deadline_limit(RC_CH1_RUD_OFFSET, pitch_channel, RC_deadband);	
	
    rc_add_yaw = yaw_channel * Yaw_RC_SEN ;
    rc_add_pit = - pitch_channel * Pitch_RC_SEN ;
	
	if(modeGimbal == CLOUD_MECH_MODE)
	{
		TOP_Rc_Switch();
#if YAW_POSITION ==	YAW_DOWN
//		Cloud_Angle_Target[YAW][MECH] -= (rc_add_yaw/5);                                 //根据云台安装方式,轴上和轴下正负不同 ,除的值保证累加的值为0.000001       /-
		
		
//		if( my_op == 10)	
//		{
//			Cloud_Angle_Target[YAW][MECH] = mid_yaw_angle;
//			my_op = 1;	
//		}
//		else
//		{
//			Cloud_Angle_Target[YAW][MECH] -= Chassis_Gyro_Error;
//			++my_op ;	
//		}
		
		
//		Cloud_Angle_Target[YAW][MECH] = mid_yaw_angle;
		Cloud_Angle_Target[YAW][MECH] =	PID_Calc(&gumbal_angle_pid,Chassis_Gyro_Error, Chassis_Gyro_Error);
		Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Measure[YAW][GYRO];
    //是否超过最大 最小值
    
    if (Cloud_Angle_Target[YAW][MECH] > max_yaw_relative_angle)                   //这里默认直接用了轴上
    {
        Cloud_Angle_Target[YAW][MECH] = max_yaw_relative_angle;
    }
    else if (Cloud_Angle_Target[YAW][MECH] < min_yaw_relative_angle)
    {
        Cloud_Angle_Target[YAW][MECH] = min_yaw_relative_angle;
    }
			
		
		Cloud_Angle_Target[PITCH][MECH]+= rc_add_pit;
		                               //注意正负 jj
		Cloud_Angle_Target[PITCH][GYRO] = Cloud_Angle_Measure[PITCH][GYRO];
    //是否超过最大 最小值
    if (Cloud_Angle_Target[PITCH][MECH] > max_pitch_relative_angle)
    {
        Cloud_Angle_Target[PITCH][MECH] = max_pitch_relative_angle;
    }
    else if (Cloud_Angle_Target[PITCH][MECH] < min_pitch_relative_angle)
    {
        Cloud_Angle_Target[PITCH][MECH] = min_pitch_relative_angle;
    }

#else 
//		Cloud_Angle_Target[YAW][MECH] += (rc_add_yaw/5);                                 //根据云台安装方式,轴上和轴下正负不同        /-
		Cloud_Angle_Target[YAW][MECH] = mid_yaw_angle;
		Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Measure[YAW][GYRO];
    //是否超过最大 最小值
    
    if (Cloud_Angle_Target[YAW][MECH] > max_yaw_relative_angle)                   //这里默认直接用了轴上
    {
        Cloud_Angle_Target[YAW][MECH] = max_yaw_relative_angle;
    }
    else if (Cloud_Angle_Target[YAW][MECH] < min_yaw_relative_angle)
    {
        Cloud_Angle_Target[YAW][MECH] = min_yaw_relative_angle;
    }
			
		
		Cloud_Angle_Target[PITCH][MECH]-= rc_add_pit;
		                               //注意正负                /+
		Cloud_Angle_Target[PITCH][GYRO] = Cloud_Angle_Measure[PITCH][GYRO];
    //是否超过最大 最小值
    if (Cloud_Angle_Target[PITCH][MECH] > max_pitch_relative_angle)
    {
        Cloud_Angle_Target[PITCH][MECH] = max_pitch_relative_angle;
    }
    else if (Cloud_Angle_Target[PITCH][MECH] < min_pitch_relative_angle)
    {
        Cloud_Angle_Target[PITCH][MECH] = min_pitch_relative_angle;
    }
		
#endif 	
	}
	else if(modeGimbal == CLOUD_GYRO_MODE)
	{
		TOP_Rc_Switch();
//		Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Measure[YAW][MECH];
//		Cloud_Angle_Target[PITCH][GYRO] = Cloud_Angle_Measure[PITCH][MECH];
		
    Cloud_Angle_Target[YAW][GYRO] += rc_add_yaw;                                     //注意正负
    Cloud_Angle_Target[PITCH][MECH] += rc_add_pit * 1.5;
		#if YAW_POSITION ==	YAW_DOWN
    
		Cloud_Angle_Target[PITCH][MECH]+= rc_add_pit;
		                               //注意正负 jj
		Cloud_Angle_Target[PITCH][GYRO] = Cloud_Angle_Measure[PITCH][GYRO];
    //是否超过最大 最小值
    if (Cloud_Angle_Target[PITCH][MECH] > max_pitch_relative_angle)
    {
        Cloud_Angle_Target[PITCH][MECH] = max_pitch_relative_angle;
    }
    else if (Cloud_Angle_Target[PITCH][MECH] < min_pitch_relative_angle)
    {
        Cloud_Angle_Target[PITCH][MECH] = min_pitch_relative_angle;
    }

#else 
		Cloud_Angle_Target[YAW][MECH] += (rc_add_yaw/5);                                 //根据云台安装方式,轴上和轴下正负不同        /-
		
		Cloud_Angle_Target[PITCH][MECH]-= rc_add_pit;
		                               //注意正负                /+
		Cloud_Angle_Target[PITCH][GYRO] = Cloud_Angle_Measure[PITCH][GYRO];
    //是否超过最大 最小值
    if (Cloud_Angle_Target[PITCH][MECH] > max_pitch_relative_angle)
    {
        Cloud_Angle_Target[PITCH][MECH] = max_pitch_relative_angle;
    }
    else if (Cloud_Angle_Target[PITCH][MECH] < min_pitch_relative_angle)
    {
        Cloud_Angle_Target[PITCH][MECH] = min_pitch_relative_angle;
    }
		
#endif 	
	}
	
	else if(modeGimbal == CLOUD_TOP_MODE)
	{
		//应该要在这里加调整枪口的程序
		if(TOP_Rc_Switch() == TRUE)
		{
			Cloud_Angle_Target[YAW][TOP]=Cloud_Angle_Measure[YAW][GYRO];
			Cloud_Angle_Target[PITCH][MECH]=Cloud_Angle_Measure[PITCH][MECH];
		}
    	Cloud_Angle_Target[YAW][TOP] += rc_add_yaw;                                     //注意正负
    	Cloud_Angle_Target[PITCH][MECH] += rc_add_pit;
//		Cloud_Angle_Target[YAW][MECH] = Cloud_Angle_Measure[YAW][MECH];
//		Cloud_Angle_Target[PITCH][MECH] = Cloud_Angle_Measure[PITCH][MECH];
		Cloud_Angle_Target[YAW][GYRO]=Cloud_Angle_Measure[YAW][GYRO];
	}
}

/*-----------------------------------------------云台键盘控制模式选择和键盘目标值计算--------------------------------------------------*/
/**
  * @brief  云台键盘控制模式
  * @param  void
  * @retval void
  * @attention 
  */
void KEY_Set_Mode(void)
{
	switch(actGimbal)
	{
		/*--------------云台模式选择----------------*/
		case GIMBAL_NORMAL:
			GIMBAL_NORMAL_Mode_Ctrl();//在此选择控制模式		  
		break;	

		/*--------------V  180°调头----------------*/
		case GIMBAL_AROUND:
			modeGimbal = CLOUD_GYRO_MODE;//进入陀螺仪模式
		
			if (TURNMode_Yaw_Back_Total == 0)
			{
				actGimbal = GIMBAL_NORMAL;
			}
			else
			{
				Cloud_Angle_Target[YAW][GYRO] = RampInc_float( &TURNMode_Yaw_Back_Total, Cloud_Angle_Target[YAW][GYRO], TurnSpeed );
			}
		break;

		
		/*------------弹仓开启,禁止抬头-----------------*/
		case GIMBAL_LEVEL:
			GIMBAL_LEVEL_Mode_Ctrl();
		    Cloud_Angle_Target[YAW][MECH] = Cloud_Angle_Target_GD[YAW][MECH];
		    Cloud_Angle_Target[PITCH][MECH] = Cloud_Angle_Target_GD[PITCH][MECH];
		break;
		
		/*--------------Q E  90°调头----------------*/
		case GIMBAL_TURN:				
			modeGimbal = CLOUD_GYRO_MODE;//进入陀螺仪模式

		  if (TURNMode_Yaw_Turn_Total == 0)
			{
				actGimbal = GIMBAL_NORMAL;
			}
			else
			{
				Cloud_Angle_Target[YAW][GYRO] = RampInc_float( &TURNMode_Yaw_Turn_Total, Cloud_Angle_Target[YAW][GYRO], TurnSpeed );
			}
		break;			
			
		case GIMBAL_AUTO:
		    modeGimbal = CLOUD_GYRO_MODE;  //进入陀螺仪模式
		  	if(!IF_MOUSE_PRESSED_RIGH)  //松开右键退出自瞄模式
			{
				actGimbal = GIMBAL_NORMAL;
				//自瞄目标偏差清零,避免切换时云台跳动
				VisionRecvData.identify_target = FALSE;
				Auto_KF_Delay = 0;//清零给下次延迟预测用
				Mobility_Prediction_Yaw = FALSE;//标记预测没开启
				Mobi_Pre_Yaw_Fire = FALSE;//默认标记预测没到位，禁止开火
				
				mobpre_yaw_left_delay  = 0;//重置左预测的开火延迟
				mobpre_yaw_right_delay = 0;//重置右预测的开火延迟	
				mobpre_yaw_stop_delay = 0;//停止预测开火延时重置
				
//				Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Measure[YAW][MECH];
//				Cloud_Angle_Measure[YAW][GYRO] = Cloud_Angle_Measure[YAW][MECH];
			}
			else if(IF_MOUSE_PRESSED_RIGH)
			{
				GIMBAL_AUTO_Mode_Ctrl();
				
			}
		break;
	}
}

/**
  * @brief  云台键盘模式，目标值计算
  * @param  void
  * @retval void
  * @attention 
  */
void GIMBAL_Set_Key_Control(void)
{
	static uint32_t Mouse_Yaw_Stop  = 0;//鼠标不动，结束响应
	static uint32_t Mouse_Pitch_Stop  = 0;//鼠标不动，结束响应
	
	if(modeGimbal == CLOUD_MECH_MODE)
	{
			Cloud_Angle_Target[PITCH][MECH] += MOUSE_Y_MOVE_SPEED * Pitch_Mouse_Sen;
			Cloud_Angle_Target[YAW][MECH]   += MOUSE_X_MOVE_SPEED * Yaw_Mouse_Sen;	    //yaw保持不动,永远在中间
			
			Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Measure[YAW][GYRO];		
	}
	
	else
	{
      	Mouse_Gyro_Yaw   += MOUSE_X_MOVE_SPEED * Yaw_Mouse_Sen;//记录目标变化角度
		Mouse_Gyro_Pitch += MOUSE_Y_MOVE_SPEED * Pitch_Mouse_Sen;//pitch仍旧使用机械模式
/*-------鼠标长时间不动，云台停止移动------*/
		if(MOUSE_X_MOVE_SPEED == 0)
	
		{
			Mouse_Yaw_Stop ++ ;
			if(Mouse_Yaw_Stop > 25)//鼠标长时间停留，停止移动
			{
				Mouse_Gyro_Yaw = 0;
			}
		}
		else
		{
			Mouse_Yaw_Stop = 0;
		}
			
		if(MOUSE_Y_MOVE_SPEED == 0)
		{
			Mouse_Pitch_Stop ++ ;
			if(Mouse_Pitch_Stop > 25)//鼠标长时间停留，停止移动
			{
				Mouse_Gyro_Pitch = 0;
			}
		}
		else
		{
			Mouse_Pitch_Stop = 0;
		}
/*-----------------------------------------*/
	    if(modeGimbal == CLOUD_TOP_MODE)
		{
			//应该要在这里加调整枪口的程序
			if(IF_KEY_PRESSED_F)
			{
				Cloud_Angle_Target[YAW][TOP]=Cloud_Angle_Measure[YAW][GYRO];
				Cloud_Angle_Target[PITCH][MECH]=Cloud_Angle_Measure[PITCH][MECH];
			}
    		Cloud_Angle_Target[YAW][TOP] += RampInc_float( &Mouse_Gyro_Yaw, Cloud_Angle_Target[YAW][TOP], Yaw_Mouse_ramp );                              //注意正负
    		Cloud_Angle_Target[PITCH][MECH] +=  RampInc_float( &Mouse_Gyro_Pitch, Cloud_Angle_Target[PITCH][MECH], Pitch_Mouse_ramp );
//			Cloud_Angle_Target[YAW][MECH] = Cloud_Angle_Measure[YAW][MECH];
//			Cloud_Angle_Target[PITCH][MECH] = Cloud_Angle_Measure[PITCH][MECH];
			Cloud_Angle_Target[YAW][GYRO]=Cloud_Angle_Measure[YAW][GYRO];
		}
		else if (modeGimbal == CLOUD_GYRO_MODE)
		{
			Cloud_Angle_Target[YAW][GYRO]   = RampInc_float( &Mouse_Gyro_Yaw, Cloud_Angle_Target[YAW][GYRO], Yaw_Mouse_ramp );
			Cloud_Angle_Target[PITCH][MECH] = RampInc_float( &Mouse_Gyro_Pitch, Cloud_Angle_Target[PITCH][MECH], Pitch_Mouse_ramp );		
		}
	}
}

void GIMBAL_AUTO_Mode_Ctrl(void)//上左+   下右-
{
	static float pitch_angle_ref;
	static float yaw_angle_ref;

	Vision_Error_Angle_Pitch(&Auto_Error_Pitch[NOW]);
	Vision_Error_Angle_Yaw(&Auto_Error_Yaw[NOW]);
	Vision_Get_Distance(&Auto_Distance);
	if(Vision_If_Update() == TRUE)                    //数据更新
	{
		pitch_angle_ref = (Cloud_Angle_Measure[PITCH][MECH]-Auto_Error_Pitch[NOW]);
		yaw_angle_ref = (Cloud_Angle_Measure[YAW][GYRO]+Auto_Error_Yaw[NOW]);
		Vision_Clean_Update_Flag();//清零,否则会一直执行
		Gimbal_Vision_Time[NOW]=xTaskGetTickCount();//获取新数据到来的时间
	}
//	if(Gimbal_Vision_Time[NOW] != Gimbal_Vision_Time[LAST])                  //更新卡尔曼滤波测量值
//	{
//		pitch_angle_raw = pitch_angle_ref;
//		yaw_angle_raw = yaw_angle_ref;
//		Gimbal_Vision_Time[LAST] = Gimbal_Vision_Time[NOW];
//	}
	
	if(VisionRecvData.identify_target == TRUE)                     //识别到了目标
	{
		Cloud_Angle_Target[YAW][GYRO] = (Cloud_Angle_Measure[YAW][GYRO] - Auto_Error_Yaw[NOW]);
		Cloud_Angle_Target[PITCH][MECH] = (Cloud_Angle_Measure[PITCH][MECH] + Auto_Error_Pitch[NOW]);
	}
	//float temp = Auto_Error_Pitch[NOW];
	//modeGimbal = CLOUD_GYRO_MODE;
}


void GIMBAL_AUTO_PREDICT_Mode_Ctrl()
{
	static uint32_t Mouse_Yaw_Stop  = 0;//鼠标不动，结束响应
	static uint32_t Mouse_Pitch_Stop  = 0;//鼠标不动，结束响应
	
	static float yaw_angle_raw, pitch_angle_raw;//卡尔曼滤波角度测量值
	static float yaw_angle_ref;//记录目标角度
	static float pitch_angle_ref;//记录目标角度
	
	float kf_delay_open = 0;
	
	Mobility_Prediction_Yaw = FALSE;
	Mobi_Pre_Yaw_Fire = FALSE;
	
	//获取角度偏差量,已经转化为弧度类型
	Vision_Error_Angle_Pitch(&Auto_Error_Pitch[NOW]);
	Vision_Error_Angle_Yaw(&Auto_Error_Yaw[NOW]);
	Vision_Get_Distance(&Auto_Distance);
	
	Auto_Distance = KalmanFilter(&Vision_Distance_Kalman,Auto_Distance);
	
	kf_delay_open = debug_kf_delay;
	
	if(Vision_If_Update() == TRUE)                    //数据更新
	{
		pitch_angle_ref = (Cloud_Angle_Measure[PITCH][GYRO]+Auto_Error_Pitch[NOW]);//得到的角度误差后面可能需要放大或者加上补偿
		yaw_angle_ref = (Cloud_Angle_Measure[YAW][GYRO]+Auto_Error_Yaw[NOW]);//得到的角度误差后面可能需要放大或者加上补偿
		Vision_Clean_Update_Flag();//清零,否则会一直执行
		Gimbal_Vision_Time[NOW]=xTaskGetTickCount();//获取新数据到来的时间
	}
	if(Gimbal_Vision_Time[NOW] != Gimbal_Vision_Time[LAST])                  //更新卡尔曼滤波测量值
	{
		vision_time_update_time = Gimbal_Vision_Time[NOW] - Gimbal_Vision_Time[LAST];//计算视觉延迟
		pitch_angle_raw = pitch_angle_ref;//更新二阶卡尔曼滤波测量值
		yaw_angle_raw = yaw_angle_ref;
		Gimbal_Vision_Time[LAST] = Gimbal_Vision_Time[NOW];
	}
	
	if(VisionRecvData.identify_target == TRUE)                     //识别到了目标
	{
		Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct,Gimbal_Vision_Time[NOW],yaw_angle_raw);
		Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct,Gimbal_Vision_Time[NOW],pitch_angle_raw);
		
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter,yaw_angle_raw,Vision_Angle_Speed_Yaw);//对角度和速度进行二阶卡尔曼滤波融合
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter,pitch_angle_raw,Vision_Angle_Speed_Pitch);
		
		Auto_KF_Delay++;//滤波延迟开启
		
		//目标距离过近时减小预测                              这里没写!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		
		//是否在瞄准哨兵,设定值不同                           这里没写!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		
		yaw_speed_k = debug_y_sk; 
		kf_yaw_angcon = debug_kf_y_angcon;
		kf_speed_yl = debug_kf_speed_yl;
		
		//扭腰模式下的延迟预测需要改变                             这里没写!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
	
	  if(fabs(Auto_Error_Yaw[NOW])<debug_auto_err_y&&Auto_KF_Delay > kf_delay_open && fabs(yaw_kf_result[1]) > kf_speed_yl && fabs(pitch_kf_result[1]) < debug_kf_speed_yh)//预测开启条件
	  {
			if(yaw_kf_result[1] >= 0)
	  	{
	  		debug_kf_angle_temp = yaw_speed_k * (yaw_kf_result[1] - kf_speed_yl) * 1;
	  	}
			
			else if(yaw_kf_result[1] <0)
			{
				debug_kf_angle_temp = yaw_speed_k * (yaw_kf_result[1] + kf_speed_yl) * 1;
			}
			
			debug_kf_angle_temp = constrain_float(debug_kf_angle_temp,-debug_kf_y_angcon,debug_kf_y_angcon);//预测暂存量限幅
			debug_kf_y_angle = RAMP_float(debug_kf_angle_temp,debug_kf_y_angle,debug_kf_angle_ramp);//预测量缓慢变化
			debug_kf_y_angle = constrain_float(debug_kf_y_angle,-debug_kf_y_angcon,debug_kf_y_angcon);
		  Cloud_Angle_Target[YAW][GYRO] = yaw_kf_result[0]+debug_kf_y_angle;
			
			if((yaw_kf_result[1]>0) && (Auto_Error_Yaw[NOW]<0.3f)) //具体正负比较还是需要debug看
			{
				mobpre_yaw_right_delay = 0;//向右预测开火延时重置
				mobpre_yaw_left_delay++;
				
				if(mobpre_yaw_left_delay > 0)//具体延时时间
				{
					Mobi_Pre_Yaw_Fire = TRUE;//预测到位,可以开火
				}
				else 
				{
					Mobi_Pre_Yaw_Fire = FALSE;//预测不到位
				}
			}
			
			else if((yaw_kf_result[1]<0) && (Auto_Error_Yaw[NOW]>-0.3f))//具体正负比较还是需要debug看
			{
				mobpre_yaw_left_delay = 0;//向右预测开火延时重置
				mobpre_yaw_right_delay++;
				
				if(mobpre_yaw_right_delay > 0)//具体延时时间更改数字
				{
					Mobi_Pre_Yaw_Fire = TRUE;//预测到位,可以开火
				}
				else 
				{
					Mobi_Pre_Yaw_Fire = FALSE;//预测不到位
				}
			}
			
	  	else
		  {
				Mobi_Pre_Yaw_Fire = FALSE;//预测不到位
				
				mobpre_yaw_left_delay = 0;//向左预测开火延时重置
				mobpre_yaw_right_delay = 0;//向右预测开火延时重置
	  	}
			
			Mobility_Prediction_Yaw = TRUE;
			mobpre_yaw_stop_delay = 0;
			

//				pitch_speed_k = debug_p_sk/2.f;
//		  	kf_pitch_angcon = debug_kf_p_angcon/1.5f;
//        debug_kf_p_angle = pitch_speed_k * (pitch_kf_result[1] + debug_kf_speed_pl);

		//		Cloud_Angle_Target[PITCH][MECH] = pitch_angle_ref;
	  }
		
		if(Auto_KF_Delay > debug_kf_delay && fabs(Auto_Error_Pitch[NOW]) > debug_auto_err_p && fabs(pitch_kf_result[KF_SPEED]) > debug_kf_speed_pl && VisionRecvData.distance/100 < 4.f)
		{
		  if(VisionRecvData.auto_too_close == TRUE)
		  {
		  	pitch_speed_k = debug_p_sk/2.f;
		  	kf_pitch_angcon = debug_kf_p_angcon/1.5f;
		  }
			
			if(pitch_kf_result[KF_SPEED]>=0)
			{
				debug_kf_p_angle = pitch_speed_k * (pitch_kf_result[KF_SPEED] - debug_kf_speed_pl);
			}
			
			else 
			{
				pitch_speed_k = debug_p_sk;
				kf_pitch_angcon = debug_kf_p_angcon;
				debug_kf_p_angle = pitch_speed_k * (pitch_kf_result[KF_SPEED] + debug_kf_speed_pl);
			}
			
			debug_kf_p_angle = constrain_float(debug_kf_p_angle, -kf_pitch_angcon, kf_pitch_angcon);//Pitch限幅
			
			Cloud_Angle_Target[PITCH][MECH] = pitch_kf_result[KF_ANGLE] + debug_kf_p_angle;
		}
		else
		{
			Cloud_Angle_Target[PITCH][MECH] = pitch_angle_ref;
		}
	}
	
	else  //未识别到目标
	{
		//		//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
		Vision_Angle_Speed_Yaw = Target_Speed_Calc(&Vision_Yaw_speed_Struct, xTaskGetTickCount(), Cloud_Angle_Measure[YAW][GYRO]);
		Vision_Angle_Speed_Pitch = Target_Speed_Calc(&Vision_Pitch_speed_Struct, xTaskGetTickCount(), Cloud_Angle_Measure[PITCH][MECH]);
//		//对角度和速度进行二阶卡尔曼滤波融合,0位置,1速度
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter, Cloud_Angle_Measure[YAW][GYRO], 0);
		pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter, Cloud_Angle_Measure[PITCH][MECH], 0);
		debug_kf_angle_temp = 0;
		
		
		yaw_kf_result = kalman_filter_calc(&yaw_kalman_filter,Cloud_Angle_Measure[YAW][GYRO],0);
    pitch_kf_result = kalman_filter_calc(&pitch_kalman_filter,Cloud_Angle_Measure[PITCH][MECH],0);		

		if(modeGimbal == CLOUD_MECH_MODE)
	  {
			Cloud_Angle_Target[PITCH][MECH] += MOUSE_Y_MOVE_SPEED * Pitch_Mouse_Sen;
			Cloud_Angle_Target[YAW][MECH]   += MOUSE_X_MOVE_SPEED * Yaw_Mouse_Sen;	    //yaw保持不动,永远在中间
			
			Cloud_Angle_Target[YAW][GYRO] = Cloud_Angle_Measure[YAW][GYRO];		
	  }
	  else if(modeGimbal == CLOUD_GYRO_MODE)
	  {
      Mouse_Gyro_Yaw   += MOUSE_X_MOVE_SPEED * Yaw_Mouse_Sen;//记录目标变化角度
			Mouse_Gyro_Pitch += MOUSE_Y_MOVE_SPEED * Pitch_Mouse_Sen;//pitch仍旧使用机械模式
/*-------鼠标长时间不动，云台停止移动------*/
			if(MOUSE_X_MOVE_SPEED == 0)
			{
				Mouse_Yaw_Stop ++ ;
				if(Mouse_Yaw_Stop > 25)//鼠标长时间停留，停止移动
				{
					Mouse_Gyro_Yaw = 0;
				}
			}
			else
			{
				Mouse_Yaw_Stop = 0;
			}
			
			if(MOUSE_Y_MOVE_SPEED == 0)
			{
				Mouse_Pitch_Stop ++ ;
				if(Mouse_Pitch_Stop > 25)//鼠标长时间停留，停止移动
				{
					Mouse_Gyro_Pitch = 0;
				}
			}
			else
			{
				Mouse_Pitch_Stop = 0;
			}
/*-----------------------------------------*/
			Cloud_Angle_Target[YAW][GYRO]   = RampInc_float( &Mouse_Gyro_Yaw, Cloud_Angle_Target[YAW][GYRO], Yaw_Mouse_ramp );
			Cloud_Angle_Target[PITCH][MECH] = RampInc_float( &Mouse_Gyro_Pitch, Cloud_Angle_Target[PITCH][MECH], Pitch_Mouse_ramp );		
	  }
		
		Auto_KF_Delay = 0;
		
		//预测延迟重置
	}
}

/*-------------------------------------------------------PID总计算在这里---------------------------------------------------------------*/
/**
  * @brief  pid计算
  * @param  void
  * @retval void
  * @attention 此处不能改变目标角度,只能用来做限幅和调用PID计算函数
  */
void GIMBAL_PositionLoop(void)
{
  if(modeGimbal == CLOUD_MECH_MODE)
	{
		Cloud_Palstance_Measure[YAW][MECH] = Cloud_Palstance_Measure[YAW][MECH]/100;
		Cloud_Palstance_Measure[PITCH][MECH] = Cloud_Palstance_Measure[PITCH][MECH]/100;
		motor_gyro_set[YAW][MECH] = GIMBAL_PID_Calc(&Gimbal_Yaw_Mech_PID,Cloud_Angle_Measure[YAW][MECH], Cloud_Angle_Target[YAW][MECH],Cloud_Palstance_Measure[YAW][MECH]);
		motor_gyro_set[PITCH][MECH] = GIMBAL_PID_Calc(&Gimbal_Pitch_Mech_PID,Cloud_Angle_Measure[PITCH][MECH], Cloud_Angle_Target[PITCH][MECH],Cloud_Palstance_Measure[PITCH][MECH]);
		current_set[YAW][MECH] = PID_Calc(&gimbal_yaw_motor_mech_pid,Cloud_Palstance_Measure[YAW][MECH],motor_gyro_set[YAW][MECH]);
		current_set[PITCH][MECH] = PID_Calc(&gimbal_pitch_motor_mech_pid,Cloud_Palstance_Measure[PITCH][MECH],motor_gyro_set[PITCH][MECH]);	
    
    given_current[YAW][MECH]	=	 current_set[YAW][MECH];
    given_current[PITCH][MECH]	=	current_set[PITCH][MECH];
	}
	else if(modeGimbal == CLOUD_GYRO_MODE)
	{
//		Cloud_Palstance_Measure[YAW][MECH] = Cloud_Palstance_Measure[YAW][MECH]/10;
//		Cloud_Palstance_Measure[PITCH][MECH] = Cloud_Palstance_Measure[PITCH][MECH]/10;
		
		motor_gyro_set[YAW][GYRO] = GIMBAL_PID_Calc(&Gimbal_Yaw_Gyro_PID,Cloud_Angle_Measure[YAW][GYRO], Cloud_Angle_Target[YAW][GYRO],Cloud_Palstance_Measure[YAW][GYRO]);
		motor_gyro_set[PITCH][MECH] = GIMBAL_PID_Calc(&Gimbal_Pitch_Mech_PID,Cloud_Angle_Measure[PITCH][MECH], Cloud_Angle_Target[PITCH][MECH],Cloud_Palstance_Measure[PITCH][MECH]);
//		motor_gyro_set[PITCH][GYRO] = GIMBAL_PID_Calc(&Gimbal_Pitch_Gyro_PID,Cloud_Angle_Measure[PITCH][MECH], Cloud_Angle_Target[PITCH][GYRO],Cloud_Palstance_Measure[PITCH][GYRO]); 
		
		current_set[YAW][GYRO] = PID_Calc(&gimbal_yaw_motor_gyro_pid,Cloud_Palstance_Measure[YAW][GYRO],motor_gyro_set[YAW][GYRO]);
		current_set[PITCH][MECH] = PID_Calc(&gimbal_pitch_motor_mech_pid,Cloud_Palstance_Measure[PITCH][MECH],motor_gyro_set[PITCH][MECH]);	
//		current_set[PITCH][GYRO] = PID_Calc(&gimbal_pitch_motor_gyro_pid,Cloud_Palstance_Measure[PITCH][GYRO],motor_gyro_set[PITCH][GYRO]);	
    
    given_current[YAW][GYRO]	=	 current_set[YAW][GYRO]; 
    given_current[PITCH][MECH]	=	current_set[PITCH][MECH];
	}
	else if(modeGimbal == CLOUD_TOP_MODE)
	{
		motor_gyro_set[YAW][TOP] = GIMBAL_PID_Calc(&Gimbal_Yaw_Gyro_PID,Cloud_Angle_Measure[YAW][GYRO], Cloud_Angle_Target[YAW][TOP],Cloud_Palstance_Measure[YAW][GYRO]);
		motor_gyro_set[PITCH][MECH] = GIMBAL_PID_Calc(&Gimbal_Pitch_Mech_PID,Cloud_Angle_Measure[PITCH][MECH], Cloud_Angle_Target[PITCH][MECH],Cloud_Palstance_Measure[PITCH][MECH]);
		
		current_set[YAW][TOP] = PID_Calc(&gimbal_yaw_motor_gyro_pid,Cloud_Palstance_Measure[YAW][GYRO],motor_gyro_set[YAW][TOP]);
		current_set[PITCH][MECH] = PID_Calc(&gimbal_pitch_motor_mech_pid,Cloud_Palstance_Measure[PITCH][MECH],motor_gyro_set[PITCH][MECH]);	
		
		given_current[YAW][TOP]	=	 current_set[YAW][TOP];
    given_current[PITCH][MECH]	=	current_set[PITCH][MECH];
	}
}


/**
  * @brief  自瞄pid计算
  * @param  void
  * @retval void
  * @attention 此处不能改变目标角度,只能用来做限幅和调用PID计算函数
  */
void GIMBAL_PositionLoop_AUTO(void)
{
  if(modeGimbal == CLOUD_MECH_MODE)
	{
		Cloud_Palstance_Measure[YAW][MECH] = Cloud_Palstance_Measure[YAW][MECH]/100;
		Cloud_Palstance_Measure[PITCH][MECH] = Cloud_Palstance_Measure[PITCH][MECH]/100;
		motor_gyro_set[YAW][MECH] = GIMBAL_PID_Calc(&Gimbal_yaw_key_PID,Cloud_Angle_Measure[YAW][MECH], Cloud_Angle_Target[YAW][MECH],Cloud_Palstance_Measure[YAW][MECH]);
		motor_gyro_set[PITCH][MECH] = GIMBAL_PID_Calc(&Gimbal_Pitch_key_PID,Cloud_Angle_Measure[PITCH][MECH], Cloud_Angle_Target[PITCH][MECH],Cloud_Palstance_Measure[PITCH][MECH]);
		current_set[YAW][MECH] = PID_Calc(&gimbal_yaw_motor_mech_pid,Cloud_Palstance_Measure[YAW][MECH],motor_gyro_set[YAW][MECH]);
		current_set[PITCH][MECH] = PID_Calc(&gimbal_pitch_motor_mech_pid,Cloud_Palstance_Measure[PITCH][MECH],motor_gyro_set[PITCH][MECH]);	
    
    given_current[YAW][MECH]	=	 current_set[YAW][MECH];
    given_current[PITCH][MECH]	=	current_set[PITCH][MECH];
	}
	else if(modeGimbal == CLOUD_GYRO_MODE)
	{
//		Cloud_Palstance_Measure[YAW][GYRO] = Cloud_Palstance_Measure[YAW][MECH]/100;
//		Cloud_Palstance_Measure[PITCH][MECH] = Cloud_Palstance_Measure[PITCH][MECH]/10;
		motor_gyro_set[YAW][GYRO] = GIMBAL_PID_Calc(&Gimbal_yaw_key_PID,Cloud_Angle_Measure[YAW][GYRO], Cloud_Angle_Target[YAW][GYRO],Cloud_Palstance_Measure[YAW][GYRO]);
		motor_gyro_set[PITCH][MECH] = GIMBAL_PID_Calc(&Gimbal_Pitch_key_PID,Cloud_Angle_Measure[PITCH][MECH], Cloud_Angle_Target[PITCH][MECH],Cloud_Palstance_Measure[PITCH][MECH]);
//		motor_gyro_set[PITCH][GYRO] = GIMBAL_PID_Calc(&Gimbal_Pitch_Gyro_PID,Cloud_Angle_Measure[PITCH][MECH], Cloud_Angle_Target[PITCH][GYRO],Cloud_Palstance_Measure[PITCH][GYRO]); 
		
		current_set[YAW][GYRO] = PID_Calc(&gimbal_yaw_motor_gyro_pid,Cloud_Palstance_Measure[YAW][GYRO],motor_gyro_set[YAW][GYRO]);
		current_set[PITCH][MECH] = PID_Calc(&gimbal_pitch_motor_mech_pid,Cloud_Palstance_Measure[PITCH][MECH],motor_gyro_set[PITCH][MECH]);	
//		current_set[PITCH][GYRO] = PID_Calc(&gimbal_pitch_motor_gyro_pid,Cloud_Palstance_Measure[PITCH][GYRO],motor_gyro_set[PITCH][GYRO]);	
    
    given_current[YAW][GYRO]	=	 current_set[YAW][GYRO]; 
    given_current[PITCH][MECH]	=	current_set[PITCH][MECH];
	}
	else if(modeGimbal == CLOUD_TOP_MODE)
	{
		motor_gyro_set[YAW][TOP] = GIMBAL_PID_Calc(&Gimbal_Yaw_Gyro_PID,Cloud_Angle_Measure[YAW][GYRO], Cloud_Angle_Target[YAW][TOP],Cloud_Palstance_Measure[YAW][GYRO]);
		motor_gyro_set[PITCH][MECH] = GIMBAL_PID_Calc(&Gimbal_Pitch_Mech_PID,Cloud_Angle_Measure[PITCH][MECH], Cloud_Angle_Target[PITCH][MECH],Cloud_Palstance_Measure[PITCH][MECH]);
		
		current_set[YAW][TOP] = PID_Calc(&gimbal_yaw_motor_gyro_pid,Cloud_Palstance_Measure[YAW][GYRO],motor_gyro_set[YAW][TOP]);
		current_set[PITCH][MECH] = PID_Calc(&gimbal_pitch_motor_mech_pid,Cloud_Palstance_Measure[PITCH][MECH],motor_gyro_set[PITCH][MECH]);	
		
		given_current[YAW][TOP]	=	 current_set[YAW][TOP];
    given_current[PITCH][MECH]	=	current_set[PITCH][MECH];
	}
}

/*-------------------------------------------------------电流发送函数在这里---------------------------------------------------------------*/
void GIMBAL_CanSend(void)
{
	float fMotorOutput[2] = {0};
		
	if(modeGimbal == CLOUD_MECH_MODE)
		{
			fMotorOutput[YAW]   = given_current[YAW][MECH];
			fMotorOutput[PITCH] = given_current[PITCH][MECH];
		}
	else if(modeGimbal == CLOUD_GYRO_MODE)
		{
			
			fMotorOutput[YAW]   = given_current[YAW][GYRO] * (-YAW_POSITION);
			fMotorOutput[PITCH] = given_current[PITCH][MECH];
		}
		else if(modeGimbal == CLOUD_TOP_MODE)
	{
		fMotorOutput[YAW]   = given_current[YAW][TOP] * (-YAW_POSITION);
		fMotorOutput[PITCH] = given_current[PITCH][MECH];
	}
		
		
		CAN_CMD_GIMBAL(fMotorOutput[YAW],fMotorOutput[PITCH],0,0);
//		CAN_CMD_GIMBAL(1000,1000,0,0);
}





/*------------------------------------------------------------辅助函数------------------------------------------------------------*/

/*        临界值结构体初始化    获取陀螺仪角度，角速度    目标速度计算函数   YAW轴偏离中心角度    限制云台与底盘分离角度  函数          */
/**
  * @brief 临界值结构体初始化
  * @param  critical:临界值结构体指针
  *    get:当前读取到的角度（陀螺仪角或机械角度）
  * @retval void
  */
void Critical_Handle_Init(Critical_t *critical, float get)
{
	
	critical->AngleSum = get;//0;
	critical->CurAngle = get;
	critical->LastAngle = get;  
	
	Cloud_Angle_Target[YAW][GYRO] = get; 

}


float speed_threshold = 5.f;//速度过快
float debug_speed;//左正右负,一般都在1左右,debug看
float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position)
{
	S->delay_cnt++;

	if (time != S->last_time)
	{
		S->speed = (position - S->last_position) / (time - S->last_time) * 2;//计算速度
		
		
//		if ((S->speed - S->processed_speed) < -speed_threshold)
//		{
//			S->processed_speed = S->processed_speed - speed_threshold;//速度斜坡变化
//		}                                                                                           //DEBUG用
//		else if ((S->speed - S->processed_speed) > speed_threshold)
//		{
//			S->processed_speed = S->processed_speed + speed_threshold;//速度斜坡变化
//		}
		

		S->processed_speed = S->speed;

		S->last_time = time;
		S->last_position = position;
		S->last_speed = S->speed;
		S->delay_cnt = 0;
	}

	if(S->delay_cnt > 300/*100*/) // delay 200ms speed = 0
	{
		S->processed_speed = 0;//时间过长则认为速度不变
	}
	debug_speed = S->processed_speed;
	return S->processed_speed;//计算出的速度
}

/**
  * @brief  计算YAW偏离中心角度,底盘跟随模式用
  * @param  void
  * @retval sAngleError,偏离角度值,CAN反馈的机械角度
  */
float GIMBAL_GetOffsetAngle(void)
{
  float sAngleError = 0;
	
	if((angleMotorYaw < 4469.0f) && (angleMotorYaw > 4369.0f))//0.034.f   4250    3750
	{
		sAngleError = 0;
	}
	else
	{
		#if YAW_POSITION ==	YAW_DOWN
            sAngleError =  Cloud_Angle_Measure[YAW][MECH];
        #else
		    sAngleError =  Cloud_Angle_Measure[YAW][CHANGE];//CHANGE
		#endif
	}
	
	return  sAngleError;
}


///**
//  * @brief  计算YAW偏离中心角度,底盘跟随模式用
//  * @param  void
//  * @retval sAngleError,偏离角度值,CAN反馈的机械角度
//  */
//float GIMBAL_GetOffsetAngle_Half(void)
//{
//  float sAngleError = 0;
//	
//	if((angleMotorYaw < 4696.0f) && (angleMotorYaw > 4296.0f))//0.034.f   4250    3750
//	{
//		sAngleError = 0;
//	}
//	else
//	{
//		#if YAW_POSITION ==	YAW_DOWN	
//            sAngleError =  Cloud_Angle_Measure[YAW][CHANGE];
//        #else
//		    sAngleError =  Cloud_Angle_Measure[YAW][MECH];//CHANGE
//		#endif
//	}
//	
//	return  sAngleError;
//}

/*---------------------------------------------------------------------------------自瞄，预测yaw轴的辅助函数------------------------------------------------------------------------------*/
/**
  * @brief  自瞄yaw轴预测是否已经开启
  * @param  void
  * @retval TRUE开启  FALSE关闭
  * @attention 
  */
bool GIMBAL_IfAuto_MobPre_Yaw(void)
{
return FALSE;
}

/**
  * @brief  yaw轴开启预测的时候云台是否到位
  * @param  void
  * @retval TRUE到位可打弹   FALSE没到位禁止打弹
  * @attention 左右各有延迟，换向时记得清零反向和静止时的延迟
  */
bool GIMBAL_MOBPRE_YAW_FIRE(void)
{
return FALSE;
}

/*---------------------------------------------------------------------------------------打符yaw，pitch轴是否移动到位-----------------------------------------------------------------*/


/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------哨兵的自瞄和预测辅助函数-----------------------------------------------------------------*/
/**
  * @brief  是否在自瞄哨兵
  * @param  void
  * @retval TRUE   FALSE
  * @attention 自瞄抬头角度太大则认为在打哨兵
  */
bool GIMBAL_AUTO_PITCH_SB(void)
{
return FALSE;
}


/**
  * @brief  是否在中等距离自瞄哨兵,加大预测
  * @param  void
  * @retval TRUE   FALSE
  * @attention 自瞄抬头角度太大则认为在打哨兵
  */
float pitch_sb_error = 0;
bool GIMBAL_AUTO_PITCH_SB_SK(void)
{
return FALSE;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------是否开启吊射基地模式---------------------------------------------------------------------*/
/**
  * @brief  是否开启吊射模式
  * @param  void
  * @retval TRUE开启  FALSE关闭
  * @attention 
  */
bool GIMBAL_If_Base(void)
{
    if (actGimbal == GIMBAL_BASE)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------是否打符，是否手打，是否大符，是否小符---------------------------------------------------------------*/
/**
  * @brief  是否开启打符模式
  * @param  void
  * @retval TRUE开启  FALSE关闭
  * @attention 
  */
bool GIMBAL_IfBuffHit(void)
{
 if (actGimbal == GIMBAL_BUFF || actGimbal == GIMBAL_SM_BUFF|| actGimbal == GIMBAL_MANUAL)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  是否开启手动打符模式
  * @param  void
  * @retval TRUE开启  FALSE关闭
  * @attention 
  */
bool GIMBAL_IfManulHit(void)
{
    if (actGimbal == GIMBAL_MANUAL)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  是否开启打大符模式
  * @param  void
  * @retval TRUE开启  FALSE关闭
  * @attention 
  */
bool GIMBAL_If_Big_Buff(void)
{
    if (actGimbal == GIMBAL_BUFF)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  是否开启打小符模式
  * @param  void
  * @retval TRUE开启  FALSE关闭
  * @attention 
  */
bool GIMBAL_If_Small_Buff(void)
{
    if (actGimbal == GIMBAL_SM_BUFF)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

/**
  * @brief  打符yaw是否移动到位
  * @param  void
  * @retval TRUE到位可打弹   FALSE没到位禁止打弹
  * @attention 
  */

bool GIMBAL_BUFF_YAW_READY(void)
{
return FALSE;
}

/**
  * @brief  打符pitch是否移动到位
  * @param  void
  * @retval TRUE到位可打弹   FALSE没到位禁止打弹
  * @attention 
  */

bool GIMBAL_BUFF_PITCH_READY(void)
{
 return FALSE;
}
/*---------------------------------------------------更新角度的机械角度和陀螺仪角度的函数------------------------------------------------------------*/

//计算相对云台中值的角度
static fp32 motor_ecd_to_angle_change(uint16_t ecd)
{
    int32_t relative_ecd = ecd - 4396;
    if (relative_ecd > Half_ecd_range)
    {
        relative_ecd -= ecd_range;
    }
    else if (relative_ecd < -Half_ecd_range)
    {
        relative_ecd += ecd_range;
    }

    return relative_ecd * Motor_Ecd_to_Rad ;
}


//计算相对云台中值的角度
static fp32 motor_ecd_to_angle_change1(uint16_t ecd)
{
    int32_t relative_ecd = ecd;
    if (relative_ecd > Half_ecd_range)
    {
        relative_ecd -= ecd_range;
    }
    else if (relative_ecd < -Half_ecd_range)
    {
        relative_ecd += ecd_range;
    }

    return relative_ecd * Motor_Ecd_to_Rad ;
}


fp32 Angle_Measure[2][3];
/**
  * @brief  更新云台机械角度,角速度,电流值,can1中断中调用
  * @param  void
  * @retval void
  * @attention 
  */
void GIMBAL_UpdateAngle( char ID, int16_t angle )
{
if (ID == PITCH)
	{
		angleMotorPit = angle;
		Cloud_Angle_Measure[PITCH][MECH]  = motor_ecd_to_angle_change(angleMotorPit);	
	}
	else if (ID == YAW)
	{
		angleMotorYaw = angle;
		Cloud_Angle_Measure[YAW][MECH]  = motor_ecd_to_angle_change(angleMotorYaw);
		Angle_Measure[YAW][MECH] = angleMotorYaw;
		Cloud_Angle_Measure[YAW][CHANGE]  = motor_ecd_to_angle_change(angleMotorYaw);
	}
}

void GIMBAL_UpdateSpeed( char ID, int16_t speed )
{
	if (ID == PITCH)
	{
		speedMotorPit = speed;
		Cloud_Speed_Measure[PITCH][MECH]  = speedMotorPit;
	}
	else if (ID == YAW)
	{
		speedMotorYaw = speed;
		Cloud_Speed_Measure[YAW][MECH]  = speedMotorYaw;
	}
}

void GIMBAL_UpdateCurrent( char ID, int16_t current )
{
	if (ID == PITCH)
	{
		currentMotorPit = current;
		Cloud_Current_Measure[PITCH][MECH]  = currentMotorPit;
	}
	else if (ID == YAW)
	{
		currentMotorYaw = current;
		Angle_Measure[YAW][MECH] = angleMotorYaw;
		Cloud_Current_Measure[YAW][MECH]  = currentMotorYaw;
	}
}

/**
  * @brief  更新云台姿态,500HZ,loop中调用
  * @param  void
  * @retval void
  * @attention 角度适度放大
  */
float AngleMpuYaw[2];
float AngleMpuPitch[2];
float AngleMpuRoll[2];

void GIMBAL_MPU_Update(void)
{
	//读取陀螺仪  角度   角速度   
	mpu_dmp_get_data( &angleMpuRoll, &angleMpuPitch, &angleMpuYaw );
	MPU_Get_Gyroscope( &palstanceMpuPitch, &palstanceMpuRoll, &palstanceMpuYaw );
	
	AngleMpuYaw[NOW]   = angleMpuYaw-AngleMpuYaw[LAST];			 //22
	AngleMpuPitch[NOW] = angleMpuPitch-AngleMpuPitch[LAST];  //-166
	AngleMpuRoll[NOW]  = angleMpuRoll-AngleMpuRoll[LAST]; 	 //4.2
	
			//将陀螺仪角度放大,不放大则陀螺仪模式内环P要给很大,会不好调
		Cloud_Angle_Measure[PITCH][GYRO]  =  (AngleMpuPitch[NOW]*PI)/180;
	  Cloud_Angle_Measure[YAW][GYRO] = (AngleMpuYaw[NOW]*PI)/180 ;
	  //theta_format(Cloud_Angle_Measure[YAW][GYRO]);
	
		//角速度更新
		Cloud_Palstance_Measure[PITCH][MECH] = ((palstanceMpuPitch + PALST_COMPS_PITCH)*PI)/180;
		Cloud_Palstance_Measure[YAW][MECH]   = ((palstanceMpuYaw+PALST_COMPS_YAW)*PI)/180;
		
		Cloud_Palstance_Measure[PITCH][GYRO] = (palstanceMpuPitch + PALST_COMPS_PITCH)/10;   //直接读陀螺仪角速度
		Cloud_Palstance_Measure[YAW][GYRO]   = (palstanceMpuYaw+PALST_COMPS_YAW)/10;  //经过计算得出的角速度
}


void MPU_Update_last(void)
{
	mpu_dmp_get_data( &angleMpuRoll, &angleMpuPitch, &angleMpuYaw );
	MPU_Get_Gyroscope( &palstanceMpuPitch, &palstanceMpuRoll, &palstanceMpuYaw );
	AngleMpuYaw[LAST] = angleMpuYaw;
	AngleMpuPitch[LAST] = angleMpuPitch;
	AngleMpuRoll[LAST] = angleMpuRoll;
}

float Error[2][2];
void Gimbal_Error_Read(void)
{
	Error[YAW][MECH]= Cloud_Angle_Target[YAW][MECH] -Cloud_Angle_Measure[YAW][MECH];
	Error[YAW][GYRO]= Cloud_Angle_Target[YAW][GYRO] -Cloud_Angle_Measure[YAW][GYRO];
	Error[PITCH][MECH]= Cloud_Angle_Target[PITCH][MECH] -Cloud_Angle_Measure[PITCH][MECH];
	Error[PITCH][GYRO]= Cloud_Angle_Target[PITCH][GYRO] -Cloud_Angle_Measure[PITCH][GYRO];
}

static void GIMBAL_PID_Init(Gimbal_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;

}
static fp32 GIMBAL_PID_Calc(Gimbal_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

//pid数据清理
static void Gimbal_PID_clear(Gimbal_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }
    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}

#define TOP_STEP0    0		//失能标志
#define TOP_STEP1    1		//SW1复位标志
#define TOP_STEP2    2		//弹仓开关标志
uint8_t	TOP_Switch = 0;//弹仓遥控模式开关标志位转换
bool TOP_Rc_Switch(void)
{
	if (IF_RC_SW2_DOWN)//机械模式
	{
			if (TOP_Switch == TOP_STEP1)
			{
				TOP_Switch = TOP_STEP2;
			}
			 else if (TOP_Switch == TOP_STEP2)
			{
				TOP_Switch = TOP_STEP0;
			}
	}
	else
	{
		TOP_Switch = TOP_STEP1;
	}
	
	if (TOP_Switch == TOP_STEP2)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
	
}


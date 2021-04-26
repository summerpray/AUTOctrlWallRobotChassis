#include "gimbal_key_control.h"
#include "kalman.h"
#include "kalman_filter.h"

#include "mpu6050.h"
#include "inv_mpu.h"
#include "vision.h"
#include "magazine.h"
#include "chassis_task.h"

#include "pid.h"
#include "start_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "arm_math.h"

#include "user_lib.h"

#include "remote_control.h"
#include "CAN_Receive.h"


extern  RC_ctrl_t rc_ctrl;    //定义遥控器结构体参数
extern GimbalCtrlMode  modeGimbal;   //定义云台控制模式    机械/陀螺仪
extern eGimbalAction  actGimbal;     //定义云台运动模式  调头 自瞄 打符等


//调头模式角度目标
float TURNMode_Yaw_Back_Total;//按下C,yaw需要改变的角度值
float TURNMode_Yaw_Turn_Total;//按下QE,yaw需要改变的角度值,正负代表左右转

float Cloud_Angle_Target_GD[2][2];   //定义目标角度过度变量   yaw/pitch    mech/gyro


/**
  * @brief  云台键盘模式选择,按键响应
  * @param  void
  * @retval void
  * @attention 云台键盘控制状态下的所有模式切换都在这
  * 无模式切换时一直处于此模式
  */
	
void GIMBAL_NORMAL_Mode_Ctrl(void)
{
	//按键延时响应,防止手贱狂按
	static portTickType  Key_Ctrl_CurrentTime = 0;
	static uint32_t PressV_Time  = 0;//调头,500ms延时响应,1秒最多按2下
	static uint32_t PressQ_Time  = 0;//90°,250ms延时响应,1秒最多按4下
  static uint32_t PressE_Time  = 0;//90°,250ms延时响应,1秒最多按4下
//	static uint32_t PressCF_Time  = 0;//打大符,400ms延时响应
//	static uint32_t PressCV_Time  = 0;//打小符,400ms延时响应
	
	Key_Ctrl_CurrentTime = xTaskGetTickCount( );//获取实时时间,用来做按键延时判断	
	
	
	if ( CHASSIS_IfActiveMode() == TRUE || Magazine_IfOpen() ==	 TRUE)//获取底盘模式,true为机械模式
	{
		modeGimbal = CLOUD_MECH_MODE;
	} 
	else					//注释掉loop中的底盘会令陀螺仪模式失效
	{
		modeGimbal = CLOUD_GYRO_MODE;
	}
	
	if ( !IF_KEY_PRESSED_CTRL && IF_KEY_PRESSED_V && Key_Ctrl_CurrentTime > PressV_Time)  //Ctrl不处于按下状态时按V调头
	{   
		actGimbal  =  GIMBAL_AROUND;//切换成调头模式

		PressV_Time = Key_Ctrl_CurrentTime + TIME_STAMP_500MS;//500ms延时防手贱狂按

		if(IF_KEY_PRESSED_A)//AV左调头
		{
			TURNMode_Yaw_Back_Total = PI;
		}
		else if(IF_KEY_PRESSED_D)//DV右调头
		{
			TURNMode_Yaw_Back_Total = -PI;
		}
		else//默认右调头
		{
			TURNMode_Yaw_Back_Total = -PI;
		}
	}
	/*---------------------------------*/	
	else if ( !IF_KEY_PRESSED_CTRL && ( (IF_KEY_PRESSED_Q && Key_Ctrl_CurrentTime > PressQ_Time) || (IF_KEY_PRESSED_E && Key_Ctrl_CurrentTime > PressE_Time) ) ) //Ctrl不处于按下状态时按Q(左),E(右)90°调头
	{   
		actGimbal = GIMBAL_TURN;//切换成快速扭头模式
		
		//注意方向
		if ( IF_KEY_PRESSED_Q)
		{
			PressQ_Time = Key_Ctrl_CurrentTime + TIME_STAMP_250MS;//250ms延时防手贱狂按
			
			TURNMode_Yaw_Turn_Total = PI/2;
		}
		else if (IF_KEY_PRESSED_E)
		{
			PressE_Time = Key_Ctrl_CurrentTime + TIME_STAMP_250MS;//250ms延时防手贱狂按
			
			TURNMode_Yaw_Turn_Total = -PI/2;
		}
			
	}	
	/*---------------------------------*/
	else if ( Magazine_IfWait() == TRUE || Magazine_IfOpen() == TRUE )			//弹仓开启或正在开启,云台归中不给动

	{
		
		actGimbal = GIMBAL_LEVEL;

	}
	else if (IF_MOUSE_PRESSED_RIGH && !IF_RC_SW1_MID)//若SW1不在中,则右键自瞄
	{
		actGimbal = GIMBAL_AUTO;

	}
//	/*----------------小符-----------------*/
//	else if(IF_KEY_PRESSED_V && IF_KEY_PRESSED_CTRL && Key_Ctrl_CurrentTime > PressCV_Time)//Ctrl+V打符,400ms响应一次
//	{
//		PressCV_Time = Key_Ctrl_CurrentTime + TIME_STAMP_400MS;
//		actGimbal = GIMBAL_SM_BUFF;
//	}
//	/*----------------大符-----------------*/
//	else if(IF_KEY_PRESSED_F && IF_KEY_PRESSED_CTRL && Key_Ctrl_CurrentTime > PressCF_Time)//Ctrl+F打符,400ms响应一次
//	{
//		PressCF_Time = Key_Ctrl_CurrentTime + TIME_STAMP_400MS;
//		actGimbal = GIMBAL_BUFF;
//	}
//	/*----------------吊射-----------------*/
//	else if(IF_KEY_PRESSED_C && !IF_MOUSE_PRESSED_RIGH && !IF_RC_SW1_MID)
//	{
//		actGimbal = GIMBAL_BASE;
//	}
	else
	{
		GIMBAL_Set_Key_Control();   //普通模式下的目标角度计算
	}
}	

/**
  * @brief  补弹模式
  * @param  void
  * @retval void
  * @attention 此模式下禁止控制pitch
  */
void GIMBAL_LEVEL_Mode_Ctrl(void)
{
	modeGimbal = CLOUD_MECH_MODE;//补弹时进入机械模式
	
	//补弹完毕,退出补弹模式
	if( Magazine_IfWait() == FALSE	&& Magazine_IfOpen() == FALSE )
	{
		actGimbal = GIMBAL_NORMAL;
	}
	else//补弹未完成,角度固定在中间
	{
		Cloud_Angle_Target_GD[YAW][MECH]   = mid_yaw_angle;
		Cloud_Angle_Target_GD[PITCH][MECH] = mid_pitch_angle;
	}
}

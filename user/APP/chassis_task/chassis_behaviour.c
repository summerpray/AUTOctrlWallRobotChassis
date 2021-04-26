#include "chassis_behaviour.h"
#include "arm_math.h"
#include "gimbal_task.h"


extern  RC_ctrl_t rc_ctrl;

extern fp32 angle_swing_set;

void chassis_follow_gimbal_yaw_control(void)
{
    //摇摆角度是利用sin函数生成，swing_time 是sin函数的输入值
    static fp32 swing_time = 0.0f;
    //swing_time 是计算出来的角度
    static fp32 swing_angle = 0.0f;
    //max_angle 是sin函数的幅值
    static fp32 max_angle = SWING_NO_MOVE_ANGLE;
    //add_time 是摇摆角度改变的快慢，最大越快
    static fp32 const add_time = PI / 300.0f;
    //使能摇摆标志位
    static uint8_t swing_flag = 0;

    //计算遥控器的原始输入信号
    //判断是否要摇摆
    if (IF_RC_SW1_DOWN)
    {
        if (swing_flag == 0)
        {
            swing_flag = 1;
            swing_time = 0.0f;
        }
    }
    else
    {
        swing_flag = 0;
    }
		
    //判断键盘输入是不是在控制底盘运动，底盘在运动减小摇摆角度
    if (rc_ctrl.rc.ch[2]>10 || rc_ctrl.rc.ch[3]>10)
    {
        max_angle = SWING_MOVE_ANGLE;
    }
    else
    {
        max_angle = SWING_NO_MOVE_ANGLE;
    }
		
    //sin函数生成控制角度
    if (swing_flag)
    {
        swing_angle = max_angle * arm_sin_f32(swing_time);
        swing_time += add_time;
    }
    else
    {
        swing_angle = 0.0f;
    }
    //sin函数不超过2pi
    if (swing_time > 2 * PI)
    {
        swing_time -= 2 * PI;
    }
		angle_swing_set = swing_angle;
}



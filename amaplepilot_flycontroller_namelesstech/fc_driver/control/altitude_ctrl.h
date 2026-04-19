#ifndef __ALTITUDE_CTRL_H
#define __ALTITUDE_CTRL_H

#include <stdint.h>


//高度控制周期
#define ALT_POS_PERIOD       PERIOD_UNIT_MIN
#define ALT_POS_CTRL_PERIOD  5//ms
#define ALT_POS_CTRL_CNT     (ALT_POS_CTRL_PERIOD/ALT_POS_PERIOD)



#define THR_MOTOR_START  	 1050//起转油门量，油门倾角补偿用，太大会导致过补偿
#define THR_HOVER_DEFAULT  1500
#define NUL 0

/****************************************************************
当油门推重比较小时，若上升下降期望加速度比较大，
会导致输出映射的里面的姿态控制量得不到充分输出，
使得快速上升下降时，姿态不平稳，若此过程持续时间长，
会导致姿态长期得不到修正，直致最后炸鸡，故推重比较
小时，可将期望加速度限小一点，或者做控制量优先级处理
*******************************************************************/
#define  Climb_Up_Acceleration_Max     500//向上最大攀爬加速度，cm/s^2  1000  400  250
#define  Climb_Down_Acceleration_Max   250//向下最大下降加速度，cm/s^2   500   250  150


typedef enum 
{
  ALTHOLD_MANUAL_CTRL=0,		//高度手动控制
  ALTHOLD_AUTO_POS_CTRL,		//高度直接位置控制
	ALTHOLD_AUTO_VEL_CTRL,    //高度直接速度控制
}ALTHOLD_CTRL_MODE;


void flight_altitude_control(uint8_t mode,float target_alt,float target_vel);
uint16_t throttle_angle_compensate(uint16_t _throttle);

void landon_earth_check(uint8_t shield);
uint8_t get_landon_state(void);
void reset_landon_state(void);


extern uint16_t throttle_output;


#endif



//	hover_thr_tmp=maple_ctrl.height_accel_ctrl.control_output+(throttle_hover_value-THR_MOTOR_START);
//	hover_thr_tmp=hover_thr_tmp*(1+maple_ctrl.height_accel_ctrl.expect/980);
//	throttle_output=(uint16_t)(THR_MOTOR_START+hover_thr_tmp);
//	throttle_output=constrain_int16(throttle_output,THR_CONTROL_WORK,THR_MAX_OUTPUT*0.9f);








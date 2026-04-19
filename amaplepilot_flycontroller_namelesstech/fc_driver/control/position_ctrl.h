#ifndef __POSITION_CTRL_H
#define __POSITION_CTRL_H


enum POSHOLD_CTRL_MODE
{
  POSHLOD_MANUAL_CTRL=0,
  POSHOLD_SEMI_AUTO_POS_CTRL,//半自动,中途允许手动控制
	POSHOLD_FULL_AUTO_POS_CTRL,//全自动,中途手动控制无效
	POSHOLD_AUTO_VEL_CTRL,
	POSHOLD_HOR_MANUAL_CTRL,
	POSHOLD_HOR_AUTO_CTRL,
};

#define POSHOLD_SPEED_0              10      // speed below which it is always safe to switch to loiter
#define Max_Horvel 500
#define Max_Opticalflow_Speed  250




//位置控制周期
#define POS_PERIOD       PERIOD_UNIT_MIN
#define POS_CTRL_PERIOD  40
#define VEL_CTRL_PERIOD  20
#define POS_CTRL_CNT     (POS_CTRL_PERIOD/POS_PERIOD)
#define VEL_CTRL_CNT     (VEL_CTRL_PERIOD/POS_PERIOD)



void flight_position_control(uint8_t mode,
														 int32_t target_lng,int32_t target_lat,//目标经、纬度
														 float target_e_vel,float target_n_vel,  //目标巡航速度
														 float max_cruising_speed,
														 uint8_t target_update,															 
														 uint8_t force_fixed_flag);
														 
														 
void opticalflow_position_ctrl(uint8_t force_brake_flag);
void indoor_position_control(uint8_t force_brake_flag);														 
void vioslam_position_ctrl(uint8_t force_brake_flag);	
void opticalflow_speed_ctrl(vector2f target);

void Horizontal_Navigation(float x,float y,float z,uint8_t nav_mode,uint8_t frame_id);


extern vector2f accel_target;
														 
#endif
														 
														 
														 
														 
														 
														 

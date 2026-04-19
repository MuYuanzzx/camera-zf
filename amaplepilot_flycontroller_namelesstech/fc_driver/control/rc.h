#ifndef __RC_H
#define __RC_H



#include <stdint.h>
#include <stdbool.h>


#define RC_MAX_DEFAULT  					2000		//最大值
#define RC_MIN_DEFAULT  					1000		//最小值
#define RC_MIDDLE_DEFAULT  	  		1500		//中间值
#define RC_DEADBAND_DEFAULT   		100		  //中位死区值
#define RC_REVERSE_FLAG  					false		//反向标志位
#define RC_DEADBAND_PERCENT   		0.1f		//中位死区占实际行程的百分比
#define RC_THR_DEADBAND_PERCENT   0.15f		//油门中位死区占实际行程的百分比	0.2
	
#define  Scale_Pecent_Max  0.75f   	 //最大解锁幅值量程因子
#define  Pit_Rol_Max 35.0f           //最大俯仰、横滚期望  40
#define  Yaw_Max     200.0f          //最大偏航角速度期望  300
#define  Buttom_Safe_Deadband  50.0f //油门底部安全死区





typedef enum 
{
	RC_ROLL_CHANNEL=0,
	RC_PITCH_CHANNEL,
	RC_THR_CHANNEL,
	RC_YAW_CHANNEL,
	RC_AUX1_CHANNEL,
	RC_AUX2_CHANNEL,
	RC_AUX3_CHANNEL,
	RC_AUX4_CHANNEL,
	RC_CHL_MAX
}RC_CHL_MAP;


typedef enum 
{
  RC_ROLL=0,
	RC_PITCH,
	RC_YAW,
	RC_THR
}RC_RPYT;

typedef enum 
{
  AUX1=0,
	AUX2,
	AUX3,
	AUX4
}AUX;

typedef enum 
{
  LOCK=0,
	UNLOCK,
}Lock_Mode;


typedef enum 
{
	THR_BUTTOM=0,
	THR_MIDDLE=1,
	THR_UP=2,
}THR_POSITION;

typedef enum 
{
	FREE_STYLE=0,
	BUTTOM_TO_MIDDLE,
	MIDDLE_TO_BUTTOM,
	MIDDLE_TO_UP,
	UP_TO_MIDDLE,
}THR_PUSH_MODE;



typedef struct
{
  uint16_t max_value;		//最大值
	uint16_t min_value;		//最小值
	uint16_t middle_value;//中间值
	uint16_t deadband;    //中位死区值
	bool reverse_flag;    //反向标志位
	float scale;
}rc_calibration;


typedef struct
{
  uint16_t rcdata[RC_CHL_MAX];
	float rc_q[4];		//期望姿态四元数
  float rc_rpyt[4]; //期望姿态角度(deg)、Z速度
	float rc_rpy[3];  //期望姿态角度(rad)
  uint16_t thr,throttle;
  uint16_t aux[4];
	uint8_t height_mode;
	uint8_t sdk_duty_mode;
	uint8_t gps_mode;
	uint8_t position_mode;
	THR_PUSH_MODE thr_push_over_state;
	
	rc_calibration cal[RC_CHL_MAX];
	uint16_t unlock_makesure_cnt;
	uint16_t lock_makesure_cnt;
	Lock_Mode lock_state;
	Lock_Mode last_lock_state;
	int16_t auto_relock_cnt;
	uint8_t auto_relock_flag;
	uint8_t rc_return_flag;
	uint8_t unwanted_lock_flag;
	uint8_t fc_ready_flag;
	float debug_parameter[4];
}rc;

void rc_init(void);
void rc_prase(ppm *_ppm);
void unlock_state_check(void);


extern rc rc_data;

#endif

#ifndef __LAND_CTRL_H
#define __LAND_CTRL_H


#include <stdint.h>
#include <stdbool.h>


enum
{
	FARAWAY_MODE=0,
	NEAR_MODE,
	REACH_MODE
};


#define FARAWAY_DIS_CM 600//600cm
#define NEAR_DIS_CM    150//150cm
#define REACH_DIS_CM   50//50cm


#define SAFE_GROUND_DISTANCE      800//800cm
#define NEAR_GROUND_DISTANCE      100//100cm


#define RISE_VEL_DEFAULT   100//cm/s
#define DECLINE_VEL_FAR_GROUND    -100//cm/s
#define DECLINE_VEL_DEFAULT       -50//cm/s
#define DECLINE_VEL_NEAR_GROUND   -30//cm/s


#define NAV_SPEED_FIRST   	150//从A->B，开启一级巡航速度，单位cm       100
#define NAV_SPEED_SECOND  	100 //从B->C，开启二级巡航速度，单位cm       80
#define NAV_SPEED_THIRD   	80 //处于C点附近，开启三级巡航速度, 单位cm  60
#define NAV_SPEED_DEFAULT 	50 //默认巡航速度                           50

typedef struct
{
	uint8_t curr_distance_mode;//当前位置模式
	uint8_t last_distance_mode;//上次位置模式
	uint8_t home_fixed_flag; 
	//
	uint8_t first_swich_in;		//首次切入
	guide_gps target;					//水平目标位置
	float max_cruising_speed;	//返航时水平最大巡航速度
	
	float alt_target_pos;
	float alt_target_vel;
	uint16_t alt_target_time;
	uint16_t alt_cnt;
	uint8_t poshold_mode;
	uint8_t althold_mode;
	uint8_t process_state;
}maple_land_state;


void land_without_gps(void);
void land_with_gps(int32_t lng,int32_t lat);
void land_run(void);

#endif







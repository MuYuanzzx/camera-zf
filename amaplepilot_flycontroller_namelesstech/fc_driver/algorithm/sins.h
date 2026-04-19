#ifndef __SINS_H
#define __SINS_H

#include "arm_math.h"



typedef struct
{
	float p[3][3]; 			//协方差矩阵
	float qp,qv;   			//加速度计过程噪声
	float	qb;      			//加速度计过程偏移噪声
	float rp[6],rv[6];  //位置观测噪声、速度观测噪声
	float cp,cv,cb;			//修正量
	float k[3][2]; 			//增益矩阵
	float err[2];
	uint8_t init;
}kalman_filter;

typedef struct
{
	float p[3][3]; //协方差矩阵
	float qp,qv;   //加速度计过程噪声
	float	qb;      //加速度计过程偏移噪声
	float rp,rv;   //位置观测噪声、速度观测噪声
	float cp[2],cv[2],cb[2];//修正量
	float k[3][2]; ////增益矩阵
	float err[2][2];
	uint8_t init;
}_kalman_filter;

void kalman_filter_init(kalman_filter *kf,float *p,float qp,float qv,float qb,float *rp,float *rv);
void altitude_kalman_filter(kalman_filter *kf,sins *_ins,float acc,float alt_obs,float alt_vel_obs,float dt);
void third_order_complementarity(float ground_height,vector2f acc,opticalflow *opt);
void kalmanfilter_horizontal(void);
void indoor_position_fusion(void);
void strapdown_ins_height(void);


bool gps_fix_health(void);
void from_vio_to_body_frame(float map_x,float map_y,float *bod_x,float *bod_y);
void from_body_to_nav_frame(float bod_x,float bod_y,float *map_x,float *map_y);


extern Location gps_home_point;   //初始定位成功点信息
extern Location gps_present_point;//当前位置点信息


extern kalman_filter alt_kf;


extern float pos_z_fusion,vel_z_fusion,acc_z_fusion;
	

#endif



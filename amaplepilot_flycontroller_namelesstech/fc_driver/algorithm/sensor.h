#ifndef __SENSOR_H
#define __SENSOR_H

#include "FusionAhrs.h"



#define player_level_default 3//0:普通玩家;1:专业玩家;2硬核玩家1;3:硬核玩家2;4:硬核玩家3

#define gyro_lpf_param_default1         30//50
#define accel_lpf_param_default1        30//30
#define ins_lpf_param_default1          30//30
#define accel_fb_lpf_param_default1     5//5

#define gyro_lpf_param_default2         95//60
#define accel_lpf_param_default2        30//20
#define ins_lpf_param_default2          60//60
#define accel_fb_lpf_param_default2     5//10


//电池低压报警区间
#define no_voltage_enable_default 1
#define no_voltage_upper_default 11.2
#define no_voltage_lower_default 6.0


void inertial_acceleration_get(void);
void sensor_raw_update(void);
void sensor_filter_init(void);
void flymaple_nav_update(void);
void calculate_quaternion_init(vector3f acc,vector3f mag,float *q);
void sensor_fusion_update(void);

float get_earth_declination(void);


void battery_voltage_detection(void);


extern FusionAhrs ahrs;
extern filter_parameter ins_lpf_param,accel_lpf_param,gyro_lpf_param,accel_fb_lpf_param; 


#endif




//	float yaw_fus_beta=0.025f;
//	//俯仰、横滚角度来源于madgwick
//	flymaple.rpy_fusion_deg[0] =	flymaple.rpy_transition_deg[0];
//	flymaple.rpy_fusion_deg[1] =	flymaple.rpy_transition_deg[1];
//	//偏航角度来源于一阶互补滤波
//	flymaple.yaw_angle_enu+=flymaple.yaw_gyro_enu*0.005f;
//	if(flymaple.yaw_angle_enu<0)   flymaple.yaw_angle_enu+=360;
//	if(flymaple.yaw_angle_enu>360) flymaple.yaw_angle_enu-=360;
//	
//	float yaw_obs_err=flymaple.rpy_obs_deg[2]-flymaple.yaw_angle_enu;
//	if(flymaple.rpy_obs_deg[2]>270&&flymaple.yaw_angle_enu<90)      
//		yaw_obs_err=360-yaw_obs_err;
//	else if(flymaple.rpy_obs_deg[2]<90&&flymaple.yaw_angle_enu>270) 
//		yaw_obs_err=360+yaw_obs_err;
//	
//	if(flymaple.mag_update_flag==1)
//	{
//		flymaple.mag_update_flag=0;			
//		flymaple.yaw_angle_enu +=yaw_obs_err* yaw_fus_beta;
//		//偏航角数据量化到0~360
//	}
//	if(flymaple.yaw_angle_enu<0)   flymaple.rpy_fusion_deg[2]=flymaple.yaw_angle_enu+360;
//	else flymaple.rpy_fusion_deg[2]=flymaple.yaw_angle_enu;
//	
//	if(ins.gps_home_fixed_flag==1)//如果GPS home点已设置，获取当地磁偏角，得到地理真北  
//		flymaple.rpy_fusion_deg[2]=flymaple.rpy_fusion_deg[2]-get_earth_declination();


//	madgwick_updateimu(&flymaple.gyro,&flymaple.accel_ahrs,&flymaple.mag,t1.period/1000.0f,ahrs_sync);
//	madgwick_getangle(flymaple.rpy_transition_deg);
//	madgwick_get_quaternion(flymaple.quaternion);


//  IIR_High_Order_Filter(&accel_x_fb,flymaple.accel_raw_data_g[0]);
//	IIR_High_Order_Filter(&accel_y_fb,flymaple.accel_raw_data_g[1]);
//	IIR_High_Order_Filter(&accel_z_fb,flymaple.accel_raw_data_g[2]);	
//	flymaple.accel_fb.x=flymaple.accel_scale.x*accel_x_fb.output[accel_x_fb.N-1]-flymaple.accel_offset.x/GRAVITY_MSS;
//	flymaple.accel_fb.y=flymaple.accel_scale.y*accel_y_fb.output[accel_y_fb.N-1]-flymaple.accel_offset.y/GRAVITY_MSS;
//	flymaple.accel_fb.z=flymaple.accel_scale.z*accel_z_fb.output[accel_z_fb.N-1]-flymaple.accel_offset.z/GRAVITY_MSS;


//ButterFilterStruct accel_x_fb,accel_y_fb,accel_z_fb;
//accel_x_fb=filterIIRButterLowpass(5,0,60,0,1,80,sampling_frequent,FILTER_IIR_BUTTER_LOW);
//butterSbValue(&accel_x_fb);
//butterLowOrHigh(&accel_x_fb);

//accel_y_fb=filterIIRButterLowpass(5,0,60,0,1,80,sampling_frequent,FILTER_IIR_BUTTER_LOW);
//butterSbValue(&accel_y_fb);
//butterLowOrHigh(&accel_y_fb);

//accel_z_fb=filterIIRButterLowpass(5,0,60,0,1,80,sampling_frequent,FILTER_IIR_BUTTER_LOW);
//butterSbValue(&accel_z_fb);
//butterLowOrHigh(&accel_z_fb);














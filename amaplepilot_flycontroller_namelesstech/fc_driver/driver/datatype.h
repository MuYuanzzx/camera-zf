/* Copyright (c)  2019-2040 Wuhan Nameless Innovation Technology Co.,Ltd. All rights reserved.*/
/*----------------------------------------------------------------------------------------------------------------------/																					重要的事情说三遍
                先驱者的历史已经证明，在当前国内略浮躁+躺平+内卷的大环境下，对于毫无收益的开源项目，单靠坊间飞控爱好者、
                个人情怀式、自发地主动输出去参与开源项目的方式行不通，好的开源项目需要请专职人员做好售后技术服务、配套
                手册和视频教程要覆盖新手入门到进阶阶段，使用过程中对用户反馈问题和需求进行统计、在实践中完成对产品的一
                次次完善与迭代升级。
-----------------------------------------------------------------------------------------------------------------------
*                                                 为什么选择无名创新？
*                                         感动人心价格厚道，最靠谱的开源飞控；
*                                         国内业界良心之作，最精致的售后服务；
*                                         追求极致用户体验，高效进阶学习之路；
*                                         萌新不再孤单求索，合理把握开源尺度；
*                                         响应国家扶贫号召，促进教育体制公平；
*                                         新时代奋斗最出彩，建人类命运共同体。 
-----------------------------------------------------------------------------------------------------------------------
*               生命不息、奋斗不止；前人栽树，后人乘凉！！！
*               开源不易，且学且珍惜，祝早日逆袭、进阶成功！！！
*               学习优秀者，简历可推荐到DJI、ZEROTECH、XAG、AEE、GDU、AUTEL、EWATT、HIGH GREAT等公司就业
*               求职简历请发送：15671678205@163.com，需备注求职意向单位、岗位、待遇等
*               飞跃雷区组第21届智能汽车竞赛交流群：579581554
*               无名创新开源飞控QQ群：2号群465082224、1号群540707961
*               CSDN博客：http://blog.csdn.net/u011992534
*               B站教学视频：https://space.bilibili.com/67803559/#/video				
*               无名创新国内首款TI开源飞控设计初衷、知乎专栏:https://zhuanlan.zhihu.com/p/54471146
*               淘宝店铺：https://shop348646912.taobao.com/
*               公司官网:www.nameless.tech
*               修改日期:2025/10/01                  
*               版本：枫叶飞控MaplePilot_V1.0
*               版权所有，盗版必究。
*               Copyright(C) 2019-2040 武汉无名创新科技有限公司 
*               All rights reserved
----------------------------------------------------------------------------------------------------------------------*/
/******************************觉得代码很有帮助，欢迎打赏一碗热干面（武科大二号门老汉口红油热干面4元一碗），无名小哥支付宝：1094744141@qq.com********/


#ifndef __DATATYPE_H
#define __DATATYPE_H


#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <wp_math.h>



#ifndef NUL
#define NUL 0
#endif


#define  min_ctrl_dt  0.005f
#define  duty_dt_ms   5
#define  sampling_frequent  200

#define PERIOD_UNIT_MIN duty_dt_ms



typedef struct
{
  float cf;
  float a[3];
  float b[3];
}filter_parameter;

typedef struct
{
  float input[3];
  float output[3];
}filter_buffer;


typedef struct
{
  float a[4];
  float b[4];
}filter4_parameter;

typedef struct
{
  float input[4];
  float output[4];
}filter4_buffer;

enum 
{
  _ROL=0,
  _PIT,
  _YAW
};

enum 
{
  _EAST=0,
  _NORTH,
  _UP,
};

enum INAV
{
  EAST=0,
  NORTH,
  UP
};

enum BODY
{
  _Z=0,
  _FORWARD,
  _RIGHT
};



enum RPYT
{
  ROLL=0,
  PITCH,
  YAW,
  THR
};


typedef struct
{
  int16_t x;
  int16_t y;
  int16_t z;	
}vector3i;


typedef struct
{
  int32_t x;
  int32_t y;
}vector2int32;



typedef struct
{
  float pos;
  float vel;
  float acc;
}vector3s;


typedef struct{
  int32_t lng;
  int32_t lat;
  uint8_t update_flag;
}guide_gps;


typedef struct
{
  volatile float last_time;
  volatile float current_time;
  volatile float period;
  volatile uint16_t period_int;//单位ms
}systime;


typedef struct
{
  float value;
  uint8_t enable;
  float upper;
  float lower;
  uint16_t low_vbat_cnt;
}low_voltage;




typedef struct
{
  //传感器采样原始数据
  float accel_raw_data[3];
  float gyro_raw_data[3];
  float mag_raw_data_tesla[3];
  
  vector3f accel_raw_correct;//raw
  //低通滤波后数据
  vector3f accel_raw_filter_data,gyro_raw_filter_data;//raw
  vector3f accel_fb_filter_data;//raw
  vector3f gyro_correct_raw_data;//raw
  
  //低通滤波+校准后数据
  vector3f gyro_dps;//dps
  vector3f accel_g;//g
  vector3f mag;
  float t,_t;
  vector3f gyro_backups[20];//dps
  vector3f accel_g_fb;//g
  vector3f gyro_rps;
  //
  float rpy_fusion_deg[3];//融合后的姿态角度
  float rpy_transition_deg[3];//中间过渡姿态角度
  float rpy_obs_deg[3];				//观测量姿态角度
  float rpy_contrast_deg[3]; 
  float cos_rpy[3];
  float sin_rpy[3];
  float quaternion[4];
  float cb2n[9];//载体系b到导航系n的旋转矩阵
  float quaternion_obs[4];
  float yaw_obs_deg;				//偏航观测量姿态角度
  float total_gyro_dps;
  float rpy_integral_deg[3];
  float mag_intensity;
  //
  float yaw_gyro_enu;
  float yaw_angle_enu;
  
  vector3f accel_scale;
  vector3f accel_offset;
  vector3f gyro_offset;
  vector3f mag_offset;
  vector3f accel_hor_offset;
  float rpy_angle_offset[3];
  float quaternion_zero[4];
  float quaternion_init[4];//初始四元数
  uint8_t quaternion_init_ok;
  uint8_t temperature_stable_flag;
  uint8_t imu_gyro_calibration_flag;
  uint8_t accel_calibration_way;
  //
  _IMU_ACCEL_SCALE imu_acc_scale;
  _IMU_GYRO_SCALE imu_gyro_scale;
  float gyro_scale_raw_to_dps,accel_scale_raw_to_g;	
  
  float gyro_total,accel_total;
  //
  vector3f accel_earth_g;
  uint8_t imu_data_offline_flag;//数据不在线标志位
  /*****************/
  float baro_p;
  float baro_t;
  float baro_height;
  float baro_pressure_offset;
  float baro_temp_offset;
  float baro_vel;
  float baro_acc;
  uint8_t baro_cal_finished_flag;
  uint16_t baro_period_ms;
  uint8_t baro_init;
  float baro_height_backups[20];
  //float baro_height_standard_deviation;
  //float baro_average;
  //
  uint8_t mag_update_flag;
  uint8_t baro_update_flag;
  uint8_t rangefinder_update_flag;
  //
  uint8_t gps_idle_flag;
  
  float vbat,mcu_t;
  uint8_t compass_health_flag;
  /***************************************************/
  int8_t player_level;
  int8_t icm_gyro_inner_lpf_config;
  int8_t icm_accel_inner_lpf_config;
}sensor;

typedef struct{
  int16_t x;
  int16_t y;
  uint16_t dt;
  uint16_t ground_distance;
  uint8_t qual;
  uint8_t version;
  //
  uint8_t valid;
  unsigned char update;
  unsigned char ctrl_valid;
  
  vector2f flow;
  vector2f flow_filter;
  vector2f gyro_filter;
  vector2f flow_correct;
  float ms;
  uint8_t is_okay;
}opticalflow;



#define INS_NUM  50

typedef struct
{
  vector2f position;
  vector2f speed;
  vector2f acceleration;
  vector2f pos_backups[INS_NUM];//历史惯导位置
  vector2f vel_backups[INS_NUM];//历史惯导速度
  vector2f acce_bias;//惯导加速度漂移量估计量
  vector2f speed_obs;
  vector2f position_obs;
  vector2f position_err;
  vector2f speed_err;
  //
  vector2f position_ctrl;
  vector2f speed_ctrl;
}opt_fusion;




typedef struct
{
  float position[3];//位置估计量
  float speed[3];//速度估计量
  float acceleration[3];//加速度估计量
  float acceleration_initial[3];//加速度原始量
  float pos_backups[3][INS_NUM];//历史惯导位置
  float vel_backups[3][INS_NUM];//历史惯导速度
  float acce_backups[3][INS_NUM];//历史惯导速度
  
  float obs_pos_backups[3][INS_NUM];//历史惯导位置
  float obs_vel_backups[3][INS_NUM];//历史惯导速度	
  float obs_pos[INS_NUM];
  float obs_vel[INS_NUM];
  
  float position_z,speed_z,acceleration_z;
  
  float acce_bias[3];//惯导加速度漂移量估计量
  float acce_bias_all[3];//惯导加速度漂移量估计量
  float last_acceleration[3];
  float last_speed[3];
  float accel_feedback[3];
  vector2f horizontal_acceleration;
  float alt_obs_cm;
  uint8_t alt_obs_update;
  uint16_t alt_obs_sync_ms;
  float alt_obs_update_period;
  uint16_t alt_obs_type;
  opt_fusion opt;
  //
  uint8_t gps_home_fixed_flag;
  int32_t gps_lat;
  int32_t gps_lng;
  int32_t gps_alt;
  int32_t gps_home_lat;
  int32_t gps_home_lng;
  int32_t gps_home_alt;
  float gps_alt_cm;
  float gps_home_alt_cm;
  float magnetic_declination;
  uint8_t gps_update_flag;
  uint8_t gps_east_update_flag;
  uint8_t gps_north_update_flag;
  uint8_t gps_state_update_flag;
  float gps_obs_pos_enu[3];
  float gps_obs_vel_enu[3];
  float gps_obs_pos_body[3];
  float gps_heading;
  float gps_hacc;
  float gps_vacc;
  float gps_sacc;
  float gps_pdop;
  float gps_lat_deg_f,gps_lng_deg_f;
  float gps_lat_min_f,gps_lng_min_f;	
  //
  float gps_hacc_m;
  float gps_vacc_m;
  float gps_sacc_mps;
  float gps_alt_standard_deviation;	  //对地测距传感器标准差
  float gps_alt_history[INS_NUM];
  float return_distance_m;
}sins;

typedef struct
{
  float position[3];//位置估计量
  float speed[3];//速度估计量
  float accel_cmpss[3];//加速度估计量
  float pos_backups[3][INS_NUM];//历史惯导位置
  float vel_backups[3][INS_NUM];//历史惯导速度
  // float accel_backups[Axis_Num][Num];//历史惯导速度
  float acce_bias[3];//惯导加速度漂移量估计量
  // float acce_bias_All[Axis_Num];//惯导加速度漂移量估计量
  // float last_accel_cmpss[Axis_Num];
  // float last_speed[Axis_Num];
  // float ins_accel_cmpss[Axis_Num];
}sins_lite;



typedef struct 
{
  uint16_t data[8];
  uint8_t  update_flag;
  float accuracy;
}ppm;

typedef struct
{
  uint16_t year;
  uint8_t mon;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint32_t ms;
}log_time;

typedef struct
{
  uint16_t target_height;
  uint16_t safe_height;
  uint16_t safe_vbat;
  uint16_t max_height;
  uint16_t max_radius;
  uint16_t max_upvel;
  uint16_t max_downvel;
  uint16_t max_horvel;
  uint16_t reserved_uart;
  uint16_t near_ground_height;
  uint16_t inner_uart;
  uint16_t opticalcal_type;
}_params;

typedef union 
{
  _params params;
  uint16_t hw[12];
}_other_params;


typedef struct
{
  float value;
  uint16_t period_cnt;
  uint16_t right_cnt;
  uint16_t wrong_cnt;
}fault_rate;

enum ESC
{
  f_50hz=1,
  f_100hz=2,
  f_200hz=4,
  f_400hz=8
};


extern sensor flymaple;
extern sins ins;

#endif


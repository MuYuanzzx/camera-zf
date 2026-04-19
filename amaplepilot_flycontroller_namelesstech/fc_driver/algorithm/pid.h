#ifndef __PID_H
#define __PID_H 

#include <stdint.h>



typedef enum
{
  _roll_gyro_ctrl=0,
  _pitch_gyro_ctrl=1,
  _yaw_gyro_ctrl=2,
  _roll_angle_ctrl=3,
  _pitch_angle_ctrl=4,
  _yaw_angle_ctrl=5,
  /*************高度控制器****************/
  _height_position_ctrl=6,
  _height_speed_ctrl=7,
  _height_accel_ctrl=8,
  /*************GPS位置控制器****************/
  _longitude_position_ctrl=9,
  _longitude_speed_ctrl=10,
  _latitude_position_ctrl=11,
  _latitude_speed_ctrl=12,
  
  /*************SDK位置控制器****************/
  _sdk_position_ctrl_x=13,
  _sdk_position_ctrl_y=14,
  /*************光流控制器****************/
  _optical_position_ctrl_x=15,
  _optical_speed_ctrl_x=16,
  _optical_position_ctrl_y=17,
  _optical_speed_ctrl_y=18,
  
  _imu_temperature_ctrl=19
}ctrl_name;



typedef struct
{
  ctrl_name name;
  uint8_t err_limit_flag :1;//偏差限幅标志
  uint8_t integrate_limit_flag :1;//积分限幅标志
  uint8_t integrate_separation_flag :1;//积分分离标志
  float expect;//期望
  float feedback;//反馈值	
  float err;//偏差
  float last_err;//上次偏差
  float err_max;//偏差限幅值
  float integrate_separation_err;//积分分离偏差值
  float integrate;//积分值
  float integrate_max;//积分限幅值
  float kp;//控制参数Kp
  float ki;//控制参数Ki
  float kd;//控制参数Kd
  float control_output;//控制器总输出
  float last_control_output;//上次控制器总输出
  float control_output_limit;//输出限幅
  /***************************************/
  float last_expect;	//上次期望
  float expect_div;		//期望的微分
  float last_feedback;//上次反馈值 
  float feedback_div;	//反馈信号的微分
  float combine_div;	//组合微分
  float pre_last_err;	//上上次偏差
  float adaptable_kd;	//自适应微分参数
  float dis_err;			//微分量
  float derivative,last_derivative;//上次微分量
  float dis_error_history[5];//历史微分量
  float err_lpf;
  float last_err_lpf;
  float dis_err_lpf;
  float last_dis_err_lpf;
  float pre_last_dis_err_lpf;
  filter_buffer lpf_buf;		 //控制器低通输入输出缓冲
  filter_parameter lpf_param;//控制器低通滤波器参数
  float d_lpf_alpha;
  systime _time;
}pid_ctrl;



typedef struct
{
  pid_ctrl roll_gyro_ctrl;
  pid_ctrl pitch_angle_ctrl;
  pid_ctrl yaw_gyro_ctrl;
  
  pid_ctrl roll_angle_ctrl; 
  pid_ctrl pitch_gyro_ctrl; 
  pid_ctrl yaw_angle_ctrl;
  /*************高度控制器****************/  
  pid_ctrl height_position_ctrl;
  pid_ctrl height_speed_ctrl;
  pid_ctrl height_accel_ctrl;	
  /*************GPS位置控制器****************/ 
  pid_ctrl longitude_position_ctrl;
  pid_ctrl longitude_speed_ctrl;
  pid_ctrl latitude_position_ctrl;
  pid_ctrl latitude_speed_ctrl;
  
  /*************SDK位置控制器****************/
  pid_ctrl sdk_position_ctrl_x;
  pid_ctrl sdk_position_ctrl_y;
  /*************光流控制器****************/
  pid_ctrl optical_position_ctrl_x;
  pid_ctrl optical_speed_ctrl_x;
  pid_ctrl optical_position_ctrl_y;
  pid_ctrl optical_speed_ctrl_y;
  pid_ctrl imu_temperature_ctrl;
}maple_controller;


typedef enum 
{
  direct_diff=0,		//直接微分
  interval_diff=1,	//区间微分
  incomplete_diff=2,//微分先行
}diff_mode;

typedef enum 
{
  noneed_lpf=0,		    //微分无需低通
  first_order_lpf=1,	//微分一阶低通
  second_order_lpf=2, //微分二阶低通
}lpf_mode;




void pid_params_init(pid_ctrl *ctrl,ctrl_name n);
void pid_init_default(void);
float pid_ctrl_general(pid_ctrl *ctrl,float period_second);
float pid_ctrl_rpy_gyro(pid_ctrl *ctrl,float period_second,diff_mode _diff_mode,lpf_mode _lpf_mode);
float pid_ctrl_yaw(pid_ctrl *ctrl,float period_second);
float pid_ctrl_err_lpf(pid_ctrl *ctrl,float period_second);
float pid_ctrl_div_lpf(pid_ctrl *ctrl,float period_second);
float pid_ctrl_div_gyro_lpf(pid_ctrl *ctrl,float period_second);


void  pid_integrate_reset(pid_ctrl *ctrl);
void takeoff_ctrl_reset(void);
void optical_ctrl_reset(uint8_t fixed);


extern maple_controller maple_ctrl;


#endif


//	{1  ,1 ,0 ,0 ,0 ,0 , 0 ,600 ,0  ,0 , 400,  0.80   ,3.00  ,4.00  ,0  ,0 , 1000 },//Roll_Gyro;偏航角速度
//	{1  ,1 ,0 ,0 ,0 ,0 , 0 ,600 ,0  ,0 , 400,  0.80   ,3.00  ,4.00  ,0  ,0 , 1000 },//Pitch_Gyro;横滚角速度	
//	{1  ,1 ,0 ,0 ,0 ,0 , 0 ,300 ,0  ,0 , 300,  1.50   ,1.00  ,1.00  ,0  ,0 , 800  },//Yaw_Gyro;偏航角速度	
//		
//  {1  ,1 ,0 ,0 ,0 ,0 , 0 ,30  ,0  ,0 , 100,  4.00   ,0.00  ,0.00  ,0  ,0 , 500  },//Roll_Angle;偏航角度
//  {1  ,1 ,0 ,0 ,0 ,0 , 0 ,30  ,0  ,0 , 100,  4.00   ,0.00  ,0.00  ,0  ,0 , 500  },//Pitch_Angle;横滚角
//  {1  ,1 ,0 ,0 ,0 ,0 , 0 ,45  ,0  ,0 , 200 , 4.00   ,0.00  ,0.00  ,0  ,0 , 500  },//Yaw_Angle;偏航角



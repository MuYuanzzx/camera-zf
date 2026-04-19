#ifndef __CONTROL_OUTPUT_MIX_H
#define __CONTROL_OUTPUT_MIX_H


#define  THR_MAX_OUTPUT      2000//油门输出最大值
#define  THR_MIN_OUTPUT      1000//油门输出最小值
#define  THR_IDEL_OUTPUT     1100//油门怠速，取接近起转油门值即可1150
#define  THR_CONTROL_WORK    1100//俯仰、横滚、偏航控制器开始起作用的油门量 1150
#define  THR_STARTUP_MIN     1050//起转油门量，油门倾角补偿用，太大会导致过补偿


void Motor_Control_Rate_Pure(float _thr,float _rol,float _pit,float _yaw,float *motor_output);


#endif





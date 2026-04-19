#ifndef __MAPLE_AHRS_H
#define __MAPLE_AHRS_H


#include "datatype.h"


void ahrs_ekf_init(void);
void ahrs_attitude_ekf(vector3f *gyro, vector3f *accel,float *obs,float dt);
void ahrs_get_quaternion(float *q);
void ahrs_get_euler(float *rpy);



void madgwick_init(void);
void madgwick_updateimu(vector3f *gyro_r,vector3f *accel_g,vector3f *mag,float dt,uint16_t sync);
void madgwick_getangle(float* rpy);
void madgwick_get_quaternion(float *q);
void madgwick_set_quaternion(float *q);



void mahony_updateimu(vector3f *gyro_r,vector3f *accel_g,float dt);

extern float q_backups[20][4];
#endif




//	ahrs_attitude_ekf(&flymaple.gyro,&flymaple.accel,flymaple.quaternion_obs,t1.period/1000.0f);
//	ahrs_get_euler(flymaple.rpy_fusion_deg);
//	ahrs_get_quaternion(flymaple.quaternion);



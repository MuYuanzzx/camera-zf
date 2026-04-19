
#include "arm_math.h"


#define INS_EKF_STATE_DIM 6         //系统状态维数
#define INS_EKF_MEASUREMENT_DIM 4   //观测状态维数


typedef struct
{
	float P[INS_EKF_STATE_DIM * INS_EKF_STATE_DIM];
	float Q[INS_EKF_STATE_DIM * INS_EKF_STATE_DIM];
	float R[INS_EKF_MEASUREMENT_DIM * INS_EKF_MEASUREMENT_DIM];
  float F[INS_EKF_STATE_DIM * INS_EKF_STATE_DIM];
  float H[INS_EKF_MEASUREMENT_DIM * INS_EKF_STATE_DIM];
	//state
	float HT[INS_EKF_STATE_DIM*INS_EKF_MEASUREMENT_DIM];
	float X[INS_EKF_STATE_DIM];
	float KY[INS_EKF_STATE_DIM];
	//measurement
	float Y[INS_EKF_MEASUREMENT_DIM];
	float PX[INS_EKF_STATE_DIM * INS_EKF_STATE_DIM];
	float PXX[INS_EKF_STATE_DIM * INS_EKF_STATE_DIM];
	float PXY[INS_EKF_STATE_DIM * INS_EKF_MEASUREMENT_DIM];
	float K[INS_EKF_STATE_DIM * INS_EKF_MEASUREMENT_DIM];
	float S[INS_EKF_MEASUREMENT_DIM * INS_EKF_MEASUREMENT_DIM];
	float SI[INS_EKF_MEASUREMENT_DIM * INS_EKF_MEASUREMENT_DIM];

	arm_matrix_instance_f32 I_Matrix;
	arm_matrix_instance_f32 P_Matrix;
	arm_matrix_instance_f32 Q_Matrix;
	arm_matrix_instance_f32 R_Matrix;
	arm_matrix_instance_f32 F_Matrix;
	arm_matrix_instance_f32 H_Matrix;
	arm_matrix_instance_f32 X_Matrix;
	arm_matrix_instance_f32 KY_Matrix;
	arm_matrix_instance_f32 Y_Matrix;
	arm_matrix_instance_f32 PX_Matrix;
	arm_matrix_instance_f32 PXX_Matrix;
	arm_matrix_instance_f32 PXY_Matrix;
	arm_matrix_instance_f32 K_Matrix;
	arm_matrix_instance_f32 S_Matrix;
	arm_matrix_instance_f32 SI_Matrix;
	 
	arm_matrix_instance_f32 HT_Matrix;
	
	bool init;
}EKF_FILTER;


void gps_ekf_init(void);
void gps_ekf_update(sins *g_ins,float dt,uint8_t *obs_update_flag);


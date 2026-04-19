#ifndef __PARAMETER_SERVER_H
#define __PARAMETER_SERVER_H

#include "drv_w25qxx.h"

typedef enum
{
  PITCH_OFFSET1=0,
  ROLL_OFFSET1=1,
  ACCEL_X_OFFSET1=2,
  ACCEL_Y_OFFSET1=3,
  ACCEL_Z_OFFSET1=4,
  ACCEL_X_SCALE1=5,
  ACCEL_Y_SCALE1=6,
  ACCEL_Z_SCALE1=7,
  MAG_X_OFFSET=8,
  MAG_Y_OFFSET=9,
  MAG_Z_OFFSET=10,
  RC_CH1_MAX=11,
  RC_CH1_MIN=12,
  RC_CH2_MAX=13,
  RC_CH2_MIN=14,
  RC_CH3_MAX=15,
  RC_CH3_MIN=16,
  RC_CH4_MAX=17,
  RC_CH4_MIN=18,
  RC_CH5_MAX=19,
  RC_CH5_MIN=20,
  RC_CH6_MAX=21,
  RC_CH6_MIN=22,
  RC_CH7_MAX=23,
  RC_CH7_MIN=24,
  RC_CH8_MAX=25,
  RC_CH8_MIN=26,
  PID1_PARAMETER_KP=27,
  PID1_PARAMETER_KI=28,
  PID1_PARAMETER_KD=29,
  PID2_PARAMETER_KP=30,
  PID2_PARAMETER_KI=31,
  PID2_PARAMETER_KD=32,
  PID3_PARAMETER_KP=33,
  PID3_PARAMETER_KI=34,
  PID3_PARAMETER_KD=35,
  PID4_PARAMETER_KP=36,
  PID4_PARAMETER_KI=37,
  PID4_PARAMETER_KD=38,
  PID5_PARAMETER_KP=39,
  PID5_PARAMETER_KI=40,
  PID5_PARAMETER_KD=41,
  PID6_PARAMETER_KP=42,
  PID6_PARAMETER_KI=43,
  PID6_PARAMETER_KD=44,
  PID7_PARAMETER_KP=45,
  PID7_PARAMETER_KI=46,
  PID7_PARAMETER_KD=47,
  PID8_PARAMETER_KP=48,
  PID8_PARAMETER_KI=49,
  PID8_PARAMETER_KD=50,
  PID9_PARAMETER_KP=51,
  PID9_PARAMETER_KI=52,
  PID9_PARAMETER_KD=53,
  PID10_PARAMETER_KP=54,
  PID10_PARAMETER_KI=55,
  PID10_PARAMETER_KD=56,
  PID11_PARAMETER_KP=57,
  PID11_PARAMETER_KI=58,
  PID11_PARAMETER_KD=59,
  PID12_PARAMETER_KP=60,
  PID12_PARAMETER_KI=61,
  PID12_PARAMETER_KD=62,
  PID13_PARAMETER_KP=63,
  PID13_PARAMETER_KI=64,
  PID13_PARAMETER_KD=65,
  PID14_PARAMETER_KP=66,
  PID14_PARAMETER_KI=67,
  PID14_PARAMETER_KD=68,
  PID15_PARAMETER_KP=69,
  PID15_PARAMETER_KI=70,
  PID15_PARAMETER_KD=71,
  PID16_PARAMETER_KP=72,
  PID16_PARAMETER_KI=73,
  PID16_PARAMETER_KD=74,   
  PID17_PARAMETER_KP=75,
  PID17_PARAMETER_KI=76,
  PID17_PARAMETER_KD=77,
  PID18_PARAMETER_KP=78,
  PID18_PARAMETER_KI=79,
  PID18_PARAMETER_KD=80,   
  PID19_PARAMETER_KP=81,
  PID19_PARAMETER_KI=82,
  PID19_PARAMETER_KD=83,
  ESC_CALIBRATION_FLAG=84,
  LOG_NUM=85,
  HOR_CAL_ACCEL_X1=86,
  HOR_CAL_ACCEL_Y1=87,
  HOR_CAL_ACCEL_Z1=88,
  SDK1_MODE_DEFAULT=89,
  GROUND_DISTANCE_DEFAULT=90,
  GYRO_X_OFFSET1=91,
  GYRO_Y_OFFSET1=92,
  GYRO_Z_OFFSET1=93,
  PITCH_ROLL_FEEDFORWARD_KP=94,
  PITCH_ROLL_FEEDFORWARD_KD=95,
  
  TARGET_HEIGHT=96,
  SAFE_HEIGHT=97,
  SAFE_VBAT=98,
  MAX_HEIGHT=99,
  MAX_RADIUS=100,
  MAX_UPVEL=101,
  MAX_DOWNVEL=102,
  MAX_HORVEL=103,
  RESERVED_UART_FUNCTION=104,
  NEAR_GROUND_HEIGHT=105,
  UART5_FUNCTION=106,
  OPTICALFLOW_TYPE=107,
  
  OPTICAL_TYPE=108,
  SDK2_MODE_DEFAULT=109,
  ACCEL_SIMPLE_MODE=110,
  ADC_KEY1_ENABLE=111,
  ADC_KEY2_ENABLE=112,
  WIRELESS_ADC_KEY_ENABLE=113,
  RES_SERVO_PINCH_PWM_US=114,
  RES_SERVO_RELEASE_PWM_US=115,
  TASK_SELECT_AFTER_TAKEOFF=116,	 
  IMU_SENSE_TYPE=117,
  PITCH_OFFSET2=118,
  ROLL_OFFSET2=119,
  ACCEL_X_OFFSET2=120,
  ACCEL_Y_OFFSET2=121,
  ACCEL_Z_OFFSET2=122,
  ACCEL_X_SCALE2=123,
  ACCEL_Y_SCALE2=124,
  ACCEL_Z_SCALE2=125,
  GYRO_X_OFFSET2=126,
  GYRO_Y_OFFSET2=127,
  GYRO_Z_OFFSET2=128,	 
  HOR_CAL_ACCEL_X2=129,
  HOR_CAL_ACCEL_Y2=130,
  HOR_CAL_ACCEL_Z2=131,
  PITCH_OFFSET3=132,
  ROLL_OFFSET3=133,
  ACCEL_X_OFFSET3=134,
  ACCEL_Y_OFFSET3=135,
  ACCEL_Z_OFFSET3=136,
  ACCEL_X_SCALE3=137,
  ACCEL_Y_SCALE3=138,
  ACCEL_Z_SCALE3=139,
  GYRO_X_OFFSET3=140,
  GYRO_Y_OFFSET3=141,
  GYRO_Z_OFFSET3=142,	 
  HOR_CAL_ACCEL_X3=143,
  HOR_CAL_ACCEL_Y3=144,
  HOR_CAL_ACCEL_Z3=145,
  GYRO_LPF_CF=146,
  ACCEL_LPF_CF=147,
  INS_LPF_CF=148,
  FB_LPF_CF=149,
  DRONE_PLAYER_LEVEL=150,	 
  ESC_OUTPUT_FREQUENCY=151,
  
 IMU_GYRO_INNER_LPF=159,	
 IMU_ACCEL_INNER_LPF=160,	
 
 
  RESERVED_PARAM=170,
}FLIGHT_PARAMETER_TABLE;







typedef struct
{
  float p;
  float i;
  float d;	
}vector3pid;



void pid_parameter_server(void);
void pid_init_from_parameter_server(void);

void rc_init_from_parameter_server(void);
void imu_calibration_init_from_server(void);
void other_parameter_init_from_server(uint8_t mode);
void reserved_params_init_server(void);


extern _other_params other_params;

#endif



//			WriteFlashParameter(TARGET_HEIGHT,other_params.params.target_height,&Flight_Params);
//			WriteFlashParameter(SAFE_HEIGHT,other_params.params.safe_height,&Flight_Params);
//			WriteFlashParameter(SAFE_VBAT,other_params.params.safe_vbat,&Flight_Params);
//			WriteFlashParameter(MAX_HEIGHT,other_params.params.max_height,&Flight_Params);
//			WriteFlashParameter(MAX_RADIUS,other_params.params.max_radius,&Flight_Params);
//			WriteFlashParameter(MAX_UPVEL,other_params.params.max_upvel,&Flight_Params);
//			WriteFlashParameter(MAX_DOWNVEL,other_params.params.max_downvel,&Flight_Params);
//			WriteFlashParameter(MAX_HORVEL,other_params.params.max_horvel,&Flight_Params);
//			WriteFlashParameter(RESERVED_UART_FUNCTION,other_params.params.reserved_uart,&Flight_Params);
//			WriteFlashParameter(NEAR_GROUND_HEIGHT,other_params.params.near_ground_height,&Flight_Params);
//			WriteFlashParameter(UART5_FUNCTION,other_params.params.inner_uart,&Flight_Params);
//			WriteFlashParameter(AVOID_OBSTACLE,other_params.params.obstacle,&Flight_Params);	
//			other_params.params.target_height=Flight_Params.parameter_table[TARGET_HEIGHT];
//			other_params.params.safe_height=Flight_Params.parameter_table[SAFE_HEIGHT];
//			other_params.params.safe_vbat=Flight_Params.parameter_table[SAFE_VBAT];
//			other_params.params.max_height=Flight_Params.parameter_table[MAX_HEIGHT];
//			other_params.params.max_radius=Flight_Params.parameter_table[MAX_RADIUS];
//			other_params.params.max_upvel=Flight_Params.parameter_table[MAX_UPVEL];
//			other_params.params.max_downvel=Flight_Params.parameter_table[MAX_DOWNVEL];
//			other_params.params.max_horvel=Flight_Params.parameter_table[MAX_HORVEL];
//			other_params.params.reserved_uart=Flight_Params.parameter_table[RESERVED_UART_FUNCTION];
//			other_params.params.near_ground_height=Flight_Params.parameter_table[NEAR_GROUND_HEIGHT];
//			other_params.params.inner_uart=Flight_Params.parameter_table[UART5_FUNCTION];
//			other_params.params.obstacle=Flight_Params.parameter_table[AVOID_OBSTACLE];
//		uint16_t Label=PID1_PARAMETER_KP+i*3;
//		Flight_Params.parameter_table[Label]  =_pid_parameter[i].p;//将需要更改的字段赋新值
//		Flight_Params.parameter_table[Label+1]=_pid_parameter[i].i;//将需要更改的字段赋新值
//		Flight_Params.parameter_table[Label+2]=_pid_parameter[i].d;//将需要更改的字段赋新值	
//	uint32_t WriteAddress = (uint32_t)(PARAMETER_TABLE_STARTADDR_EEPROM+PID1_PARAMETER_KP*4);
//	W25QXX_Write_N_Data(WriteAddress,&Flight_Params.parameter_table[PID1_PARAMETER_KP],3*PID_USE_NUM);






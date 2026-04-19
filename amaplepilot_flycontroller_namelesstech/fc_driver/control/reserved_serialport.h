#ifndef _RESERVED_SERIALPORT_H_
#define _RESERVED_SERIALPORT_H_





void Reserved_Serialport_Init(void);
void Uart2_Serialport_Init(void);


void NCLink_Data_Prase_Prepare_Lite(uint8_t data);
void NCLink_Send_IMU_Feedback_PC(void);
void reserved_serialport_init(void);

void quad_getangle(float *q,float* rpy);
void NCLink_Data_Prase_Process_Lite(uint8_t *data_buf,uint8_t num);

void reserved_serialport_prase(void);


extern uint8_t send_check_back;



#define Laser_Min_Info_Num  20
extern float min_dis_cm,min_dis_angle,target_yaw_err;
extern float min_dis_cm_backups[Laser_Min_Info_Num],min_dis_angle_backups[Laser_Min_Info_Num];


#endif


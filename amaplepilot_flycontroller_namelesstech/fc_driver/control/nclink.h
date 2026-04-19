#ifndef __NCLINK_H
#define __NCLINK_H

#include <stdint.h>
#include <stdbool.h>


#define BYTE0(dwTemp)  (*((char *)(&dwTemp)))
#define BYTE1(dwTemp)  (*((char *)(&dwTemp)+1))
#define BYTE2(dwTemp)  (*((char *)(&dwTemp)+2))
#define BYTE3(dwTemp)  (*((char *)(&dwTemp)+3))
	

#define NCLINK_STATUS         0x01//发送飞控状态数据，功能字节
#define NCLINK_SENSOR         0x02//发送传感器原始数据，功能字节
#define NCLINK_RCDATA         0x03//发送遥控器解析数据，功能字节
#define NCLINK_GPS 		        0x04//发送GPS经纬度、海拔数据，功能字节
#define NCLINK_OBS_NE         0x05//发送NE方向观测数据，功能字节
#define NCLINK_OBS_UOP        0x06//发送U方向观测数据，功能字节
#define NCLINK_FUS_U          0x07//发送U方向融合数据，功能字节
#define NCLINK_FUS_NE         0x08//发送NE方向融合数据，功能字节
#define NCLINK_USER           0x09//发送用户数据，功能字节

#define NCLINK_SEND_PID1_3    0x0A//PID01-03数据，功能字节+应答字节
#define NCLINK_SEND_PID4_6    0x0B//PID04-06数据，功能字节+应答字节
#define NCLINK_SEND_PID7_9    0x0C//PID07-09数据，功能字节+应答字节
#define NCLINK_SEND_PID10_12  0x0D//PID10-12数据，功能字节+应答字节
#define NCLINK_SEND_PID13_15  0x0E//PID13-15数据，功能字节+应答字节
#define NCLINK_SEND_PID16_18  0x0F//PID16-18数据，功能字节+应答字节

#define NCLINK_SEND_PARA      0x10//飞控发送其它参数，功能字节+应答字节
#define NCLINK_SEND_CAL_RAW1  0x11//发送校准原始数据1，功能字节
#define NCLINK_SEND_CAL_RAW2  0x12//发送校准原始数据2，功能字节
#define NCLINK_SEND_CAL_PARA1 0x13//发送校准结果数据1，功能字节
#define NCLINK_SEND_CAL_PARA2 0x14//发送校准结果数据2，功能字节
#define NCLINK_SEND_CAL_PARA3 0x15//发送校准结果数据3，功能字节

#define NCLINK_SEND_PARA_RESERVED		0x16//飞控发送预留参数，功能字节+应答字节
#define NCLINK_SEND_3D_TRACK				0x17//飞控3D位姿数据发送参数



#define NCLINK_SEND_CHECK     					0xF0//发送应答数据，功能字节
//应答数据
#define NCLINK_SEND_RC        					0xF1//飞控解析遥控器控制指令，应答字节
#define NCLINK_SEND_DIS       					0xF2//飞控解析位移控制指令，应答字节
#define NCLINK_SEND_CAL       					0xF3//飞控传感器校准完毕，应答字节
#define NCLINK_SEND_CAL_READ  					0xF4//读取校准参数成功，应答字节
#define NCLINK_SEND_FACTORY   					0xF5//恢复出厂设置成功，应答字节
#define NCLINK_SEND_NAV_CTRL  					0xF6//飞控解析导航控制指令，应答字节
#define NCLINK_SEND_NAV_CTRL_FINISH     0xF7//飞控导航控制完毕，应答字节
#define NCLINK_SEND_SLAM_SYSTEM_RESET   0xF8//SLAM导航复位完毕，控制字节
#define NCLINK_SEND_SLAM_STOP_MOTOR     0xF9//SLAM控制电机停止，控制字节
#define NCLINK_SEND_SLAM_START_MOTOR    0xFA//SLAM控制电机开始，控制字节

#define NCLINK_SEND_AHRS	0x18//发送AHRS数据


#define RESERVED_PARAM_NUM 200


typedef enum
{
	SDK_FRONT = 0x00,
	SDK_BEHIND,
	SDK_LEFT,
	SDK_RIGHT,
	SDK_UP,
	SDK_DOWN
}SDK_MODE;


typedef struct
{
  bool sdk_front_flag;
	bool sdk_behind_flag;
	bool sdk_left_flag;
	bool sdk_right_flag;
	bool sdk_up_flag;
	bool sdk_down_flag;
}sdk_mode_flag;

typedef struct
{
  uint8_t move_mode;//SDK移动标志位
  uint8_t mode_order;//SDK移动顺序
  uint16_t move_distance;//SDK移动距离
	bool update_flag;
	sdk_mode_flag move_flag;
	float f_distance;
}ngs_sdk_control;


typedef enum 
{
	NO_SLAM=0,
	RPLIDAR_SLAM,	//位置导航模式
	T265_SLAM,   //GPS定点模式
	LOAM,	     
}slam_sensor_mode;

typedef struct
{
	float position_x;
	float position_y;
	float position_z;
	float velocity_x;
	float velocity_y;
	float velocity_z;
	float q[4];
	float quality;
	uint8_t update_flag;
	uint8_t byte[8];
	float rpy[3];
	
	uint8_t rec_update_flag;
	uint8_t rec_head_update_flag;
	uint8_t valid;
	uint8_t fault,last_fault;
	slam_sensor_mode slam_sensor;
	uint8_t loam_update_flag;
}third_party_state;


typedef enum 
{
  BODY_FRAME=0,//机体坐标系
	MAP_FRAME,	 //导航坐标系
}Navigation_Frame;


typedef enum 
{
  RELATIVE_MODE=0,//相对模式
	GLOBAL_MODE,	  //全局模式
	CMD_VEL_MODE,   //速度模式
	TRANSITION_MODE //过渡模式
}Navigation_Mode;


typedef struct{
	uint16_t number;								//任务编号
	float x;												//x方向位置/速度期望
	float y;												//y方向位置/速度期望
	float z;												//z方向位置/偏航角速度期望
	uint8_t nav_mode;								//导航模式
	uint8_t frame_id;								//坐标系类型
	
	uint8_t update_flag;						//指令更新标志位
	uint8_t ctrl_finish_flag;				//控制完毕标志位
	
	uint16_t cnt; 									//位置控制完毕判断计数器
	float dis_cm;										//总位置偏差
	const float dis_limit_cm;       //位置阈值
	const float cmd_vel_max;        //最大线速度限制
	const float cmd_angular_max;    //最大角速度限制
	
	uint8_t cmd_vel_update;					//速度控制指令更新标志位
	float cmd_vel_x;								//期望x方向速度
	float cmd_vel_y;                //期望y方向速度
	float cmd_vel_angular_z;				//期望偏航方向角速度
	uint32_t cmd_vel_during_cnt;    //速度控制计数器
	uint8_t cmd_vel_suspend_flag;   //控制中止标志位
}nav_ctrl;


typedef struct{
	int32_t lng;
	int32_t lat;
	uint8_t update_flag;
}nclink_guide_gps;


typedef struct
{
	uint8_t cal_flag;
	uint8_t cal_step;
	uint8_t cal_cmd;
	uint8_t shutdown_now_cal_flag;//传感器校准标志位
	bool update_flag;
}nclink_cal_state;


void NCLink_Data_Prase_Process(uint8_t *data_buf,uint8_t num);
void NCLink_Data_Prase_Prepare(uint8_t data);
void NCLink_SEND_StateMachine(void);
void Float2Byte(float *FloatValue, unsigned char *Byte, unsigned char Subscript);
void Byte2Float(unsigned char *Byte, unsigned char Subscript, float *FloatValue);
void Pilot_Status_Tick(void);
											
											

extern ngs_sdk_control ngs_sdk;
extern third_party_state current_state;
extern nav_ctrl ngs_nav_ctrl;
extern nclink_cal_state cal_state;
extern nclink_guide_gps guide_ctrl;

extern uint8_t nclink_send_ask_flag[10];//飞控接收获取参数命令请求，给地面站发送标志位
extern uint8_t nclink_send_check_flag[20];//数据解析成功，飞控给地面站发送标志位
extern float param_value[RESERVED_PARAM_NUM];
#endif



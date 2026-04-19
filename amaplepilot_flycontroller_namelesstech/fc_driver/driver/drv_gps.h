#ifndef __DRV_GPS_H
#define __DRV_GPS_H


typedef struct
{
	uint8_t validdata :1;
	uint8_t validtime :1;
	uint8_t fullyresolved :1;
	uint8_t nc:5;
}validity_flag;



typedef struct
{
	uint8_t gnssfixok :1;
	uint8_t diffsoln :1;
	uint8_t psmstate :3;
	uint8_t headvehvalid :1;
	uint8_t nc:2;
}gps_flags;




typedef struct
{
	uint32_t itow;//4
	uint16_t year;//2
	uint8_t month;//1
	uint8_t day;//1
  uint8_t hour;//1
  uint8_t min;//1
  uint8_t sec;//1
	validity_flag valid;//1
  uint32_t tacc;//4
	int32_t nano;//4
	uint8_t fixtype;//1
	gps_flags flags;//1
	uint8_t reserved1;//1
	uint8_t numsv;//1
	int32_t lon;     //经度 scale=1e-7deg
	int32_t lat;     //纬度 scale=1e-7deg
	int32_t height;  //椭球高度 mm 
	int32_t hmsl;    //海平面高度 mm 
	uint32_t hacc;   //水平位置精度估计 mm
	uint32_t vacc;   //竖直位置精度估计 mm
	int32_t veln;    //北向速度 mm/s
	int32_t vele;    //东向速度 mm/s
	int32_t veld;    //地向速度 mm/s
	int32_t gspeed;  //航向水平速度 mm/s 
	int32_t headmot; //运动航向角scale=1e-5deg
	uint32_t sacc;   //速度估计精度mm/s
	uint32_t headacc;//运动航向角精度scale=1e-5deg
	uint16_t pdop;   //定位质量 scale=0.01
	uint8_t reserved2[6];
	int32_t headveh; //航向角scale=1e-5deg  
	uint8_t reserved3[4];
}pvt_data;

typedef struct
{
	pvt_data gps;
	uint8_t update;
	uint16_t gps_rec_time_ms;
}pvt_gps;

void Ublox_Set_Output_PVT_10hz_Baud_Set(unsigned long Baud);
void gps_uart_init(void);
void gps_data_prase(uint8_t *update_flag);


extern pvt_gps gps_data;
#endif



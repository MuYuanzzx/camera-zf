#ifndef __FLYMAPLE_SDK_H
#define __FLYMAPLE_SDK_H


#include <math.h> // M_PI, sqrtf, atan2f, asinf
#include <stdbool.h>
#include <stdint.h>

#define SDK_DATA_LEN  53*2//45

#define SDK_TARGET_X_OFFSET  0
#define SDK_TARGET_Y_OFFSET  0//-12


/*************************************************************************/
#define  Pixel_Size_MV    0.0024f//6um=0.000006m=0.0006cm
//320---0.0012
//160---0.0024
//80 ---0.0048
#define  Focal_Length_MV  0.42f

#define OV7725_Sensor_Width_MM    		3.984f//3984um
#define OV7725_Sensor_Height_MM   		2.952f//2952um
#define Pixel_Image_Width_MV    		160//320
#define Pixel_Image_Height_MV   		120//240
#define Pixel_Image_Focal_MM_MV 		3.6f
#define Pixel_Image_View_Angle_X_MV  (65.9/2)//(56.72/2)//deg(50.75/2)
#define Pixel_Image_View_Angle_Y_MV  (51.8/2)//(44.07/2)//deg(38.72/2)
/*************************************************************************/
#define  Pixel_Size_CV    0.00056f//cm
//2592:1944——0.00014cm
//640:480——0.00056cm
#define  Focal_Length_CV  0.36f  //焦距3.6mm

#define OV5647_Sensor_Width_MM    		3.674f//3674um
#define OV5647_Sensor_Height_MM   		2.738f//2738.4um#
#define Pixel_Image_Width_CV    			640//640
#define Pixel_Image_Height_CV   			480//480
#define Pixel_Image_Focal_MM_CV 			3.6f
#define Pixel_Image_View_Angle_X_CV  (53.5/2)//约为66deg广角
#define Pixel_Image_View_Angle_Y_CV  (41.4/2)


#define AprilTag_Side_Length  13.6f//cm13.6






typedef struct
{
  uint16_t x;
  uint16_t y; 
  uint16_t pixel;  
  uint8_t flag;
  uint8_t state;		
  int16_t angle;
  uint16_t distance;
  uint16_t apriltag_id;
  uint16_t width;
  uint16_t height;
  uint8_t fps;
  uint8_t reserved1;
  uint8_t reserved2;
  uint8_t reserved3;
  uint8_t reserved4;
  uint16_t range_sensor1;
  uint16_t range_sensor2;
  uint16_t range_sensor3;
  uint16_t range_sensor4;
  uint8_t camera_id;
  int32_t reserved1_int32;
  int32_t reserved2_int32;
  int32_t reserved3_int32;
  int32_t reserved4_int32;
  //
  uint8_t sdk_mode;
  float x_cm;
  float y_cm;
  float z_cm;	
  float x_pixel_size;
  float y_pixel_size;
  float z_pixel_size;
  float apriltag_distance;
  uint16_t trust_cnt;
  uint16_t trust_flag;
  uint8_t line_ctrl_enable;
  uint8_t target_ctrl_enable;
  vector3f sdk_target,sdk_target_offset;
  float sdk_angle;
}vision_target_check;//目标检测




typedef enum 
{
  UART2_SDK=0,
  UART5_SDK,
}COM_SDK;





void flymaple_sdk_init(void);
void sdk_data_receive_prepare_1(uint8_t data);
void sdk_send_check(unsigned char mode,COM_SDK com);
void sdk_data_receive_prepare_2(uint8_t data);


extern vision_target_check lookdown_vision;
extern int16_t sdk1_mode_setup;
#endif



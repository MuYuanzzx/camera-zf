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


#ifndef __WP_MATH_H
#define __WP_MATH_H

#include "stdint.h"
#include <string.h>
#include "math.h"

typedef struct{
  // by making alt 24 bit we can make p1 in a command 16 bit,
  // allowing an accurate angle in centi-degrees. This keeps the
  // storage cost per mission item at 15 bytes, and allows mission
  // altitudes of up to +/- 83km
  int32_t alt:24; ///< param 2 - Altitude in centimeters (meters * 100)
  int32_t lat;    ///< param 3 - Lattitude * 10**7
  int32_t lng;    ///< param 4 - Longitude * 10**7
}Location;

typedef struct
{
  float x;
  float y;
}vector2f;

typedef struct
{
  float x;
  float y;
  float z;	
}vector3f;


typedef enum
{
  WHO_AM_I_MPU6050  =0x68,
  WHO_AM_I_ICM20689 =0x98,
  WHO_AM_I_ICM20608D=0xAE,
  WHO_AM_I_ICM20608G=0xAF,
  WHO_AM_I_ICM20602=0x12,
  WHO_AM_I_BMI088_A=0x1E,
  WHO_AM_I_BMI088_G=0x0F,
  WHO_AM_I_BMI088=0x1F,
  WHO_AM_I_ICM42688=0x47,
  WHO_AM_I_NONE=0xFF,
}_IMU_ID_READ;

typedef enum
{
  SCALE_250  = 0x00,
  SCALE_500  = 0x08,
  SCALE_1000 = 0x10,
  SCALE_2000 = 0x18,
}_IMU_GYRO_SCALE;

typedef enum
{
  SCALE_2G  = 0x00,
  SCALE_4G  = 0x08,
  SCALE_8G  = 0x10,
  SCALE_16G = 0x18,
  SCALE_3G  = 0x19,
  SCALE_6G  = 0x1A,
  SCALE_12G = 0x1B,
  SCALE_24G = 0x1C,
}_IMU_ACCEL_SCALE;


#define DEG_TO_RAD_RATIO    0.0174532f
#define RAD_TO_DEG_RATIO    57.2957795f

//****************************************
// acceleration due to gravity in m/s/s
#define GRAVITY_MSS 9.80665f
// Gyroscope scale (uncertain where the 0.01745 value comes from)
#define GYRO_SCALE_2000_RPS  (DEG_TO_RAD_RATIO / 16.4f)
#define GYRO_SCALE_1000_RPS  (DEG_TO_RAD_RATIO / 32.8f)
#define GYRO_SCALE_500_RPS   (DEG_TO_RAD_RATIO / 65.5f)
#define GYRO_SCALE_250_RPS   (DEG_TO_RAD_RATIO / 131.0f)
#define GYRO_SCALE_2000_DPS  (2000.0f/32768.0f)
#define GYRO_SCALE_1000_DPS  (1000.0f/32768.0f)
#define GYRO_SCALE_500_DPS   (500.0f/32768.0f)
#define GYRO_SCALE_250_DPS   (250.0f/32768.0f)
// Accelerometer scale adjustment
#define ACCEL_SCALE_16G   (GRAVITY_MSS / 2048.0f)
#define ACCEL_SCALE_8G    (GRAVITY_MSS / 4096.0f)
#define ACCEL_SCALE_4G    (GRAVITY_MSS / 8192.0f)
#define ACCEL_SCALE_2G    (GRAVITY_MSS / 16384.0f)


#define GRAVITY_RAW  2048.0f
#define RAW_TO_G     GRAVITY_MSS/GRAVITY_RAW
#define G_TO_RAW  	 GRAVITY_RAW/GRAVITY_MSS

#define DEG2RAD (PI / 180.0f)
#define RAD2DEG (180.0f / PI)


/*************WGS84地心坐标参考系数**************/
#define WGS84_RADIUS_EQUATOR        6378137.0f//半长轴，单位m
#define WGS84_INVERSE_FLATTENING    298.257223563f//扁率
#define WGS84_FLATTENING            (1/WGS84_INVERSE_FLATTENING)//扁率导数
#define WGS84_RADIUS_POLAR          (WGS84_RADIUS_EQUATOR*(1-WGS84_FLATTENING))//短轴
#define WGS84_ECCENTRICITY_SQUARED  (2*WGS84_FLATTENING-WGS84_FLATTENING*WGS84_FLATTENING)
/*********************************************
经度方向距离：LON_TO_CM*经度差，LON_TO_CM对应武汉地区所在纬度平面圆周长
纬度方向距离：LAT_TO_CM*纬度差，
*********************************************/

#define LON_COSINE_LOCAL 0.860460f//约等于当地纬度的余弦值，cos(Lat*DEG_TO_RAD)
//#define LAT_TO_CM  (2.0f * WGS84_RADIUS_EQUATOR * PI / (360.0f * 100000.0f))
//#define LON_TO_CM  (2.0f * WGS84_RADIUS_EQUATOR * PI / (360.0f * 100000.0f))*LON_COSINE_LOCAL
#define LAT_TO_CM  (2.0f * WGS84_RADIUS_EQUATOR * PI/360.0f)*100.0f
#define LON_TO_CM  (2.0f * WGS84_RADIUS_EQUATOR * PI*LON_COSINE_LOCAL/360.0f)*100.0f

#define LAT_TO_M  (2.0f * WGS84_RADIUS_EQUATOR * PI/360.0f)
#define LON_TO_M  (2.0f * WGS84_RADIUS_EQUATOR * PI*LON_COSINE_LOCAL/360.0f)

#ifndef M_PI_F
#define M_PI_F 3.141592653589793f
#endif
#ifndef PI
# define PI M_PI_F
#endif
#ifndef M_PI_2
# define M_PI_2 1.570796326794897f
#endif
//Single precision conversions
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f

// radius of earth in meters
#define RADIUS_OF_EARTH 6378100
// scaling factor from 1e-7 degrees to meters at equater
// == 1.0e-7 * DEG_TO_RAD * RADIUS_OF_EARTH
#define LOCATION_SCALING_FACTOR 0.011131884502145034f
// inverse of LOCATION_SCALING_FACTOR
#define LOCATION_SCALING_FACTOR_INV 89.83204953368922f


float safe_asin(float v);
float safe_sqrt(float v);
float fast_atan(float v);
float fast_atan2(float y, float x);
float constrain_float(float amt, float low, float high);
int16_t constrain_int16(int16_t amt, int16_t low, int16_t high);
int32_t constrain_int32(int32_t amt, int32_t low, int32_t high);
float radians(float deg);
float degrees(float rad);
float sq(float v);
float pythagorous2(float a, float b);
float pythagorous3(float a, float b, float c);
float cube(float v);



#define ROOT_HALF (0.70710678118654752440084436210485f)
#define LOGA_COEF0 (-4.649062303464e-1f)
#define LOGA_COEF1 (+1.360095468621e-2f)
#define LOGB_COEF0 (-5.578873750242f)

#define LOGDA_COEF0 (-6.4124943423745581147e1f)
#define LOGDA_COEF1 (+1.6383943563021534222e1f)
#define LOGDA_COEF2 (-7.8956112887491257267e-1f)
#define LOGDB_COEF0 (-7.6949932108494879777e2f)
#define LOGDB_COEF1 (+3.1203222091924532844e2f)
#define LOGDB_COEF2 (-3.5667977739034646171e1f)
#define LN2_DC1 (0.693359375f)
#define LN2_DC2 (-2.121944400e-4f)
#define LN2_DC3 (-5.46905827678e-14f)

float FastLn(float x);
//////////////////////////////////////////////////////////////////////////
///Coefficients used for pow
#define POWP_COEF1    (+8.33333286245e-2f)
#define POWP_COEF2    (+1.25064850052e-2f)
#define POWQ_COEF1    (+6.93147180556341e-1f)
#define POWQ_COEF2    (+2.40226506144710e-1f)
#define POWQ_COEF3    (+5.55040488130765e-2f)
#define POWQ_COEF4    (+9.61620659583789e-3f)
#define POWQ_COEF5    (+1.30525515942810e-3f)

#define POW_BIGNUM (+2046.0f)
#define POW_SMALLNUM (-2015.0f)

#define LOG2E_MINUS1 (0.44269504088896340735992468100189f)
//
#define FLT_EPSILON          1.1920928955078125E-07F 
#define FLT_MAX              3.4028234663852886E+38F
//
float FastPow(float x,float y);
//////////////////////////////////////////////////////////////////////////
//
#define X_MAX (+9.099024257348e3f)
#define INV_PI_2 ( 0.63661977236758134307553505349006f)
#define PI_2_C1             ( 1.5703125f)
#define PI_2_C2             ( 4.84466552734375e-4f)
#define PI_2_C3 (-6.39757837755768678308360248557e-7f)

#define TANP_COEF1    (-1.113614403566e-1f)
#define TANP_COEF2    (+1.075154738488e-3f)
#define TANQ_COEF0    (+1.000000000000f)
#define TANQ_COEF1    (-4.446947720281e-1f)
#define TANQ_COEF2    (+1.597339213300e-2f)
float FastTan(float x);
//////////////////////////////////////////////////////////////////////////

#define _2_PI 6.283185307179586476925286766559f
#define RADTODEG(x) ((x) * 57.295779513082320876798154814105f)
#define DEGTORAD(x) ((x) * 0.01745329251994329576923690768489f)

//translate from the DSP instruction of a DSP Library.
#ifndef PI
#define PI (3.1415926535897932384626433832795f)
#endif
#define PI_2 (1.5707963267948966192313216916398f)
#define PI_3 (1.0471975511965977461542144610932f)
#define PI_4 (0.78539816339744830961566084581988f)
#define PI_6 (0.52359877559829887307710723054658f)
#define TWO_MINUS_ROOT3 (0.26794919243112270647255365849413f)
#define SQRT3_MINUS_1 (0.73205080756887729352744634150587f)
#define SQRT3 (1.7320508075688772935274463415059f)
#define EPS_FLOAT (+3.452669830012e-4f)
//Coefficients used for atan/atan2
#define ATANP_COEF0 (-1.44008344874f)
#define ATANP_COEF1 (-7.20026848898e-1f)
#define ATANQ_COEF0 (+4.32025038919f)
#define ATANQ_COEF1 (+4.75222584599f)
//Coefficients used for asin/acos
#define ASINP_COEF1 (-2.7516555290596f)
#define ASINP_COEF2 (+2.9058762374859f)
#define ASINP_COEF3 (-5.9450144193246e-1f)
#define ASINQ_COEF0 (-1.6509933202424e+1f)
#define ASINQ_COEF1 (+2.4864728969164e+1f)
#define ASINQ_COEF2 (-1.0333867072113e+1f)





#define ABS(X)  (((X)>0)?(X):-(X))
#define MAX(a,b)  ((a)>(b)?(a):(b))
#define MIN(a,b)  ((a)>(b)?(b):(a))



float FastAsin(float x);
float FastAtan2(float y, float x);

float FastSqrtI(float x);
float FastSqrt(float x);

float FastSin(float x);
float FastCos(float x);
void FastSinCos(float x, float *sinVal, float *cosVal);
float FastAbs(float x);


float invSqrt(float x);

float sine(float x) ;
float cosine(float x);

float calculate_rmse(float* data1,float *data2,int n);
float calculate_variance(float* data,int n);
float calculate_standard_deviation(float* data,int n);
float calculate_average(float* data,int n);

#define sq2(sq) (((float)sq) * ((float)sq))


//////////////////////////////////////////////////////////////////////////
float longitude_scale(Location loc);
vector2f location_diff(Location loc1,Location loc2);
float get_distance(Location loc1,Location loc2);
float get_declination(float lat, float lon);


void vector3f_mul_sub_to_rawdata(vector3f a,vector3f b,vector3f sub,vector3f *c,float scale);
void vector3f_sub(vector3f a,vector3f b,vector3f *c,float scale);


#endif


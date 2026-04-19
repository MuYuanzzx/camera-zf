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



#ifndef __DRV_MPU6050_H
#define __DRV_MPU6050_H






#define	SMPLRT_DIV		0x19
#define	MPU_CONFIG		0x1A
#define	GYRO_CONFIG		0x1B
#define	ACCEL_CONFIG	0x1C
#define ACCEL_CONFIG2 0x1D
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	PWR_MGMT_1		0x6B
#define	WHO_AM_I			0x75
#define USER_CTRL			0x6A
#define INT_PIN_CFG		0x37

#define MPU6050_ADRESS		(0x68)
//#define MPU6050_ADRESS		(0x68<<1)

#define ICM20689_ADRESS MPU6050_ADRESS

uint8_t icm20608x_init(void);
void get_imu_data(float *accel,float *gyro,float *temp,_IMU_ACCEL_SCALE *imu_acc_scale,_IMU_GYRO_SCALE *imu_gyro_scale,uint8_t *offline);



extern int8_t imu_sensor_type;
extern _IMU_ID_READ imu_id;


















/***************************************************************************************************************/
#define BMI088_ACC_I2C_ADDR0 0x19
#define BMI088_GYR_I2C_ADDR0 0x69

#define BMI088_ACC_I2C_ADDR1 0x19
#define BMI088_GYR_I2C_ADDR1 0x69


#define BMI088_ACCEL_3G_SEN  0.0008974358974f
#define BMI088_ACCEL_6G_SEN  0.00179443359375f
#define BMI088_ACCEL_12G_SEN 0.0035888671875f
#define BMI088_ACCEL_24G_SEN 0.007177734375f

#define BMI088_ACCEL_3G_SEN_G  (32768.0f/3)
#define BMI088_ACCEL_6G_SEN_G  (32768.0f/6)
#define BMI088_ACCEL_12G_SEN_G (32768.0f/12)
#define BMI088_ACCEL_24G_SEN_G (32768.0f/24)

#define BMI088_GYRO_2000_SEN_RAD 0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN_RAD 0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN_RAD  0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN_RAD  0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN_RAD  0.000066579027251980956150958662738366f

#define BMI088_SHORT_DELAY_TIME 10
#define BMI088_LONG_DELAY_TIME 100
#define BMI088_COM_WAIT_SENSOR_TIME 150
#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f


/***************************************************************************************************************/
#define BMI088_ACC_CHIP_ID 0x00 // the register is  " Who am I "
#define BMI088_ACC_CHIP_ID_VALUE 0x1E

#define BMI088_ACC_ERR_REG 0x02
#define BMI088_ACCEL_CONGIF_ERROR_SHFITS 0x2
#define BMI088_ACCEL_CONGIF_ERROR (1 << BMI088_ACCEL_CONGIF_ERROR_SHFITS)
#define BMI088_FATAL_ERROR_SHFITS 0x0
#define BMI088_FATAL_ERROR (1 << BMI088_FATAL_ERROR)

#define BMI088_ACC_STATUS 0x03
#define BMI088_ACCEL_DRDY_SHFITS 0x7
#define BMI088_ACCEL_DRDY (1 << BMI088_ACCEL_DRDY_SHFITS)

#define BMI088_ACCEL_XOUT_L 0x12
#define BMI088_ACCEL_XOUT_M 0x13
#define BMI088_ACCEL_YOUT_L 0x14
#define BMI088_ACCEL_YOUT_M 0x15
#define BMI088_ACCEL_ZOUT_L 0x16
#define BMI088_ACCEL_ZOUT_M 0x17

#define BMI088_SENSORTIME_DATA_L 0x18
#define BMI088_SENSORTIME_DATA_M 0x19
#define BMI088_SENSORTIME_DATA_H 0x1A

#define BMI088_ACC_INT_STAT_1 0x1D
#define BMI088_ACCEL_DRDY_INTERRUPT_SHFITS 0x7
#define BMI088_ACCEL_DRDY_INTERRUPT (1 << BMI088_ACCEL_DRDY_INTERRUPT_SHFITS)

#define BMI088_TEMP_M 0x22

#define BMI088_TEMP_L 0x23

#define BMI088_ACC_CONF 0x40
#define BMI088_ACC_CONF_MUST_Set 0x80
#define BMI088_ACC_OSR4 (0x08 << 4)
#define BMI088_ACC_OSR2 (0x09 << 4)
#define BMI088_ACC_NORMAL (0x0A << 4)

#define BMI088_ACC_12_5_HZ (0x5 << 0)
#define BMI088_ACC_25_HZ (0x6 << 0)
#define BMI088_ACC_50_HZ (0x7 << 0)
#define BMI088_ACC_100_HZ (0x8 << 0)
#define BMI088_ACC_200_HZ (0x9 << 0)
#define BMI088_ACC_400_HZ (0xA << 0)
#define BMI088_ACC_800_HZ (0xB << 0)
#define BMI088_ACC_1600_HZ (0xC << 0)

enum acc_odr_type_t { // output data rate
    ODR_12 = 0x05, //
    ODR_25 = 0x06, //
    ODR_50 = 0x07, //
    ODR_100 = 0x08, //
    ODR_200 = 0x09, //
    ODR_400 = 0x0A, //
    ODR_800 = 0x0B, //
    ODR_1600 = 0x0C, //
};

#define BMI088_ACC_RANGE 0x41

#define BMI088_ACC_RANGE_3G (0x00 << 0)
#define BMI088_ACC_RANGE_6G (0x01 << 0)
#define BMI088_ACC_RANGE_12G (0x02 << 0)
#define BMI088_ACC_RANGE_24G (0x03 << 0)

enum acc_scale_type_t { // measurement rage
    RANGE_3G = 0x00, //
    RANGE_6G = 0x01, //
    RANGE_12G = 0x02, //
    RANGE_24G = 0x03, //
};

#define BMI088_INT1_IO_CTRL 0x53
#define BMI088_ACC_INT1_IO_ENABLE (0x1 << 3)
#define BMI088_ACC_INT1_GPIO_PP (0x0 << 2)
#define BMI088_ACC_INT1_GPIO_OD (0x1 << 2)
#define BMI088_ACC_INT1_GPIO_LOW (0x0 << 1)
#define BMI088_ACC_INT1_GPIO_HIGH (0x1 << 1)

#define BMI088_INT2_IO_CTRL 0x54
#define BMI088_ACC_INT2_IO_ENABLE (0x1 << 3)
#define BMI088_ACC_INT2_GPIO_PP (0x0 << 2)
#define BMI088_ACC_INT2_GPIO_OD (0x1 << 2)
#define BMI088_ACC_INT2_GPIO_LOW (0x0 << 1)
#define BMI088_ACC_INT2_GPIO_HIGH (0x1 << 1)

#define BMI088_INT_MAP_DATA 0x58
#define BMI088_ACC_INT2_DRDY_INTERRUPT (0x1 << 6)
#define BMI088_ACC_INT1_DRDY_INTERRUPT (0x1 << 2)

#define BMI088_ACC_SELF_TEST 0x6D
#define BMI088_ACC_SELF_TEST_OFF 0x00
#define BMI088_ACC_SELF_TEST_POSITIVE_SIGNAL 0x0D
#define BMI088_ACC_SELF_TEST_NEGATIVE_SIGNAL 0x09

#define BMI088_ACC_PWR_CONF 0x7C
#define BMI088_ACC_PWR_SUSPEND_MODE 0x03
#define BMI088_ACC_PWR_ACTIVE_MODE 0x00
enum acc_power_type_t { // power mode
    ACC_ACTIVE = 0x00, //
    ACC_SUSPEND = 0x03, //
};


#define BMI088_ACC_PWR_CTRL 0x7D
#define BMI088_ACC_ENABLE_ACC_OFF 0x00
#define BMI088_ACC_ENABLE_ACC_ON 0x04

#define BMI088_ACC_SOFTRESET 0x7E
#define BMI088_ACC_SOFTRESET_VALUE 0xB6

#define BMI088_GYRO_CHIP_ID 0x00
#define BMI088_GYRO_CHIP_ID_VALUE 0x0F

#define BMI088_GYRO_X_L 0x02
#define BMI088_GYRO_X_H 0x03
#define BMI088_GYRO_Y_L 0x04
#define BMI088_GYRO_Y_H 0x05
#define BMI088_GYRO_Z_L 0x06
#define BMI088_GYRO_Z_H 0x07

#define BMI088_GYRO_INT_STAT_1 0x0A
#define BMI088_GYRO_DYDR (0x1 << 7)

#define BMI088_GYRO_RANGE 0x0F
#define BMI088_GYRO_2000 (0x0 << 0)
#define BMI088_GYRO_1000 (0x1 << 0)
#define BMI088_GYRO_500 (0x2 << 0)
#define BMI088_GYRO_250 (0x3 << 0)
#define BMI088_GYRO_125 (0x4 << 0)
enum gyro_scale_type_t { // measurement rage
    RANGE_2000 = 0x00, //
    RANGE_1000 = 0x01, //
    RANGE_500 = 0x02, //
    RANGE_250 = 0x03, //
    RANGE_125 = 0x04, //
};

#define BMI088_GYRO_BANDWIDTH 0x10
// the first num means Output data  rate, the second num means bandwidth
#define BMI088_GYRO_BANDWIDTH_MUST_Set 0x80
#define BMI088_GYRO_2000_532_HZ 0x00
#define BMI088_GYRO_2000_230_HZ 0x01
#define BMI088_GYRO_1000_116_HZ 0x02
#define BMI088_GYRO_400_47_HZ 0x03
#define BMI088_GYRO_200_23_HZ 0x04
#define BMI088_GYRO_100_12_HZ 0x05
#define BMI088_GYRO_200_64_HZ 0x06
#define BMI088_GYRO_100_32_HZ 0x07
enum gyro_odr_type_t { // output data rate
    ODR_2000_BW_532 = 0x00, //
    ODR_2000_BW_230 = 0x01, //
    ODR_1000_BW_116 = 0x02, //
    ODR_400_BW_47 = 0x03, //
    ODR_200_BW_23 = 0x04, //
    ODR_100_BW_12 = 0x05, //
    ODR_200_BW_64 = 0x06, //
    ODR_100_BW_32 = 0x07, //
};

#define BMI088_GYRO_LPM1 0x11
#define BMI088_GYRO_NORMAL_MODE 0x00
#define BMI088_GYRO_SUSPEND_MODE 0x80
#define BMI088_GYRO_DEEP_SUSPEND_MODE 0x20
enum gyro_power_type_t { // power mode
    GYRO_NORMAL = 0x00, //
    GYRO_SUSPEND = 0x80, //
    GYRO_DEEP_SUSPEND = 0x20, //
};

#define BMI088_GYRO_SOFTRESET 0x14
#define BMI088_GYRO_SOFTRESET_VALUE 0xB6

#define BMI088_GYRO_CTRL 0x15
#define BMI088_DRDY_OFF 0x00
#define BMI088_DRDY_ON 0x80

#define BMI088_GYRO_INT3_INT4_IO_CONF 0x16
#define BMI088_GYRO_INT4_GPIO_PP (0x0 << 3)
#define BMI088_GYRO_INT4_GPIO_OD (0x1 << 3)
#define BMI088_GYRO_INT4_GPIO_LOW (0x0 << 2)
#define BMI088_GYRO_INT4_GPIO_HIGH (0x1 << 2)
#define BMI088_GYRO_INT3_GPIO_PP (0x0 << 1)
#define BMI088_GYRO_INT3_GPIO_OD (0x1 << 1)
#define BMI088_GYRO_INT3_GPIO_LOW (0x0 << 0)
#define BMI088_GYRO_INT3_GPIO_HIGH (0x1 << 0)

#define BMI088_GYRO_INT3_INT4_IO_MAP 0x18

#define BMI088_GYRO_DRDY_IO_OFF 0x00
#define BMI088_GYRO_DRDY_IO_INT3 0x01
#define BMI088_GYRO_DRDY_IO_INT4 0x80
#define BMI088_GYRO_DRDY_IO_BOTH (BMI088_GYRO_DRDY_IO_INT3 | BMI088_GYRO_DRDY_IO_INT4)

#define BMI088_GYRO_SELF_TEST 0x3C
#define BMI088_GYRO_RATE_OK (0x1 << 4)
#define BMI088_GYRO_BIST_FAIL (0x1 << 2)
#define BMI088_GYRO_BIST_RDY (0x1 << 1)
#define BMI088_GYRO_TRIG_BIST (0x1 << 0)













/* ICM42688 registers
https://media.digikey.com/pdf/Data%20Sheets/TDK%20PDFs/ICM-42688-P_DS_Rev1.2.pdf
*/
// User Bank 0
#define ICM42688_DEVICE_CONFIG             0x11
#define ICM42688_DRIVE_CONFIG              0x13
#define ICM42688_INT_CONFIG                0x14
#define ICM42688_FIFO_CONFIG               0x16
#define ICM42688_TEMP_DATA1                0x1D
#define ICM42688_TEMP_DATA0                0x1E
#define ICM42688_ACCEL_DATA_X1             0x1F
#define ICM42688_ACCEL_DATA_X0             0x20
#define ICM42688_ACCEL_DATA_Y1             0x21
#define ICM42688_ACCEL_DATA_Y0             0x22
#define ICM42688_ACCEL_DATA_Z1             0x23
#define ICM42688_ACCEL_DATA_Z0             0x24
#define ICM42688_GYRO_DATA_X1              0x25
#define ICM42688_GYRO_DATA_X0              0x26
#define ICM42688_GYRO_DATA_Y1              0x27
#define ICM42688_GYRO_DATA_Y0              0x28
#define ICM42688_GYRO_DATA_Z1              0x29
#define ICM42688_GYRO_DATA_Z0              0x2A
#define ICM42688_TMST_FSYNCH               0x2B
#define ICM42688_TMST_FSYNCL               0x2C
#define ICM42688_INT_STATUS                0x2D
#define ICM42688_FIFO_COUNTH               0x2E
#define ICM42688_FIFO_COUNTL               0x2F
#define ICM42688_FIFO_DATA                 0x30
#define ICM42688_APEX_DATA0                0x31
#define ICM42688_APEX_DATA1                0x32
#define ICM42688_APEX_DATA2                0x33
#define ICM42688_APEX_DATA3                0x34
#define ICM42688_APEX_DATA4                0x35
#define ICM42688_APEX_DATA5                0x36
#define ICM42688_INT_STATUS2               0x37
#define ICM42688_INT_STATUS3               0x38
#define ICM42688_SIGNAL_PATH_RESET         0x4B
#define ICM42688_INTF_CONFIG0              0x4C
#define ICM42688_INTF_CONFIG1              0x4D
#define ICM42688_PWR_MGMT0                 0x4E
#define ICM42688_GYRO_CONFIG0              0x4F
#define ICM42688_ACCEL_CONFIG0             0x50
#define ICM42688_GYRO_CONFIG1              0x51
#define ICM42688_GYRO_ACCEL_CONFIG0        0x52
#define ICM42688_ACCEL_CONFIG1             0x53
#define ICM42688_TMST_CONFIG               0x54
#define ICM42688_APEX_CONFIG0              0x56
#define ICM42688_SMD_CONFIG                0x57
#define ICM42688_FIFO_CONFIG1              0x5F
#define ICM42688_FIFO_CONFIG2              0x60
#define ICM42688_FIFO_CONFIG3              0x61
#define ICM42688_FSYNC_CONFIG              0x62
#define ICM42688_INT_CONFIG0               0x63
#define ICM42688_INT_CONFIG1               0x64
#define ICM42688_INT_SOURCE0               0x65
#define ICM42688_INT_SOURCE1               0x66
#define ICM42688_INT_SOURCE3               0x68
#define ICM42688_INT_SOURCE4               0x69
#define ICM42688_FIFO_LOST_PKT0            0x6C
#define ICM42688_FIFO_LOST_PKT1            0x6D
#define ICM42688_SELF_TEST_CONFIG          0x70
#define ICM42688_WHO_AM_I                  0x75 // should return 0x47
#define ICM42688_REG_BANK_SEL              0x76

// User Bank 1
#define ICM42688_SENSOR_CONFIG0            0x03
#define ICM42688_GYRO_CONFIG_STATIC2       0x0B
#define ICM42688_GYRO_CONFIG_STATIC3       0x0C
#define ICM42688_GYRO_CONFIG_STATIC4       0x0D
#define ICM42688_GYRO_CONFIG_STATIC5       0x0E
#define ICM42688_GYRO_CONFIG_STATIC6       0x0F
#define ICM42688_GYRO_CONFIG_STATIC7       0x10
#define ICM42688_GYRO_CONFIG_STATIC8       0x11
#define ICM42688_GYRO_CONFIG_STATIC9       0x12
#define ICM42688_GYRO_CONFIG_STATIC10      0x13
#define ICM42688_XG_ST_DATA                0x5F
#define ICM42688_YG_ST_DATA                0x60
#define ICM42688_ZG_ST_DATA                0x61
#define ICM42688_TMSTAL0                   0x63
#define ICM42688_TMSTAL1                   0x64
#define ICM42688_TMSTAL2                   0x62
#define ICM42688_INTF_CONFIG4              0x7A
#define ICM42688_INTF_CONFIG5              0x7B
#define ICM42688_INTF_CONFIG6              0x7C

// User Bank 2
#define ICM42688_ACCEL_CONFIG_STATIC2      0x03
#define ICM42688_ACCEL_CONFIG_STATIC3      0x04
#define ICM42688_ACCEL_CONFIG_STATIC4      0x05
#define ICM42688_XA_ST_DATA                0x3B
#define ICM42688_YA_ST_DATA                0x3C
#define ICM42688_ZA_ST_DATA                0x3D

// User Bank 4
#define ICM42688_APEX_CONFIG1              0x40
#define ICM42688_APEX_CONFIG2              0x41
#define ICM42688_APEX_CONFIG3              0x42
#define ICM42688_APEX_CONFIG4              0x43
#define ICM42688_APEX_CONFIG5              0x44
#define ICM42688_APEX_CONFIG6              0x45
#define ICM42688_APEX_CONFIG7              0x46
#define ICM42688_APEX_CONFIG8              0x47
#define ICM42688_APEX_CONFIG9              0x48
#define ICM42688_ACCEL_WOM_X_THR           0x4A
#define ICM42688_ACCEL_WOM_Y_THR           0x4B
#define ICM42688_ACCEL_WOM_Z_THR           0x4C
#define ICM42688_INT_SOURCE6               0x4D
#define ICM42688_INT_SOURCE7               0x4E
#define ICM42688_INT_SOURCE8               0x4F
#define ICM42688_INT_SOURCE9               0x50
#define ICM42688_INT_SOURCE10              0x51
#define ICM42688_OFFSET_USER0              0x77
#define ICM42688_OFFSET_USER1              0x78
#define ICM42688_OFFSET_USER2              0x79
#define ICM42688_OFFSET_USER3              0x7A
#define ICM42688_OFFSET_USER4              0x7B
#define ICM42688_OFFSET_USER5              0x7C
#define ICM42688_OFFSET_USER6              0x7D
#define ICM42688_OFFSET_USER7              0x7E
#define ICM42688_OFFSET_USER8              0x7F

#define ICM42688_ADDRESS           0x68+1   // Address of ICM42688 accel/gyro when ADO = 0

#define AFS_2G  0x03
#define AFS_4G  0x02
#define AFS_8G  0x01
#define AFS_16G 0x00 // default

#define GFS_2000DPS   0x00   // default
#define GFS_1000DPS   0x01
#define GFS_500DPS    0x02
#define GFS_250DPS    0x03
#define GFS_125DPS    0x04
#define GFS_62_50DPS  0x05
#define GFS_31_25DPS  0x06
#define GFS_15_625DPS 0x07

// Low Noise mode
#define AODR_32kHz    0x01   
#define AODR_16kHz    0x02
#define AODR_8kHz     0x03
#define AODR_4kHz     0x04
#define AODR_2kHz     0x05
#define AODR_1kHz     0x06  // default
//Low Noise or Low Power modes
#define AODR_500Hz    0x0F
#define AODR_200Hz    0x07
#define AODR_100Hz    0x08
#define AODR_50Hz     0x09
#define AODR_25Hz     0x0A
#define AODR_12_5Hz   0x0B
// Low Power mode
#define AODR_6_25Hz   0x0C  
#define AODR_3_125Hz  0x0D
#define AODR_1_5625Hz 0x0E

#define GODR_32kHz  0x01   
#define GODR_16kHz  0x02
#define GODR_8kHz   0x03
#define GODR_4kHz   0x04
#define GODR_2kHz   0x05
#define GODR_1kHz   0x06 // default
#define GODR_500Hz  0x0F
#define GODR_200Hz  0x07
#define GODR_100Hz  0x08
#define GODR_50Hz   0x09
#define GODR_25Hz   0x0A
#define GODR_12_5Hz 0x0B

#define aMode_OFF 0x01
#define aMode_LP  0x02
#define aMode_LN  0x03

#define gMode_OFF 0x00
#define gMode_SBY 0x01
#define gMode_LN  0x03
/*****************************************************************************/




typedef struct 
{
  uint8_t cfg[8];
  uint16_t name[8];
}_icm_inner_lpf_config;
extern  _icm_inner_lpf_config icmg_inner_lpf_config;
extern  _icm_inner_lpf_config icma_inner_lpf_config;

#define icm_gyro_inner_lpf_config_default  0x03//41hz
#define icm_accel_inner_lpf_config_default 0x03//44.8hz 

uint8_t bmi088_init(void);

#endif





/*

uint8_t bmi_read_register[9]={
  //accel configure		
  BMI088_ACC_SOFTRESET_VALUE,
  BMI088_ACC_OSR4 | BMI088_ACC_1600_HZ,
  BMI088_ACC_RANGE_12G,	
  BMI088_ACC_PWR_ACTIVE_MODE,	
  BMI088_ACC_ENABLE_ACC_ON,
  //gyro configure	
  BMI088_GYRO_SOFTRESET_VALUE,
  BMI088_GYRO_2000,
  BMI088_GYRO_2000_230_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set,
  BMI088_GYRO_NORMAL_MODE,
};

  //accel configure		
  BMI088_ACC_SOFTRESET_VALUE,
  BMI088_ACC_OSR4 | BMI088_ACC_800_HZ,
  BMI088_ACC_RANGE_24G,	
  BMI088_ACC_PWR_ACTIVE_MODE,	
  BMI088_ACC_ENABLE_ACC_ON,
  //gyro configure	
  BMI088_GYRO_SOFTRESET_VALUE,
  BMI088_GYRO_2000,
  BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set,
  BMI088_GYRO_NORMAL_MODE,

*/ 
  
/*	
switch(tmp_imu_gyro_scale)
{
case SCALE_250: 		
{
if(flymaple.gyro_total<200*200) single_writei2c(imu_address,0x1B, SCALE_250);
			else single_writei2c(imu_address,0x1B, SCALE_500);
}
break;
case SCALE_500: 		
{
if(flymaple.gyro_total<200*200)      single_writei2c(imu_address,0x1B, SCALE_250);
			else if(flymaple.gyro_total<400*400) single_writei2c(imu_address,0x1B, SCALE_500);
			else single_writei2c(imu_address,0x1B, SCALE_1000);
}
break;
case SCALE_1000:		
{
if(flymaple.gyro_total<200*200) 		 single_writei2c(imu_address,0x1B, SCALE_250);
			else if(flymaple.gyro_total<400*400) single_writei2c(imu_address,0x1B, SCALE_500);
			else if(flymaple.gyro_total<800*800) single_writei2c(imu_address,0x1B, SCALE_1000);
			else single_writei2c(imu_address,0x1B, SCALE_2000);
}
break;
case SCALE_2000:
{
if(flymaple.gyro_total<200*200) 		 single_writei2c(imu_address,0x1B, SCALE_250);
			else if(flymaple.gyro_total<400*400) single_writei2c(imu_address,0x1B, SCALE_500);
			else if(flymaple.gyro_total<800*800) single_writei2c(imu_address,0x1B, SCALE_1000);
			else single_writei2c(imu_address,0x1B, SCALE_2000);
}
break;
default:
{
single_writei2c(imu_address,0x1B, SCALE_2000);
}
}

switch(tmp_imu_accel_scale)
{
case SCALE_2G: 		
{
single_writei2c(imu_address,0x1C, SCALE_4G);
}
break;
case SCALE_4G: 		
{
if(flymaple.accel_total<3.0f*3.0f) single_writei2c(imu_address,0x1C, SCALE_4G);
			else single_writei2c(imu_address,0x1C, SCALE_8G);
}
break;
case SCALE_8G:		
{
if(flymaple.accel_total<3.0f*3.0f)  single_writei2c(imu_address,0x1C, SCALE_4G);
			else if(flymaple.accel_total<6.0f*6.0f)  single_writei2c(imu_address,0x1C, SCALE_8G);
			else single_writei2c(imu_address,0x1C, SCALE_16G);
}
break;
case SCALE_16G:
{
if(flymaple.accel_total<3.0f*3.0f)  single_writei2c(imu_address,0x1C, SCALE_4G);
			else if(flymaple.accel_total<6.0f*6.0f)  single_writei2c(imu_address,0x1C, SCALE_8G);
			else single_writei2c(imu_address,0x1C, SCALE_16G);
}
break;
default:
{
single_writei2c(imu_address,0x1C, SCALE_16G);
}
}
*/







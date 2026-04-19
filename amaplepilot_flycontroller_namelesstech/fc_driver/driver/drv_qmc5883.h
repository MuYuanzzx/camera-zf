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



#define QMC5883L_DATA_READ_X_LSB	0x00
#define QMC5883L_DATA_READ_X_MSB	0x01
#define QMC5883L_DATA_READ_Y_LSB	0x02
#define QMC5883L_DATA_READ_Y_MSB	0x03
#define QMC5883L_DATA_READ_Z_LSB	0x04
#define QMC5883L_DATA_READ_Z_MSB	0x05
#define QMC5883L_TEMP_READ_LSB		0x07
#define QMC5883L_TEMP_READ_MSB		0x08 
#define QMC5883L_STATUS		0x06 // DOR | OVL | DRDY
#define QMC5883L_CONFIG_1		0x09 // OSR | RNG | ODR | MODE
#define QMC5883L_CONFIG_2		0x0A // SOFT_RST | ROL_PNT | INT_ENB
#define QMC5883L_CONFIG_3		0x0B // SET/RESET Period FBR [7:0]
#define QMC5883L_ID			0x0D



#define QMC5883L_WR_ADDRESS  				(0x0D)
#define QMC5883L_RD_ADDRESS  				(0x0D)

//#define QMC5883L_WR_ADDRESS  				(0x0D<<1)
//#define QMC5883L_RD_ADDRESS  				(0x0D<<1)


#ifndef M_PI 
#define M_PI 3.14159265358979323846264338327950288f 
#endif 

#define QMC5883L_SCALE_FACTOR 		0.732421875f
#define QMC5883L_CONVERT_GAUSS_2G 	12000.0f
#define QMC5883L_CONVERT_GAUSS_8G 	3000.0f
#define QMC5883L_CONVERT_MICROTESLA 	100
#define QMC5883L_DECLINATION_ANGLE	93.67/1000  // radian, Tekirdag/Turkey

typedef enum STATUS_VARIABLES
{
  NORMAL,
  NO_NEW_DATA,
  NEW_DATA_IS_READY,
  DATA_OVERFLOW,
  DATA_SKIPPED_FOR_READING
}_qmc5883l_status;

typedef enum MODE_VARIABLES
{
  MODE_CONTROL_STANDBY=0x00,
  MODE_CONTROL_CONTINUOUS=0x01
}_qmc5883l_MODE;

typedef enum ODR_VARIABLES
{
  OUTPUT_DATA_RATE_10HZ=0x00,
  OUTPUT_DATA_RATE_50HZ=0x04,
  OUTPUT_DATA_RATE_100HZ=0x08,
  OUTPUT_DATA_RATE_200HZ=0x0C
}_qmc5883l_ODR;

typedef enum RNG_VARIABLES
{
  FULL_SCALE_2G=0x00,
  FULL_SCALE_8G=0x10
}_qmc5883l_RNG;


typedef enum OSR_VARIABLES
{
  OVER_SAMPLE_RATIO_512=0x00,
  OVER_SAMPLE_RATIO_256=0x40,
  OVER_SAMPLE_RATIO_128=0x80,
  OVER_SAMPLE_RATIO_64=0xC0
}_qmc5883l_OSR;



typedef enum INTTERRUPT_VARIABLES
{
  INTERRUPT_DISABLE,INTERRUPT_ENABLE
}_qmc5883l_INT;



void qmc5883l_init(void);
void get_mag_data(float *mag,uint8_t *update);






// QMC5883P I2C address
#define QMC5883P_ADDR 0x2C  //0101100_0

// QMC5883P Chip ID
#define QMC5883P_CHIP_ID 0x80


// QMC5883P Registers
#define QMC5883P_REG_CHIP_ID       0x00  // Chip ID register
#define QMC5883P_REG_XOUT_L        0x01  // X-axis data LSB
#define QMC5883P_REG_XOUT_H        0x02  // X-axis data MSB
#define QMC5883P_REG_YOUT_L        0x03  // Y-axis data LSB
#define QMC5883P_REG_YOUT_H        0x04  // Y-axis data MSB
#define QMC5883P_REG_ZOUT_L        0x05  // Z-axis data LSB
#define QMC5883P_REG_ZOUT_H        0x06  // Z-axis data MSB
#define QMC5883P_REG_STATUS        0x09  // Status register (OVFL, DRDY)
#define QMC5883P_REG_CONTROL_1     0x0A  // Control register 1 (OSR2, OSR1, ODR, MODE)
#define QMC5883P_REG_CONTROL_2     0x0B  // Control register 2 (SOFT_RST, SELF_TEST, RNG, SET/RESET MODE)


// QMC5883P Status Register Bits
#define QMC5883P_STATUS_DRDY       0x01  // Data Ready
#define QMC5883P_STATUS_OVL        0x02  // Overflow

// QMC5883P Control Register 1 Bits
#define QMC5883P_CONTROL_1_MODE_SUSPEND  0x00  // Suspend mode
#define QMC5883P_CONTROL_1_MODE_NORMAL   0x01  // Normal mode
#define QMC5883P_CONTROL_1_MODE_SINGLE   0x02  // Single mode
#define QMC5883P_CONTROL_1_MODE_CONT     0x03  // Continuous mode

#define QMC5883P_CONTROL_1_OSR1_8        0x00  // Over Sample Rate: 8
#define QMC5883P_CONTROL_1_OSR1_4        0x10  // Over Sample Rate: 4
#define QMC5883P_CONTROL_1_OSR1_2        0x20  // Over Sample Rate: 2
#define QMC5883P_CONTROL_1_OSR1_1        0x30  // Over Sample Rate: 1
#define QMC5883P_CONTROL_1_OSR2_1        0x00  // Down Sample Rate: 1
#define QMC5883P_CONTROL_1_OSR2_2        0x40  // Down Sample Rate: 2
#define QMC5883P_CONTROL_1_OSR2_4        0x80  // Down Sample Rate: 4
#define QMC5883P_CONTROL_1_OSR2_8        0xC0  // Down Sample Rate: 8

// QMC5883P Control Register 2 Bits
#define QMC5883P_CONTROL_2_SOFT_RESET    0x80  // Soft reset
#define QMC5883P_CONTROL_2_SELF_TEST     0x40  // Self test
#define QMC5883P_CONTROL_2_MODE          0x00  // SET/RESET MODE

// QMC5883P RATE
#define QMC5883P_CONTROL_1_ODR_10HZ      0x00  // Output Data Rate: 10Hz
#define QMC5883P_CONTROL_1_ODR_50HZ      0x04  // Output Data Rate: 50Hz
#define QMC5883P_CONTROL_1_ODR_100HZ     0x08  // Output Data Rate: 100Hz
#define QMC5883P_CONTROL_1_ODR_200HZ     0x0C  // Output Data Rate: 200Hz

// QMC5883P RANGE
#define QMC5883P_CONTROL_2_RNG_2G        0x0C  // Range: ±2G
#define QMC5883P_CONTROL_2_RNG_8G        0x08  // Range: ±8G
#define QMC5883P_CONTROL_2_RNG_12G       0x04  // Range: ±12G
#define QMC5883P_CONTROL_2_RNG_30G       0x00  // Range: ±30G

// QMC5883P SENSITIVITY
#define QMC5883_RNG_SENSITIVITY_2G       15000.0f // LSB/Gauss
#define QMC5883_RNG_SENSITIVITY_8G       3750.0f  // LSB/Gauss
#define QMC5883_RNG_SENSITIVITY_12G      2500.0f  // LSB/Gauss
#define QMC5883_RNG_SENSITIVITY_30G      1000.0f  // LSB/Gauss

// 参数调整
#define QMC5883P_CONTROL_1_ODR  QMC5883P_CONTROL_1_ODR_200HZ     //输出速率，详见下方QMC5883P RATE
#define QMC5883P_CONTROL_2_RNG  QMC5883P_CONTROL_2_RNG_8G        //采样范围，详见下方QMC5883P RANGE
#define QMC5883_RNG_SENSITIVITY QMC5883_RNG_SENSITIVITY_8G       //灵敏度，详见下方QMC5883P SENSITIVITY，范围必须和QMC5883P RANGE一样

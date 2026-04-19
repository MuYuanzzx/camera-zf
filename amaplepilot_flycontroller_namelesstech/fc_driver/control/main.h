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







//中断优先级定义
#define ppm_irq_chl_pre_priority  0  //ppm
#define ppm_irq_chl_sub_priority  0

#define uart1_irq_chl_pre_priority  0  //GCS
#define uart1_irq_chl_sub_priority  1

#define uart2_irq_chl_pre_priority  1  //GPS/ROS,921600/460800
#define uart2_irq_chl_sub_priority  0

#define uart5_irq_chl_pre_priority  2  //sdk,256000/921600
#define uart5_irq_chl_sub_priority  0

#define uart3_irq_chl_pre_priority  3	 //tofsense/mt1,115200
#define uart3_irq_chl_sub_priority  0

#define uart4_irq_chl_pre_priority  4  //opt,19200
#define uart4_irq_chl_sub_priority  0

#define timerA_irq_chl_pre_priority  5  //timerA,1ms
#define timerA_irq_chl_sub_priority  0

#define timerB_irq_chl_pre_priority  6  //timerB,5ms￡o6 0
#define timerB_irq_chl_sub_priority  0

#define timerC_irq_chl_pre_priority  6  //timerC,5ms
#define timerC_irq_chl_sub_priority  1

#define timerD_irq_chl_pre_priority  7  //timerD,100ms
#define timerD_irq_chl_sub_priority  0


#include "zf_common_headfile.h"

//温控IO定义
#define IMU1_HEATER_PIN P00_3
#define IMU2_HEATER_PIN P00_2
//蜂鸣器、预留IO定义
#define BEEP_PIN   P23_7
#define LASERLIGHT_PIN  P06_4
//PWM接口定义
#define MOTOR_PWM_1 TCPWM_CH37_P12_1
#define MOTOR_PWM_2 TCPWM_CH53_P18_4
#define MOTOR_PWM_3 TCPWM_CH51_P18_6
#define MOTOR_PWM_4 TCPWM_CH50_P18_7
#define MOTOR_PWM_5 TCPWM_CH36_P12_0
#define MOTOR_PWM_6 TCPWM_CH45_P13_3
#define MOTOR_PWM_7 TCPWM_CH54_P18_3
#define MOTOR_PWM_8 TCPWM_CH52_P18_5
//IMU的I2C端口定义
#define IMU_I2C_SCL_PIN  P08_0
#define IMU_I2C_SDA_PIN  P08_1
//EEPROM的I2C端口定义
#define AT24CXX_I2C_SCL_PIN  P05_0
#define AT24CXX_I2C_SDA_PIN  P05_1
//OLED的I2C端口定义
#define OLED_I2C_SCL_PIN  P11_0
#define OLED_I2C_SDA_PIN  P06_3
//电压检测端口定义
#define BATTERY_VOLTAGE_ADC_PIN  ADC1_CH22_P14_2
#define ADC_KEY1_ADC_PIN         ADC0_CH06_P06_6 
#define ADC_KEY2_ADC_PIN         ADC0_CH18_P07_2  

//主任务定时器调度中断定义
#define USER1_PIT_NUM            (PIT_CH0)// 使用的周期中断编号
#define USER2_PIT_NUM            (PIT_CH1)// 使用的周期中断编号
#define PIT_PRIORITY            (CPUIntIdx2_IRQn)// 对应周期中断的中断编号

//SOFT_I2C端口定义
#define SOFT_I2C_SCL_PIN  P13_1
#define SOFT_I2C_SDA_PIN  P13_0

#define  OMV_TOF_VL53L8CX_ENABLE 1

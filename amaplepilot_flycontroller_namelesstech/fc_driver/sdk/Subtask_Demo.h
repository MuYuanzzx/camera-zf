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



#ifndef _SUBTASK_DEMO_
#define _SUBTASK_DEMO_




void Color_Block_Control_Pilot(void);
void Top_APrilTag_Control_Pilot(void);
void Self_Track_Control_Pilot(void);



#define FLIGHT_SUBTASK_NUM 50

#define WORK_HEIGHT_CM  125//150

void flight_subtask_reset(void);


void simple_auto_flight(void);


void flight_subtask_1(void);
void flight_subtask_2(void);
void flight_subtask_3(void);
void flight_subtask_4(void);
void flight_subtask_5(void);
void flight_subtask_6(void);	
void flight_subtask_7(void);
void flight_subtask_8(void);

uint8_t Auto_Takeoff(float target);
void basic_auto_flight_support(void);
void ros_flight_support(void);

void Agriculture_UAV_Basic(void);
void Agriculture_UAV_Closeloop(void);
void Agriculture_UAV_Innovation(void);

void Deliver_UAV_Basic(void);
void Deliver_UAV_Innovation(void);
void Deliver_UAV_Hulahoop(void);	


void Navigation_User_Setpoint(void);


extern uint16_t flight_subtask_cnt[FLIGHT_SUBTASK_NUM];
extern uint32_t execute_time_ms[FLIGHT_SUBTASK_NUM];
extern uint32_t flight_global_cnt[FLIGHT_SUBTASK_NUM];

extern vector3f base_position; 
extern uint16_t barcode_id;
extern uint8_t barcode_flag;


extern float min_dis_cm,min_dis_angle,target_yaw_err;





#endif






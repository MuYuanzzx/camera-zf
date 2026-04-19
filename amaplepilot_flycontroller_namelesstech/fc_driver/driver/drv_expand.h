#ifndef __DRV_EXPAND_H
#define __DRV_EXPAND_H

#include "zf_common_headfile.h"

typedef struct
{
  uint16_t times;				 //预设闪烁总次数
  uint8_t  reset;				 //闪烁进程复位标志
  uint16_t cnt;					 //闪烁控制计数器
  uint16_t times_cnt;		 //记录已闪烁次数
  uint8_t  end;					 //闪烁完成标志位
  //GPIO_TypeDef *port;				 //闪烁所在的端口
  gpio_pin_enum pin;					 //闪烁所在的GPIO
  uint32_t period;			 //闪烁周期
  float light_on_percent;//单个周期内点亮时间百分比
}_laser_light;


void reserved_io_init(void);
void laser_light_work(_laser_light *light);
void buzzer_setup(uint32_t _period,float _light_on_percent,uint16_t _times);

extern _laser_light laser_light_1,beep;;
#endif




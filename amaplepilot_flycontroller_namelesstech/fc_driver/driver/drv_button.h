#ifndef __DRV_BUTTON_H__
#define __DRV_BUTTON_H__

#include "stdint.h"


typedef enum {
  IUP=0,
  IDOWN,
  //
  UP_3D,
  DN_3D,
  LT_3D,
  RT_3D,
  ME_3D,
  BUTTON_NUM
}BUTTON;

typedef enum
{
  NO_PRESS=0,
  SHORT_PRESS,
  LONG_PRESS,
  IN_PRESS,	
  KEEP_LONG_PRESS,
}press_state;

typedef struct
{
  press_state press;
  uint8_t value;
  uint8_t last_value;	
  //GPIO_TypeDef *port;
  gpio_pin_enum pin;
  uint32_t press_time;
  uint32_t release_time; 
  uint32_t in_time; 
  uint32_t in_press_cnt;
  uint32_t state_lock_time;
}button_state;

typedef struct
{
  button_state state[BUTTON_NUM];
}rc_buttton;

#define LONG_PRESS_LIMIT  1000//1000ms
#define IN_PRESS_LIMIT  	250//250ms
#define KEEP_LONG_PRESS_LIMIT 	5000//持续按下5S



void nkey_init(void);
void read_button_state_all(void);
void read_button_state_one(button_state *button);


extern rc_buttton _button;


#endif



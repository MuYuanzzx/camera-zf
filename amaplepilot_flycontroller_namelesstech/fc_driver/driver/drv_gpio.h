#ifndef __DRV_GPIO_H
#define __DRV_GPIO_H

#include "drv_expand.h"


extern _laser_light  beep;
void ngpio_init(void);
void laser_light_work(_laser_light *light);

#endif


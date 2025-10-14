#ifndef BSP_FRIC_H
#define BSP_FRIC_H
#include "../../Application/struct_typedef.h"

#define FRIC_UP 1400
#define FRIC_DOWN 1320
#define FRIC_OFF 1000

extern void fire_off(void);
extern void fire_init(void);
extern void fire_on(uint16_t cmd);
#endif

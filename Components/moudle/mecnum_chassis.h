#ifndef MECNUM_CHASSIS_H
#define MECNUM_CHASSIS_H

#include "DJI_Motor.h"

typedef struct _ {
  DJI_Motor_t motor_lf;
  DJI_Motor_t motor_rf;
  DJI_Motor_t motor_lb;
  DJI_Motor_t motor_rb;

} mecnum_chassis_t;

#endif

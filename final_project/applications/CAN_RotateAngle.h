#ifndef CAN_ROTATEANGLE_H
#define CAN_ROTATEANGLE_H

#include "struct_typedef.h"

#define CHASSIS_CAN hcan1
#define GIMBAL_CAN hcan2

typedef struct
{
  int16_t initialAngle[2];
	
	int32_t targetAngle[2];
	int16_t targetSpeed[2];
	
	int16_t currentAngle[2];
	int16_t currentLoop[2];
	int16_t currentSpeed[2];
	
	int16_t lastAngle[2];
	
} motor_InfoSet;

typedef struct
{
    int16_t set;
    int16_t get;

    int16_t err;
    int16_t last_err;

    int16_t pout;
    int16_t iout;
    int16_t dout;
    int16_t out;
} pid;

extern void M3508_SetAngle(int16_t angle);

extern void M3508_initialize(void);

extern void M3508_update(void);

extern int16_t pid_calculate(pid motor_pid, int16_t get, int16_t set);

extern void abs_limit(int16_t *a, int16_t ABS_MAX);

#endif

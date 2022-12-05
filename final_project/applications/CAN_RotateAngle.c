#include "CAN_receive.h"
#include "CAN_RotateAngle.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

motor_InfoSet M3508_Info;
int16_t M3508_Output[2];
int16_t speed_err[2];
int16_t angle_diff[2];
int16_t pout[2];
float dout[2];
int16_t out[2];
int16_t direction;
float loop;

void M3508_SetAngle(int16_t angle)
{
	int16_t speed = 75;
	int16_t maxout = 4000;
	float kp = 2;
	float kd = 1;
	//Initialize
	if(angle > 0)
	{
		direction = +1;
	}
	else
	{
		direction = -1;
	}

	for(int16_t i=0; i<2; i++)
	{
		M3508_Output[i] = 0;
		speed_err[i] = 0;
		angle_diff[i] = 0;
		pout[i] = 0;
		dout[i] = 0;
		out[i] = 0;
	}

	//Rotate Command
	while(direction != 0)
	{
		for(int16_t i=0; i<2; i++)
		{
			speed_err[i] = (float)speed * (3591/187) *direction - get_chassis_measure(i) -> speed_rpm;
			angle_diff[i] = (int16_t)get_chassis_measure(i) -> ecd - (int16_t)get_chassis_measure(i) ->last_ecd;
			if(direction == 1 && angle_diff[i] < -1)
			{
				angle_diff[i] = (float)dout[i] / loop;
			}
			if(direction == -1 && angle_diff[i] > +1)
			{
				angle_diff[i] = (float)dout[i] / loop;
			}
			pout[i] = speed_err[i] * kp;
			dout[i] += (float)angle_diff[i] * 0.005f;
			out[i] = pout[i] + (angle - dout[i]) * kd;
			if(out[i] > maxout || out[i] < -maxout)
			{
				out[i] = maxout;
			}
			M3508_Output[i] = out[i];
			
		}

			loop++;

		CAN_cmd_chassis(M3508_Output[0],M3508_Output[1],0,0);
		HAL_Delay(1);
		
		//Data Feedback
		M3508_update();
		
		//Target Judgement

		if(dout[0] >= (int32_t)angle || dout[1] >= (int32_t)angle)
		{
			CAN_cmd_chassis(0,0,0,0);
			direction *= -1;
			loop = 0;
			break;
		}
	}
}

void M3508_update()
{
//	HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
//	HAL_CAN_RxFifo0MsgPendingCallback(&hcan2);
		
	for(int16_t i=0; i<2; i++)
	{
		M3508_Info.lastAngle[i] = M3508_Info.currentAngle[i];
		M3508_Info.currentAngle[i] = (int16_t)get_chassis_measure(i) -> ecd;
		M3508_Info.currentSpeed[i] = get_chassis_measure(i) -> speed_rpm;
	}
}

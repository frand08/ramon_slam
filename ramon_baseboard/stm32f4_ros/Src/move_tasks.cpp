#include "stm32f4xx_hal.h"

#include <std_msgs/String.h>
#include "cmsis_os.h"

extern TIM_HandleTypeDef htim1;

extern osMessageQId MovementQueueHandle;

volatile uint32_t movement_counter = 0;

/* TODO: Ver que onda con lo de que esto no sea una tarea y como lo ejecuta ROS */
void movementCallback(const std_msgs::String& mov)
//void movementCallback(const std_msgs::UInt32 mov)
{
	// 0: L
	// 1: R
	// 2: U
	// 3: D
	// 4: S
	uint32_t mov_value;

	if(strncmp(mov.data,"L",strlen("L")) == 0)
	{
		mov_value = 0;
	}

	if(strncmp(mov.data,"R",strlen("R")) == 0)
	{
		mov_value = 1;
	}

	if(strncmp(mov.data,"U",strlen("U")) == 0)
	{
		mov_value = 2;
	}

	if(strncmp(mov.data,"D",strlen("D")) == 0)
	{
		mov_value = 3;

	}

	if(strncmp(mov.data,"S",strlen("S")) == 0)
	{
		mov_value = 4;
	}
	osMessagePut(MovementQueueHandle,mov_value,osWaitForever);
}

void StartMovementTask(void const * argument)
{
	// 0: L
	// 1: R
	// 2: U
	// 3: D
	// 4: S
	uint32_t velocidad = 210;
	osEvent event;
	for(;;)
	{
		event = osMessageGet(MovementQueueHandle,osWaitForever);
		if(event.status == osEventMessage)
		{
			HAL_GPIO_TogglePin(LD1_GPIO_Port,LD1_Pin);
			switch(event.value.v)
			{
				case 0:		// Left
					movement_counter = 0;
					__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
					__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,velocidad + 40);
					__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
					__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0);
					break;

				case 1:		// Right
					movement_counter = 0;
					  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
					  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
					  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,velocidad + 40);
					  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0);
					break;

				case 2:		// Up
					movement_counter = 0;
					__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
					__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,velocidad);
					__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,velocidad);
					__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0);
					break;

				case 3:		// Down
					movement_counter = 0;
					__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,velocidad);
					__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
					__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
					__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,velocidad);
					break;

				case 4:		// Stop
					movement_counter = 0;
					__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
					__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
					__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
					__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0);
					break;

				default:	// Stop
					movement_counter = 0;
					__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
					__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
					__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
					__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0);
					break;
			}
		}
	}
}

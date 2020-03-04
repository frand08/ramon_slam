#include "stm32f4xx_hal.h"
#include "mainpp.h"
#include "cmsis_os.h"

#include <ros.h>
#include <sensor_msgs/NavSatFix.h>
#include "gps.h"

extern UART_HandleTypeDef huart4;
UART_HandleTypeDef huart_gps = huart4;

extern DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart_gps_rx = hdma_uart4_rx;
extern volatile uint8_t buffer[PINGPONG_SIZE];

extern osSemaphoreId GPSIntSemHandle;

extern ros::NodeHandle nh;

extern osMessageQId PublishQueueHandle;

/* TODO: Pasar GPS */
void StartGPSTask(void const * argument)
{
	struct gps_rmcdata gps_data;
	uint8_t gps_config[] = "$PUBX,40,GLL,0,0,0,0,0,0*5C\r\n";		//Tiene que tener el \r\n!!!!
	sensor_msgs::NavSatFix *gps = (sensor_msgs::NavSatFix *)argument;
	//Config del GPS

	gps->header.frame_id = "gps_data";

	HAL_Delay(2000);
	HAL_UART_Transmit(&huart_gps,&gps_config[0],sizeof(gps_config),300);
	HAL_Delay(200);
	strlcpy((char*)&gps_config[0],(const char*)"$PUBX,40,VTG,0,5,0,0,0,0*5B\r\n",sizeof(gps_config));
	HAL_UART_Transmit(&huart_gps,&gps_config[0],sizeof(gps_config),300);
	HAL_Delay(200);
	strlcpy((char*)&gps_config[0],(const char*)"$PUBX,40,GGA,0,0,0,0,0,0*5A\r\n",sizeof(gps_config));
	HAL_UART_Transmit(&huart_gps,&gps_config[0],sizeof(gps_config),300);
	HAL_Delay(200);
	strlcpy((char*)&gps_config[0],(const char*)"$PUBX,40,GSA,0,0,0,0,0,0*4E\r\n",sizeof(gps_config));
	HAL_UART_Transmit(&huart_gps,&gps_config[0],sizeof(gps_config),300);
	HAL_Delay(200);
	strlcpy((char*)&gps_config[0],(const char*)"$PUBX,40,GSV,0,0,0,0,0,0*59\r\n",sizeof(gps_config));
	HAL_UART_Transmit(&huart_gps,&gps_config[0],sizeof(gps_config),300);
	HAL_Delay(200);
	strlcpy((char*)&gps_config[0],(const char*)"$PUBX,40,RMC,0,1,0,0,0,0*46\r\n",sizeof(gps_config));
	HAL_UART_Transmit(&huart_gps,&gps_config[0],sizeof(gps_config),300);
	HAL_Delay(200);
	strlcpy((char*)&gps_config[0],(const char*)"$PUBX,40,GRS,0,0,0,0,0,0*5D\r\n",sizeof(gps_config));
	HAL_UART_Transmit(&huart_gps,&gps_config[0],sizeof(gps_config),300);
	HAL_Delay(200);
	strlcpy((char*)&gps_config[0],(const char*)"$PUBX,40,GST,0,0,0,0,0,0*5B\r\n",sizeof(gps_config));
	HAL_UART_Transmit(&huart_gps,&gps_config[0],sizeof(gps_config),300);
	HAL_Delay(200);
	strlcpy((char*)&gps_config[0],(const char*)"$PUBX,40,ZDA,0,0,0,0,0,0*44\r\n",sizeof(gps_config));
	HAL_UART_Transmit(&huart_gps,&gps_config[0],sizeof(gps_config),300);
	HAL_Delay(200);
	strlcpy((char*)&gps_config[0],(const char*)"$PUBX,40,GBS,0,0,0,0,0,0*4D\r\n",sizeof(gps_config));
	HAL_UART_Transmit(&huart_gps,&gps_config[0],sizeof(gps_config),300);
	HAL_Delay(200);
	strlcpy((char*)&gps_config[0],(const char*)"$PUBX,40,DTM,0,0,0,0,0,0*46\r\n",sizeof(gps_config));
	HAL_UART_Transmit(&huart_gps,&gps_config[0],sizeof(gps_config),300);
	HAL_Delay(200);
	strlcpy((char*)&gps_config[0],(const char*)"$PUBX,40,THS,0,0,0,0,0,0*54\r\n",sizeof(gps_config));
	HAL_UART_Transmit(&huart_gps,&gps_config[0],sizeof(gps_config),300);

	HAL_Delay(2000);

	__HAL_UART_ENABLE_IT(&huart_gps, UART_IT_IDLE);   						// enable idle line interrupt
	__HAL_DMA_ENABLE_IT(&hdma_uart_gps_rx, DMA_IT_TC);  					// enable DMA Tx cplt interrupt

	HAL_UART_Receive_DMA(&huart_gps,(uint8_t*)&buffer[0],PINGPONG_SIZE);
	hdma_uart_gps_rx.Instance->CR &= ~DMA_SxCR_HTIE;  					// disable uart half tx interrupt

	gps->position_covariance[0] = 0.019600000000000002;
	gps->position_covariance[1] = 0.0;
	gps->position_covariance[2] = 0.0;
	gps->position_covariance[3] = 0.0;
	gps->position_covariance[4] = 0.019600000000000002;
	gps->position_covariance[5] = 0.0;
	gps->position_covariance[6] = 0.0;
	gps->position_covariance[7] = 0.0;
	gps->position_covariance[8] = 0.04;
	gps->position_covariance_type = gps->COVARIANCE_TYPE_APPROXIMATED;
	gps->status.service = gps->status.SERVICE_GPS;
	gps->status.status = gps->status.STATUS_FIX;

	for(;;)
	{
		if(osSemaphoreWait(GPSIntSemHandle,osWaitForever) > 0)
		{
			gps->header.stamp.sec = nh.now().sec;
			gps->header.stamp.nsec = nh.now().nsec;
			if(gps_getfromRMC(&gps_data) == true)
			{
	//			gps_getfromRMC(&gps_data);
	//			gps.status.status = gps_data.status;
				gps->latitude = gps_data.latitude;
				gps->longitude = gps_data.longitude;
				gps->altitude = 0.0;
	//			gps.header.stamp.sec = (uint32_t)gps_data.time;
				/* FIXME: Ver si osWaitForever es adecuado, ademas de cambiar arg */
				osMessagePut(PublishQueueHandle,1,osWaitForever);
			}
		}
//		osDelay(100);
	}
}

#include <ros.h>
#include "stm32f4xx_hal.h"
#include "mainpp.h"
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include "MPU9250.h"
#include "gps.h"
//#include <nmea_msgs/Sentence.h>

void mpu9250_init(void);
uint32_t mpu9250_loop(sensor_msgs::Imu *imu);


volatile uint8_t gps_int_state = 0;
extern volatile uint32_t mpu_int_state;
extern volatile uint8_t buffer[PINGPONG_SIZE];
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
UART_HandleTypeDef huart_gps = huart4;
extern DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart_gps_rx = hdma_uart4_rx;
extern TIM_HandleTypeDef htim1;
uint32_t movement_counter = 0;

int intbuf[13];

uint8_t send, mongo = 0;
uint32_t transmit = 0;
uint8_t data_mongo[PINGPONG_SIZE*2];



void movement_callback(const std_msgs::String& mov);


//Creo el nodo
ros::NodeHandle nh;

sensor_msgs::Imu imu;
ros::Publisher pub_imu("imu/data",&imu);

sensor_msgs::NavSatFix gps;
ros::Publisher pub_gps("gps/fix",&gps);

//Creo subscriber para recibir mensajes de movimiento
//std_msgs::String str_msg;
ros::Subscriber<std_msgs::String> sub_movement("move", &movement_callback);
//char hola[13] = "Hola mundo";

struct gps_rmcdata gps_data;


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	nh.getHardware()->flush();
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	// Si no esta esto, rbuflen overflow, que se traduce en:
	// Lost sync with device, restarting...
	if(huart->Instance == USART3)
		nh.getHardware()->reset_rbuf();
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
//	if(huart->Instance == USART3)
//	{
//		pingpongbuf = 1;
//		uart3_status = 1;
//	}
}

/**
  * @brief  Funcion que inicializa los nodos
  * @param  none
  * @retval None
  */
void setup(void)
{
	uint8_t gps_config[] = "$PUBX,40,GLL,0,0,0,0,0,0*5C\r\n";		//Tiene que tener el \r\n!!!!

	//Inicializo el nodo
	nh.initNode();

//	nh.advertise(pub_imu);

//	nh.advertise(pub_gps);

	nh.subscribe(sub_movement);

	//Config de IMU

	mpu9250_init();

	imu.header.frame_id = "imu_data";

	//Covarianzas: Se toman solo las varianzas (xx,yy,zz), las covarianzas son despreciadas

	//Total RMS Noise del gyro = 0.1 º/s-rms (Datasheet)
	//RMS = MEAN² + VAR, con VAR = SIGMA² -> Como es de ruido : MEAN = 0
	//Esto ya es a 1 sigma ya que asi se define a la varianza

	//Quiero que este en rad/s
	imu.angular_velocity_covariance[0] = 0.1f*PI/180.0f;
	imu.angular_velocity_covariance[1] = 0;
	imu.angular_velocity_covariance[2] = 0;
	imu.angular_velocity_covariance[3] = 0;
	imu.angular_velocity_covariance[4] = 0.1f*PI/180.0f;
	imu.angular_velocity_covariance[5] = 0;
	imu.angular_velocity_covariance[6] = 0;
	imu.angular_velocity_covariance[7] = 0;
	imu.angular_velocity_covariance[8] = 0.1f*PI/180.0f;

	//Total RMS Noise del acc = 8 mg-rms (Datasheet)
	//RMS = MEAN² + VAR, con VAR = SIGMA² -> Como es de ruido : MEAN = 0
	//Esto ya es a 1 sigma ya que asi se define a la varianza

	//Quiero que este en m/s²
	imu.linear_acceleration_covariance[0] = 0.008f*GRAVITY;
	imu.linear_acceleration_covariance[1] = 0;
	imu.linear_acceleration_covariance[2] = 0;
	imu.linear_acceleration_covariance[3] = 0;
	imu.linear_acceleration_covariance[4] = 0.008f*GRAVITY;
	imu.linear_acceleration_covariance[5] = 0;
	imu.linear_acceleration_covariance[6] = 0;
	imu.linear_acceleration_covariance[7] = 0;
	imu.linear_acceleration_covariance[8] = 0.008f*GRAVITY;

	imu.orientation_covariance[0] = 0.01745f;
	imu.orientation_covariance[1] = 0.0f;
	imu.orientation_covariance[2] = 0.0f;
	imu.orientation_covariance[3] = 0.0f;
	imu.orientation_covariance[4] = 0.01745f;
	imu.orientation_covariance[5] = 0.0f;
	imu.orientation_covariance[6] = 0.0f;
	imu.orientation_covariance[7] = 0.0f;
	imu.orientation_covariance[8] = 0.15708f;

	imu.header.stamp = nh.now();
	//Config del GPS

	gps.header.frame_id = "gps_data";

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

	gps.position_covariance[0] = 0.019600000000000002;
	gps.position_covariance[1] = 0.0;
	gps.position_covariance[2] = 0.0;
	gps.position_covariance[3] = 0.0;
	gps.position_covariance[4] = 0.019600000000000002;
	gps.position_covariance[5] = 0.0;
	gps.position_covariance[6] = 0.0;
	gps.position_covariance[7] = 0.0;
	gps.position_covariance[8] = 0.04;
	gps.position_covariance_type = gps.COVARIANCE_TYPE_APPROXIMATED;
	gps.status.service = gps.status.SERVICE_GPS;
	gps.status.status = gps.status.STATUS_FIX;
	gps.header.stamp = nh.now();

}

/**
  * @brief  Loop principal
  * @param  none
  * @retval None
  */
void loop(void)
{
//	uint8_t aux[] = "HOLA\r\n";
	int gps_transmit = 0, imu_transmit = 0;
//	char buffer[600];
//	Gpsneo gps;

	if(mpu_int_state)
	{
		mpu_int_state = 0;
		imu_transmit = mpu9250_loop(&imu);
//		HAL_UART_Transmit(&huart3,aux,strlen((const char*)aux),200);
	}

	if(gps_int_state)
	{
		if(gps_getfromRMC(&gps_data) == true)
		{
//			gps_getfromRMC(&gps_data);
			gps_transmit = 1;
//			gps.status.status = gps_data.status;
			gps.latitude = gps_data.latitude;
			gps.longitude = gps_data.longitude;
			gps.altitude = 0.0;
//			gps.header.stamp.sec = (uint32_t)gps_data.time;
		}
		gps_int_state = 0;
	}

	if(nh.connected())
	{
/*
		if(imu_transmit)
		{
			imu_transmit = 0;
			imu.header.stamp = nh.now();
			pub_imu.publish(&imu);
		}

		if(gps_transmit)
		{
			gps_transmit = 0;
			gps.header.stamp = nh.now();
			pub_gps.publish(&gps);
		}
*/
	}

	nh.spinOnce();

	HAL_Delay(10);
}







void movement_callback(const std_msgs::String& mov)
{
	static uint32_t velocidad = 210;
	HAL_GPIO_TogglePin(LD1_GPIO_Port,LD1_Pin);
	if(strncmp(mov.data,"U",strlen("U")) == 0)
	{
		movement_counter = 0;
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,velocidad);
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,velocidad);
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0);
	}

	if(strncmp(mov.data,"D",strlen("D")) == 0)
	{
		movement_counter = 0;
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,velocidad);
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
		__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,velocidad);
	}

	if(strncmp(mov.data,"R",strlen("R")) == 0)
	{
		movement_counter = 0;
		  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
		  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
		  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,velocidad + 40);
		  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0);
	}

	if(strncmp(mov.data,"L",strlen("L")) == 0)
	{
		movement_counter = 0;
		  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
		  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,velocidad + 40);
		  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
		  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0);

	}
	if(strncmp(mov.data,"S",strlen("S")) == 0)
	{
		movement_counter = 0;
		  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
		  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
		  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
		  __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0);

	}
}

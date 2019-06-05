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
#include <string>
//#include <nmea_msgs/Sentence.h>


#include "cmsis_os.h"

//void mpu9250_init(void);
//uint32_t mpu9250_loop(sensor_msgs::Imu *imu);


//volatile uint8_t gps_int_state = 0;
//extern volatile uint32_t mpu_int_state;
extern volatile uint8_t buffer[PINGPONG_SIZE];
//extern DMA_HandleTypeDef hdma_usart3_rx;
//extern UART_HandleTypeDef huart3;
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

extern osMessageQId PublishQueueHandle;

void movement_callback(const std_msgs::String& mov);


extern void StartMPU9250Task(void const * argument);
extern void StartGPSTask(void const * argument);


extern osSemaphoreId MPUIntSemHandle;
extern osSemaphoreId GPSIntSemHandle;
extern osMessageQId PublishQueueHandle;
extern osThreadId MPU9250SendTaskHandle;

extern osThreadId MPU9250TaskHandle;
extern osThreadId GPSTaskHandle;

extern "C"
{
	void StartPublishTask(void const * argument)
	{
		/* Node creation */
		ros::NodeHandle nh;

		/* IMU publisher */
		sensor_msgs::Imu imu;
		ros::Publisher pub_imu("imu/data",&imu);

		/* GPS publisher */
		sensor_msgs::NavSatFix gps;
		ros::Publisher pub_gps("gps/fix",&gps);

		/* Movement subscriber */
		ros::Subscriber<std_msgs::String> sub_movement("move", &movement_callback);

		/* Definition of mpu9250_task */
		const osThreadDef_t os_thread_def_mpu9250_task = \
		{ (char*)"mpu9250_task", StartMPU9250Task, osPriorityNormal, 0, 128};

		/* Definition of gps_task */
		const osThreadDef_t os_thread_def_gps_task = \
		{ (char*)"mpu9250_task", StartGPSTask, osPriorityNormal, 0, 128};

		osEvent event;

		/* FIXME: QUE SE YO, ASI NO ME TIRA EL WARNING DE QUE NO LO USO */
		if(argument != NULL)
		{
			return;
		}
		nh.initNode();

		nh.advertise(pub_imu);
		nh.advertise(pub_gps);

		nh.subscribe(sub_movement);

		/* Creation of mpu9250_task */
		MPU9250TaskHandle = osThreadCreate(&os_thread_def_mpu9250_task, (void*)&imu);

		/* Creation of gps_task */
		GPSTaskHandle = osThreadCreate(&os_thread_def_gps_task, (void*)&gps);


		for(;;)
		{
			// Chequea si hubo evento (no lo espera). Si se encuentra conectado, transmite.
			event = osMessageGet(PublishQueueHandle,0);
			if(event.status == osEventMessage && nh.connected())
			{
				switch(event.value.v)
				{
					/* IMU */
					case 0:
						imu.header.stamp = nh.now();
						pub_imu.publish(&imu);
						break;
					/* GPS */
					case 1:
						gps.header.stamp = nh.now();
						pub_gps.publish(&gps);
						break;
						/* FIXME: Ver si esta bien que se hagan aca los casos de UART */
					/* UART TX CALLBACK */
					case 2:
						nh.getHardware()->flush();
						break;
					/* UART RX CALLBACK */
					case 3:
						nh.getHardware()->reset_rbuf();
						break;
					default:
						break;
				}
			}
			nh.spinOnce();
			osDelay(1);
		}
	}
}


typedef struct IMU_Send_t
{
	sensor_msgs::Imu *imu;
	MPU9250_data data;
}IMU_Send;


void StartMPU9250SendTask(void const * argument)
{
	IMU_Send *imu_data = (IMU_Send *) argument;

	for(;;)
	{
		osDelay(50);
		/* raw accel to m/s^2 */
		imu_data->imu->linear_acceleration.x = imu_data->data.accel.x*GRAVITY;
		imu_data->imu->linear_acceleration.y = imu_data->data.accel.y*GRAVITY;
		imu_data->imu->linear_acceleration.z = imu_data->data.accel.z*GRAVITY;

		/* raw angular velocity to rad/sec */
		imu_data->imu->angular_velocity.x = imu_data->data.gyro.x*PI/180.0f;
		imu_data->imu->angular_velocity.y = imu_data->data.gyro.y*PI/180.0f;
		imu_data->imu->angular_velocity.z = imu_data->data.gyro.z*PI/180.0f;
		//Ponemos la duracion total de lo realizado
//		imu->header.stamp.nsec = sum;		//Lo hago al transmitir

		imu_data->imu->orientation.w = imu_data->data.q[0];
		imu_data->imu->orientation.x = imu_data->data.q[1];
		imu_data->imu->orientation.y = imu_data->data.q[2];
		imu_data->imu->orientation.z = imu_data->data.q[3];

		/* FIXME: Ver si osWaitForever es adecuado, ademas de cambiar arg */
		osMessagePut(PublishQueueHandle,0,osWaitForever);
	}
}



void StartMPU9250Task(void const * argument)
{
	IMU_Send_t imu_send;
	c_MPU9250 mpu9250;


	const osThreadDef_t os_thread_def_mpu9250send_task = \
	{ (char*)"mpu9250send_task", StartMPU9250SendTask, osPriorityNormal, 0, 128};

	imu_send.imu = (sensor_msgs::Imu *) argument;
	imu_send.imu->header.frame_id = "imu_data";

	//Covarianzas: Se toman solo las varianzas (xx,yy,zz), las covarianzas son despreciadas

	//Total RMS Noise del gyro = 0.1 º/s-rms (Datasheet)
	//RMS = MEAN² + VAR, con VAR = SIGMA² -> Como es de ruido : MEAN = 0
	//Esto ya es a 1 sigma ya que asi se define a la varianza

	//Quiero que este en rad/s
	imu_send.imu->angular_velocity_covariance[0] = 0.1f*PI/180.0f;
	imu_send.imu->angular_velocity_covariance[1] = 0;
	imu_send.imu->angular_velocity_covariance[2] = 0;
	imu_send.imu->angular_velocity_covariance[3] = 0;
	imu_send.imu->angular_velocity_covariance[4] = 0.1f*PI/180.0f;
	imu_send.imu->angular_velocity_covariance[5] = 0;
	imu_send.imu->angular_velocity_covariance[6] = 0;
	imu_send.imu->angular_velocity_covariance[7] = 0;
	imu_send.imu->angular_velocity_covariance[8] = 0.1f*PI/180.0f;

	//Total RMS Noise del acc = 8 mg-rms (Datasheet)
	//RMS = MEAN² + VAR, con VAR = SIGMA² -> Como es de ruido : MEAN = 0
	//Esto ya es a 1 sigma ya que asi se define a la varianza

	//Quiero que este en m/s²
	imu_send.imu->linear_acceleration_covariance[0] = 0.008f*GRAVITY;
	imu_send.imu->linear_acceleration_covariance[1] = 0;
	imu_send.imu->linear_acceleration_covariance[2] = 0;
	imu_send.imu->linear_acceleration_covariance[3] = 0;
	imu_send.imu->linear_acceleration_covariance[4] = 0.008f*GRAVITY;
	imu_send.imu->linear_acceleration_covariance[5] = 0;
	imu_send.imu->linear_acceleration_covariance[6] = 0;
	imu_send.imu->linear_acceleration_covariance[7] = 0;
	imu_send.imu->linear_acceleration_covariance[8] = 0.008f*GRAVITY;

	imu_send.imu->orientation_covariance[0] = 0.01745f;
	imu_send.imu->orientation_covariance[1] = 0.0f;
	imu_send.imu->orientation_covariance[2] = 0.0f;
	imu_send.imu->orientation_covariance[3] = 0.0f;
	imu_send.imu->orientation_covariance[4] = 0.01745f;
	imu_send.imu->orientation_covariance[5] = 0.0f;
	imu_send.imu->orientation_covariance[6] = 0.0f;
	imu_send.imu->orientation_covariance[7] = 0.0f;
	imu_send.imu->orientation_covariance[8] = 0.15708f;

	MPU9250SendTaskHandle = osThreadCreate(&os_thread_def_mpu9250send_task, (void*)&imu_send);

	for(;;)
	{
		if(osSemaphoreWait(MPUIntSemHandle,osWaitForever) > 0)
		{
			mpu9250.read_accel_data(imu_send.data.accel);  // Read the x/y/z adc values

			mpu9250.read_gyro_data(imu_send.data.gyro);  // Read the x/y/z adc values

			mpu9250.read_mag_data(imu_send.data.mag);  // Read the x/y/z adc values

			mpu9250.read_temp_data(imu_send.data.temp);  // Read the adc values


			/* FIXME: ver como tomar el deltat */
			/*
			Now = HAL_GetTick();
			mpu9250.deltat = (float)((Now - lastUpdate)/1000.0f) ; // set integration time by time elapsed since last filter update
			lastUpdate = Now;
			 */

			//Calculo de quaternions
			mpu9250.madgwick_quaternion_update(
					imu_send.data.accel.x, imu_send.data.accel.y, imu_send.data.accel.z,
					imu_send.data.gyro.x*PI/180.0f, imu_send.data.gyro.y*PI/180.0f, imu_send.data.gyro.z*PI/180.0f,
					imu_send.data.mag.y,  imu_send.data.mag.x, imu_send.data.mag.z,
					imu_send.data.q);
		}
		osDelay(1);
	}
}


/**
  * @brief  Funcion que inicializa los nodos
  * @param  none
  * @retval None
  */
/*
void setup(void)
{
	uint8_t gps_config[] = "$PUBX,40,GLL,0,0,0,0,0,0*5C\r\n";		//Tiene que tener el \r\n!!!!

	//Inicializo el nodo
	nh.initNode();

	nh.advertise(pub_imu);

	nh.advertise(pub_gps);

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
*/

/**
  * @brief  Loop principal
  * @param  none
  * @retval None
  */
/*
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

//		if(imu_transmit)
//		{
//			imu_transmit = 0;
//			imu.header.stamp = nh.now();
//			pub_imu.publish(&imu);
//		}
//
//		if(gps_transmit)
//		{
//			gps_transmit = 0;
//			gps.header.stamp = nh.now();
//			pub_gps.publish(&gps);
//		}

	}

	nh.spinOnce();

	HAL_Delay(10);
}
*/


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
		osDelay(1);
	}
}


/* TODO: Ver que onda con lo de que esto no sea una tarea y como lo ejecuta ROS */
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


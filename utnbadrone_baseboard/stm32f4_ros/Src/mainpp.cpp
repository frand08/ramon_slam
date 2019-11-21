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
#include <sensor_msgs/MagneticField.h>
#include "MPU9250.h"
#include "gps.h"
#include <string>
//#include <nmea_msgs/Sentence.h>
#include <sstream>

#include "cmsis_os.h"

// En caso de querer calibrar el magnetometro unicamente uso este define, sino es modo normal
//#define MAG_CALIBRATION

//void mpu9250_init(void);
//uint32_t mpu9250_loop(sensor_msgs::Imu *imu);

extern I2C_HandleTypeDef hi2c1;

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
extern void StartMagnetometerTask(void const * argument);
extern void StartGPSTask(void const * argument);


extern osSemaphoreId MPUIntSemHandle;
extern osSemaphoreId GPSIntSemHandle;
extern osMessageQId PublishQueueHandle;
extern osThreadId MPU9250SendTaskHandle;

extern osThreadId MPU9250TaskHandle;
extern osThreadId MagnetometerTaskHandle;
extern osThreadId GPSTaskHandle;


/* Node creation */
ros::NodeHandle nh;

extern "C"
{
	void StartPublishTask(void const * argument)
	{
		osEvent event;

		/* Magnetometer publisher */
		sensor_msgs::MagneticField magnetometer;
		ros::Publisher pub_mag("mag_data",&magnetometer);

		/* IMU publisher */
		sensor_msgs::Imu imu;
		ros::Publisher pub_imu("imu_data",&imu);

		/* GPS publisher */
		sensor_msgs::NavSatFix gps;
		ros::Publisher pub_gps("gps_fix",&gps);

		/* Chatter publisher */
		std_msgs::String chatter;
		ros::Publisher pub_chatter("chatter",&chatter);

		chatter.data = "Hola\r\n";

#ifdef MAG_CALIBRATION
		/* Definition of magnetometer_task */
		const osThreadDef_t os_thread_def_magnetometer_task = \
		{ (char*)"magnetometer_task", StartMagnetometerTask, osPriorityNormal, 0, 256};

#else
		/* Movement subscriber */
		ros::Subscriber<std_msgs::String> sub_movement("move", &movement_callback);

		/* Definition of mpu9250_task */
		const osThreadDef_t os_thread_def_mpu9250_task = \
		{ (char*)"mpu9250_task", StartMPU9250Task, osPriorityNormal, 0, 256};

		/* Definition of gps_task */
		const osThreadDef_t os_thread_def_gps_task = \
		{ (char*)"mpu9250_task", StartGPSTask, osPriorityNormal, 0, 256};
#endif

		nh.initNode();

#ifdef MAG_CALIBRATION
		nh.advertise(pub_mag);

#else
		nh.advertise(pub_imu);
//		nh.advertise(pub_chatter);
//		nh.advertise(pub_gps);
//		nh.subscribe(sub_movement);
#endif

#ifdef MAG_CALIBRATION
		/* Creation of magnetometer_task */
		MagnetometerTaskHandle = osThreadCreate(&os_thread_def_magnetometer_task, (void*)&magnetometer);

#else
		/* Creation of mpu9250_task */
		MPU9250TaskHandle = osThreadCreate(&os_thread_def_mpu9250_task, (void*)&imu);

		/* Creation of gps_task */
		GPSTaskHandle = osThreadCreate(&os_thread_def_gps_task, (void*)&gps);
#endif

		for(;;)
		{
			// Chequea si hubo evento (no lo espera). Si se encuentra conectado, transmite.
			event = osMessageGet(PublishQueueHandle,osWaitForever);
			if(event.status == osEventMessage)
			{
				if(nh.connected())
				{
					switch(event.value.v)
					{
						/* IMU */
						case 0:
							imu.header.stamp.sec = nh.now().sec;
							imu.header.stamp.nsec = nh.now().nsec;
							pub_imu.publish(&imu);
							break;

						/* GPS */
						case 1:
							gps.header.stamp.sec = nh.now().sec;
							gps.header.stamp.nsec = nh.now().nsec;
							pub_gps.publish(&gps);
							break;

						/* Magnetometer */
						case 2:
							magnetometer.header.stamp.sec = nh.now().sec;
							magnetometer.header.stamp.nsec = nh.now().nsec;
							pub_mag.publish(&magnetometer);
							break;

						default:
							break;
					}
				}
			}
			nh.spinOnce();
		}
	}

	void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
	{
		if(huart->Instance == USART3)
			nh.getHardware()->flush();
	}


	void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{
		// Si no esta esto, rbuflen overflow, que se traduce en:
		// Lost sync with device, restarting...
		if(huart->Instance == USART3)
			nh.getHardware()->reset_rbuf();
	}
}


typedef struct IMU_Send_t
{
	sensor_msgs::Imu *imu;
	MPU9250_data data;
	int count;
} IMU_Send;


void StartMPU9250SendTask(void const * argument)
{
	IMU_Send *imu_data = (IMU_Send *) argument;
	mag_data mag;

	portTickType LastWakeTime;
	osDelay(1000);

	LastWakeTime = osKernelSysTick();	// Para poder calcular exactamente 100ms

	for(;;)
	{
		osDelayUntil(&LastWakeTime,100);
		/* raw accel to m/s^2 */
		if(imu_data->count < 0)
		{
			imu_data->imu->linear_acceleration.x = imu_data->data.accel.x*GRAVITY;
			imu_data->imu->linear_acceleration.y = imu_data->data.accel.y*GRAVITY;
			imu_data->imu->linear_acceleration.z = imu_data->data.accel.z*GRAVITY;

			/* raw angular velocity to rad/sec */
			imu_data->imu->angular_velocity.x = imu_data->data.gyro.x*PI/180.0f;
			imu_data->imu->angular_velocity.y = imu_data->data.gyro.y*PI/180.0f;
			imu_data->imu->angular_velocity.z = imu_data->data.gyro.z*PI/180.0f;
		}
		else if(imu_data->count > 0)
		{
			/* FIXME: Ver si tiene sentido calcular los cuaterniones con el promedio */
			imu_data->imu->linear_acceleration.x = imu_data->data.accel.x*GRAVITY/imu_data->count;
			imu_data->imu->linear_acceleration.y = imu_data->data.accel.y*GRAVITY/imu_data->count;
			imu_data->imu->linear_acceleration.z = imu_data->data.accel.z*GRAVITY/imu_data->count;

			imu_data->data.accel.x = 0;
			imu_data->data.accel.y = 0;
			imu_data->data.accel.z = 0;

			/* raw angular velocity to rad/sec */
			imu_data->imu->angular_velocity.x = imu_data->data.gyro.x*PI/180.0f/imu_data->count;
			imu_data->imu->angular_velocity.y = imu_data->data.gyro.y*PI/180.0f/imu_data->count;
			imu_data->imu->angular_velocity.z = imu_data->data.gyro.z*PI/180.0f/imu_data->count;


			imu_data->data.gyro.x = 0;
			imu_data->data.gyro.y = 0;
			imu_data->data.gyro.z = 0;
		}


		//Ponemos la duracion total de lo realizado
//		imu->header.stamp.nsec = sum;		//Lo hago al transmitir


		imu_data->imu->orientation.w = imu_data->data.q[0];
		imu_data->imu->orientation.x = imu_data->data.q[1];
		imu_data->imu->orientation.y = imu_data->data.q[2];
		imu_data->imu->orientation.z = imu_data->data.q[3];

		HAL_GPIO_TogglePin(LD1_GPIO_Port,LD1_Pin);

		/* FIXME: Ver si osWaitForever es adecuado, ademas de cambiar arg */
		osMessagePut(PublishQueueHandle,0,0);
	}
}


void i2c_read_bytes(uint8_t address, uint8_t subAddress, uint32_t count, uint8_t * dest)
{
	HAL_I2C_Mem_Read(&hi2c1,address,subAddress,I2C_MEMADD_SIZE_8BIT,&dest[0],(uint16_t)count,100);
}


void i2c_write_bytes(uint8_t address, uint8_t subAddress, uint32_t count, uint8_t *data)
{
	HAL_I2C_Mem_Write(&hi2c1,address,subAddress,I2C_MEMADD_SIZE_8BIT,&data[0],(uint16_t)count,100);
}


void StartMPU9250Task(void const * argument)
{
	IMU_Send_t imu_send;
	c_MPU9250 mpu9250(&HAL_Delay,&i2c_read_bytes,&i2c_write_bytes);

//	uint32_t timer;
	accel_data accel;
	gyro_data gyro;
	mag_data mag;
	const osThreadDef_t os_thread_def_mpu9250send_task = \
	{ (char*)"mpu9250send_task", StartMPU9250SendTask, osPriorityNormal, 0, 256};


	imu_send.count = -1;			//Para indicar que no quiero promediar, -1, sino 0
	imu_send.imu = (sensor_msgs::Imu *) argument;
	imu_send.imu->header.frame_id = "imu_data";

	if(!mpu9250.init())
	{
		HAL_GPIO_WritePin(LD1_GPIO_Port,LD1_Pin,GPIO_PIN_SET);
		osThreadTerminate(MPU9250TaskHandle);
	}

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
//	imu_send.imu->header.stamp = nh.now();

	MPU9250SendTaskHandle = osThreadCreate(&os_thread_def_mpu9250send_task, (void*)&imu_send);

	for(;;)
	{
		/* FIXME: Esta bien el == 0? */
		if(osSemaphoreWait(MPUIntSemHandle,osWaitForever) == 0)
		{
			// Esta en NED tomando ref del magnetometro

			mpu9250.read_accel_data(accel);  // Read the x/y/z adc values
			mpu9250.read_gyro_data(gyro);  // Read the x/y/z adc values


			mpu9250.read_mag_data(mag);  // Read the x/y/z adc values

			mpu9250.read_temp_data(imu_send.data.temp);  // Read the adc values

			if(imu_send.count < 0)
			{
				imu_send.data.accel.x = accel.x;
				imu_send.data.accel.y = accel.y;
				imu_send.data.accel.z = accel.z;

				imu_send.data.gyro.x = gyro.x;
				imu_send.data.gyro.y = gyro.y;
				imu_send.data.gyro.z = gyro.z;
			}

			else
			{
				imu_send.data.accel.x += accel.x;
				imu_send.data.accel.y += accel.y;
				imu_send.data.accel.z += accel.z;

				imu_send.data.gyro.x += gyro.x;
				imu_send.data.gyro.y += gyro.y;
				imu_send.data.gyro.z += gyro.z;

				imu_send.count++;
			}
			/* FIXME: Valor de mag Z muy alto!!! */
			// Calculo de quaternions
			// Busco NED, y como el gyro y accel son:
			// 				Y+
			//				^
			//				|
			//				|
			//				·----> X+
			//				Z+
			// Y el magnetometro
			// 				X+
			//				^
			//				|
			//				|
			//				x----> Y+
			//				Z+
			// Paso el gyro y accel al coordinate frame del mag (que es NED)
			// El deltat esta hardcodeado a 5ms en base a medicion con equipo
			mpu9250.mahony_quaternion_update(
					accel.y, accel.x, -accel.z,
					gyro.y*PI/180.0f, gyro.x*PI/180.0f, -gyro.z*PI/180.0f,
					mag.x,  mag.y, mag.z,
					imu_send.data.q,
					float(5.0f/1000.0f));
//						float(osKernelSysTick() - timer)/1000.0f);
//			timer = osKernelSysTick();
		}
	}
}


void StartMagnetometerTask(void const * argument)
{
	c_MPU9250 mpu9250(&HAL_Delay,&i2c_read_bytes,&i2c_write_bytes);

	sensor_msgs::MagneticField *mag;
	mag_data mag_raw, mag_prom;
	uint32_t count = 0;

	mag_prom.x = 0;
	mag_prom.y = 0;
	mag_prom.z = 0;

	mag = (sensor_msgs::MagneticField *) argument;
	mag->header.frame_id = "mag_data";

	if(!mpu9250.init())
	{
		HAL_GPIO_WritePin(LD1_GPIO_Port,LD1_Pin,GPIO_PIN_SET);
		osThreadTerminate(MPU9250TaskHandle);
	}

	for(;;)
	{
		if(osSemaphoreWait(MPUIntSemHandle,osWaitForever) == 0)
		{
			// Esta en NED tomando ref del magnetometro

			mpu9250.read_mag_data(mag_raw);  // Read the x/y/z adc values
			mag_prom.x += mag_raw.x;
			mag_prom.y += mag_raw.y;
			mag_prom.z += mag_raw.z;
			count++;
			if(count >= 10)
			{
				mag->magnetic_field.x = mag_prom.x / count;
				mag->magnetic_field.y = mag_prom.y / count;
				mag->magnetic_field.z = mag_prom.z / count;

				count = 0;
				mag_prom.x = 0;
				mag_prom.y = 0;
				mag_prom.z = 0;
				osMessagePut(PublishQueueHandle,2,0);
			}

		}
	}
}


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
		osDelay(100);
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


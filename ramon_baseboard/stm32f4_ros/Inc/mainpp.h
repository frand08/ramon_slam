#define PINGPONG_SIZE		128

//#define ACCEL_BIAS_FLASH_ADDR_0	0x080E0000
//#define ACCEL_BIAS_FLASH_ADDR_1	0x080E0004
//#define ACCEL_BIAS_FLASH_ADDR_2	0x080E0008
//
//#define GYRO_BIAS_FLASH_ADDR_0	0x080E000C
//#define GYRO_BIAS_FLASH_ADDR_1	0x080E0010
//#define GYRO_BIAS_FLASH_ADDR_2	0x080E0014
//
//#define MAG_BIAS_FLASH_ADDR_0	0x080E0018
//#define MAG_BIAS_FLASH_ADDR_1	0x080E001C
//#define MAG_BIAS_FLASH_ADDR_2	0x080E0020
//
//#define MAG_SCALE_FLASH_ADDR_0	0x080E0018
//#define MAG_SCALE_FLASH_ADDR_1	0x080E001C
//#define MAG_SCALE_FLASH_ADDR_2	0x080E0020

#define ACCEL_BIAS_FLASH_ADDR_0	0x080E0000
#define ACCEL_BIAS_FLASH_ADDR_1	0x080E0010
#define ACCEL_BIAS_FLASH_ADDR_2	0x080E0020

#define GYRO_BIAS_FLASH_ADDR_0	0x080E0030
#define GYRO_BIAS_FLASH_ADDR_1	0x080E0040
#define GYRO_BIAS_FLASH_ADDR_2	0x080E0050

#define MAG_BIAS_FLASH_ADDR_0	0x080E0060
#define MAG_BIAS_FLASH_ADDR_1	0x080E0070
#define MAG_BIAS_FLASH_ADDR_2	0x080E0080

#define MAG_SCALE_FLASH_ADDR_0	0x080E0090
#define MAG_SCALE_FLASH_ADDR_1	0x080E00A0
#define MAG_SCALE_FLASH_ADDR_2	0x080E00B0

#ifdef __cplusplus
extern "C" {
#endif
	uint32_t Flash_Read(uint32_t address);
	void Flash_Write(uint32_t address, uint32_t data);
//	static void MX_USART2_UART_Init(void);
#ifdef __cplusplus
}
#endif


// En caso de querer calibrar la IMU completa uso este define, sino es modo normal
#define IMU_CALIBRATION

// Para publicar o subscribir los topics cuando corresponda

//#define GPS_DATA_SEND

//#define MOVEMENT_DATA_RECEIVE

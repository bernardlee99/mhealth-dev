#include "main.h"
#include "ICM_20948_2.h"


/*
 *
 * SPI abstraction
 *
 */
void ICM2_readBytes(uint8_t reg, uint8_t *pData, uint16_t Size) // ***
{
	reg = reg | 0x80;
	HAL_GPIO_WritePin(ICM2_CS_GPIO_Port, ICM2_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(SPI_BUS, &reg, 1, 50);
	HAL_SPI_Receive(SPI_BUS, pData, Size, 50);
	HAL_GPIO_WritePin(ICM2_CS_GPIO_Port, ICM2_CS_Pin, GPIO_PIN_SET);
}

void ICM2_WriteBytes(uint8_t reg, uint8_t *pData, uint16_t Size) // ***
{
	reg = reg & 0x7F;
	HAL_GPIO_WritePin(ICM2_CS_GPIO_Port, ICM2_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(SPI_BUS, &reg, 1, 50);
	HAL_SPI_Transmit(SPI_BUS, pData, Size, 50);
	HAL_GPIO_WritePin(ICM2_CS_GPIO_Port, ICM2_CS_Pin, GPIO_PIN_SET);

}

void ICM2_ReadOneByte(uint8_t reg, uint8_t* pData) // ***
{
	reg = reg | 0x80;
	HAL_GPIO_WritePin(ICM2_CS_GPIO_Port, ICM2_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(SPI_BUS, &reg, 1, 50);
	while (HAL_SPI_GetState(SPI_BUS) != HAL_SPI_STATE_READY)
		;
	HAL_SPI_Receive(SPI_BUS, pData, 1, 50);
	while (HAL_SPI_GetState(SPI_BUS) != HAL_SPI_STATE_READY)
		;
	HAL_GPIO_WritePin(ICM2_CS_GPIO_Port, ICM2_CS_Pin, GPIO_PIN_SET);
}

void ICM2_WriteOneByte(uint8_t reg, uint8_t Data) // ***
{
	reg = reg & 0x7F;
	HAL_GPIO_WritePin(ICM2_CS_GPIO_Port, ICM2_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(SPI_BUS, &reg, 1, 50);
	HAL_SPI_Transmit(SPI_BUS, &Data, 1, 50);
	HAL_GPIO_WritePin(ICM2_CS_GPIO_Port, ICM2_CS_Pin, GPIO_PIN_SET);
}

/*
 *
 * AUX I2C abstraction for magnetometer
 *
 */
void i2c2_Mag_write(uint8_t reg,uint8_t value)
  {
  	ICM2_WriteOneByte(0x7F, 0x30);

  	HAL_Delay(1);
  	ICM2_WriteOneByte(0x03 ,0x0C);//mode: write

  	HAL_Delay(1);
  	ICM2_WriteOneByte(0x04 ,reg);//set reg addr

  	HAL_Delay(1);
  	ICM2_WriteOneByte(0x06 ,value);//send value

  	HAL_Delay(1);
  }

  static uint8_t ICM2_Mag_Read(uint8_t reg)
  {
  	uint8_t  Data;
  	ICM2_WriteOneByte(0x7F, 0x30);
    HAL_Delay(1);
  	ICM2_WriteOneByte(0x03 ,0x0C|0x80);
    HAL_Delay(1);
  	ICM2_WriteOneByte(0x04 ,reg);// set reg addr
    HAL_Delay(1);
  	ICM2_WriteOneByte(0x06 ,0xff);//read
  	HAL_Delay(1);
  	ICM2_WriteOneByte(0x7F, 0x00);
  	ICM2_ReadOneByte(0x3B,&Data);
    HAL_Delay(1);
  	return Data;
  }

  void ICM20948_2_READ_MAG(int16_t magn[3])
  {
    uint8_t mag_buffer[10];

      mag_buffer[0] =ICM2_Mag_Read(0x01);

      mag_buffer[1] =ICM2_Mag_Read(0x11);
  	  mag_buffer[2] =ICM2_Mag_Read(0x12);
  	  magn[0]=mag_buffer[1]|mag_buffer[2]<<8;
    	mag_buffer[3] =ICM2_Mag_Read(0x13);
      mag_buffer[4] =ICM2_Mag_Read(0x14);
    	magn[1]=mag_buffer[3]|mag_buffer[4]<<8;
  	 	mag_buffer[5] =ICM2_Mag_Read(0x15);
      mag_buffer[6] =ICM2_Mag_Read(0x16);
  		magn[2]=mag_buffer[5]|mag_buffer[6]<<8;

     	i2c2_Mag_write(0x31,0x01);
  }

/*
 *
 * Read magnetometer
 *
 */
void ICM2_ReadMag(int16_t magn[3]) {
	uint8_t mag_buffer[10];

	      mag_buffer[0] =ICM2_Mag_Read(0x01);

	      mag_buffer[1] =ICM2_Mag_Read(0x11);
	  	  mag_buffer[2] =ICM2_Mag_Read(0x12);
	  	  magn[0]=mag_buffer[1]|mag_buffer[2]<<8;
	    	mag_buffer[3] =ICM2_Mag_Read(0x13);
	      mag_buffer[4] =ICM2_Mag_Read(0x14);
	    	magn[1]=mag_buffer[3]|mag_buffer[4]<<8;
	  	 	mag_buffer[5] =ICM2_Mag_Read(0x15);
	      mag_buffer[6] =ICM2_Mag_Read(0x16);
	  		magn[2]=mag_buffer[5]|mag_buffer[6]<<8;

	     	i2c2_Mag_write(0x31,0x01);
}

/*
 *
 * Sequence to setup ICM290948 as early as possible after power on
 *
 */
void ICM2_PowerOn(void) {
	char uart_buffer[200];
	uint8_t whoami = 0xEA;
	uint8_t test = ICM2_WHOAMI();
	if (test == whoami) {
		ICM2_CSHigh();
		HAL_Delay(10);
		ICM2_SelectBank(USER_BANK_0);
		HAL_Delay(10);
		ICM2_Disable_I2C();
		HAL_Delay(10);
		ICM2_SetClock((uint8_t)CLK_BEST_AVAIL);
		HAL_Delay(10);
		ICM2_AccelGyroOff();
		HAL_Delay(20);
		ICM2_AccelGyroOn();
		HAL_Delay(10);
		ICM2_Initialize();
	} else {
		sprintf(uart_buffer, "Failed WHO_AM_I.  %i is not 0xEA\r\n", test);
		HAL_UART_Transmit(UART_BUS, (uint8_t*) uart_buffer, strlen(uart_buffer), 100);
		HAL_Delay(100);
	}
}
uint16_t ICM2_Initialize(void) {
		ICM2_SelectBank(USER_BANK_2);
		HAL_Delay(20);
		ICM2_SetGyroRateLPF(GYRO_RATE_250, GYRO_LPF_17HZ);
		HAL_Delay(10);

		// Set gyroscope sample rate to 100hz (0x0A) in GYRO_SMPLRT_DIV register (0x00)
		ICM2_WriteOneByte(0x00, 0x0A);
		HAL_Delay(10);

		// Set accelerometer low pass filter to 136hz (0x11) and the rate to 8G (0x04) in register ACCEL_CONFIG (0x14)
		ICM2_WriteOneByte(0x14, (0x04 | 0x11));

		// Set accelerometer sample rate to 225hz (0x00) in ACCEL_SMPLRT_DIV_1 register (0x10)
		ICM2_WriteOneByte(0x10, 0x00);
		HAL_Delay(10);

		// Set accelerometer sample rate to 100 hz (0x0A) in ACCEL_SMPLRT_DIV_2 register (0x11)
		ICM2_WriteOneByte(0x11, 0x0A);
		HAL_Delay(10);

		ICM2_SelectBank(USER_BANK_2);
		HAL_Delay(20);

		// Configure AUX_I2C Magnetometer (onboard ICM-20948)
		ICM2_WriteOneByte(0x7F, 0x00); // Select user bank 0
		ICM2_WriteOneByte(0x0F, 0x30); // INT Pin / Bypass Enable Configuration
		ICM2_WriteOneByte(0x03, 0x20); // I2C_MST_EN
		ICM2_WriteOneByte(0x7F, 0x30); // Select user bank 3
		ICM2_WriteOneByte(0x01, 0x4D); // I2C Master mode and Speed 400 kHz
		ICM2_WriteOneByte(0x02, 0x01); // I2C_SLV0 _DLY_ enable
		ICM2_WriteOneByte(0x05, 0x81); // enable IIC	and EXT_SENS_DATA==1 Byte

		// Initialize magnetometer
		i2c2_Mag_write(0x32, 0x01); // Reset AK8963
		HAL_Delay(1000);
		i2c2_Mag_write(0x31, 0x02); // use i2c to set AK8963 working on Continuous measurement mode1 & 16-bit output

		return 1337;
	}

void ICM2_ReadAccelGyro(void) {
	uint8_t raw_data[12];
	ICM2_readBytes(0x2D, raw_data, 12);

	accel_data_2[0] = (raw_data[0] << 8) | raw_data[1];
	accel_data_2[1] = (raw_data[2] << 8) | raw_data[3];
	accel_data_2[2] = (raw_data[4] << 8) | raw_data[5];

	gyro_data_2[0] = (raw_data[6] << 8) | raw_data[7];
	gyro_data_2[1] = (raw_data[8] << 8) | raw_data[9];
	gyro_data_2[2] = (raw_data[10] << 8) | raw_data[11];

	accel_data_2[0] = accel_data_2[0] / 8;
	accel_data_2[1] = accel_data_2[1] / 8;
	accel_data_2[2] = accel_data_2[2] / 8;

	gyro_data_2[0] = gyro_data_2[0] / 250;
	gyro_data_2[1] = gyro_data_2[1] / 250;
	gyro_data_2[2] = gyro_data_2[2] / 250;
}
void ICM2_SelectBank(uint8_t bank) {
	ICM2_WriteOneByte(USER_BANK_SEL, bank);
}
void ICM2_Disable_I2C(void) {
	ICM2_WriteOneByte(0x03, 0x78);
}
void ICM2_CSHigh(void) {
	HAL_GPIO_WritePin(ICM2_CS_GPIO_Port, ICM2_CS_Pin, SET);
}
void ICM2_CSLow(void) {
	HAL_GPIO_WritePin(ICM2_CS_GPIO_Port, ICM2_CS_Pin, RESET);
}
void ICM2_SetClock(uint8_t clk) {
	ICM2_WriteOneByte(PWR_MGMT_1, clk);
}
void ICM2_AccelGyroOff(void) {
	ICM2_WriteOneByte(PWR_MGMT_2, (0x38 | 0x07));
}
void ICM2_AccelGyroOn(void) {
	ICM2_WriteOneByte(0x07, (0x00 | 0x00));
}
uint8_t ICM2_WHOAMI(void) {
	uint8_t spiData = 0x01;
	ICM2_ReadOneByte(0x00, &spiData);
	return spiData;
}
void ICM2_SetGyroRateLPF(uint8_t rate, uint8_t lpf) {
	ICM2_WriteOneByte(GYRO_CONFIG_1, (rate|lpf));
}
/*
 *
 * Read Accelerometer and Gyro data
 *
 */

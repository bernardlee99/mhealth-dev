#ifndef ICM20948_2
#define ICM20948_2

#include "string.h"
#include "stdbool.h"
#include <stdio.h>

SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart2;

#define SPI_BUS &hspi2
#define UART_BUS &huart2

int16_t accel_data_2[3];
int16_t gyro_data_2[3];
int16_t mag_data_2[3];

#define USER_BANK_SEL	(0x7F)
#define USER_BANK_0		(0x00)
#define USER_BANK_1		(0x10)
#define USER_BANK_2		(0x20)
#define USER_BANK_3		(0x30)

#define PWR_MGMT_1 		(0x06)
#define PWR_MGMT_2		(0x07)
#define GYRO_CONFIG_1	(0x01)


#define CLK_BEST_AVAIL	(0x01)
#define GYRO_RATE_250	(0x00)
#define GYRO_RATE_500	(0x01)
#define GYRO_LPF_17HZ 	(0x29)

void ICM2_PowerOn();
uint8_t ICM2_WHOAMI(void);
void ICM2_SelectBank(uint8_t bank);
void ICM2_ReadAccelGyro(void);
void ICM2_ReadMag(int16_t magn[3]);
uint16_t ICM2_Initialize(void);
void ICM2_SelectBank(uint8_t bank);
void ICM2_Disable_I2C(void);
void ICM2_CSHigh(void);
void ICM2_CSLow(void);
void ICM2_SetClock(uint8_t clk);
void ICM2_AccelGyroOff(void);
void ICM2_AccelGyroOn(void);
void ICM2_SetGyroRateLPF(uint8_t rate, uint8_t lpf);
void ICM2_SetGyroLPF(uint8_t lpf);


void ICM2_init(void);
bool ICM2_deviceCheck(void);
void ICM2_readAddress(uint8_t address, uint8_t size);
//void test_accel(void);



#endif



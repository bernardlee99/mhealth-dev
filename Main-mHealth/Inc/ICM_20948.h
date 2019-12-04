#ifndef ICM20948
#define ICM20948

#include "string.h"
#include "stdbool.h"
#include <stdio.h>

SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;

#define SPI_BUS &hspi1
#define UART_BUS &huart1

int16_t accel_data[3];
int16_t gyro_data[3];
int16_t mag_data[3];

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

void ICM_PowerOn();
uint8_t ICM_WHOAMI(void);
void ICM_SelectBank(uint8_t bank);
void ICM_ReadAccelGyro(void);
void ICM_ReadMag(int16_t magn[3]);
uint16_t ICM_Initialize(void);
void ICM_SelectBank(uint8_t bank);
void ICM_Disable_I2C(void);
void ICM_CSHigh(void);
void ICM_CSLow(void);
void ICM_SetClock(uint8_t clk);
void ICM_AccelGyroOff(void);
void ICM_AccelGyroOn(void);
void ICM_SetGyroRateLPF(uint8_t rate, uint8_t lpf);
void ICM_SetGyroLPF(uint8_t lpf);


void ICM_init(void);
bool ICM_deviceCheck(void);
void ICM_readAddress(uint8_t address, uint8_t size);
void test_accel(void);



#endif



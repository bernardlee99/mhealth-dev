
// Includes
#include "main.h"
#include "MAX86150.h"
#include <stdint.h>
#include "stdlib.h"
//#include "stm32l4xx_hal_i2c.h"
char uartBuffer[100];
UART_HandleTypeDef huart1;

// Defines
#define I2C_BUFFER_LENGTH 32

/**********************************************************************************
 * START REGISTERS
 **********************************************************************************/
// Status Registers
static const uint8_t MAX86150_INTStatus1 = 			0x00;
static const uint8_t MAX86150_INTStatus2 = 			0x01;

static const uint8_t MAX86150_INT_EN1 = 			0x02;
static const uint8_t MAX86150_INT_EN2 = 			0x03;

// FIFO Registers
static const uint8_t MAX86150_FIFOWritePointer = 	0x04;
static const uint8_t MAX86150_OverflowCounter =		0x05;
static const uint8_t MAX86150_FIFOReadPointer = 	0x06;
static const uint8_t MAX86150_FIFODataRegister = 	0x07;
static const uint8_t MAX86150_FIFOConfig = 			0x08;

// FIFO Data Control
static const uint8_t MAX86150_FIFODataControl1 = 	0x09;
static const uint8_t MAX86150_FIFODataControl2 =	0x0A;

// System Control
static const uint8_t MAX86150_SystemControl =		0x0D;

// PPG Configuration
static const uint8_t MAX86150_PPGConfig1 =			0x0E;
static const uint8_t MAX86150_PPGConfig2 =			0x0F;
static const uint8_t MAX86150_ProxINTThreshold =	0x10;

// LED Pulse Amplitude
static const uint8_t MAX86150_LED1_IR =				0x11;
static const uint8_t MAX86150_LED2_RED =			0x12;
static const uint8_t MAX86150_LEDRange =			0x14;
static const uint8_t MAX86150_LEDPilot_PROX =		0x15;

// ECG Configuration
static const uint8_t MAX86150_ECGConfig1 =			0x3C;
static const uint8_t MAX86150_ECGConfig2 =			0x3E;

// Part Identification
static const uint8_t MAX86150_PartID =				0xFF;

/**********************************************************************************
 * END REGISTERS
 **********************************************************************************/


/**********************************************************************************
 * START COMMANDS
 **********************************************************************************/
// Status Registers
static const uint8_t MAX86150_INT_A_FULL_FLAG_MASK = 			0x7F; // 0111 1111 --> Zeroing-out bit [7]
static const uint8_t MAX86150_INT_A_FULL_ENABLE = 				0x80; // (1)000 0000 set bit [7] to "1"
static const uint8_t MAX86150_INT_A_FULL_DISABLE = 				0x00; // (0)000 0000 clear bit [7] to "0"

static const uint8_t MAX86150_INT_PPG_RDY_MASK = 				0xBF; // 1011 1111 --> Zeroing-out bit [6]
static const uint8_t MAX86150_INT_PPG_RDY_ENABLE = 				0x40; // 0(1)00 0000 set bit [6] to "1"
static const uint8_t MAX86150_INT_PPG_RDY_DISABLE = 			0x00; // 0(0)00 0000 clear bit [6] to "0"

static const uint8_t MAX86150_INT_ALC_OVF_MASK = 				0xDF; // 1101 1111 --> Zeroing-out bit [5]
static const uint8_t MAX86150_INT_ALC_OVF_ENABLE = 				0x20; // 00(1)0 0000 set bit [5] to "1"
static const uint8_t MAX86150_INT_ALC_OVF_DISABLE = 			0x00; // 00(0)0 0000 clear bit [5] to "0"

static const uint8_t MAX86150_INT_PROX_MASK = 					0xEF; // 1110 1111 --> Zeroing-out bit [4]
static const uint8_t MAX86150_INT_PROX_ENABLE = 				0x10; // 000(1) 0000 set bit [4] to "1"
static const uint8_t MAX86150_INT_PROX_DISABLE = 				0x00; // 000(0) 0000 clear bit [4] to "0"

static const uint8_t MAX86150_INT_PWR_RDY_MASK = 				0xFE; // 1111 1110 --> Zeroing-out bit [0]
static const uint8_t MAX86150_INT_PWR_RDY_ENABLE = 				0x01; // 0000 000(1) set bit [0] to "1"
static const uint8_t MAX86150_INT_PWR_RDY_DISABLE = 			0x00; // 0000 000(0) clear bit [0] to "0"


// PPG Mode Configuration 1
static const uint8_t MAX86150_ADCRange_MASK =					0x3F; // 0011 1111 --> Zeroing-out bits [7:6]
static const uint8_t MAX86150_ADCRange_4096 =					0x00; // (00)00 0000 set bits [7:6] to "00"
static const uint8_t MAX86150_ADCRange_8192 =					0x40; // (01)00 0000 set bits [7:6] to "01"
static const uint8_t MAX86150_ADCRange_16384 =					0x80; // (10)00 0000 set bits [7:6] to "10"
static const uint8_t MAX86150_ADCRange_32768 =					0xC0; // (11)00 0000 set bits [7:6] to "11"


static const uint8_t MAX86150_FIFO_ROLLOVER_MASK =				0xEF; // 1110 1111 --> Zeroing-out bit [4]
static const uint8_t MAX86150_FIFO_ROLLOVER_ENABLE =			0x10; // 000(1) 0000 set bit [4] to "1"
static const uint8_t MAX86150_FIFO_ROLLOVER_DISABLE =			0x00; // 000(0) 0000 clear bit [4] to "0"

	// Set the effective sampling rate of the PPG

static const uint8_t MAX86150_SampleRate_MASK =					0xC3; // 1100 0011 --> Zeroing-out bits [5:2]
static const uint8_t MAX86150_SampleRate_10_1Pulse =			0x00; // 00(00 00)00 set bits [5:2] to "0000"
static const uint8_t MAX86150_SampleRate_20_1Pulse  =			0x04; // 00(00 01)00 set bits [5:2] to "0001"
static const uint8_t MAX86150_SampleRate_50_1Pulse  =			0x08; // 00(00 10)00 set bits [5:2] to "0010"
static const uint8_t MAX86150_SampleRate_84_1Pulse  =			0x0C; // 00(00 11)00 set bits [5:2] to "0011"
static const uint8_t MAX86150_SampleRate_100_1Pulse  =			0x10; // 00(01 00)00 set bits [5:2] to "0100"
static const uint8_t MAX86150_SampleRate_200_1Pulse  =			0x14; // 00(01 01)00 set bits [5:2] to "0101"
static const uint8_t MAX86150_SampleRate_400_1Pulse  =			0x18; // 00(01 10)00 set bits [5:2] to "0110"
static const uint8_t MAX86150_SampleRate_800_1Pulse  =			0x1C; // 00(01 11)00 set bits [5:2] to "0111"
static const uint8_t MAX86150_SampleRate_1000_1Pulse  =			0x20; // 00(10 00)00 set bits [5:2] to "1000"
static const uint8_t MAX86150_SampleRate_1600_1Pulse  =			0x24; // 00(10 01)00 set bits [5:2] to "1001"
static const uint8_t MAX86150_SampleRate_3200_1Pulse  =			0x28; // 00(10 10)00 set bits [5:2] to "1010"
static const uint8_t MAX86150_SampleRate_10_2Pulse  =			0x2C; // 00(10 11)00 set bits [5:2] to "1011"
static const uint8_t MAX86150_SampleRate_20_2Pulse  =			0x30; // 00(11 00)00 set bits [5:2] to "1100"
static const uint8_t MAX86150_SampleRate_50_2Pulse  =			0x34; // 00(11 01)00 set bits [5:2] to "1101"
static const uint8_t MAX86150_SampleRate_84_2Pulse  =			0x38; // 00(11 10)00 set bits [5:2] to "1110"
static const uint8_t MAX86150_SampleRate_100_2Pulse  =			0x3C; // 00(11 11)00 set bits [5:2] to "1111"

	// These bits set the pulse width of the LED drivers and the integration time (us) of the PPG adc -- Resolution 19 bits

static const uint8_t MAX86150_LEDPulseWidth_MASK =				0xFC; // 1111 1100 --> Zeroing-out bits [1:0]
static const uint8_t MAX86150_LEDPulseWidth_50 =				0x00; // 0000 00(00) set bits [1:0] to "00"
static const uint8_t MAX86150_LEDPulseWidth_100 =				0x01; // 0000 00(01) set bits [1:0] to "01"
static const uint8_t MAX86150_LEDPulseWidth_200 =				0x02; // 0000 00(10) set bits [1:0] to "10"
static const uint8_t MAX86150_LEDPulseWidth_400 =				0x03; // 0000 00(11) set bits [1:0] to "11"


// PPG Mode Configuration 2
// Sample averaging
// These bits set the # of samples that are average on chip before being written to the FIFO
static const uint8_t MAX86150_SMPAVG_MASK =						0xF8; // 1111 1000 --> Zeroing-out bits [2:0]
static const uint8_t MAX86150_SMPAVG_1 =						0x00; // 0000 0(000) set bits [2:0] to "000" -- No averaging
static const uint8_t MAX86150_SMPAVG_2 =						0x01; // 0000 0(001) set bits [2:0] to "001"
static const uint8_t MAX86150_SMPAVG_4 =						0x02; // 0000 0(010) set bits [2:0] to "010"
static const uint8_t MAX86150_SMPAVG_8 =						0x03; // 0000 0(011) set bits [2:0] to "011"
static const uint8_t MAX86150_SMPAVG_16 =						0x04; // 0000 0(100) set bits [2:0] to "100"
static const uint8_t MAX86150_SMPAVG_32 =						0x05; // 0000 0(101) set bits [2:0] to "101"
//static const uint8_t MAX86150_SMPAVG_32 =						0x06; // 0000 0(110) set bits [2:0] to "110"
//static const uint8_t MAX86150_SMPAVG_32 =						0x07; // 0000 0(111) set bits [2:0] to "111"


// ECG Configuration 1
// These bits set the Over Sampling Ratio (OSR) of the ECG_ADC
// It also sets the ADC Clock frequency to set the ECG Sample Rate
static const uint8_t MAX86150_ECG_ADC_MASK =					0xFB; // 1111 1000 --> Zeroing-out bit [2:0]
static const uint8_t MAX86150_ECG_ADC_1600 =					0x00; // 0000 0(000) set bits [2:0] to "000"
static const uint8_t MAX86150_ECG_ADC_800 =						0x01; // 0000 0(001) set bits [2:0] to "001"
static const uint8_t MAX86150_ECG_ADC_400 =						0x02; // 0000 0(010) set bits [2:0] to "010"
static const uint8_t MAX86150_ECG_ADC_200 =						0x03; // 0000 0(011) set bits [2:0] to "011"
static const uint8_t MAX86150_ECG_ADC_3200 =					0x04; // 0000 0(100) set bits [2:0] to "100"
//static const uint8_t MAX86150_ECG_ADC_1600 =					0x05; // 0000 0(101) set bits [2:0] to "101"
//static const uint8_t MAX86150_ECG_ADC_800 =						0x06; // 0000 0(110) set bits [2:0] to "110"
//static const uint8_t MAX86150_ECG_ADC_400 =						0x07; // 0000 0(111) set bits [2:0] to "111"


// ECG Configuration 3
// These bits set the gain of the ECG PGA Gain Options - units V/V
static const uint8_t MAX86150_ECG_GAIN_MASK =					0xF3; // 1111 0011 --> Zeroing-out bit [3:2]
static const uint8_t MAX86150_ECG_GAIN_1 =						0x00; // 0000 (00)00 set bits [3:2] to "00"
static const uint8_t MAX86150_ECG_GAIN_2 =						0x04; // 0000 (01)00 set bits [3:2] to "01"
static const uint8_t MAX86150_ECG_GAIN_4 =						0x08; // 0000 (10)00 set bits [3:2] to "10"
static const uint8_t MAX86150_ECG_GAIN_8 =						0x0C; // 0000 (11)00 set bits [3:2] to "11"

// These bits set the gain of the instrumental Amplifier - units V/V
static const uint8_t MAX86150_IA_GAIN_MASK =					0xF3; // 1111 1100 --> Zeroing-out bit [1:0]
static const uint8_t MAX86150_IA_GAIN_5 =						0x00; // 0000 00(00) set bits [1:0] to "00"
static const uint8_t MAX86150_IA_GAIN_9_5 =						0x01; // 0000 00(01) set bits [1:0] to "01"
static const uint8_t MAX86150_IA_GAIN_20 =						0x02; // 0000 00(10) set bits [1:0] to "10"
static const uint8_t MAX86150_IA_GAIN_40 =						0x03; // 0000 00(11) set bits [1:0] to "11"

UART_HandleTypeDef huart2;

/**********************************************************************************
 * END COMMANDS
 **********************************************************************************/



/**********************************************************************************
 * FUNCTION DEFINITIONS
 **********************************************************************************/

void MAX86150_setup() {

	// =====================================
	// FIFO Configuration
	// =====================================

	// Copying this setup code from github
	//sprintf(uartBuffer, "Before: %d\r\n", readRegister8(MAX86150_Address, MAX86150_SystemControl));
	//HAL_UART_Transmit(&huart1, (uint8_t*)uartBuffer, strlen(uartBuffer), 1000);
	writeRegister8(MAX86150_Address, MAX86150_SystemControl,0x01);
	HAL_Delay(100);
	//sprintf(uartBuffer, "After: %d\r\n", readRegister8(MAX86150_Address, MAX86150_SystemControl));
	//HAL_UART_Transmit(&huart1, (uint8_t*)uartBuffer, strlen(uartBuffer), 1000);
	writeRegister8(MAX86150_Address, MAX86150_FIFOConfig,0x7F);
	// Default to average 4 samples
//	setFIFOAverage(MAX86150_SMPAVG_4);
	setFIFOAverage(0x5F);


	uint16_t FIFOCode = 0x00;

	FIFOCode = FIFOCode<<4 | 0x0009;// : FIFOCode;  //insert ECG front of ETI in FIFO
	FIFOCode = FIFOCode<<8 | 0x0021;//) : FIFOCode; //insert Red(2) and IR (1) in front of ECG in FIFO

	writeRegister8(MAX86150_Address, MAX86150_FIFODataControl1,(0b00100001));
	writeRegister8(MAX86150_Address, MAX86150_FIFODataControl2,(0b00001001));

	writeRegister8(MAX86150_Address, MAX86150_PPGConfig1,(0b11010001));

	writeRegister8(MAX86150_Address, MAX86150_PPGConfig2,0x06);
	writeRegister8(MAX86150_Address, MAX86150_LEDRange,0x00);

	writeRegister8(MAX86150_Address, MAX86150_SystemControl,0x04); // start FIFO

	writeRegister8(MAX86150_Address, MAX86150_ECGConfig1,0b00000011);
	writeRegister8(MAX86150_Address, MAX86150_ECGConfig2,0b00001101);

	setPulseAmplitudeRed(0xFF);
	setPulseAmplitudeIR(0xFF);

	clearFIFO();
	// =====================================
	// PPG Config 1
	// =====================================

	// Default to 4096
//	setADCRange(MAX86150_ADCRange_4096);
	// Default to 50
//	setSampleRate(MAX86150_SampleRate_50_1Pulse);
	// Set pulseWidth - default to 50us
//	setPulseWidth(MAX86150_LEDPulseWidth_50);

	// =====================================
	// LED Pulse Amplitude Configuration
	// =====================================

	// Default is 0x3F = 12.5mA
//	setPulseAmplitudeRed(0x3F);
//	setPulseAmplitudeIR(0x3F);
	// 0x7F = 25.4 mA
//	setPulseAmplitudeProximity(0x7F);

}

// Given a register, read from it, mask the bits, change it
void bitMask(uint8_t reg, uint8_t mask, uint8_t value) {

	// Read the contents of the register
	uint8_t originalContents = readRegister8(MAX86150_Address, reg);

	// Zero-out portions of the register I'm interested in
	originalContents = (reg & mask);

	// Change contents
	writeRegister8(MAX86150_Address, reg, (originalContents | value));
}


uint16_t MAX86150_check() {

	uint8_t readPointer = getReadPointer();
	uint8_t writePointer = getWritePointer();

	uint8_t activeDevices = 3;
	uint8_t numberOfSamples = 0;

	// Do we have new data
	if (readPointer != writePointer) {

		// Number of samples to read from the sensor
		numberOfSamples = writePointer - readPointer;
		if (numberOfSamples < 0) {
			numberOfSamples += I2C_BUFFER_LENGTH; // wrap condition
		}

		//We now have the number of readings, now calc uint8_ts to read
		//For this example we are just doing Red and IR (3 uint8_ts each)
		uint8_t bytesLeftToRead = numberOfSamples * activeDevices * 3;

		// Get ready to read a burst of data
		HAL_I2C_Master_Transmit(&hi2c1, MAX86150_Address, &MAX86150_FIFODataRegister, 1, 10);

		while(bytesLeftToRead > 0) {
			int8_t toGet = bytesLeftToRead;
			if(toGet > I2C_BUFFER_LENGTH) {
				toGet = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (activeDevices * 3));
			}

			bytesLeftToRead -= toGet;

			HAL_I2C_Master_Receive(&hi2c1, MAX86150_Address, &toGet, sizeof(toGet), 100);

			while(toGet > 0) {
				sense.head++; //Advance the head of the storage struct
				sense.head %= STORAGE_SIZE; //Wrap condition

				uint8_t temp[sizeof(uint32_t)]; //Array of 4 uint8_ts that we will convert into long
				uint32_t tempLong;

				temp[3] = 0;
				HAL_I2C_Master_Receive(&hi2c1, MAX86150_Address, &temp[2], 1, 100);
				HAL_I2C_Master_Receive(&hi2c1, MAX86150_Address, &temp[1], 1, 100);
				HAL_I2C_Master_Receive(&hi2c1, MAX86150_Address, &temp[0], 1, 100);

				//Convert array to long
				memcpy(&tempLong, temp, sizeof(tempLong));

				tempLong &= 0x7FFFF; //Zero out all but 18 bits

				sense.red[sense.head] = tempLong; //Store this reading into the sense array

				if (activeDevices > 1)
					{
					  //Burst read three more uint8_ts - IR
					  temp[3] = 0;
					  HAL_I2C_Master_Receive(&hi2c1, MAX86150_Address, &temp[2], 1, 100);
					  HAL_I2C_Master_Receive(&hi2c1, MAX86150_Address, &temp[1], 1, 100);
					  HAL_I2C_Master_Receive(&hi2c1, MAX86150_Address, &temp[0], 1, 100);

					  //Convert array to long
					  memcpy(&tempLong, temp, sizeof(tempLong));
						//Serial.println(tempLong);
					  tempLong &= 0x7FFFF; //Zero out all but 18 bits

					  sense.IR[sense.head] = tempLong;
					}

					if (activeDevices > 2)
					{
					  //Burst read three more uint8_ts - ECG
								int32_t tempLongSigned;

					  temp[3] = 0;
					  HAL_I2C_Master_Receive(&hi2c1, MAX86150_Address, &temp[2], 1, 100);
					  HAL_I2C_Master_Receive(&hi2c1, MAX86150_Address, &temp[1], 1, 100);
					  HAL_I2C_Master_Receive(&hi2c1, MAX86150_Address, &temp[0], 1, 100);
								//Serial.println(tempLong);
					  //Convert array to long
					  memcpy(&tempLongSigned, temp, sizeof(tempLongSigned));

							//tempLong &= 0x3FFFF; //Zero out all but 18 bits

					  sense.ecg[sense.head] = tempLongSigned;
					}

					toGet -= activeDevices * 3;
				  }
				} //End while (uint8_tsLeftToRead > 0)
			  } //End readPtr != writePtr
			  return (numberOfSamples); //Let the world know how much new data we found
			}


uint32_t MAX86150_getFIFORed(void)
{
  return (sense.red[sense.tail]);
}

uint32_t getFIFOIR(void)
{
  return (sense.IR[sense.tail]);
}

uint8_t getReadPointer() {

	return readRegister8(MAX86150_Address, MAX86150_FIFOReadPointer);
}

uint8_t getWritePointer() {

	return readRegister8(MAX86150_Address, MAX86150_FIFOWritePointer);
}



// I2C Communication
uint8_t readRegister8(uint8_t address, uint8_t reg) {

//	uint8_t data[2];
	uint8_t registerContents;
//	data[0] = reg;

	HAL_I2C_Master_Transmit(&hi2c1, MAX86150_Address, &reg, 1, 100);


	// Store the data from the I2C Communication in data[1]
	HAL_I2C_Master_Receive(&hi2c1, MAX86150_Address, &registerContents, 1, 100);

	return registerContents;
}

// The 7-bit address of device, the register you want to write to, the value you want to write
void writeRegister8(uint8_t SevenBitAddress, uint8_t reg, uint8_t value) {

	uint8_t data[2];

	data[0] = reg;
	data[1] = value;

	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, MAX86150_Address, data, 2, 100);

	return;

}

void setFIFOAverage(uint8_t numSamp) {

	bitMask(MAX86150_FIFOConfig, MAX86150_SampleRate_MASK, numSamp);
}

void setADCRange(uint8_t adcRange) {

	bitMask(MAX86150_PPGConfig1,MAX86150_ADCRange_MASK, adcRange);
}

void setSampleRate(uint8_t sampleRate) {

	bitMask(MAX86150_PPGConfig1,MAX86150_SampleRate_MASK, sampleRate);
}

void setPulseWidth(uint8_t pulseWidth) {

	// LEDPulseWidth_50, _100, _200, _400
	bitMask(MAX86150_PPGConfig1,MAX86150_LEDPulseWidth_MASK, pulseWidth);
}

void setPulseAmplitudeRed(uint8_t amplitude) {

	writeRegister8(MAX86150_Address, MAX86150_LED2_RED, amplitude);
}


void setPulseAmplitudeIR(uint8_t amplitude) {

	writeRegister8(MAX86150_Address, MAX86150_LED1_IR, amplitude);
}


void setPulseAmplitudeProximity(uint8_t amplitude) {

	writeRegister8(MAX86150_Address, MAX86150_LED1_IR, amplitude);
}

void setFIFORollOver_ENABLE(void) {

	bitMask(MAX86150_FIFOConfig,MAX86150_FIFO_ROLLOVER_MASK, MAX86150_FIFO_ROLLOVER_ENABLE);
}

void setFIFORollOver_DISABLE(void) {

	bitMask(MAX86150_FIFOConfig,MAX86150_FIFO_ROLLOVER_MASK, MAX86150_FIFO_ROLLOVER_DISABLE);
}

uint8_t getINT1(void) {

	return (readRegister8(MAX86150_Address, MAX86150_INTStatus1));
}
uint8_t getINT2(void) {

	return (readRegister8(MAX86150_Address, MAX86150_INTStatus2));
}


void enableA_FULL(void) {

	bitMask(MAX86150_INT_EN1, MAX86150_INT_A_FULL_FLAG_MASK, MAX86150_INT_A_FULL_ENABLE);
}

void disableA_FULL(void) {

	bitMask(MAX86150_INT_EN1, MAX86150_INT_A_FULL_FLAG_MASK, MAX86150_INT_A_FULL_DISABLE);
}

void enablePPG_RDY(void) {

	bitMask(MAX86150_INT_EN1, MAX86150_INT_PPG_RDY_MASK, MAX86150_INT_PPG_RDY_ENABLE);
}

void disablePPG_RDY(void) {

	bitMask(MAX86150_INT_EN1, MAX86150_INT_PPG_RDY_MASK, MAX86150_INT_PPG_RDY_DISABLE);
}

void enableALC_OVF(void) {

	bitMask(MAX86150_INT_EN1, MAX86150_INT_ALC_OVF_MASK, MAX86150_INT_ALC_OVF_ENABLE);
}

void disableALC_OVF(void) {

	bitMask(MAX86150_INT_EN1, MAX86150_INT_ALC_OVF_MASK, MAX86150_INT_ALC_OVF_DISABLE);
}

void enablePROX_INT(void) {

	bitMask(MAX86150_INT_EN1, MAX86150_INT_PROX_MASK, MAX86150_INT_PROX_ENABLE);
}

void disablePROX_INT(void) {

	bitMask(MAX86150_INT_EN1, MAX86150_INT_PROX_MASK, MAX86150_INT_PROX_DISABLE);
}

void clearFIFO(void) {

	writeRegister8(MAX86150_Address, MAX86150_FIFOWritePointer, 0);
	writeRegister8(MAX86150_Address, MAX86150_OverflowCounter, 0);
	writeRegister8(MAX86150_Address, MAX86150_FIFOReadPointer, 0);
}


/**********************************************************************************
 * END FUNCTION DEFINITIONS
 **********************************************************************************/



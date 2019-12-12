
// Defines
#define MAX86150_Address			0xBC    // Use this - tested
#define STORAGE_SIZE 4                      //Each long is 4 bytes so limit this to fit on your micro

// Includes
#include <stdint.h>


I2C_HandleTypeDef hi2c1;




  typedef struct Record
  {
    uint32_t red[STORAGE_SIZE];
    uint32_t IR[STORAGE_SIZE];
    int32_t ecg[STORAGE_SIZE];
    uint8_t head;
    uint8_t tail;
  } sense_struct; //This is our circular buffer of readings from the sensor

  sense_struct sense;


/**********************************************************************************
 * FUNCTION PROTOTYPES
 **********************************************************************************/

void MAX86150_setup();
void wakeUp();
void shutDown();

void setADCRange(uint8_t range);
void setSampleRate(uint8_t rate);
void setPulseWidth(uint8_t pulsewidth);


void setPulseAmplitudeGreen(uint8_t value);
void setPulseAmplitudeRed(uint8_t value);
void setPulseAmplitudeIR(uint8_t value);
void setPulseAmplitudeProximity(uint8_t value);

void setProximityThreshold(uint8_t value);

void setFIFOAverage(uint8_t samples);
void setFIFORollOver_ENABLE(void);
void setFIFORollOver_DISABLE(void);

// Interrupts
uint8_t getINT1(void);
uint8_t getINT2(void);
void enableA_FULL(void);
void disableA_FULL(void);
void enablePPG_RDY(void);
void disablePPG_RDY(void);
void enableALC_OVF(void);
void disableALC_OVF(void);
void enablePROX_INT(void);
void disablePROX_INT(void);
void clearFIFO(void);

uint32_t MAX86150_getFIFORed(void);
uint32_t getFIFOIR(void);

// Data Collection

uint16_t MAX86150_check(void);
uint8_t getReadPointer();
uint8_t getWritePointer();





// I2C Communication
uint8_t readRegister8(uint8_t address, uint8_t reg);
void writeRegister8(uint8_t SevenBitAddress, uint8_t reg, uint8_t value);
void bitMask(uint8_t reg, uint8_t mask, uint8_t value);


#ifndef _NAV_H

#define _NAV_H

#include "periph.h"

extern uint8_t i2cRxBuffer[14];
extern uint8_t i2cTxBuffer[1];
extern int16_t accel_gyro_temp[7];
extern float gForceX, gForceY, gForceZ;
extern float rotX, rotY, rotZ;
extern float temp_C;
extern uint8_t goState;

void TIM1_IRQHandler(void);
void EXTI9_5_IRQHandler(void);
void EXTI15_10_IRQHandler(void);

void I2C2_EV_IRQHandler(void);
void DMA1_Channel5_IRQHandler(void);
void MPU6050_CalcAccelRot(void);

void GetClkFreq(void);

typedef struct{
	uint16_t prev_cap;
	uint16_t new_cap;
	uint16_t diff_cnt;
	float d_wheel;/*diameter of wheel in mm*/
	float diff_ms;
	float w;/*angular velocity of wheel in rad/ms*/
	float v;/*tangential velocity of wheel in mm/s*/
	//uint8_t first_cap;
}capture;

void InitNav(void);
void InitCapture(capture* capStruct);

void ENC_SetLeftValue(uint32_t value);
void ENC_SetRightValue(uint32_t value);

uint32_t ENC_GetLeftValue();
uint32_t ENC_GetRightValue();

#endif

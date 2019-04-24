#include "nav.h"
#include "control.h"

/*encoder input capture values*/
uint32_t ENC_Left;
uint32_t ENC_Right;

/*encoder channels*/
capture RA;
//capture RB;
capture LA;
//capture LB;

uint16_t timer1_clk;

float d_wheel = 18;/*mm*/
float gear_ratio = 4;/*#cogs_wheel/#cogs_magnet*/

uint8_t i2cRxBuffer[14] = {};
uint8_t i2cTxBuffer[] = { 0x3B };
int16_t accel_gyro_temp[7];
float gForceX, gForceY, gForceZ;
float rotX, rotY, rotZ;
float temp_C;

void InitNav(void){
	InitCapture(&RA);
	InitCapture(&LA);
	GetClkFreq();
}

void InitCapture(capture* capStruct){
	capStruct->prev_cap = 0;
	capStruct->new_cap = 0;
	capStruct->diff_cnt = 0;
	capStruct->diff_ms = 0;
	capStruct->d_wheel = 18;/*mm*/
	capStruct->w = 0;/*rad/ms*/
	capStruct->v = 0;/*mm/s*/
	//capStruct->first_cap = 1;
}

void GetClkFreq(void)
{
	RCC_ClocksTypeDef   clkConfStruct;
	RCC_GetClocksFreq(&clkConfStruct);

	timer1_clk = clkConfStruct.PCLK2_Frequency/1000;/*kHz; if APB2 prescaler = 1!!!*/
}

void ENC_SetLeftValue(uint32_t value){
	ENC_Left = value;
}

void ENC_SetRightValue(uint32_t value){
	ENC_Right = value;
}

uint32_t ENC_GetLeftValue(){
	return ENC_Left;
}

uint32_t ENC_GetRightValue(){
	return ENC_Right;
}



float duty_start = 0; //MOT_PWM_MAX_DUTY/2;

uint8_t dir_L = 0, dir_R = 0;
uint8_t goState=0;	/*for enabling/disabling control, global variable*/

void EXTI9_5_IRQHandler(void){

	/* interrupt request from encoderL */
	if(SET == EXTI_GetITStatus(EXTI_Line8)){
		EXTI->PR = EXTI_Line8;
		ENC_Left++;
		return;
	}

	/* interrupt request from button */
	if(SET == EXTI_GetITStatus(EXTI_Line5)){
		EXTI->PR = EXTI_Line5;
		/*do stuff when button pressed*/

		ENC_SetLeftValue(0);
		ENC_SetRightValue(0);

		/*start/stop motors*/
		if(!goState){
			goState = 1;
//			MotCtl(0.4, MOT_R);
			v_base = 0.5; // TODO - change it
			TIM_Cmd(TIM4,ENABLE); // starts the control loop
		}
		else {
			TIM_Cmd(TIM4,DISABLE); // stops the control loop
			v_base = 0;
			MotCtl(0, MOT_L);
			MotCtl(0, MOT_R);

			goState = 0;
		}
	}
}


void EXTI15_10_IRQHandler(void){
	/* interrupt request from encoderR */
	if(SET == EXTI_GetITStatus(EXTI_Line10)){
		EXTI->PR = EXTI_Line10;
		ENC_Right++;
	}
}

void I2C2_EV_IRQHandler(void) {
	if (I2C_GetFlagStatus(I2C2, I2C_FLAG_SB) == SET) {
		if (i2cDirectionWrite) {
			// STM32 Transmitter
			I2C_Send7bitAddress(I2C2, MPU6050_DEFAULT_ADDRESS, I2C_Direction_Transmitter);
		} else {
			// STM32 Receiver
			I2C_Send7bitAddress(I2C2, MPU6050_DEFAULT_ADDRESS, I2C_Direction_Receiver);
		}
	} else if (I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == SUCCESS) {
		if (i2cDirectionWrite) {
			// STM32 Transmitter
			DMA_Cmd(DMA1_Channel4, ENABLE);
		}
	} else if (I2C_GetFlagStatus(I2C2, I2C_FLAG_BTF)) {
		if (i2cDirectionWrite) {
			// STM32 Transmitter
			DMA_Cmd(DMA1_Channel5, ENABLE);
			I2C_DMALastTransferCmd(I2C2, ENABLE);
			I2C_GenerateSTART(I2C2, ENABLE);
			i2cDirectionWrite = 0;
			I2C_ClearFlag(I2C2, I2C_FLAG_BTF);
		}
	}
}

// mpu6050 readings are ready
void DMA1_Channel5_IRQHandler(void) {
	DMA_ClearFlag(DMA1_FLAG_TC5);
	I2C_GenerateSTOP(I2C2, ENABLE);
	DMA_Cmd(DMA1_Channel4, DISABLE);
	DMA_Cmd(DMA1_Channel5, DISABLE);

	MPU6050_CalcAccelRot();
}

void MPU6050_CalcAccelRot(){
	accel_gyro_temp[0] = (int16_t) (i2cRxBuffer[0]<<8 | i2cRxBuffer[1]);
	accel_gyro_temp[1] = (int16_t) (i2cRxBuffer[2]<<8 | i2cRxBuffer[3]);
	accel_gyro_temp[2] = (int16_t) (i2cRxBuffer[4]<<8 | i2cRxBuffer[5]);
	accel_gyro_temp[6] = (int16_t) (i2cRxBuffer[6]<<8 | i2cRxBuffer[7]); //temp
	accel_gyro_temp[3] = (int16_t) (i2cRxBuffer[8]<<8 | i2cRxBuffer[9]);
	accel_gyro_temp[4] = (int16_t) (i2cRxBuffer[10]<<8 | i2cRxBuffer[11]);
	accel_gyro_temp[5] = (int16_t) (i2cRxBuffer[12]<<8 | i2cRxBuffer[13]);

//	gForceX = (float) accel_gyro_temp[0] / 16384.0 * 9810;
//	gForceY = (float) accel_gyro_temp[1] / 16384.0 * 9810;
//	gForceZ = (float) accel_gyro_temp[2] / 16384.0 * 9810;
	gForceX = (float) accel_gyro_temp[0] / 1.67;
	gForceY = (float) accel_gyro_temp[1] / 1.67;
	gForceZ = (float) accel_gyro_temp[2] / 1.67;

	rotX = (float) accel_gyro_temp[3] / 65.5; //131.0; gyro @250 [LSB / deg/s]
	rotY = (float) accel_gyro_temp[4] / 65.5; //65.5   gyro @500 [LSB / deg/s]
	rotZ = (float) accel_gyro_temp[5] / 65.5; //131.0;

	temp_C = (float) accel_gyro_temp[6] / 340 + 36.53;
}


/* not used at the moment */
//void TIM1_IRQHandler(void){
//	uint16_t timer_sr = TIM1->SR;
//
//	/*if(timer_sr&TIM_SR_CC1IF) //channel1
//	{
//		RB.prev_cap = RB.new_cap;
//		RB.new_cap = TIM1->CCR1;
//		if(RB.prev_cap<RB.new_cap)
//			RB.diff_cnt = RB.new_cap-RB.prev_cap;
//		else
//			RB.diff_cnt = (TIM1->ARR - RB.prev_cap) + RB.new_cap;
//	}*/
//
//
////	if(timer_sr&TIM_SR_CC2IF)/*channel2*/
////	{
////		RA.prev_cap = RA.new_cap;
////		RA.new_cap = TIM1->CCR2;
////		if(RA.prev_cap<RA.new_cap)
////			RA.diff_cnt = RA.new_cap-RA.prev_cap;
////		else
////			RA.diff_cnt = (TIM1->ARR - RA.prev_cap) + RA.new_cap;
////
////		RA.diff_ms = (float)RA.diff_cnt/(timer1_clk*(TIM1->PSC+1));/*ms; IC1PSC = 1, rising edge*/
////		RA.w = 512/(RA.diff_ms*gear_ratio);/*rad/ms*/
////		RA.v = RA.w*RA.d_wheel/2*1000;/*mm/s*/
////	}
////	else if(timer_sr&TIM_SR_CC3IF)/*channel3*/
////	{
////		LA.prev_cap = LA.new_cap;
////		LA.new_cap = TIM1->CCR3;
////		if(LA.prev_cap<LA.new_cap)
////			LA.diff_cnt = LA.new_cap-LA.prev_cap;
////		else
////			LA.diff_cnt = (TIM1->ARR - LA.prev_cap) + LA.new_cap;
////
////		LA.diff_ms = (float)LA.diff_cnt/(timer1_clk*(TIM1->PSC+1));/*IC2PSC = 1, rising edge*/
////		LA.w = 512/(LA.diff_ms*gear_ratio);/*rad/ms*/
////		LA.v = LA.w*LA.d_wheel/2*1000;/*mm/s*/
////	}
//
//	/*if(timer_sr&TIM_SR_CC4IF)//channel4
//	{
//		LB.prev_cap = LB.new_cap;
//		LB.new_cap = TIM1->CCR4;
//		if(LB.prev_cap<LB.new_cap)
//			LB.diff_cnt = LB.new_cap-LB.prev_cap;
//		else
//			LB.diff_cnt = (TIM1->ARR - LB.prev_cap) + LB.new_cap;
//	}*/
//}

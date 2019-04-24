#include "periph.h"

/* init every peripheral used,  */
void Init_Periph(void){

	Init_RCC();
	Init_SysTick();
	DelayUs(5);

	Init_MotorControl_GPIO();
	DelayUs(5);
	Init_Feedback_LEDs();
	DelayUs(5);

	Init_PTrs_IRLEDs();
	DelayUs(5);

	Init_Encoder();
	DelayUs(5);
	Init_Button0();
	DelayUs(5);

//	Init_MPU6050();
	DelayUs(10);
//	Init_MPU6050_I2C_DMA(i2cTxBuffer, i2cRxBuffer);
	DelayUs(10);

	InitTIM();
	DelayUs(2);

#ifdef _DEBUG
	InitDBG();
#endif
}


void Init_RCC(void){

	RCC_DeInit();
	RCC_ClockSecuritySystemCmd(DISABLE);

	/* disable HSE */
	RCC_HSEConfig(RCC_HSE_OFF);

	/* enable HSI, 8 Mhz */
	RCC_HSICmd(ENABLE);

	/* Wait till HSI is ready */
	while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET);

	RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_16);
	RCC_PLLCmd(ENABLE);
	while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

	FLASH_SetLatency(FLASH_Latency_1);
	FLASH_Unlock();

	/* Set PLLCLK as sys clock*/
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); /* 64 MHz */



	/* Set HCLK, PCLK1, and PCLK2 */
	RCC_HCLKConfig(RCC_SYSCLK_Div1);	/* 64 MHz - AHB  */
	RCC_PCLK1Config(RCC_HCLK_Div2); 	/* 32 MHz - APB1 */
	RCC_PCLK2Config(RCC_HCLK_Div1); 	/* 64 MHz - APB2 */

	/* Set ADC clk */
	RCC_ADCCLKConfig(RCC_PCLK2_Div6); /* 10.6 MHz */

#ifdef _DEBUG_MCO
	RCC_MCOConfig(RCC_MCO_SYSCLK); /*output SYSCLK on PA8, GPIO_Pin_PA8 must be AFIO */
#endif
}
void Init_MotorControl_GPIO(void){

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); /* frees up PB3 and PA15    - motor control */
	GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE); /* TIM2_CH1=PA15, TIM2_CH2=PB3 - motor control */

	GPIO_InitTypeDef GPIOInitStruct;
	GPIO_StructInit(&GPIOInitStruct);

	GPIOInitStruct.GPIO_Speed = GPIO_Speed_50MHz;

	/* GPIOA, afio */
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIOInitStruct.GPIO_Pin =PIN_DRV_AE;

	GPIO_Init(GPIOA, &GPIOInitStruct);

	/* GPIOB, outputs */
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIOInitStruct.GPIO_Pin= PIN_DRV_AP | PIN_DRV_BP;

	GPIO_Init(GPIOB, &GPIOInitStruct);

	/* GPIOB, afio */
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIOInitStruct.GPIO_Pin =PIN_DRV_BE;

	GPIO_Init(GPIOB, &GPIOInitStruct);


#ifdef _DEBUG_MCO
	/* MCO ouput. See init RCC for further options */
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIOInitStruct.GPIO_Pin = GPIO_Pin_8;

	GPIO_Init(GPIOA, &GPIOInitStruct);
#endif

}

/* init heartbeat and the other feedback led*/
void Init_Feedback_LEDs(){

	RCC_APB2PeriphClockCmd(LEDs_RCC_Ports, ENABLE);

	GPIO_InitTypeDef GPIOInitStruct;
	GPIO_StructInit(&GPIOInitStruct);
	GPIOInitStruct.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIOInitStruct.GPIO_Mode	= GPIO_Mode_Out_PP;
	GPIOInitStruct.GPIO_Pin		= LED0_Pin | LED1_Pin;
	GPIO_Init(LEDs_Port, &GPIOInitStruct);

}

/* init photo transistors and ir leds */
void Init_PTrs_IRLEDs(){

	RCC_APB2PeriphClockCmd(IRDiodes_RCC_Port, ENABLE);
	RCC_APB2PeriphClockCmd(PTrs_RCC_Port, ENABLE);

	/* init GPIO */
	GPIO_InitTypeDef GPIOInitStruct;
	GPIO_StructInit(&GPIOInitStruct);
	GPIOInitStruct.GPIO_Speed = GPIO_Speed_50MHz;

		/* infra leds */
		GPIOInitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIOInitStruct.GPIO_Pin  = IRDiode_RS_Pin | IRDiode_RF_Pin;
		GPIO_Init(IRDiode_R_Port, &GPIOInitStruct);

		GPIOInitStruct.GPIO_Pin  = IRDiode_LS_Pin | IRDiode_LF_Pin;
		GPIO_Init(IRDiode_L_Port, &GPIOInitStruct);


		/* photo tr */
		GPIOInitStruct.GPIO_Mode = GPIO_Mode_AIN;
		GPIOInitStruct.GPIO_Pin  = PTr_RS_Pin | PTr_RF_Pin | PTr_LF_Pin | PTr_LS_Pin;

		GPIO_Init(PTrs_Port, &GPIOInitStruct);


	/* init ADC */
	RCC_APB2PeriphClockCmd(PTrs_RCC_Periph, ENABLE);

	ADC_InitTypeDef ADC_InitStructure;
	ADC_DeInit(PTrs_ADC);
	ADC_StructInit(&ADC_InitStructure);
	/* ADC1 configuration */
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode =DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 4;
	ADC_Init(PTrs_ADC, &ADC_InitStructure);
	/* ADC1 regular channels configuration */
	ADC_RegularChannelConfig(PTrs_ADC, PTr_RS_ADC_Ch, 1, ADC_SAMPLE_TIME);
	ADC_RegularChannelConfig(PTrs_ADC, PTr_LF_ADC_Ch, 2, ADC_SAMPLE_TIME);
	ADC_RegularChannelConfig(PTrs_ADC, PTr_RF_ADC_Ch, 3, ADC_SAMPLE_TIME);
	ADC_RegularChannelConfig(PTrs_ADC, PTr_LS_ADC_Ch, 4, ADC_SAMPLE_TIME);

	ADC_DiscModeChannelCountConfig(PTrs_ADC,1);
	ADC_DiscModeCmd(PTrs_ADC,ENABLE);

	ADC_Cmd(PTrs_ADC,ENABLE);

	/* Enable ADC1 reset calibaration register */
	ADC_ResetCalibration(PTrs_ADC);
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(PTrs_ADC));
	/* Start ADC1 calibaration */
	ADC_StartCalibration(PTrs_ADC);
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(PTrs_ADC));
}

/* rising falling edge detection for encoders*/
void Init_Encoder(){

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitTypeDef GPIOInitStruct;
	GPIO_StructInit(&GPIOInitStruct);

	GPIOInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIOInitStruct.GPIO_Pin= PIN_ENC_IDX_R | PIN_ENC_LSB_R | PIN_ENC_DIR_R | PIN_ENC_LSB_L | PIN_ENC_DIR_L;

	GPIO_Init(GPIOA, &GPIOInitStruct);

	/* GPIOB, inputs */
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIOInitStruct.GPIO_Pin= PIN_ENC_IDX_L;

	GPIO_Init(GPIOB, &GPIOInitStruct);



	EXTI_InitTypeDef EXTI_InitStructure;

	/*encoderL DIR*/
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource8);
	EXTI_InitStructure.EXTI_Line=EXTI_Line8;
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/*encoderR DIR*/
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource10);
	EXTI_InitStructure.EXTI_Line = EXTI_Line10;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/*nvic*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	NVIC_InitTypeDef nvicStructure;

	/*button0+encL_DIR nvic*/
	nvicStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure.NVIC_IRQChannelSubPriority = 0;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);

	/*encR_LSB nvic*/
	nvicStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure.NVIC_IRQChannelSubPriority = 0;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);

}

/* init button*/
void Init_Button0(){

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitTypeDef GPIOInitStruct;
	GPIO_StructInit(&GPIOInitStruct);
	GPIOInitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOInitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIOInitStruct.GPIO_Pin = PIN_SW0;
	GPIO_Init(GPIOB, &GPIOInitStruct);

	EXTI_InitTypeDef EXTI_InitStructure;

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource5);
	EXTI_InitStructure.EXTI_Line = EXTI_Line5;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/*nvic*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
	NVIC_InitTypeDef nvicStructure;

	/*button0+encL_DIR nvic*/
	nvicStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 1;
	nvicStructure.NVIC_IRQChannelSubPriority = 0;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&nvicStructure);
}

/* init mpu6050*/
uint8_t Init_MPU6050(){
	/* functions from mpu6050 lib */
	MPU6050_I2C_Init();
	DelayUs(5);
	MPU6050_Initialize();
	DelayUs(5);
	return MPU6050_TestConnection(); /* returns 0 if it is working */
}

void InitTIM(void){
	InitTIM1(); // not used
	InitTIM2();
	InitTIM3();	// not used
	InitTIM4();
}

void InitDBG(){
	DBGMCU_Config(DBGMCU_TIM2_STOP, ENABLE); /* this will make TIM2 stop when core is halted during debug */
	DBGMCU_Config(DBGMCU_TIM3_STOP, ENABLE);
	DBGMCU_Config(DBGMCU_STOP, ENABLE);
}

void InitTIM1(void){

}

void InitTIM2(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;


	TIM_TimeBaseStructure.TIM_Prescaler =0 ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = MOT_PWM_T-1; /* 10kHz */
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* Channel 1, 2 Configuration in PWM mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;

	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;

	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC1Init(TIM2, &TIM_OCInitStructure);

	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OC2Init(TIM2, &TIM_OCInitStructure);

	/* TIM2 counter enable */
	TIM_Cmd(TIM2, ENABLE);

	/* TIM2 Main Output Enable */
	TIM_CtrlPWMOutputs(TIM2, ENABLE);
}

void InitTIM3(void){

}


/* timer for control loop */
void InitTIM4(void){
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	TIM_TimeBaseStructure.TIM_Prescaler = 1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 64000-1; /* 0.5kHz */
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);



	NVIC_InitTypeDef nvicStructure;

	nvicStructure.NVIC_IRQChannel = TIM4_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure.NVIC_IRQChannelSubPriority = 1;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&nvicStructure);

	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);

	/* TIM4 counter enable */
//	TIM_Cmd(TIM4, ENABLE);
}




/* eg.: MotCtl(0.12, MOT_FRW, MOT_L) */
void MotCtl(float duty,uint8_t side){
	if(duty<MOT_PWM_MAX_DUTY && duty>-MOT_PWM_MAX_DUTY){  // TODO - go condition removed from if
		// invalid. assert?
		uint16_t newC=(float)MOT_PWM_T*fabsf(duty);

		BitAction phase = MOT_FRW;

		if(duty<0)
			phase=MOT_REV;
		else
			phase=MOT_FRW;


		if(side==MOT_L){
			TIM_SetCompare2(TIM2,newC);
			GPIO_WriteBit(PORT_DRV_BP,PIN_DRV_BP,phase);
		}
		else if(side==MOT_R){
			TIM_SetCompare1(TIM2,newC);
			GPIO_WriteBit(PORT_DRV_AP,PIN_DRV_AP,phase);
		}
		else{
			//invalid. assert?
		}
	}
}


uint16_t ledPins[4]={	IRDiode_RS_Pin,
						IRDiode_LF_Pin,
						IRDiode_RF_Pin,
						IRDiode_LS_Pin};

GPIO_TypeDef* ledPorts[4]={	IRDiode_R_Port,
							IRDiode_L_Port,
							IRDiode_R_Port,
							IRDiode_L_Port};

uint16_t adcBuf[4]={0};

void readADC(void){
	for(uint8_t i=0;i<4;i++){
//		GPIO_SetBits(ledPorts[i], ledPins[i]);
		ledPorts[i]->BSRR = ledPins[i];

		ADC_SoftwareStartConvCmd(PTrs_ADC, ENABLE);

		while(ADC_GetFlagStatus(PTrs_ADC, ADC_FLAG_EOC) == RESET);   /* Wait until conversion completion */

//		GPIO_ResetBits(ledPorts[i], ledPins[i]);
		ledPorts[i]->BRR = ledPins[i];

		adcBuf[i]=PTrs_ADC->DR;
	}
}



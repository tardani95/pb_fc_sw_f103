/*
******************************************************************************
File        : periph.h
Author      : Peter Vass - modified and extended by Daniel Tar
Version     :
Copyright   :
Description : defines peripheral mapping and provides high level peripheral init and control functions
			  pins are not remapped, unless otherwise noted
Info        : 07.09.2018
*******************************************************************************
*/
#ifndef _PERIPH_H_
#define _PERIPH_H_

/*========================================================================*/
/*                             INCLUDES									  */
/*========================================================================*/
#include <math.h>
#include "conf.h"
#include "util.h"
#include "nav.h"
#include "mpu6050.h"

#include "stm32f10x.h"
#include "stm32f10x_rcc.h" /* Reset & Clock Control */
#include "stm32f10x_flash.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_i2c.h"


/* High level functions for NVIC and SysTick (add-on to CMSIS functions)
 * NVIC - Nested Vector Interrupt Controller
 */
#include "misc.h" /* NVIC configuration */
/*========================================================================*/

/* phototransitor ADC input pins  - DID NOT USE*/
/*#define  PIN_Q_RS GPIO_Pin_0
#define PORT_Q_RS GPIOA

#define  PIN_Q_RF GPIO_Pin_1
#define PORT_Q_RF GPIOA

#define  PIN_Q_LF GPIO_Pin_2
#define PORT_Q_LF GPIOA

#define  PIN_Q_LS GPIO_Pin_3
#define PORT_Q_LS GPIOA*/

/*
#define  PIN_ADC1_IN0 GPIO_Pin_0
#define  PIN_ADC1_IN1 GPIO_Pin_1
#define  PIN_ADC1_IN3 GPIO_Pin_3
#define  PIN_ADC1_IN4 GPIO_Pin_4

#define PORT_ADC1 GPIOA
*/

/* IR diode output pins, TIM3 channels */
/*
#define  PIN_D_RS GPIO_Pin_7
#define PORT_D_RS GPIOA

#define  PIN_D_RF GPIO_Pin_6
#define PORT_D_RF GPIOA

#define  PIN_D_LF GPIO_Pin_1
#define PORT_D_LF GPIOB

#define  PIN_D_LS GPIO_Pin_0
#define PORT_D_LS GPIOB
*/

/* status led outputs, inverted */
/*
#define  PIN_LED0 GPIO_Pin_8
#define PORT_LED0 GPIOB

#define  PIN_LED1 GPIO_Pin_9
#define PORT_LED1 GPIOB
*/


/* heartbeat led - inverted*/
#define LED0_Pin 			GPIO_Pin_8
/* status led - inverted*/
#define LED1_Pin 			GPIO_Pin_9
#define LEDs_Port 			GPIOB
#define LEDs_RCC_Ports		(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO)


/* ir diode pins - output push pull */
#define IRDiodes_RCC_Port	(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB)

#define IRDiode_RS_Pin		GPIO_Pin_6
#define IRDiode_RF_Pin		GPIO_Pin_7
#define IRDiode_R_Port		GPIOA

#define IRDiode_LS_Pin		GPIO_Pin_1
#define IRDiode_LF_Pin		GPIO_Pin_0
#define IRDiode_L_Port		GPIOB


/* photo transistor pins - analog input */
#define PTrs_RCC_Port		RCC_APB2Periph_GPIOA

#define PTr_RS_Pin			GPIO_Pin_1
#define PTr_RF_Pin			GPIO_Pin_0
#define PTr_LF_Pin			GPIO_Pin_4
#define PTr_LS_Pin			GPIO_Pin_3
#define PTrs_Port			GPIOA

/* ADC defines */
#define ADC_SAMPLE_TIME 	ADC_SampleTime_41Cycles5
#define PTrs_RCC_Periph		RCC_APB2Periph_ADC1
#define PTrs_ADC			ADC1
#define PTr_RS_ADC_Ch		ADC_Channel_1
#define PTr_RF_ADC_Ch		ADC_Channel_0
#define PTr_LF_ADC_Ch		ADC_Channel_4
#define PTr_LS_ADC_Ch		ADC_Channel_3

/* tactile switch input */
#define  PIN_SW0 GPIO_Pin_5
#define PORT_SW0 GPIOB

/* mpu6050 pins and settings */
#define MPU6050_I2C_RCC_DMA		RCC_AHBPeriph_DMA1
#define MPU6050_I2C_RCC_Periph	RCC_APB1Periph_I2C2
#define MPU6050_I2C_RCC_Port	( RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO )

#define MPU6050_I2C_SCL_Pin		GPIO_Pin_10
#define MPU6050_I2C_SDA_Pin		GPIO_Pin_11
#define MPU6050_I2C_Port		GPIOB

#define MPU6050_I2C_Speed		400000 // maximum bus speed 400kHz defined in datasheet
#define MPU6050_I2C				I2C2


/* motor driver outputs */
/* REMAP: AE=TIM2_CH2, BE=TIM2_CH1 */
#define  PIN_DRV_AE GPIO_Pin_15
#define PORT_DRV_AE GPIOA

#define  PIN_DRV_AP GPIO_Pin_12
#define PORT_DRV_AP GPIOB

#define  PIN_DRV_BE GPIO_Pin_3
#define PORT_DRV_BE GPIOB

#define  PIN_DRV_BP GPIO_Pin_14
#define PORT_DRV_BP GPIOB

/* encoder inputs */
#define  PIN_ENC_IDX_L GPIO_Pin_15
#define PORT_ENC_IDX_L GPIOA

#define  PIN_ENC_LSB_L GPIO_Pin_9 /* TIM1_CH2 */
#define PORT_ENC_LSB_L GPIOA

#define  PIN_ENC_DIR_L GPIO_Pin_8 /* TIM1_CH1 */
#define PORT_ENC_DIR_L GPIOA

#define  PIN_ENC_IDX_R GPIO_Pin_12
#define PORT_ENC_IDX_R GPIOA

#define  PIN_ENC_LSB_R GPIO_Pin_10 /* TIM1_CH3 */
#define PORT_ENC_LSB_R GPIOA

#define  PIN_ENC_DIR_R GPIO_Pin_11 /* TIM1_CH4 */
#define PORT_ENC_DIR_R GPIOA


/* i2c pins */
#define  PIN_SDA GPIO_Pin_11 /* TIM2_CH3 */
#define PORT_SDA GPIOB

#define  PIN_SCL GPIO_Pin_10 /* TIM2_CH4 */
#define PORT_SCL GPIOB

/* usart pins */
#define  PIN_RX GPIO_Pin_7
#define PORT_SDA GPIOB

#define  PIN_TX GPIO_Pin_6
#define PORT_SCL GPIOB


/* peripheral config defines */

/* motor defines */
#define MOT_PWM_T 			(16000) /* 4kHz @ 64MHz SYSCLK */
#define MOT_PWM_MAX_DUTY 	(0.8f)
#define MOT_FRW 			Bit_RESET /* forward */
#define MOT_REV 			Bit_SET   /* reverse */
#define MOT_L 				0 /* left */
#define MOT_R 				1 /* right */






/* external vars */

extern uint16_t adcBuf[4];
//extern __IO uint32_t SysTickCnt;
extern uint8_t go;

/* init functions */
void Init_Periph(void);

void Init_RCC(void);
void Init_MotorControl_GPIO(void);
void InitDBG(void);
void InitADC(void);

/* gpio */
void Init_Feedback_LEDs(void);

/* gpio with exti */
void Init_Encoder(void);
void Init_Button0(void);

/* gpio with adc */
void Init_PTrs_IRLEDs(void);


/* gpio with i2c */
uint8_t Init_MPU6050();




void InitTIM(void);
void InitTIM1(void);
void InitTIM2(void);
void InitTIM3(void);
void InitTIM4(void);

/* high level control functions */
void readADC(void);
void MotCtl(float duty,uint8_t side);


#endif






























































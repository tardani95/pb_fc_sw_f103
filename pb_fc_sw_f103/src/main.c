/******************************************************************************
File        : main.c
Author      : Daniel Tar
Version     : 0.3
Copyright   :
Description :
Info        : 14-09-2018
******************************************************************************/

/*==========================================================================*/
/*                               Includes								 	*/
/*==========================================================================*/
#include <stddef.h>
#include <stdio.h>
#include "stm32f10x.h"

#include "conf.h"
#include "periph.h"
#include "util.h"
#include "control.h"
#include "nav.h"
#include "maze.h"
#include "mpu6050.h"

/*==========================================================================*/
/*                            Private typedef								*/
/*==========================================================================*/


/*==========================================================================*/
/*                            Private define								*/
/*==========================================================================*/
//#define DEBUG

/*==========================================================================*/
/*                            Private macro									*/
/*==========================================================================*/


/*==========================================================================*/
/*                            Private variables								*/
/*==========================================================================*/


/*==========================================================================*/
/*                      Private function prototypes							*/
/*==========================================================================*/


/*==========================================================================*/
/*                          Private functions								*/
/*==========================================================================*/
#ifdef DEBUG
	#include "stm32f10x_dbgmcu.h"
#endif





/*==========================================================================*/
/*  							Main program								*/
/*==========================================================================*/
int main(void){

	/* Setup STM32 system (clock, PLL and Flash configuration) */
	SystemInit();

#ifdef DEBUG
	InitDBG();
#endif

	Init_Periph();
	DelayUs(2);
	Init_EEPROM();
	DelayUs(2);
	InitNav();




	/* TODO - Add your application code here */


//	DelayUs(500);  /* delay ms  not us*/
//	MPU6050_DMAGetRawAccelGyro();
//	DelayUs(2);
//	MPU6050_DMAGetRawAccelGyro();
//	DelayUs(2);
//	MPU6050_DMAGetRawAccelGyro();
//	DelayUs(2);

//	LEDs_Port->BSRR |= LED0_Pin;
	/* Infinite loop */
	while (1){
//		MPU6050_GetRawAccelGyro( &accel_gyro_values );
//		LEDs_Port->BRR |= LED0_Pin;
//		readADC();
//		LEDs_Port->BSRR |= LED0_Pin;
//		DelayUs(1);
	}
}




/*==========================================================================*/
/*                            Assert functions								*/
/*==========================================================================*/
#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


/*
 * Minimal __assert_func used by the assert() macro
 * */
/*extern "C" void __assert_func(const char *file, int line, const char *func, const char *failedexpr)
{
  while(1)
  {}
}*/

/*
 * Minimal __assert() uses __assert__func()
 * */
/*extern "C" void __assert(const char *file, int line, const char *failedexpr)
{
   __assert_func (file, line, NULL, failedexpr);
}*/


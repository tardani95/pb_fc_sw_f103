/*
******************************************************************************
File        : util.h
Author      : Daniel Tar
Version     :
Copyright   :
Description :
Info        : 07-09-2018
*******************************************************************************
*/

#ifndef _UTIL_H_
#define _UTIL_H_

/*========================================================================*/
/*                             INCLUDES									  */
/*========================================================================*/
#include "conf.h"

#include "stm32f10x.h"
#include "stm32f10x_rcc.h" /* Reset & Clock Control */
/*========================================================================*/

#define SYS_TICK_FREQ (1000) /* frequency of sys_tick event in Hz - used by InitSystick() function */

//extern __IO uint32_t SysTickCnt; /* make this available in all files that include periph */

void SysTick_Handler(void);
void Init_SysTick(void);
uint32_t GetSysTickCount(void);
uint32_t Millis(void);
void DelayUs(uint32_t us);
void DelayMs(uint32_t ms);
void DelaySec(uint32_t sec);


#endif /* _UTIL_H_ */

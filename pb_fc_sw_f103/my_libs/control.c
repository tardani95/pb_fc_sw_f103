#include "control.h"
#include "nav.h"

/* control loop @1kHz */
#define RS 0
#define LS 3
#define RF 2
#define LF 1

//float T=0.001; // @1   kHz
float T=0.002; // @0.5 kHz

float Kp=0.2; /* P */ //1.5
float Ki=0.0; /* I */ // 0.05
float Kd=0.0; /* D */ //0.1

float e=0;
float e_d=0;

float e_max=2500.0;
//float e_max = 4000.0;
float I=0;
float D=0;

float Imax=1.0; /* anti wind-up */


float v_max=3.0;

float v_base=0.0; //0.4

#define N_AVG 10

float s=0;

float r=0;
float v=0;
float a=0;

/* control loop @1kHz */
void TIM4_IRQHandler(void){

	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET){
        TIM_ClearITPendingBit(TIM4,  TIM_IT_Update);

        /*switch(controll_mode){

        case MOVE_FWD :{

        }break;

        case TURN :{

        }break;

        case MOVE_X_CELL :{

        }break;

        }*/

//        LEDs_Port->BSRR |= LED0_Pin;
//		MPU6050_DMAGetRawAccelGyro();
		readADC();
//		MPU6050_CalcAccelRot();
//		LEDs_Port->BRR |= LED0_Pin;

        /*test pid*/


        e_d=e;

//        e=(ENC_GetLeftValue()-ENC_GetRightValue())/e_max; // L-R
        e = (adcBuf[LS] - adcBuf[RS]) / e_max;

        D=Kd*(e-e_d)/T;

        I+=Ki*e;
        if(I>Imax)
        	I=Imax;
        if(I<-Imax)
        	I=-Imax;
        I=0;

        s=Kp*e+I+D;


        v=v_max*s/3.5;


//		MotCtl(v_base+v,MOT_L);
//		MotCtl(v_base-v,MOT_R);

		if( ENC_GetLeftValue() > 1000000){
			TIM_Cmd(TIM4,DISABLE); // stops the control loop
			v_base = 0;
			MotCtl(0, MOT_L);
			MotCtl(0, MOT_R);

			goState = 0;
		}
   }
}

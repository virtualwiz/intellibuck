#include "stm32f10x.h"
#include "system_stm32f10x.h"
#include "stm32f10x_adc.h"
#include "stm32f10x_dma.h"

#define Relay_Close() GPIO_SetBits(GPIOB , GPIO_Pin_7)
#define Relay_Open() GPIO_ResetBits(GPIOB , GPIO_Pin_7)

#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "adc.h"
#include "tsensor.h"
#include "pwm.h"
#include "bsp.h"
#include "timer.h"

#define PID_ILIMIT 40000

float IVal = 0;
float DOut = 0;

int PID_Exec(int PID_In, int PID_Target , float kP,float kI,float kD)
{
	float POut = 0;
	int PID_Out;
	static int lastValue = 0;
	
	POut = (PID_Target - PID_In) * kP;
	IVal += (PID_Target - PID_In) * kI;
	DOut = (lastValue - PID_In) * kD;
	lastValue = PID_In;

	if(IVal >= PID_ILIMIT)
		IVal = PID_ILIMIT;
	if(IVal <= -PID_ILIMIT) 
		IVal = -PID_ILIMIT;
		
	PID_Out = (int)(POut + IVal+ DOut);
	return PID_Out;
}

int DBG_DUTY;
float DBG_KP = 0;
float DBG_KI = 0;
float DBG_KD = 0;



void delay(unsigned int cycles)
{
	unsigned int t;
	for(t = 0 ; t <= cycles ; t++);
}

char lastMode;
char modeChangedFlag = 0;
char runPidAlgorithmFlag = 0;

//4 freq 5 mode 6 hv
int main(void){
	
	BSP_InitAll();
	Timer_Init(0,0);
	TIM4_Int_Init(9,71);
	while(1)
	{
		if(runPidAlgorithmFlag)
		{
			runPidAlgorithmFlag = 0;
			ADC_Transfer();
			if(BSP_StatusStructure.sw_Contactor == Contactor_On)
			{
				if(lastMode == 0)
				{
					modeChangedFlag = 1;
				}
				lastMode = 1;
				Relay_Close();
				
				if(GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_4) == 0)
				{
					if(BSP_StatusStructure.status == INF_CV)
					{
						BSP_SetLEDColor(0,0,1);
						DBG_DUTY = map(
							PID_Exec(BSP_StatusStructure.outVoltage , 
							BSP_StatusStructure.setVoltage * map(BSP_StatusStructure.setAmpere , 0,2000,0,5)/5,
							 3,0.1,1),
						0 , 40000 , 0 , 40000);
						Timer_Config(40000 , DBG_DUTY);
					}
				
					if(BSP_StatusStructure.status == INF_CC)
					{
						BSP_SetLEDColor(0,0,1);
						DBG_DUTY = map(
							PID_Exec(BSP_StatusStructure.outAmpere 
							, BSP_StatusStructure.setAmpere * map(BSP_StatusStructure.setVoltage , 0,30000,0,5)/5 ,
							 2.0 , 0.6 , 0.7),
						0 , 40000 , 0 , 40000);
						Timer_Config(40000 , DBG_DUTY);
					}					
				}

				else
				{
					if(BSP_StatusStructure.status == INF_CV)
					{
						BSP_SetLEDColor(0,1,0);
						DBG_DUTY = map(
							PID_Exec(BSP_StatusStructure.outVoltage , BSP_StatusStructure.setVoltage , 3,0.1,1),
						0 , 40000 , 0 , 40000);
						Timer_Config(40000 , DBG_DUTY);
					}
				
					if(BSP_StatusStructure.status == INF_CC)
					{
						BSP_SetLEDColor(0,1,1);
						DBG_DUTY = map(
							PID_Exec(BSP_StatusStructure.outAmpere , BSP_StatusStructure.setAmpere , 2.0 , 0.6 , 0.7),
						0 , 40000 , 0 , 40000);
						Timer_Config(40000 , DBG_DUTY);
						// 2.0 , 0.6 , 0.7
						//DBG_KP,DBG_KI,DBG_KD
					}
				}
				if(DBG_DUTY >= 38000)
				{
					BSP_SetLEDColor(1,0,0);
					BSP_StatusStructure.status = WRN_OLOAD;
				}

				if(DBG_DUTY == 40000)
				{
					BSP_SetLEDColor(1,0,0);
					BSP_StatusStructure.status = ERR_OLOAD;
					Relay_Open();
					while(GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_6) != Contactor_Off);
				}

				if(BSP_StatusStructure.temperature >= 60)
				{
					BSP_SetLEDColor(1,0,0);
					BSP_StatusStructure.status = WRN_TEMP;
				}

				if(BSP_StatusStructure.temperature >= 70)
				{
					BSP_SetLEDColor(1,0,0);
					BSP_StatusStructure.status = ERR_TEMP;
					Relay_Open();
					while(GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_6) != Contactor_Off);
				}				
			}
			else
			{
				if(lastMode == 1)
				{
					modeChangedFlag = 1;
				}
				lastMode = 0;
				Relay_Open();
			}

			if(modeChangedFlag)
			{
				modeChangedFlag = 0;
				IVal = 0;
			}
		}
	}
}

unsigned int irqCounter = 0;

void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET) 
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
		irqCounter++;
		if(irqCounter % 98 == 0)
		{
			runPidAlgorithmFlag = 1;
		}
		if(irqCounter % 46912 == 0)
		{
			BSP_StatusStructure.energy += BSP_StatusStructure.outVoltage * BSP_StatusStructure.outAmpere
			* 3.27777777e-8;
			LCD_PrepareData();
			LCD_SendData();
			//irqCounter = 0;
		}
	}
}

#define INF_WAIT	0
#define INF_CC		1
#define INF_CV		2

#define WRN_OLOAD	3
#define WRN_TEMP	4

#define ERR_OLOAD	5
#define ERR_TEMP	6
#define ERR_FAULT	7

#define Contactor_On 0
#define Contactor_Off 1
#define FrequencyMode_High 0
#define FrequencyMode_Low 1
#define DriveMode_CC 0
#define DriveMode_CV 1

#define ADC1_DR_Address    ((u32)0x4001244C) 

vu16 AD_Value[6];

#define LOWPASS_WINDOW_LENGTH_1 10
#define LOWPASS_WINDOW_LENGTH_2 10
#define LOWPASS_WINDOW_LENGTH_3 10

int value_buf_1[LOWPASS_WINDOW_LENGTH_1]; 
int value_buf_2[LOWPASS_WINDOW_LENGTH_2];
int value_buf_3[LOWPASS_WINDOW_LENGTH_3];

int Filter_1(int Data_In) 
{
	static char i = 0;
	char count; 
	int sum=0; 
	value_buf_1[i++] = Data_In; 
	if ( i == LOWPASS_WINDOW_LENGTH_1 )   
		i = 0;
	for ( count=0 ; count < LOWPASS_WINDOW_LENGTH_1 ; count++) 
		sum += value_buf_1[count];
	return (int)(sum/LOWPASS_WINDOW_LENGTH_1); 
}

int Filter_2(int Data_In) 
{
	static char i = 0;
	char count; 
	int sum=0; 
	value_buf_2[i++] = Data_In; 
	if ( i == LOWPASS_WINDOW_LENGTH_2 )   
		i = 0;
	for ( count=0 ; count < LOWPASS_WINDOW_LENGTH_2 ; count++) 
		sum += value_buf_2[count];
	return (int)(sum/LOWPASS_WINDOW_LENGTH_2); 
}

int Filter_3(int Data_In) 
{
	static char i = 0;
	char count; 
	int sum=0; 
	value_buf_3[i++] = Data_In; 
	if ( i == LOWPASS_WINDOW_LENGTH_3 )   
		i = 0;
	for ( count=0 ; count < LOWPASS_WINDOW_LENGTH_3 ; count++) 
		sum += value_buf_3[count];
	return (int)(sum/LOWPASS_WINDOW_LENGTH_3); 
}

typedef struct
{
	int setVoltage;
	int setAmpere;
	int outVoltage;
	int outAmpere;
	int inAmpere;
	unsigned char effeciency;
	float energy;
	unsigned char temperature;
	unsigned char status;
	
	unsigned char sw_Contactor;
	unsigned char sw_FrequencyMode;
	unsigned char sw_DriveMode;
	
}	BSP_StatusTypeDef;

BSP_StatusTypeDef BSP_StatusStructure;
char serialTransmitBuffer[31];  //USART Send data buffer

void NVIC_Configuration(void) 
{ 
//    NVIC_InitTypeDef NVIC_InitStructure; 

#ifdef  VECT_TAB_RAM 
    // Set the Vector Table base location at 0x20000000 
    NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0); 
#else  /* VECT_TAB_FLASH  */ 
    // Set the Vector Table base location at 0x08000000 
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0); 
#endif 

    //设置NVIC优先级分组为Group2：0-3抢占式优先级，0-3的响应式优先级 
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
} 

void IO_Init()
{
	GPIO_InitTypeDef  GPIO_InitStructure;
 	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_4 | GPIO_Pin_5;  //4 freq 5 mode 6 hv
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 ;   //PB7 Relay Output
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB , GPIO_Pin_7);

}

void Timer_Init(u16 arr,u16 psc)
{    
	RCC->APB2ENR|= 1<<2;			//GPIOAÊ±ÖÓÊ¹ÄÜ
	RCC->APB1ENR |=1<<1;       //TIM3Ê±ÖÓÊ¹ÄÜ     
	GPIOA->CRL&=0X0FFFFFFF;//PA7Êä³ö
	GPIOA->CRL|=0XB0000000;//¸´ÓÃ¹¦ÄÜÊä³ö PWMÄ£Ê½ 
		
	TIM3->ARR=arr;  //Éè¶¨¼ÆÊýÆ÷×Ô¶¯ÖØ×°Öµ 
	TIM3->PSC=psc;  //Ô¤·ÖÆµÆ÷
	
	TIM3->CCMR1|=7<<12;  //CH2 PWM2Ä£Ê½£¨¸ßµçÆ½ÎªÕ¼¿Õ±È£©  
	TIM3->CCMR1|=1<<11; //CH2Ô¤×°ÔØÊ¹ÄÜ    
	TIM3->CCER|=1<<4;   //OC2 Êä³öÊ¹ÄÜ    
	TIM3->CR1=0x8000;   //ARPEÊ¹ÄÜ 
	TIM3->CR1|=0x01;    //¿ª¶¨Ê±Æ÷3    
}       

void Timer_Config(int dARR,int dCCR2)
{
	TIM3->CNT = 0;
	//TIM3->ARR = 100000 / dARR;
	//TIM3->CCR2 = (1000 / dARR) * dCCR2;
	TIM3->ARR = dARR;
	TIM3->CCR2 = dCCR2;
}

void AD_DMA_Init()
{
	ADC_InitTypeDef ADC_InitStructure; 
	DMA_InitTypeDef DMA_InitStructure; 
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); 
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); 

    DMA_DeInit(DMA1_Channel1); 
    DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address; 
    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&AD_Value; 
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; 
    //BufferSize=2，因为ADC转换序列有2个通道 
    //如此设置，使序列1结果放在AD_Value[0]，序列2结果放在AD_Value[1] 
    DMA_InitStructure.DMA_BufferSize = 6; 
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; 
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; 
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; 
    //循环模式开启，Buffer写满后，自动回到初始地址开始传输 
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; 
    DMA_InitStructure.DMA_Priority = DMA_Priority_High; 
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 
    DMA_Init(DMA1_Channel1, &DMA_InitStructure); 
    //配置完成后，启动DMA通道 
    DMA_Cmd(DMA1_Channel1, ENABLE); 

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent; 
    ADC_InitStructure.ADC_ScanConvMode = ENABLE; 
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;  //连续转换开启 
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None; 
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right; 
    ADC_InitStructure.ADC_NbrOfChannel = 6;
    ADC_Init(ADC1, &ADC_InitStructure); 
    
    //ADC内置温度传感器使能
    ADC_TempSensorVrefintCmd(ENABLE); 
    
    //常规转换序列1：通道10 
 	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_239Cycles5); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_239Cycles5); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_239Cycles5); 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 5, ADC_SampleTime_239Cycles5); 
    //常规转换序列2：通道16（内部温度传感器），采样时间>2.2us,(239cycles) 
    ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 6, ADC_SampleTime_239Cycles5); 
    
    // Enable ADC1 
    ADC_Cmd(ADC1, ENABLE); 
    // 开启ADC的DMA支持（要实现DMA功能，还需独立配置DMA通道等参数） 
    ADC_DMACmd(ADC1, ENABLE); 
    
    // 下面是ADC自动校准，开机后需执行一次，保证精度 
    // Enable ADC1 reset calibaration register 
    ADC_ResetCalibration(ADC1); 
    // Check the end of ADC1 reset calibration register 
    while(ADC_GetResetCalibrationStatus(ADC1)); 

    // Start ADC1 calibaration 
    ADC_StartCalibration(ADC1); 
    // Check the end of ADC1 calibration 
    while(ADC_GetCalibrationStatus(ADC1)); 
    // ADC自动校准结束--------------- 
}

void BSP_InitAll()
{
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_Configuration();
	IO_Init();
	delay_init();	    	 //延时函数初始化	  
	AD_DMA_Init();
	uart_init(115200);
	ADC_SoftwareStartConvCmd(ADC1, ENABLE); 
}

int map(int value ,int xmin ,int xmax ,int ymin ,int ymax)
{
	float k,b;
	if(value <= xmin) value = xmin;
	if(value >= xmax) value = xmax;
	k = ((float)(ymax - ymin) / (float)(xmax - xmin));
	//k = 9.0 / 2.0;
	b = (ymin - xmin * k);
	return (int)(k * value + b);
}

void BSP_SetLEDColor(char r,char g,char b)
{
	if(r)
	GPIO_SetBits(GPIOB,GPIO_Pin_11);
	else
	GPIO_ResetBits(GPIOB,GPIO_Pin_11);
	if(g)
	GPIO_SetBits(GPIOB,GPIO_Pin_12);
	else
	GPIO_ResetBits(GPIOB,GPIO_Pin_12);
	if(b)
	GPIO_SetBits(GPIOB,GPIO_Pin_13);
	else
	GPIO_ResetBits(GPIOB,GPIO_Pin_13);
}

void ADC_Transfer()
{
	BSP_StatusStructure.sw_Contactor = GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_6);
	BSP_StatusStructure.sw_FrequencyMode = GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_4);
	BSP_StatusStructure.sw_DriveMode = GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_5);
	BSP_StatusStructure.setVoltage = map(Filter_1(AD_Value[0]) , 20 , 4070 , 0 , 30000);
	BSP_StatusStructure.setAmpere = map(Filter_2(AD_Value[1]) , 20 , 4070 , 0 , 2000);
	BSP_StatusStructure.outVoltage = map(AD_Value[2] , 0 , 4095 , 0 , 30000);
	BSP_StatusStructure.outAmpere = map(AD_Value[3] , 0 , 4095 , 0 , 2000);
	BSP_StatusStructure.inAmpere =  map(AD_Value[4] , 0 , 4095 , 0 , 999);

	BSP_StatusStructure.effeciency = Filter_3((float)BSP_StatusStructure.outVoltage
	 * (float)BSP_StatusStructure.outAmpere / 10000 / map(BSP_StatusStructure.inAmpere,300,400,0,20) * 1.7);
	if(GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_5) == DriveMode_CC)
	{
		BSP_StatusStructure.status = INF_CC;
	}
	else 
	{
		BSP_StatusStructure.status = INF_CV;
	}
	if(GPIO_ReadInputDataBit(GPIOB , GPIO_Pin_6) == Contactor_Off)
	{
		BSP_StatusStructure.status = INF_WAIT;
		BSP_SetLEDColor(0,0,0);
	}
	BSP_StatusStructure.temperature = (1.43 - AD_Value[5] * 3.3 / 4096) * 1000 / 4.35 + 255;

}

void LCD_PrepareData()
{
	serialTransmitBuffer[0] = '$';
	
	serialTransmitBuffer[1] = BSP_StatusStructure.setVoltage / 10000 + 0x30;
	serialTransmitBuffer[2] = BSP_StatusStructure.setVoltage % 10000 / 1000 + 0x30;
	serialTransmitBuffer[3] = BSP_StatusStructure.setVoltage % 1000 / 100 + 0x30;
	serialTransmitBuffer[4] = BSP_StatusStructure.setVoltage % 100 / 10 + 0x30;
	serialTransmitBuffer[5] = BSP_StatusStructure.setVoltage % 10 + 0x30;
	
	serialTransmitBuffer[6] = BSP_StatusStructure.setAmpere / 1000 + 0x30;
	serialTransmitBuffer[7] = BSP_StatusStructure.setAmpere % 1000 / 100 + 0x30;
	serialTransmitBuffer[8] = BSP_StatusStructure.setAmpere % 100 / 10 + 0x30;
	serialTransmitBuffer[9] = BSP_StatusStructure.setAmpere % 10 + 0x30;
	
	serialTransmitBuffer[10] = BSP_StatusStructure.outVoltage / 10000 + 0x30;
	serialTransmitBuffer[11] = BSP_StatusStructure.outVoltage % 10000 / 1000 + 0x30;
	serialTransmitBuffer[12] = BSP_StatusStructure.outVoltage % 1000 / 100 + 0x30;
	serialTransmitBuffer[13] = BSP_StatusStructure.outVoltage % 100 / 10 + 0x30;
	serialTransmitBuffer[14] = BSP_StatusStructure.outVoltage % 10 + 0x30;
	
	serialTransmitBuffer[15] = BSP_StatusStructure.outAmpere / 1000 + 0x30;
	serialTransmitBuffer[16] = BSP_StatusStructure.outAmpere % 1000 / 100 + 0x30;
	serialTransmitBuffer[17] = BSP_StatusStructure.outAmpere % 100 / 10 + 0x30;
	serialTransmitBuffer[18] = BSP_StatusStructure.outAmpere % 10 + 0x30;
	
	serialTransmitBuffer[19] = BSP_StatusStructure.inAmpere / 100 + 0x30;
	serialTransmitBuffer[20] = BSP_StatusStructure.inAmpere % 100 / 10 + 0x30;
	serialTransmitBuffer[21] = BSP_StatusStructure.inAmpere % 10 + 0x30;
	
	serialTransmitBuffer[22] = BSP_StatusStructure.effeciency / 10 + 0x30;
	serialTransmitBuffer[23] = BSP_StatusStructure.effeciency % 10 + 0x30;
	
	serialTransmitBuffer[24] = (int)BSP_StatusStructure.energy / 1000 + 0x30;
	serialTransmitBuffer[25] = (int)BSP_StatusStructure.energy % 1000 / 100 + 0x30;
	serialTransmitBuffer[26] = (int)BSP_StatusStructure.energy % 100 / 10 + 0x30;
	serialTransmitBuffer[27] = (int)BSP_StatusStructure.energy % 10 + 0x30;
	
	serialTransmitBuffer[28] = BSP_StatusStructure.temperature / 10 + 0x30;
	serialTransmitBuffer[29] = BSP_StatusStructure.temperature % 10 + 0x30;
	
	serialTransmitBuffer[30] = BSP_StatusStructure.status + 0x30;
		
}
 
void LCD_SendData()
{
	int dataPtr;
	for(dataPtr=0;dataPtr<=30;dataPtr++)
	{
		//USART1->DR=serialTransmitBuffer[dataPtr];
		printf("%c",serialTransmitBuffer[dataPtr]);
		while((USART1->SR&0X40)==0);//等待发送结束
	}
}


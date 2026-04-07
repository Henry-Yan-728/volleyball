/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */
uint32_t ADC_Value[80]={0};
uint32_t X_Value=0, Y_Value=0,C_Value=0;
uint32_t X_Ref=0,Y_Ref=0,C_Ref=0;
float	Bat_Value=0;
uint8_t i=0;
int X_Value1=0,Y_Value1=0,C_Value1=0;	
int test=0;
void ProcessADCData()
{
	TX_buffer[34]=0;
	for (i=0,X_Value=Y_Value=C_Value=Bat_Value=0;i<20;)
			{
					X_Value += ADC_Value[i++];
					Y_Value += ADC_Value[i++];
					C_Value += ADC_Value[i++];
				  Bat_Value += ADC_Value[i++];
			}
			X_Value=(1320-660*(X_Value/15.51515)/X_Ref);
			Y_Value=1320-660*(Y_Value/15.51515)/Y_Ref;
			C_Value=660*(C_Value/15.51515)/C_Ref;
			Bat_Value=Bat_Value/2068.68685;

//remote1
			if(V_Flag_1==0)
			{
					if(X_Value<720&&X_Value>590)
					{
						X_Value1=660;
					}
					else if(X_Value<591)
					{
						X_Value=X_Value+70;
						X_Value1=X_Value-(660-X_Value)/1.98;
					}
					else if(X_Value>719)
					{
						X_Value=X_Value-60;
						X_Value1=X_Value+(X_Value-660)/2.27;
					}
					if(X_Value1>1319){
						X_Value1=1320;
					}
					else if(X_Value1<1){
							X_Value1=0;
						}
					if(Y_Value<715&&Y_Value>620)
					{
						Y_Value1=660;
					}
					else if(Y_Value<621)
					{
						Y_Value=Y_Value+40;
						Y_Value1=Y_Value-(660-Y_Value)/1.95;
					}
					else if(Y_Value>714)
					{
						Y_Value=Y_Value-55;
						Y_Value1=Y_Value+(Y_Value-660)/2.52;
					}
					if(Y_Value1>1319){
						Y_Value1=1320;
					}
					else if(Y_Value1<1){
							Y_Value1=0;
						}
////////���ڸ��õ��������֣���С���������ҷ����������������
//////					if(X_Value<670&&X_Value>650)
//////					{
//////						X_Value1=660;
//////					}
//////					else if(X_Value<651)
//////					{
//////						X_Value1=X_Value;
//////					}
//////					else if(X_Value>669)
//////					{
//////						X_Value1=X_Value;
//////					}
//////					if(X_Value1>1319){
//////						X_Value1=1320;
//////					}
//////					else if(X_Value1<1){
//////							X_Value1=0;
//////						}
//////					if(Y_Value<670&&Y_Value>650)
//////					{
//////						Y_Value1=660;
//////					}
//////					else if(Y_Value<651)
//////					{
//////						Y_Value1=Y_Value;
//////					}
//////					else if(Y_Value>669)
//////					{
//////						Y_Value1=Y_Value;
//////					}
//////					if(Y_Value1>1319){
//////						Y_Value1=1320;
//////					}
//////					else if(Y_Value1<1){
//////							Y_Value1=0;
//////						}

					if(C_Value<715&&C_Value>645)
					{
						C_Value1=660;
					}
					else if(C_Value<646)
					{
						C_Value1=C_Value/0.97727;
					}
					else if(C_Value>714)
					{
						C_Value1=C_Value-55;
					//C_Value1=C_Value1/0.91666;
					}
					if(C_Value1>1319){
						C_Value1=1319;
					}
					else if(C_Value1<1){
							C_Value1=0;
						}
//////			x_stick=X_Value1/13.2;
//////			y_stick=Y_Value1/13.2;
//////			z_stick=C_Value1/13.2;
//////			Printf(&huart4,"check.j0.val=%d\xff\xff\xff",x_stick);//׼ȷx��ʾ
//////			Printf(&huart4,"check.j1.val=%d\xff\xff\xff",y_stick);//׼ȷy��ʾ
//////			Printf(&huart4,"check.j2.val=%d\xff\xff\xff",z_stick);//׼ȷz��ʾ
//////			x_stick=X_Value1-660;
//////			y_stick=Y_Value1-660;
//////			z_stick=C_Value1-660;
//////			Printf(&huart4,"check.n0.val=%d\xff\xff\xff",x_stick);//׼ȷx��ʾ
//////			Printf(&huart4,"check.n1.val=%d\xff\xff\xff",y_stick);//׼ȷy��ʾ
//////			Printf(&huart4,"check.n2.val=%d\xff\xff\xff",z_stick);//׼ȷz��ʾ
			}
			TX_buffer[1]=X_Value1/1000;
			TX_buffer[2]=(X_Value1%1000)/100;
			TX_buffer[3]=(X_Value1%100)/10;
			TX_buffer[4]=X_Value1%10;
			TX_buffer[5]=Y_Value1/1000;
			TX_buffer[6]=(Y_Value1%1000)/100;
			TX_buffer[7]=(Y_Value1%100)/10;
			TX_buffer[8]=Y_Value1%10;
			TX_buffer[9]=C_Value1/1000;
			TX_buffer[10]=(C_Value1%1000)/100;
			TX_buffer[11]=(C_Value1%100)/10;
			TX_buffer[12]=C_Value1%10;
////			TX_buffer[23]=__HAL_TIM_GetCounter(&htim1);
//			TX_buffer[34]=0;
//			for(i=1;i<=23;i++)
//			{
//				TX_buffer[34]=TX_buffer[34]+TX_buffer[i];
//			}
//			Printf(&huart3,"%c%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%c\n",TX_buffer[0],TX_buffer[1],TX_buffer[2],TX_buffer[3],TX_buffer[4],TX_buffer[5],TX_buffer[6],TX_buffer[7],TX_buffer[8],TX_buffer[9],TX_buffer[10],TX_buffer[11],TX_buffer[12],TX_buffer[13],TX_buffer[14],TX_buffer[15],TX_buffer[16],TX_buffer[17],TX_buffer[18],TX_buffer[19],TX_buffer[20],TX_buffer[21],TX_buffer[22],TX_buffer[23],TX_buffer[34]);


}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc == &hadc1 && test!=1) {
        // �ڴ˴���ADC���ݣ�ADC_Value����ȫ���£�
//        Printf(&huart3,"%d  %d  %d    ",ADC_Value[0], ADC_Value[1], ADC_Value[2]);
			ProcessADCData();
    }
		else
		{
			for (i = 0,X_Value=Y_Value=C_Value=0; i < 20;)
			{
					X_Value += ADC_Value[i++];
					Y_Value += ADC_Value[i++];
					C_Value += ADC_Value[i++];
					Bat_Value += ADC_Value[i++];
			}
			X_Ref=X_Value/15.51515;
			Y_Ref=Y_Value/15.51515;
			C_Ref=C_Value/15.51515;
			Bat_Value=Bat_Value/2068.68685;
			test = 0;
			BEEP();
			BEEP();
			BEEP();
			BEEP();
			BEEP();
		}
}

//ADCУ׼
void ADC_adjust(void)
{
	test = 1;
	HAL_ADC_Start_DMA(&hadc1, ADC_Value, 80);//ADC_DMA ����
	
}
void OPEN_adjust(void)//����У��
{
	ADC_adjust();
//////	Printf(&huart1,"setting.t0.txt=\"%.2f\"\xff\xff\xff",Bat_Value);//׼ȷ��ѹ��ʾ
//////	Printf(&huart1,"check.j0.val=%d\xff\xff\xff",50);//׼ȷ��ѹ��ʾ
//////	Printf(&huart1,"check.j1.val=%d\xff\xff\xff",50);//׼ȷ��ѹ��ʾ
//////	Printf(&huart1,"check.j2.val=%d\xff\xff\xff",50);//׼ȷ��ѹ��ʾ
//////			if(Bat_Value>8.0&&Bat_Value<8.4)//���
//////			{
//////			Printf(&huart1,"main.p1.pic=14\xff\xff\xff");
//////			}
//////			else if(Bat_Value>7.8&&Bat_Value<8.0)//�ĸ�
//////			{
//////			Printf(&huart1,"main.p1.pic=13\xff\xff\xff");
//////			}
//////			else if(Bat_Value>7.6&&Bat_Value<7.8)//����
//////			{
//////			Printf(&huart1,"main.p1.pic=12\xff\xff\xff");
//////			}
//////			else if(Bat_Value>7.5&&Bat_Value<7.6)//����
//////			{
//////			Printf(&huart1,"main.p1.pic=11\xff\xff\xff");
//////			}
//////			else if(Bat_Value>7.3&&Bat_Value<7.5)//һ��
//////			{
//////			Printf(&huart1,"main.p1.pic=10\xff\xff\xff");
//////			}
//////			else if(Bat_Value>7&&Bat_Value<7.3)//���
//////			{
//////			Printf(&huart1,"main.p1.pic=9\xff\xff\xff");
//////			BEEP ();
//////			}
}

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */
  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */
  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA2     ------> ADC1_IN2
    PA3     ------> ADC1_IN3
    PA5     ------> ADC1_IN5
    PA6     ------> ADC1_IN6
    */
    GPIO_InitStruct.Pin = X_Pin|Y_Pin|GPIO_PIN_5|C_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc1.Init.Mode = DMA_NORMAL;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);

  /* USER CODE BEGIN ADC1_MspInit 1 */
  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */
  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA2     ------> ADC1_IN2
    PA3     ------> ADC1_IN3
    PA5     ------> ADC1_IN5
    PA6     ------> ADC1_IN6
    */
    HAL_GPIO_DeInit(GPIOA, X_Pin|Y_Pin|GPIO_PIN_5|C_Pin);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);
  /* USER CODE BEGIN ADC1_MspDeInit 1 */
  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/* USER CODE END 1 */

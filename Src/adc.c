/**
  ******************************************************************************
  * File Name          : ADC.c
  * Description        : This file provides code for the configuration
  *                      of the ADC instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2021 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "variable.h"

#define OFF_VALUE 0
#define SIDE_VALUE 1
#define FRONT_VALUE 2
#define FINISH_CONVERT 3

#define ADC_CONVERT_DATA_SIZE 8

extern uint16_t ADCBuff[ADC_CONVERT_DATA_SIZE];     // AD変換の結果を保持するバッファ
extern uint16_t ADCOffData[ADC_CONVERT_DATA_SIZE];   // AD変換の消した時の値を保持
extern uint16_t ADCOntData[ADC_CONVERT_DATA_SIZE];   // AD変換の光って??��?��?るとき�???��?��値を保持
extern int16_t adc_counter;                          // AD変換のタイミングを指??��?��?

static uint8_t ctr_irled = 0;
/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
  
    /**ADC1 GPIO Configuration    
    PB1     ------> ADC1_IN9 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
    PB1     ------> ADC1_IN9 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_1);

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
///////////////////////////////////////////////////////////////////////
// batt voltage calculation
// [argument] nothinh
// [Substitutiong] batt_voltage
// [return] nothing
///////////////////////////////////////////////////////////////////////
float battMonitor( void )
{
  int16_t batt_analog = 0;
  float batt_voltage;

  HAL_ADC_Start( &hadc1 );
  HAL_ADC_PollForConversion( &hadc1,100 );  // trans
  batt_analog = HAL_ADC_GetValue( &hadc1 );   // get value
  HAL_ADC_Stop( &hadc1 );
  
  batt_voltage = (float)( batt_analog / 4095.0f * 2.0f * 3.3f );

  return batt_voltage;
}

void battChecker(void){
  float batt=0;
  for(int i=0;i<10;i++){
    batt+=battMonitor();
  }
  batt/=10.0f;
  printf("batt_voltage:%f\r\n",batt);
  if(batt < 3.6f){
    while(1){
      Led(15);
      HAL_Delay(500);
      Led(0);
      HAL_Delay(500);
    }
  }
}

///////////////////////////////////////////////////////////////////////
// able ir led
// [argument] ired 1:paluse emit ON 0:OFF
// [Substitutiong] nothing
// [return] nothing
///////////////////////////////////////////////////////////////////////
void setIrledPwm( uint8_t able )
{
	ctr_irled = able;
}


void setSensorConstant( t_sensor *sen, int16_t reference, int16_t threshold )
{
  sen->reference = reference;
  sen->threshold = threshold;
}

void update_sensor_data( void )
{

  sen_front.now = ( sen_fl.now + sen_fr.now ) / 2;

  sen_fr.diff = ( sen_fr_log.now - sen_fr_log.before_5ms );
  if ( sen_fr.diff < 0 ) sen_fr.diff = -1 * sen_fr.diff;
  sen_fr.diff_1ms = ( sen_fr_log.now - sen_fr_log.before_1ms );
  if ( sen_fr.diff_1ms < 0 ) sen_fr.diff_1ms = -1 * sen_fr.diff_1ms;

  sen_r.diff = ( sen_r_log.now - sen_r_log.before_5ms );
  //if ( sen_r.diff < 0 ) sen_r.diff = -1 * sen_r.diff;
  sen_r.diff_1ms = ( sen_r_log.now - sen_r_log.before_1ms );
  if ( sen_r.diff_1ms < 0 ) sen_r.diff_1ms = -1 * sen_r.diff_1ms;

  sen_l.diff = ( sen_l_log.now - sen_l_log.before_5ms );
 // if ( sen_l.diff < 0 ) sen_l.diff = -1 * sen_l.diff;
  sen_l.diff_1ms = ( sen_l_log.now - sen_l_log.before_1ms );
  if ( sen_l.diff_1ms < 0 ) sen_l.diff_1ms = -1 * sen_l.diff_1ms;

  sen_fl.diff = ( sen_fl_log.now - sen_fl_log.before_5ms );
  if ( sen_fl.diff < 0 ) sen_fl.diff = -1 * sen_fl.diff;
  sen_fl.diff_1ms = ( sen_fl_log.now - sen_fl_log.before_1ms );
  if ( sen_fl.diff_1ms < 0 ) sen_fl.diff_1ms = -1 * sen_fl.diff_1ms;

  if ( sen_front.now < sen_front.threshold ){
    sen_front.is_wall = 0;
  } else {
    sen_front.is_wall = 1;
  }

  if ( sen_l.now < sen_l.threshold ){
    sen_l.is_wall = 0;
  } else {
    sen_l.is_wall = 1;
  }

  if ( sen_r.now < sen_r.threshold ){
    sen_r.is_wall = 0;
  } else {
    sen_r.is_wall = 1;
  }
}

void adcStart( void )
{
  setIrledPwm( IRLED_ON );
  adc_counter = 0;
  HAL_ADC_Start_DMA( &hadc1, (uint32_t *)ADCBuff, sizeof(ADCBuff) );
}

void adcEnd( void )
{
  setIrledPwm( IRLED_OFF );
  adc_counter = FINISH_CONVERT;
}

void adcCheckConvert( void )
{
  if ( adc_counter == FINISH_CONVERT ){
    batt_monitor = battMonitor();
    if ( ctr_irled == 1 ){
      adc_counter = 0;
      HAL_ADC_Start_DMA( &hadc1, (uint32_t *)ADCBuff, sizeof(ADCBuff) );
    }
  }
}

// DMA の変換式を記�?
void getADSensor( int16_t *adcount )
{
  volatile int i;
  switch( *adcount ) {
    case OFF_VALUE:
      HAL_ADC_Stop_DMA( &hadc1 );
      ADCOffData[0] = ADCBuff[0];
      ADCOffData[1] = ADCBuff[1];
      ADCOffData[2] = ADCBuff[2];
      ADCOffData[3] = ADCBuff[3];

      HAL_GPIO_WritePin( ir_side_GPIO_Port, ir_side_Pin, GPIO_PIN_SET );
      for( i = 0; i < 200; i++ ){

      }

      *adcount = SIDE_VALUE;

      HAL_ADC_Start_DMA( &hadc1, (uint32_t *)ADCBuff, sizeof(ADCBuff) );
      break;

    case SIDE_VALUE:
      HAL_GPIO_WritePin( ir_side_GPIO_Port, ir_side_Pin, GPIO_PIN_RESET );
      HAL_ADC_Stop_DMA( &hadc1 );

      ADCOntData[2] = ADCBuff[2];
      ADCOntData[3] = ADCBuff[3];

      sen_r.now = ADCOntData[2] - ADCOffData[2];
      sen_r_log.before_5ms = sen_r_log.before_4ms;
      sen_r_log.before_4ms = sen_r_log.before_3ms;
      sen_r_log.before_3ms = sen_r_log.before_2ms;
      sen_r_log.before_2ms = sen_r_log.before_1ms;
      sen_r_log.before_1ms = sen_r_log.now;
      sen_r_log.now = sen_r.now;

      sen_l.now = ADCOntData[3] - ADCOffData[3];
      sen_l_log.before_5ms = sen_l_log.before_4ms;
      sen_l_log.before_4ms = sen_l_log.before_3ms;
      sen_l_log.before_3ms = sen_l_log.before_2ms;
      sen_l_log.before_2ms = sen_l_log.before_1ms;
      sen_l_log.before_1ms = sen_l_log.now;
      sen_l_log.now = sen_l.now;


      HAL_GPIO_WritePin( ir_front_GPIO_Port, ir_front_Pin, GPIO_PIN_SET );
      for( i = 0; i < 400; i++ ){

      }

      *adcount = FRONT_VALUE;

      HAL_ADC_Start_DMA( &hadc1, (uint32_t *)ADCBuff, sizeof(ADCBuff) );
      break;

    case FRONT_VALUE:
      HAL_GPIO_WritePin( ir_front_GPIO_Port, ir_front_Pin, GPIO_PIN_RESET );
      HAL_ADC_Stop_DMA( &hadc1 );

      ADCOntData[0] = ADCBuff[0];
      ADCOntData[1] = ADCBuff[1];

      sen_fr.now = ADCOntData[0] - ADCOffData[0];
      sen_fr_log.before_5ms = sen_fr_log.before_4ms;
      sen_fr_log.before_4ms = sen_fr_log.before_3ms;
      sen_fr_log.before_3ms = sen_fr_log.before_2ms;
      sen_fr_log.before_2ms = sen_fr_log.before_1ms;
      sen_fr_log.before_1ms = sen_fr_log.now;
      sen_fr_log.now = sen_fr.now;

      sen_fl.now = ADCOntData[1] - ADCOffData[1];
      sen_fl_log.before_5ms = sen_fl_log.before_4ms;
      sen_fl_log.before_4ms = sen_fl_log.before_3ms;
      sen_fl_log.before_3ms = sen_fl_log.before_2ms;
      sen_fl_log.before_2ms = sen_fl_log.before_1ms;
      sen_fl_log.before_1ms = sen_fl_log.now;
      sen_fl_log.now = sen_fl.now;

      *adcount = FINISH_CONVERT;
      update_sensor_data();
      break;

    default:
      break;
  }
}
/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

/**
  ******************************************************************************
  * File Name          : SPI.c
  * Description        : This file provides code for the configuration
  *                      of the SPI instances.
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
#include "spi.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "variable.h"
#include "arm_math.h"

#define MPU6500_RA_SELF_TEST_X_GYRO	0x00
#define MPU6500_RA_SELF_TEST_Y_GYRO	0x01
#define MPU6500_RA_SELF_TEST_Z_GYRO	0x02
#define MPU6500_RA_SELF_TEST_X_ACCEL	0x0D
#define MPU6500_RA_SELF_TEST_Y_ACCEL	0x0E
#define MPU6500_RA_SELF_TEST_Z_ACCEL	0x0F
#define MPU6500_RA_XG_OFFSET_H		0x13
#define MPU6500_RA_XG_OFFSET_L		0x14
#define MPU6500_RA_YG_OFFSET_H		0x15
#define MPU6500_RA_YG_OFFSET_L		0x16
#define MPU6500_RA_ZG_OFFSET_H		0x17
#define MPU6500_RA_ZG_OFFSET_L		0x18
#define MPU6500_RA_SMPLRT_DIV		0x19
#define MPU6500_RA_CONFIG			0x1A
#define MPU6500_RA_GYRO_CONFIG		0x1B
#define MPU6500_RA_ACCEL_CONFIG		0x1C
#define MPU6500_RA_ACCEL_CONFIG2	0x1D
#define MPU6500_RA_LP_ACCEL_ODR		0x1E
#define MPU6500_RA_WOM_THR			0x1F
#define MPU6500_RA_FIFO_EN			0x23
// ...
// å¤–éƒ¨I2Cå‘¨ã‚�?ï¿½????¿½?¿½??¿½?¿½çœ�?ç•¥
// ...
#define MPU6500_RA_INT_PIN_CFG		0x37
#define MPU6500_RA_INT_ENABLE		0x38
#define MPU6500_RA_INT_STATUS		0x3A
#define MPU6500_RA_ACCEL_XOUT_H		0x3B
#define MPU6500_RA_ACCEL_XOUT_L		0x3C
#define MPU6500_RA_ACCEL_YOUT_H		0x3D
#define MPU6500_RA_ACCEL_YOUT_L		0x3E
#define MPU6500_RA_ACCEL_ZOUT_H		0x3F
#define MPU6500_RA_ACCEL_ZOUT_L		0x40
#define MPU6500_RA_TEMP_OUT_H		0x41
#define MPU6500_RA_TEMP_OUT_L		0x42
#define MPU6500_RA_GYRO_XOUT_H		0x43
#define MPU6500_RA_GYRO_XOUT_L		0x44
#define MPU6500_RA_GYRO_YOUT_H		0x45
#define MPU6500_RA_GYRO_YOUT_L		0x46
#define MPU6500_RA_GYRO_ZOUT_H		0x47
#define MPU6500_RA_GYRO_ZOUT_L		0x48
// ...
// å¤–éƒ¨I2Cå‘¨ã‚�?ï¿½????¿½?¿½??¿½?¿½çœ�?ç•¥
// ...
#define MPU6500_RA_SIGNAL_PATH_RESET	0x68
#define MPU6500_RA_MOT_DETECT_CTRL		0x69
#define MPU6500_RA_USER_CTRL		0x6A
#define MPU6500_RA_PWR_MGMT_1		0x6B
#define MPU6500_RA_PWR_MGMT_2		0x6C
#define MPU6500_RA_FIFO_COUNTH		0x72
#define MPU6500_RA_FIFO_COUNTL			0x73
#define MPU6500_RA_FIFO_R_W			0x74
#define MPU6500_RA_WHO_AM_I			0x75
#define MPU6500_RA_XA_OFFSET_H		0x77
#define MPU6500_RA_XA_OFFSET_L		0x78
#define MPU6500_RA_YA_OFFSET_H		0x7A
#define MPU6500_RA_YA_OFFSET_L		0x7B
#define MPU6500_RA_ZA_OFFSET_H		0x7D
#define MPU6500_RA_ZA_OFFSET_L		0x7E

#define MPU6500_DEVICE_ID			0x70

#define SETTING       0x80  //0b1000 0000 8bitの上位bitを立てると

#define GYRO_FACTOR  16.4f

#define ACCEL_FACTOR 2048.0f

#define NOP 0x0000
#define ERRFL 0x0001 //error check
#define DIAAGC 0x3ffc
#define ANGLECOM 0x3fff
#define ANGLEUNC 0x7ffe
#define DIAAGC 0x3ffc
#define ZPOSM 0x0016
#define ZPOSL 0x0017
#define SETTINGS1 0x0018
#define SETTINGS2 0x0019

#define TRUE 0
#define FALSE 1

#define dt 0.001f

static int16_t gyro_offset_cnt = 0; 
static int8_t  gyro_calc_flag = 1;
static float gyro_z_offset = 0.0f;

/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* SPI2 init function */
void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspInit 0 */

  /* USER CODE END SPI1_MspInit 0 */
    /* SPI1 clock enable */
    __HAL_RCC_SPI1_CLK_ENABLE();
  
    /**SPI1 GPIO Configuration    
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI1_MspInit 1 */

  /* USER CODE END SPI1_MspInit 1 */
  }
  else if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspInit 0 */

  /* USER CODE END SPI2_MspInit 0 */
    /* SPI2 clock enable */
    __HAL_RCC_SPI2_CLK_ENABLE();
  
    /**SPI2 GPIO Configuration    
    PB10     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN SPI2_MspInit 1 */

  /* USER CODE END SPI2_MspInit 1 */
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{

  if(spiHandle->Instance==SPI1)
  {
  /* USER CODE BEGIN SPI1_MspDeInit 0 */

  /* USER CODE END SPI1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI1_CLK_DISABLE();
  
    /**SPI1 GPIO Configuration    
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);

  /* USER CODE BEGIN SPI1_MspDeInit 1 */

  /* USER CODE END SPI1_MspDeInit 1 */
  }
  else if(spiHandle->Instance==SPI2)
  {
  /* USER CODE BEGIN SPI2_MspDeInit 0 */

  /* USER CODE END SPI2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SPI2_CLK_DISABLE();
  
    /**SPI2 GPIO Configuration    
    PB10     ------> SPI2_SCK
    PB14     ------> SPI2_MISO
    PB15     ------> SPI2_MOSI 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_15);

  /* USER CODE BEGIN SPI2_MspDeInit 1 */

  /* USER CODE END SPI2_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
/********************************************************************
 * Overview : spi read register
 * Argument : register
 * Return : 1byte data
********************************************************************/
uint8_t read_byte(uint8_t reg){
  uint8_t ret,val;
  HAL_GPIO_WritePin(gyro_cs_GPIO_Port,gyro_cs_Pin,GPIO_PIN_RESET);
  ret = reg|SETTING ;
  HAL_SPI_Transmit(&hspi2,&ret,1,100);
  HAL_SPI_Receive(&hspi2,&val,1,100);
  HAL_GPIO_WritePin(gyro_cs_GPIO_Port,gyro_cs_Pin,GPIO_PIN_SET);
  return val;
}


/****************************************************************
 * Overview : shift 8bit and spi read register
 * Argument : register
 * Return : 2byte data (shift 8bit)
 ************************************************************/
int16_t read_shift_byte(uint8_t reg){
  uint8_t address,val_1;
  int16_t val_2;
  HAL_GPIO_WritePin(gyro_cs_GPIO_Port,gyro_cs_Pin,GPIO_PIN_RESET);
  address = reg | SETTING ;
  HAL_SPI_Transmit(&hspi2,&address,1,100);
  HAL_SPI_Receive(&hspi2,&val_1,1,100);
  val_2 = (int16_t)(val_1 << 8);
  HAL_GPIO_WritePin(gyro_cs_GPIO_Port,gyro_cs_Pin,GPIO_PIN_SET);
  return val_2;
}


/**************************************************************
 * Overview : spi write 1byte
 * Argument : register
 * Return : 
 *************************************************************/
void write_byte( uint8_t reg,uint8_t val){
  uint8_t ret;
  ret = reg & 0x7F;
  HAL_GPIO_WritePin(gyro_cs_GPIO_Port,gyro_cs_Pin,GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi2,&ret,1,100);
  HAL_SPI_Transmit(&hspi2,&val,1,100);
  HAL_GPIO_WritePin(gyro_cs_GPIO_Port,gyro_cs_Pin,GPIO_PIN_SET);
}


///////////////////////////////////////////////////////////////////////
// set up mpu-6500
// [argument] nothing
// [Substitutiong] nothing
// [return] nothong
///////////////////////////////////////////////////////////////////////
void MPU6500_init( void )
{
  uint8_t who_am_i;
  uint8_t recheck_who_am_i;
  
  HAL_Delay( 100 );
  who_am_i = read_byte( MPU6500_RA_WHO_AM_I );
  printf( "\r\n0x%x\r\n",who_am_i );

  if ( who_am_i != 0x70 ){
    HAL_Delay( 300 );
    recheck_who_am_i = read_byte( MPU6500_RA_WHO_AM_I );
    if ( recheck_who_am_i != 0x70 ){
      while(1){
        printf( "gyro_error\r");
      }
    } else {
      printf( "recheck_who_am_i = 0x%x\r\n", recheck_who_am_i );
    }
  }

  HAL_Delay( 50 );

  // solve sleep 
  write_byte( MPU6500_RA_PWR_MGMT_1, 0x00 );

  HAL_Delay( 50 );

	// DLPF_CFG = 0 : Gyro's LPF is not avaiable
	// not avaiable FIFO
  write_byte( MPU6500_RA_CONFIG, 0x00 );

  HAL_Delay( 50 );

  // Gyro scope is full scale
	write_byte(MPU6500_RA_GYRO_CONFIG, 0x18 );
  HAL_Delay( 50 );

  // Accel scope is full scale
  write_byte( MPU6500_RA_ACCEL_CONFIG, 0x18 );

  gyro_calc_flag = 1;

}

///////////////////////////////////////////////////////////////////////
// get mpu-6500 read z axis data
// [argument] nothing
// [Substitutiong] nothing
// [return] read z ( rad / sec)
///////////////////////////////////////////////////////////////////////
float MPU6500_read_gyro_z( void )
{
  int16_t gyro_z;
  float omega;

  gyro_z = (int16_t)( read_shift_byte(MPU6500_RA_GYRO_ZOUT_H) | read_byte(MPU6500_RA_GYRO_ZOUT_L) );

  // GYRO FACTOR 
  // 180 * PI ( rad/sec )
  
  omega = (float)( ( gyro_z - gyro_z_offset ) / GYRO_FACTOR);

  return omega;
}
///////////////////////////////////////////////////////////////////////
// get mpu-6500 read z axis offset start
// [argument] nothing
// [Substitutiong] nothing
// [return] read z ( rad / sec)
///////////////////////////////////////////////////////////////////////
void MPU6500_z_axis_offset_calc_start( void )
{
  gyro_z_offset = 0.0f;
  gyro_offset_cnt = 0; 
  gyro_calc_flag = 0;
  rotation_real.distance=0;
}

///////////////////////////////////////////////////////////////////////
// get mpu-6500 read z axis offset calc
// [argument] nothing
// [Substitutiong] nothing
// [return] read z ( rad / sec)
///////////////////////////////////////////////////////////////////////
void MPU6500_z_axis_offset_calc( void )
{
  int16_t gyro_z;

  gyro_z = (int16_t)( read_shift_byte(MPU6500_RA_GYRO_ZOUT_H) | read_byte(MPU6500_RA_GYRO_ZOUT_L) );

  // GYRO FACTOR ( deg / sec )

  if ( gyro_offset_cnt < 1023 ){
    gyro_z_offset += (float)gyro_z;
    gyro_offset_cnt++;
  } else {
    gyro_z_offset /= 1023.0f;
    //gyro_z_offset = roundf( gyro_z_offset );
    gyro_calc_flag = 1;
  }
}

int8_t MPU6500_calc_check( void )
{
  return gyro_calc_flag;
}

// debug ON
float checkGyroOffset()
{
  return gyro_z_offset;
}

///////////////////////////////////////////////////////////////////////
// get mpu-6500 read x axis accel
// [argument] nothing
// [Substitutiong] nothing
// [return] read x ( m / sec^2)
///////////////////////////////////////////////////////////////////////
float MPU6500_read_accel_x( void )
{
  int16_t accel_x;
  float accel;

  accel_x = (int16_t)( read_shift_byte(MPU6500_RA_ACCEL_XOUT_H) | read_byte(MPU6500_RA_ACCEL_XOUT_L) );

  // GYRO FACTOR 
  
  accel = (float)( accel_x / ACCEL_FACTOR);

  return accel;
}

void UpdataGyro(void){
  if(gyro_calc_flag){
    rotation_real.velocity = MPU6500_read_gyro_z();
    rotation_real.distance += rotation_real.velocity/1000.0f;
  }else{
    MPU6500_z_axis_offset_calc();
  }
}

uint8_t parity(uint16_t val) {
	val ^= val >> 8;
	val ^= val >> 4;
	val ^= val >> 2;
	val ^= val >> 1;

	return (uint8_t)(val & 0x0001);
}

/********************************************************************
 * Overview : spi read register
 * Argument : register
 * Return : 2byte data
********************************************************************/
uint16_t enc_read_byte(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint16_t reg){
  uint16_t out;
  uint8_t val[2]={0x00,0x00};
  uint8_t ret[2]={(uint8_t)(reg>>8),(uint8_t)(reg&0xff)};
  uint8_t dummy[2]={0x00,0x00};

  ret[0] |= 0x40;//read
  ret[0] |= parity(((uint16_t)ret[0]<<8)|((uint16_t)ret[1]&0xff))<<7;//parity

  HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1,ret,dummy,1,100);
  //HAL_SPI_Transmit(&hspi1,ret,1,100);
  //HAL_SPI_Receive(&hspi1,dummy,1,100);
  HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_SET);
  for(volatile uint16_t i=0;i<20;i++);//350ns

  HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1,dummy,val,1,100);
  //HAL_SPI_Transmit(&hspi1,dummy,1,100);
  //HAL_SPI_Receive(&hspi1,val,1,100);
  HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_SET);
  for(volatile uint16_t i=0;i<20;i++);//350ns
  out=((uint16_t)val[0]<<8)|((uint16_t)val[1]&0xff);
  return out;
}

/********************************************************************
 * Overview : spi write register
 * Argument : register
 * Return : 2byte data
********************************************************************/
void enc_write_byte(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint16_t reg, uint16_t value){
  uint8_t val[2]={(uint8_t)(value>>8),(uint8_t)(value&0xff)};
  uint8_t ret[2]={(uint8_t)(reg>>8),(uint8_t)(reg&0xff)};
  uint8_t dummy[2]={0x00,0x00};
  ret[0] |= (uint8_t)parity(((uint16_t)ret[0]<<8)|(uint16_t)ret[1])<<7;//parity
  val[0] |= (uint8_t)parity(((uint16_t)val[0]<<8)|(uint16_t)val[1])<<7;//parity
  HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1,ret,dummy,1,100);
  //HAL_SPI_Transmit(&hspi1,ret,1,100);
  //HAL_SPI_Receive(&hspi1,dummy,1,100);
  HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_SET);
  for(volatile uint16_t i=0;i<20;i++);

  HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_RESET);
  dummy[0]=0x00;
  dummy[1]=0x00;
  HAL_SPI_TransmitReceive(&hspi1,val,dummy,1,100);
  //HAL_SPI_Transmit(&hspi1,val,1,100);
  //HAL_SPI_Receive(&hspi1,dummy,1,100);
  HAL_GPIO_WritePin(GPIOx,GPIO_Pin,GPIO_PIN_SET);
  for(volatile uint16_t i=0;i<20;i++);
}

void set_enc(void){
  /*enc_write_byte(csl_GPIO_Port,csl_Pin,SETTINGS1,0x0001);//def 0x0001
  enc_write_byte(csl_GPIO_Port,csl_Pin,SETTINGS2,0b0000);//def 0x0000
  enc_write_byte(csl_GPIO_Port,csl_Pin,ZPOSM,0x0000);//ZPOSM[0:7]
  enc_write_byte(csl_GPIO_Port,csl_Pin,ZPOSL,0x0000);//comp_h_error_en[7]|comp_l_error_en[6]|ZPOSL[0:5]

  enc_write_byte(csr_GPIO_Port,csr_Pin,SETTINGS1,0x0001);//def 0x0001
  enc_write_byte(csr_GPIO_Port,csr_Pin,SETTINGS2,0x0000);//0x0000
  enc_write_byte(csr_GPIO_Port,csr_Pin,ZPOSM,0x0000);//ZPOSM[0:7]
  enc_write_byte(csr_GPIO_Port,csr_Pin,ZPOSL,0x0000);//comp_h_error_en[7]|comp_l_error_en[6]|ZPOSL[0:5]
  */


  printf("nop_l:%x\tnop_r:%x\r\n",enc_read_byte(csl_GPIO_Port,csl_Pin,NOP),enc_read_byte(csr_GPIO_Port,csr_Pin,NOP));
  //while(((enc_read_byte(csl_GPIO_Port,csl_Pin,DIAAGC)>>8)&0b1)==0){
    for (volatile uint8_t i = 0; i < 20; i++);
    printf("DIAAGC_l:%x\tDIAAGC_r:%x\r",enc_read_byte(csl_GPIO_Port,csl_Pin,DIAAGC),enc_read_byte(csr_GPIO_Port,csr_Pin,DIAAGC));
  //}
  printf("\n");
}

void get_enc_val(void){
  uint16_t buff_l,buff_r;
  buff_l = enc_read_byte(csl_GPIO_Port,csl_Pin,ANGLECOM);
  buff_r = enc_read_byte(csr_GPIO_Port,csr_Pin,ANGLECOM);
  if(parity(buff_l)==0){
    enc_l = buff_l&0x3fff;
  }
  if(parity(buff_r)==0){
    enc_r = buff_r&0x3fff;
  }
  distance_l += (float)enc_l/16383.0f*PI*12.6f*0.001f; 
  distance_r += (float)enc_r/16383.0f*PI*12.6f*0.001f;
}


/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

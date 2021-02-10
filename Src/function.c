#include "function.h"
// peripheeral
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "tim.h"
#include "spi.h"
#include "gpio.h"

//common
//#include "flash.h"
//#include "logger.h"

///////////////////////////////////////////////////////////////////////
// machine_init
// [argument] nothinh
// [Substitutiong] batt_voltage
// [return] nothing
///////////////////////////////////////////////////////////////////////
void machine_init( void )
{
  setbuf( stdout, NULL );
  failSafe_flag = 0;
  //buzzerSetMonophonic( NORMAL, 100 );
  //HAL_Delay( 101 );
  //MPU6500_init();
  //buzzerSetMonophonic( NORMAL, 100 );
  //HAL_TIM_Encoder_Start( &htim2, TIM_CHANNEL_ALL ); // encoder
  //HAL_TIM_Encoder_Start( &htim8, TIM_CHANNEL_ALL ); // encoder
  //log_init();
  battChecker();
  //buzzerSetMonophonic( NORMAL, 100 );
  HAL_Delay(1000);
  MPU6500_z_axis_offset_calc_start();
}
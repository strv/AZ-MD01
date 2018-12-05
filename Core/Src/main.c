/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dac.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pwm.h"
#include "encoder.h"
#include "uart_util.h"
#include "xprintf.h"
#include "control.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
volatile int64_t tick;
const int64_t INTERVAL = 100;
const int32_t ADC_CAL_DURATION = 50;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void tick_inc(){
	tick++;
}

int64_t tick_get(){
	return tick;
}

bool cmd_duty_set_func(int32_t argc,int32_t* argv){
	float duty = 0.;
	if(argc != 1){
		return false;
	}
	if(argv[0] < -100){
		duty = -100.;
	}else if(argv[0] > 100){
		duty = 100.;
	}else{
		duty = argv[0];
	}
	pwm_set_duty(duty);
	return true;
}
UU_ConsoleCommand cmd_duty_set = {
		"DUTY",
		cmd_duty_set_func,
		"DUTY [duty]\r\nSet PWM duty. "
};

bool cmd_set_cur_func(int32_t argc, int32_t* argv){
	ctrl_set_cur((float)argv[0] / 1000.);
	return true;
}
UU_ConsoleCommand cmd_set_cur = {
		"SETCUR",
		cmd_set_cur_func,
		"SETCUR [current in mA]\r\nSet target current in current control mode."
};

bool cmd_set_cur_gain_func(int32_t argc, int32_t* argv){
	if(argc != 3){
		return false;
	}
	ctrl_set_cur_gain((float)argv[0] / 1000., (float)argv[1] / 1000., (float)argv[2] / 1000.);
	return true;
}
UU_ConsoleCommand cmd_set_cur_gain = {
		"CURGAIN",
		cmd_set_cur_gain_func,
		"CURGAIN [kp] [ti] [td]\r\nAll value are divide with 1000."
};

bool cmd_set_mode_func(int32_t argc, int32_t* argv){
	if(argc != 1){
		return false;
	}
	ctrl_set_mode(argv[0]);
	return true;
}
UU_ConsoleCommand cmd_set_mode = {
		"MODE",
		cmd_set_mode_func,
		"MODE [mode number]\r\n0 : DUTY\r\n1 : CURRENT\r\n2 : SPEED with BEMF\r\n3 : SPEED with ENCODER"
};
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void){
  /* USER CODE BEGIN 1 */
	int64_t tick_last = 0;
	int64_t enc_total;
	int32_t vel;
	uint32_t enc_prev = 0;
	uint32_t enc;
	float cur=0.;
	float cur_prev=0.;
	float cur_fo=0.2;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

  /* System interrupt init*/
  /* SysTick_IRQn interrupt configuration */
  NVIC_SetPriority(SysTick_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(),1, 0));

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  LL_SYSTICK_EnableIT();

  uu_init();
  uu_push_command(&cmd_duty_set);
  uu_push_command(&cmd_set_cur);
  uu_push_command(&cmd_set_cur_gain);
  uu_push_command(&cmd_set_mode);
  xputs("AZ-MD01\r\n");

  adc_init();

  pwm_init();
  pwm_set_duty(0.);

  enc_init();
  dac_init();
  pwm_md_enable();

  {
	  xputs("Start ADC calibration...");
	  int32_t adc_cal_cnt = 0;
	  int32_t cz_a, cz_b, cz_a_prev, cz_b_prev;
	  while(true){
		  adc_cur_cal_proc();
		  cz_a = adc_get_cur_zero(0);
		  cz_b = adc_get_cur_zero(1);
		  if(abs(cz_a - cz_a_prev) < 1 && abs(cz_b - cz_b_prev) < 1){
			  adc_cal_cnt++;
			  if(adc_cal_cnt > 10){
				  break;
			  }
		  }else{
			  adc_cal_cnt = 0;
		  }
		  cz_a_prev = cz_a;
		  cz_b_prev = cz_b;
		  LL_mDelay(1);
	  }
	  xputs("Done!!!\r\n");
	  xprintf("Current zero value : %d %d\r\n", adc_get_cur_zero(0), adc_get_cur_zero(1));
  }

  ctrl_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(tick >= tick_last + INTERVAL){
		  tick_last += INTERVAL;

		  enc = enc_get();
		  vel = enc - enc_prev;
		  enc_prev = enc;
		  enc_total += vel;
/*
		  xprintf("ADC :\r\n");
		  for(volatile int i = 0; i < 20; i++){
			  xprintf("\t%02d : %4d\r\n", i, adc_get(i));
		  }
		  xputs("\r\n");
*/
		  xprintf("V batt  : %5d\r\n", (int32_t)(adc_get_vbatt() * 1000.));
		  cur = cur_prev * (1. - cur_fo) + adc_get_cur_ave() * cur_fo;
		  cur_prev = cur;
		  xprintf("Current : %5d %5d\r\n", (int32_t)(adc_get_cur() * 1000.), (int32_t)(cur * 1000.));
		  xprintf("ADC EXT : %5d %5d\r\n", (int32_t)(adc_get_ext() * 1000.), (int32_t)(adc_get_ext_rate() * 100.));
	  }
	  uu_proc_command();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);

   if(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2)
  {
  Error_Handler();  
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {
    
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_16);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {
    
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB1_DIV_2);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {
  
  }
  LL_Init1msTick(64000000);
  LL_SYSTICK_SetClkSource(LL_SYSTICK_CLKSOURCE_HCLK);
  LL_SetSystemCoreClock(64000000);
  LL_RCC_SetTIMClockSource(LL_RCC_TIM1_CLKSOURCE_PLL);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

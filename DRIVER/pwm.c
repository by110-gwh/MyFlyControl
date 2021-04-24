#include "pwm.h"
#include "stm32f1xx_hal.h"

//最大PWM值 400Hz
#define MAX_PWM 2500

static TIM_HandleTypeDef htim3;

/**********************************************************************************************************
*函 数 名: pwm_init
*功能说明: 电调输出PWM初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void pwm_init()
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	
	__HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_TIM3_CLK_ENABLE();
	
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 72 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = MAX_PWM - 1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_Base_Init(&htim3);
	
	HAL_TIM_PWM_Init(&htim3);
	
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
}

/**********************************************************************************************************
*函 数 名: pwm_set
*功能说明: PWM输出设置
*形    参: 通道一PWM值 通道二PWM值 通道三PWM值 通道四PWM值
*返 回 值: 无
**********************************************************************************************************/
void pwm_set(uint16_t pwm1, uint16_t pwm2, uint16_t pwm3, uint16_t pwm4)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm1);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm2);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwm3);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwm4);
}

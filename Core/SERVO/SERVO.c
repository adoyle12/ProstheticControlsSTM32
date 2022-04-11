/*
 * File: SERVO.c
 * Driver Name: [[ SERVO Motor ]]
 * SW Layer:   ECUAL
 * Created on: Jun 28, 2020
 * Author:     Khaled Magdy
 * -------------------------------------------
 * For More Information, Tutorials, etc.
 * Visit Website: www.DeepBlueMbedded.com
 *
 */

#include "SERVO.h"
#include "SERVO_cfg.h"
#include "DWT_Delay.h"
#include "config.h"

typedef struct
{
	uint16_t  CCR_Min;
	uint16_t  CCR_Max;
}SERVO_info;

static SERVO_info gs_SERVO_info[SERVO_NUM] = {0};


void SERVO_Init(uint16_t au16_SERVO_Instance)
{
    uint32_t ARR_Value = SERVO_CfgParam[au16_SERVO_Instance].TIM_Instance->Init.Period;
    printf("ARR: %d\n", ARR_Value);

    gs_SERVO_info[au16_SERVO_Instance].CCR_Min = (uint16_t) (ARR_Value * STARTING_PULSE/PWM_PERIOD);
    float MaxPulse = (SERVO_CfgParam[au16_SERVO_Instance].DegreesToMove * WIDTH_PER_DEGREE) + STARTING_PULSE;
    gs_SERVO_info[au16_SERVO_Instance].CCR_Max = (uint16_t) (ARR_Value * MaxPulse / PWM_PERIOD);
	HAL_TIM_PWM_Start(SERVO_CfgParam[au16_SERVO_Instance].TIM_Instance, SERVO_CfgParam[au16_SERVO_Instance].PWM_TIM_CH);
    printf("PWM Started on Servo %d \r\n",au16_SERVO_Instance);
}

/* Moves A Specific Motor To A Specific Degree That Can Be Float Number */
void SERVO_MoveTo(uint16_t au16_SERVO_Instance, float af_Angle)
{
	uint16_t au16_Pulse = 0;
	au16_Pulse = ((af_Angle*(gs_SERVO_info[au16_SERVO_Instance].CCR_Max - gs_SERVO_info[au16_SERVO_Instance].CCR_Min)) / 180.0)
			+ gs_SERVO_info[au16_SERVO_Instance].CCR_Min;

	*(SERVO_CfgParam[au16_SERVO_Instance].TIM_CCRx) = au16_Pulse;
}

/* Moves A Specific Motor With A Raw Pulse Width Value */
void SERVO_RawMove(uint16_t au16_SERVO_Instance, uint16_t au16_Pulse)
{

//    printf("Raw Move: Servo %d, Pulse: %d  \r\n",au16_SERVO_Instance,au16_Pulse);
    if (au16_Pulse >= gs_SERVO_info[au16_SERVO_Instance].CCR_Min && au16_Pulse <= gs_SERVO_info[au16_SERVO_Instance].CCR_Max)
    {
        *(SERVO_CfgParam[au16_SERVO_Instance].TIM_CCRx) = au16_Pulse;
    }
}

/* Gets The Maximum Pulse Width Value For A Specific Motor */
uint16_t SERVO_Get_MaxPulse(uint16_t au16_SERVO_Instance)
{
	return (gs_SERVO_info[au16_SERVO_Instance].CCR_Max);
}


/* Gets The Minimum Pulse Width Value For A Specific Motor */
uint16_t SERVO_Get_MinPulse(uint16_t au16_SERVO_Instance)
{
	return (gs_SERVO_info[au16_SERVO_Instance].CCR_Min);
}


/* Move A Motor From 0 deg to 180 And Back to 0 again */
void SERVO_Sweep(uint16_t au16_SERVO_Instance)
{
	uint8_t au8_Angle = 0;

	SERVO_MoveTo(au16_SERVO_Instance, 0);

    HAL_Delay(250);
	while(au8_Angle < 180)
	{
		SERVO_MoveTo(au16_SERVO_Instance, au8_Angle++);
        HAL_Delay(5);
	}
    HAL_Delay(250);
	while(au8_Angle > 0)
	{
		SERVO_MoveTo(au16_SERVO_Instance, au8_Angle--);
        HAL_Delay(5);
	}
}

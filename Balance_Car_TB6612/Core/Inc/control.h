/**
  *************************************************************************************************************************
  * @file    : xxx.h
  * @author  :Confident
  * @version : V1.0
  * @date    : 2022-XX-XX
  * @brief   : STM32Fxxx核心板核心板驱动程序--xxx模块H文件
  *************************************************************************************************************************
  * @attention
  * 
  * 
  * 
  *************************************************************************************************************************
  */


/* Define to prevent recursive inclusion --------------------------------------------------------------------------------*/
#ifndef __CONTROL_H__
#define __CONTROL_H__
/* Includes -------------------------------------------------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "iwdg.h"
#include "gpio.h"
#include "oled.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "IIC.h"
/* define ---------------------------------------------------------------------------------------------------------------*/
#define IN1_SET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET)
#define IN1_RESET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET)

#define IN2_SET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET)
#define IN2_RESET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET)

//左电机
#define IN3_SET HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET)
#define IN3_RESET HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET)

#define IN4_SET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET)
#define IN4_RESET HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET)

//使能位
#define STBY_SET HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)
#define STBY_RESET HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)


typedef struct 
{
	int16_t aacx;//加速度
	int16_t aacy;
	int16_t aacz;
	
	int16_t gyrox;//角速度
	int16_t gyroy;
	int16_t gyroz;
	
	float pitch;//俯仰角
	float roll;//横滚角 
	float yaw;//航向角 
	float temp;
	
}MPU6050TypeDef;
typedef struct
{
	float balance_UP_KP; 	 // 小车直立环KP 305
	float balance_UP_KD;              //1.4   微分 
	
	float velocity_KP;
	float velocity_KI;
	float Turn_KP;
	float Turn_KD;
	float Forward_KP;
}PIDTypeDef;
typedef struct
{
	int Balance_Pwm;
	int Velocity_Pwm;
	int Turn_Pwm;
	int Forward_Pwm;
	int Movement;
	int Mechanical_angle;
	int Turn_Target;
}OTHERTypeDef;
extern OTHERTypeDef Other;
extern PIDTypeDef PID;
extern  MPU6050TypeDef OutMpu;
extern int8_t delay_flag,delay_50ms;
extern uint8_t UART_Receive_buffer[1];//串口缓冲区
/* function -------------------------------------------------------------------------------------------------------------*/
void OLED_Init_show(void);
void OLED_Proc(void);
void Set_Pwm(int moto1,int moto2);
int abs(int p);
void TIM_Init(void);
uint8_t KEY_Scan(uint8_t mode);
void Key_Proc(void);
int Read_Speed(int TIMx);
void PID_Init(void);
void Limit_Pwm(int Moto1,int Moto2);

int balance_UP(float Angle,float Mechanical_balance,float Gyro);
int velocity(int encoder_left,int encoder_right,float Angle,float Movement);
int turn(int Turn_Target,float gyro);
uint16_t Get_Adc_Average(uint8_t times);
uint16_t Get_Adc(void)  ;
int Foward(float object_yaw,float now_yaw);
#endif /* __XXX_H */
/*****************************************************END OF FILE*********************************************************/	



/**
  *************************************************************************************************************************
  * @file    : xxx.c
  * @author  : Confident
  * @version : V1.0
  * @date    : 2022-XX-XX
  * @brief   : STM32Fxxx核心板核心板驱动程序--xxx模块C文件
  *************************************************************************************************************************
  * @attention
  *执行文件
  * 
  *
  *************************************************************************************************************************
  */

/* Includes -------------------------------------------------------------------------------------------------------------*/
 #include "control.h"
/* define ---------------------------------------------------------------------------------------------------------------*/
MPU6050TypeDef OutMpu;
PIDTypeDef PID;
OTHERTypeDef Other;
int8_t delay_flag,delay_50ms;
int16_t Left_PWM,Right_PWM;//计算的左PWM 右PWM
int16_t Moto1,Moto2;
int16_t Lcnt,Rcnt;
uint8_t balance_state=1;//上电启动电机
uint8_t UART_Receive_buffer[1]={0};//串口缓冲区
uint8_t bluetooth;
uint8_t forward=0;//前进反馈
uint16_t object_yaw=0;
uint8_t flag = 0;
uint16_t adc_volatge=0;
void PID_Init(void)//380是死区   
{
	//直立环
	PID.balance_UP_KP = 395;//183   0-680//400
	PID.balance_UP_KD = 0.84;//0.84   1.3
//	//速度环
	PID.velocity_KP = 65 ;//0-106
	PID.velocity_KI = PID.velocity_KP/200;
//	//转向环
	PID.Turn_KP = 16;
	PID.Turn_KD = PID.Turn_KP/100;
	PID.Forward_KP = 100;
	
	
}


#define ANGLE_a 45
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance ==TIM3)
	{
		if(++delay_50ms==10)	 {delay_flag = 1,delay_50ms=0;}//给主函数提供100ms的精准延时
		mpu_dmp_get_data(&OutMpu.pitch, &OutMpu.roll, &OutMpu.yaw);
		MPU_Get_Accelerometer(&OutMpu.aacx,&OutMpu.aacy, &OutMpu.aacz);		//得到加速度传感器数据
		MPU_Get_Gyroscope(&OutMpu.gyrox, &OutMpu.gyroy, &OutMpu.gyroz);		//得到陀螺仪数据
		OutMpu.temp=MPU_Get_Temperature();						//得到温度信息
		
		Rcnt = Read_Speed(2);
		Lcnt = -Read_Speed(4);//左右轮速度
		if(abs(Rcnt)>85)
		{
			flag++;
			}else flag=0;
		if(flag==100)balance_state=1;
		Key_Proc();
		if(abs(OutMpu.pitch)>ANGLE_a||balance_state==1){IN1_SET;IN2_SET;IN3_SET;IN4_SET;STBY_RESET;}
		else{STBY_SET;}
		Other.Balance_Pwm = balance_UP(OutMpu.pitch,Other.Mechanical_angle,OutMpu.gyroy);
		//蓝牙控制
		if(bluetooth == 1){Other.Movement++;Other.Turn_Target=0;forward=1;if(Other.Movement>80)Other.Movement = 80;}//前进  右轮快
		else if(bluetooth == 2){Other.Movement--;Other.Turn_Target=0;forward=1;if(Other.Movement<-60)Other.Movement = -60;}//后退   右轮慢
		else if(bluetooth == 3){Other.Turn_Target=45;forward=0;}//左转
		else if(bluetooth == 4){Other.Turn_Target=-45;forward=0;}//右转
		else if(bluetooth==0){Other.Movement=0;forward=0;Other.Turn_Target=0;object_yaw=OutMpu.yaw;}//平衡状态
		
		Other.Velocity_Pwm = velocity(Lcnt,Rcnt,OutMpu.pitch,Other.Movement);
		Other.Turn_Pwm = turn(Other.Turn_Target,OutMpu.gyroz);
//		if(forward==1){Other.Forward_Pwm = Foward(OutMpu.yaw,OutMpu.yaw);}
//		else if(forward==0){Other.Forward_Pwm=0;}
		Left_PWM = Other.Balance_Pwm+Other.Velocity_Pwm-Other.Turn_Pwm;
		Right_PWM = Other.Balance_Pwm+Other.Velocity_Pwm+Other.Turn_Pwm;
		Limit_Pwm(Left_PWM,Right_PWM);//PWM限幅
		Set_Pwm(Left_PWM,Right_PWM);//装载
	}
}

int Foward(float object_yaw,float now_yaw)//输入目标角度,目标角度，当前角度
{
	 int object=0,now=0,bias;
		object=(int)object_yaw;
		now=(int)now_yaw;
		bias=object-now;
  	//=============转向PD控制器=======================//
		forward=bias*PID.Forward_KP;                 
	  return forward/2;//   
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

		if(huart->Instance==USART2)
		{
			HAL_UART_Receive_IT(&huart2,UART_Receive_buffer, 1);
			switch(UART_Receive_buffer[0])
			{
				case('A'):bluetooth = 1;break;//直行
				case('B'):bluetooth = 2;break;//后退
				case('C'):bluetooth = 3;break;//左转
				case('D'):bluetooth = 4;break;//右转
				case('1'):balance_state=1;break;//控制小车关闭
				case('2'):balance_state=0;break;//控制小车开启
				default: bluetooth = 0;
			}
		}

}

int balance_UP(float Angle,float Mechanical_balance,float Gyro)
{  
   int Bias;
	 int balance;
	 Bias=Angle-Mechanical_balance;    							 //===求出平衡的角度中值和机械相关

	 balance=PID.balance_UP_KP*Bias+PID.balance_UP_KD*(Gyro);  //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	return balance;
}
int velocity(int encoder_left,int encoder_right,float Angle,float Movement)
{  
    static float Velocity,Encoder_Least,Encoder;
	  static float Encoder_Integral;
   //=============速度PI控制器=======================//	
		Encoder_Least =(encoder_left+encoder_right)-0;                    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） 
		Encoder *= 0.85f;		                                                //===一阶低通滤波器       
		Encoder += Encoder_Least*0.15f;	                                    //===一阶低通滤波器    
		Encoder_Integral +=Encoder;                                       //===积分出位移 积分时间：10ms
		Encoder_Integral=Encoder_Integral-Movement;                       //===接收遥控器数据，控制前进后退
		if(Encoder_Integral>10000)  		Encoder_Integral=10000;             //===积分限幅
		if(Encoder_Integral<-10000)		Encoder_Integral=-10000;            //===积分限幅	
		Velocity=Encoder*PID.velocity_KP+Encoder_Integral*PID.velocity_KI;        //===速度控制	
	  //if(pitch<-40||pitch>40) 				Encoder_Integral=0;     						//===电机关闭后清除积分
	if(abs(Angle)>ANGLE_a||balance_state==1){Encoder_Integral=0;}
	  return Velocity;
}

int turn(int Turn_Target,float gyro)//转向控制
{
	 float Turn;
  	//=============转向PD控制器=======================//
		Turn=-Turn_Target*PID.Turn_KP-gyro*PID.Turn_KD;                 //===结合Z轴陀螺仪进行PD控制
	  return Turn;//1500    //-16 - 16;Kd = Kp/100;
}

//获得ADC值
//ch: 通道值 0~16，取值范围为：ADC_CHANNEL_0~ADC_CHANNEL_16
//返回值:转换结果
uint16_t Get_Adc(void)   
{
 
    HAL_ADC_Start(&hadc1);                               //开启ADC
	
    HAL_ADC_PollForConversion(&hadc1,10);                //轮询转换
 
	return (uint16_t)HAL_ADC_GetValue(&hadc1);	        	//返回最近一次ADC1规则组的转换结果
}
//获取指定通道的转换值，取times次,然后平均 
//times:获取次数
//返回值:通道ch的times次转换结果平均值

uint16_t Get_Adc_Average(uint8_t times)
{
	uint32_t temp_val=0;
	uint8_t t;
	uint16_t volt,battery;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc();
		HAL_Delay(5);
	}
	volt = temp_val/times;
	battery = volt*4.9*11*100/4096;
	return battery;
}
#define PWM_MAX 7000
#define PWM_MIN -7000

/**
  * @brief  限制PWM赋值
  * @param  左右电机PWM值
  * @retval None
  */
void Limit_Pwm(int Moto1,int Moto2)
{
	 //===PWM满幅是7200 限制在7000
    if(Moto1<PWM_MIN ) Moto1=PWM_MIN ;
		if(Moto1>PWM_MAX )  Moto1=PWM_MAX ;
	  if(Moto2<PWM_MIN ) Moto2=PWM_MIN ;
		if(Moto2>PWM_MAX )  Moto2=PWM_MAX ;

}
#define KEY3 HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10)
#define KEY2 HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15)
#define KEY3_PRES   3	//KEY3按下
#define KEY2_PRES   2	//KEY3按下
uint8_t KEY_Scan(uint8_t mode)
{	 
	static uint8_t key_up=1;//按键按松开标志
	if(mode)key_up = 1;  //支持连按		  
	if(key_up&&(KEY3==0||KEY2==0))
	{
		key_up=0;
		if(KEY2==0)return  KEY2_PRES;
		else if(KEY3==0)return  KEY3_PRES;
	}else if (KEY3==1&&KEY2==1)key_up=1; 	    
 	return 0;// 无按键按下
}
void Key_Proc(void)
{
  uint8_t KeyRead;
	KeyRead= KEY_Scan(0);
	if(KeyRead)
	{
	 switch (KeyRead)
	 {
		 case KEY2_PRES:balance_state=0;break;
		 case KEY3_PRES:balance_state=1;break;
		 default:break;
	 }	
	}
}
int Read_Speed(int TIMx)
{
	int value;
	switch(TIMx)
	{
		case 2:value=(short)__HAL_TIM_GET_COUNTER(&htim2);__HAL_TIM_SET_COUNTER(&htim2,0);break;//IF是定时器2，1.采集编码器的计数值并保存。2.将定时器的计数值清零。
		case 4:value=(short)__HAL_TIM_GET_COUNTER(&htim4);__HAL_TIM_SET_COUNTER(&htim4,0);break;
		default:value=0;
	}
	return value;
}
void TIM_Init(void)
{
	//定时器1 PWW
	//2 编码器
	//3 中段
	//4 编码器B
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);//PWM 1kZ 7199
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);	
	
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1);//编码器 
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_2);
	
	HAL_TIM_Base_Start_IT(&htim3);//每10ms触发一次中断  
	
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_2);	
	
}

#define PWM_LINE 300  //死区 
void Set_Pwm(int moto1,int moto2)
{
			
    	if(moto1<0)			 {IN1_SET;IN2_RESET;Moto1=abs(moto1);}//反转  后退
			else 	          {IN1_RESET;IN2_SET;Moto1=abs(moto1);}//正转    前进
	//
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Moto1+PWM_LINE);//右
			
		  if(moto2<0)	 {IN3_RESET;IN4_SET;Moto2=abs(moto2);}//反转you 后退
			else        {IN3_SET;IN4_RESET;Moto2=abs(moto2);}//正转    前进

			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Moto2+PWM_LINE);//左
			
}
int abs(int p)
{
	int q;
	q=p>0?p:(-p);
	return q;
}
void OLED_Proc(void)
{
	//欧拉角数据
//	OLED_ShowString(0,2,"pit:",12);
//	if(OutMpu.pitch>=0)OLED_ShowString(32,2,"+",12);
//	else OLED_ShowString(32,2,"-",12);
//	OLED_ShowNum(40,2,abs(OutMpu.pitch),3,12);
//	OLED_ShowString(8-8,3,"Gyy:",12);
//	if(OutMpu.roll>=0)OLED_ShowString(32,3,"+",12);
//	else OLED_ShowString(32,3,"-",12);
//	OLED_ShowNum(48-8,3,abs(OutMpu.gyroy),5,12);
//	
//	OLED_ShowString(8-8,4,"Gzz:",12);
//	if(OutMpu.yaw>=0)OLED_ShowString(32,4,"+",12);
//	else OLED_ShowString(32,4,"-",12);
//	OLED_ShowNum(48-8,4,abs(OutMpu.gyroz),3,12);
//	
//	OLED_ShowString(96-8,2,"temp:",12);OLED_ShowNum(96-8,3,OutMpu.temp,4,12);
//		//编码器数据
//	OLED_ShowString(8-8,5,"L:",12);
//	if(Lcnt>0)OLED_ShowString(8+8,5,"+",12);
//	else OLED_ShowString(8+8,5,"-",12);
//	OLED_ShowNum(8+8+8,5,abs(Lcnt),4,12);
//	
//	OLED_ShowString(56,5,"R:",12);
//	if(Rcnt>0)OLED_ShowString(56+8+8,5,"+",12);
//	else OLED_ShowString(56+8+8,5,"-",12);
//	OLED_ShowNum(56+8+8+8,5,abs(Rcnt),4,12);
OLED_ShowCHinese(3,3,0);
OLED_ShowCHinese(3+8+8,3,1);
	adc_volatge=Get_Adc_Average(2);
	OLED_ShowNum(3+8+8+8+8+8,3,adc_volatge,4,12);
	
	OLED_ShowCHinese(3,6,2);
OLED_ShowCHinese(3+8+8,6,3);
OLED_ShowNum(3+8+8+8+8+8,7,OutMpu.temp,4,12);
}

void OLED_Init_show(void)
{
	//GPIO先初始化
	HAL_Delay(200);
	OLED_Init();
	
	//上电显示
	OLED_Clear(); 
	OLED_ColorTurn(0);//0正常显示，1 反色显示
	OLED_DisplayTurn(1);//0正常显示 1 屏幕翻转显示
	OLED_ShowString(16,0,"Balance Car",16);
}

/*****************************************************END OF FILE*********************************************************/	

#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "LED.h"
#include "Timer.h"
#include "Key.h"
#include "MPU6050.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include "BlueSerial.h"
#include "PID.h"
#include "NRF24L01.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

/*预计算常量，避免在中断中重复计算浮点乘除法*/
#define GYRO_TO_ANGLE_SCALE   0.00061035f    /* GY / 32768 * 2000 * 0.01 的合并常量 */
#define ENCODER_TO_SPEED      0.0489993f     /* 1 / (44 * 0.05 * 9.27666) 编码器增量转速度 */

/**
  * 函    数：快速 atan2 角度计算（返回角度值，单位：度）
  * 参    数：y atan2 的 y 参数
  * 参    数：x atan2 的 x 参数
  * 返 回 值：atan2(y, x) 的角度值，单位：度，范围：-180~180
  * 说    明：使用分段线性逼近替代浮点 atan2，最大误差 < 0.3°
  *           在 Cortex-M3 无硬件 FPU 时，比较件浮点 atan2 快 10 倍以上
  */
static inline float FastAtan2Deg(float y, float x)
{
	float abs_y = (y > 0.0f) ? y : -y;
	float angle;
	if (x >= 0.0f)
	{
		float r = (x - abs_y) / (x + abs_y);
		angle = 45.0f - 45.0f * r;			/* 第一、四象限 */
	}
	else
	{
		float r = (x + abs_y) / (abs_y - x);
		angle = 135.0f - 45.0f * r;		/* 第二、三象限 */
	}
	return (y > 0.0f) ? angle : -angle;
}

/*定义全局变量*/
int16_t AX, AY, AZ, GX, GY, GZ;		//读取MPU6050的原始数据
int8_t LV,RH,LH,RV;
uint8_t TimerErrorFlag;	//定时器错误标志位，如果定时中断函数执行时间超过了定时时间，则此标志位置1
uint16_t TimerCount;	//定时器计数值，此值可用于计算定时中断函数具体的执行时间
volatile uint32_t g_Millis;			//全局毫秒计数器，在1ms定时中断中自增
volatile uint8_t NRF24L01_Busy;		//NRF24L01忙标志，防止中断与主循环的SPI冲突
volatile uint8_t NRF24L01_RxReady;	//NRF24L01数据就绪标志，由中断设置，主循环处理

float AngleAcc;			//由加速度计得到的角度值
float AngleGyro;		//由陀螺仪得到的角度值，执行互补滤波后，此值基本与Angle相等
float Angle;			//互补滤波后的角度值，准确且无漂移

uint8_t KeyNum, RunFlag,NRFKeyNum;			//按键键码，运行标志位

int16_t LeftPWM, RightPWM;			//左PWM，右PWM
int16_t AvePWM, DifPWM;				//平均PWM，差分PWM

float LeftSpeed, RightSpeed;		//左速度，右速度
float AveSpeed, DifSpeed;			//平均速度，差分速度

/*定义PID结构体变量*/

PID_t AnglePID = {					//角度环PID结构体变量，定义的时候同时给部分成员赋初值
	.Kp = 6,						//比例项权重
	.Ki = 0.1,						//积分项权重
	.Kd = 6,						//微分项权重

	.OutMax = 100,					//输出限幅的最大值
	.OutMin = -100,					//输出限幅的最小值

	.OutOffset = 4.5,					//输出偏移

	.ErrorIntMax = 600,				//误差积分的最大值
	.ErrorIntMin = -600,			//误差积分的最小值
};

PID_t SpeedPID = {					//角度环PID结构体变量，定义的时候同时给部分成员赋初值
	.Kp = 2,						//比例项权重
	.Ki = 0.05,						//积分项权重
	.Kd = 0.5,						//微分项权重

	.OutMax = 20,					//输出限幅的最大值
	.OutMin = -20,					//输出限幅的最小值

	.ErrorIntMax = 150,				//误差积分的最大值
	.ErrorIntMin = -150,			//误差积分的最小值
};

PID_t TurnPID = {					//转向环PID结构体变量，定义的时候同时给部分成员赋初值
	.Kp = 4,						//比例项权重
	.Ki = 3,						//积分项权重
	.Kd = 0,						//微分项权重

	.OutMax = 50,					//输出限幅的最大值
	.OutMin = -50,					//输出限幅的最小值

	.ErrorIntMax = 20,				//误差积分的最大值
	.ErrorIntMin = -20,				//误差积分的最小值
};

int main(void)
{


	/*模块初始化*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	//统一的NVIC优先级分组，抢占优先级0~3，响应优先级0~3
	OLED_Init();		//OLED初始化
	MPU6050_Init();		//MPU6050初始化
	BlueSerial_Init();	//蓝牙串口初始化
	LED_Init();			//LED初始化
	Key_Init();			//按键初始化
	Motor_Init();		//电机初始化
	Encoder_Init();		//编码器初始化
	Serial_Init();		//串口初始化
	NRF24L01_Init();	//NRF24L01初始化

	Timer_Init();		//定时器初始化，定时中断时间1ms



	while (1)
	{
		/*LED指示程序运行状态*/
		if (RunFlag) {LED_ON();} else {LED_OFF();}		//RunFlag非0时LED点亮

		
		/*按键控制*/
		KeyNum = Key_GetNum();		//获取键码
		if (KeyNum == 1||NRFKeyNum==1)			//如果K1按下
		{
			NRFKeyNum = 0;
			/*RunFlag变量取非*/
			if (RunFlag == 0)		//如果RunFlag为0
			{
				PID_Init(&AnglePID);//启动程序时给PID初始化，清除之前可能遗留的参数
				PID_Init(&SpeedPID);
				PID_Init(&TurnPID);
				RunFlag = 1;		//则RunFlag置1
			}
			else					//否则，RunFlag非0
			{
				RunFlag = 0;		//则RunFlag置0
			}
		}



		/*NRF24L01接收数据包处理*/
		if (NRF24L01_Receive() == 1)				//如果收到数据包
		{
			uint8_t ID = NRF24L01_RxPacket[0];		//字节0，规定为ID，用于区分不同的数据包
			
			if (ID == 0x00)			//如果ID是0x00，则是遥控数据包
			{
//				LV = NRF24L01_RxPacket[1];	//字节1，规定为左摇杆横向值
				LH   = NRF24L01_RxPacket[2];	//字节2，规定为左摇杆纵向值
				RV  = NRF24L01_RxPacket[3];	//字节3，规定为右摇杆横向值
//				RH  = NRF24L01_RxPacket[4];	//字节4，规定为右摇杆纵向值
				uint8_t KEY = NRF24L01_RxPacket[5];	//字节4，规定为按键键码
				  NRFKeyNum = KEY;
			    OLED_Clear();
//			    OLED_Printf(0,  0, OLED_8X16, "LV:%+05d",LV);
//				  OLED_Printf(0, 16,  OLED_8X16, "RH:%+05d",RH);
//				  OLED_Printf(0, 32, OLED_8X16, "KEY:%02d",NRFKeyNum);
		OLED_Printf(0, 8, OLED_6X8, "P:%05.2f", AnglePID.Kp);		//角度环Kp
		OLED_Printf(0, 16, OLED_6X8, "I:%05.2f", AnglePID.Ki);		//角度环Ki
		OLED_Printf(0, 24, OLED_6X8, "D:%05.2f", AnglePID.Kd);		//角度环Kd
			    OLED_Update();	
			
				/*执行摇杆操作*/
				SpeedPID.Target = LH  / 25.0;	//摇杆值LV缩放后，控制速度环目标值，前后行进控制
				TurnPID.Target = -RV / 25.0;		//摇杆值RH缩放后，控制转向环目标值，左右转弯控制	
			}
		}
			/*蓝牙串口接收数据包处理*/
		/*规定的数据包格式为：[数据1,数据2,数据3,...]*/
		if (BlueSerial_RxFlag == 1)		//如果收到数据包
		{
			char *Tag = strtok(BlueSerial_RxPacket, ",");	//提取数据1，定义为标签Tag
			if (strcmp(Tag, "key") == 0)					//Tag为key，收到按键数据包
			{
				char *Name = strtok(NULL, ",");				//提取数据2，定义为按键名称
				char *Action = strtok(NULL, ",");			//提取数据3，定义为按键动作

				/*此处可执行按键操作，目前程序暂时没用到按键*/
			}
			else if (strcmp(Tag, "slider") == 0)			//Tag为slider，收到滑杆数据包
			{
				char *Name = strtok(NULL, ",");				//提取数据2，定义为滑杆名称
				char *Value = strtok(NULL, ",");			//提取数据3，定义为滑杆值

				/*执行滑杆操作*/
				if (strcmp(Name, "AngleKp") == 0)			//如果滑杆名称是AngleKp
				{
					AnglePID.Kp = atof(Value);				//则把滑杆值赋值给角度环Kp
				}
				else if (strcmp(Name, "AngleKi") == 0)		//如果滑杆名称是AngleKi
				{
					AnglePID.Ki = atof(Value);				//则把滑杆值赋值给角度环Ki
				}
				else if (strcmp(Name, "AngleKd") == 0)		//如果滑杆名称是AngleKd
				{
					AnglePID.Kd = atof(Value);				//则把滑杆值赋值给角度环Kd
				}
				else if (strcmp(Name, "SpeedKp") == 0)		//如果滑杆名称是SpeedKp
				{
					SpeedPID.Kp = atof(Value);				//则把滑杆值赋值给速度环Kp
				}
				else if (strcmp(Name, "SpeedKi") == 0)		//如果滑杆名称是SpeedKi
				{
					SpeedPID.Ki = atof(Value);				//则把滑杆值赋值给速度环Ki
				}
				else if (strcmp(Name, "SpeedKd") == 0)		//如果滑杆名称是SpeedKd
				{
					SpeedPID.Kd = atof(Value);				//则把滑杆值赋值给速度环Kd
				}
				else if (strcmp(Name, "TurnKp") == 0)		//如果滑杆名称是TurnKp
				{
					TurnPID.Kp = atof(Value);				//则把滑杆值赋值给转向环Kp
				}
				else if (strcmp(Name, "TurnKi") == 0)		//如果滑杆名称是TurnKi
				{
					TurnPID.Ki = atof(Value);				//则把滑杆值赋值给转向环Ki
				}
				else if (strcmp(Name, "TurnKd") == 0)		//如果滑杆名称是TurnKd
				{
					TurnPID.Kd = atof(Value);				//则把滑杆值赋值给转向环Kd
				}
				else if (strcmp(Name, "Offset") == 0)		//如果滑杆名称是Offset
				{
					AnglePID.OutOffset = atof(Value);		//则把滑杆值赋值给OutOffset
				}
			}
			else if (strcmp(Tag, "joystick") == 0)			//Tag为joystick，收到摇杆数据包
			{
				int8_t LH = atoi(strtok(NULL, ","));		//提取数据2，定义为摇杆值LH
				int8_t LV = atoi(strtok(NULL, ","));		//提取数据3，定义为摇杆值LV
				int8_t RH = atoi(strtok(NULL, ","));		//提取数据4，定义为摇杆值RH
				int8_t RV = atoi(strtok(NULL, ","));		//提取数据5，定义为摇杆值RV

				/*执行摇杆操作*/
				SpeedPID.Target = LV / 25.0;	//摇杆值LV缩放后，控制速度环目标值，前后行进控制
				TurnPID.Target = RH / 25.0;		//摇杆值RH缩放后，控制转向环目标值，左右转弯控制
			}

			BlueSerial_RxFlag = 0;				//处理完成后，标志位置0，允许接收下一个数据包
		}


	}
}

void TIM1_UP_IRQHandler(void)
{
	static uint16_t Count0, Count1;

	if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		/*定时中断函数1ms自动执行一次*/

		/*进入中断函数后，立刻清标志位*/
		/*如果中断函数退出前，标志位又置1了，说明中断函数执行时间超过了定时时间（1ms）*/
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

		Key_Tick();				//调用按键的Tick函数

		/*计次分频*/
		Count0 ++;				//计次自增
		if (Count0 >= 10)		//如果计次10次，则if成立，即if每隔10ms进一次
		{
			Count0 = 0;			//计次清零，便于下次计次

			/*在中断里读取MPU6050，可以保证读取间隔严格为1ms*/
			/*但要保证MPU6050_GetData执行时间不超过1ms*/
			MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);

			/*校准陀螺仪Y轴零漂*/
			/*此值需实测确定，不同的设备零漂一般不同*/
			/*实测方法是，在完全静止时，观察OLED显示的GY值，即为零漂值*/
			/*然后在此处将零漂值减去，使得完全静止时，GY值为0*/
			GY -= 16;

			/*由加速度计计算得到角度值*/
			/*使用快速 atan2 近似替代浮点 atan2，大幅减少 CPU 占用（M3 无硬件 FPU）*/
			/*公式：-atan2(AX, AZ) * 180 / PI，结果单位为度*/
			AngleAcc = -FastAtan2Deg((float)AX, (float)AZ);

			/*校准中心角度*/
			/*此值需实测确定，不同的设备中心角度一般不同*/
			/*实测方法是，使平衡车绝对竖直，观察OLED显示的角度环实际值，即为中心角度偏移值*/
			/*然后在此处将偏移值减去，使得绝对竖直时，角度环实际值为0*/
			AngleAcc += 3.8;

			/*由陀螺仪积分得到角度值*/
			/*互补滤波下，角度积分要在上次滤波后的Angle上进行*/
			/*GYRO_TO_ANGLE_SCALE = 2000 * 0.01 / 32768，三个常量预计算合并，减少中断内运算*/
			AngleGyro = Angle + GY * GYRO_TO_ANGLE_SCALE;

			/*执行互补滤波*/
			/*系数 Alpha = 0.01，在 10ms 采样周期下，时间常数约 1s*/
			/*值越大越偏向加速度计（响应快但噪声大），值越小越偏向陀螺仪（平滑但易漂移）*/
			float Alpha = 0.01f;
			Angle = Alpha * AngleAcc + (1.0f - Alpha) * AngleGyro;	/* 一阶互补滤波 */

			/*平衡车倒地后自动停止PID程序*/
			if (Angle > 50 || Angle < -50)	//角度超过-50度~50度的范围，认为平衡车倒地了
			{
				RunFlag = 0;				//RunFlag置0，停止PID程序
			}

			/*执行PID调控程序*/
			if (RunFlag)					//RunFlag非0时，启动PID程序
			{
				/*根据PWM大小动态调整角度环PID参数*/
				if (LH != 0 || RV != 0)
				{
					AnglePID.Kp = 3;
					AnglePID.Ki = 0.1;
					AnglePID.Kd = 3;
				}
				else
				{
					AnglePID.Kp = 6;
					AnglePID.Ki = 0.1;
					AnglePID.Kd = 6;
				}

				/*角度环PID控制*/
				AnglePID.Actual = Angle;	//角度环实际值为Angle
				PID_Update(&AnglePID);		//调用封装好的函数，一步完成PID计算和更新
				AvePWM = -AnglePID.Out;		//角度环的输出值给到电机平均PWM，用于控制前后行进

				/*控制量转换*/
				LeftPWM = AvePWM + DifPWM / 2;		//由平均PWM和差分PWM计算得到左轮PWM
				RightPWM = AvePWM - DifPWM / 2;		//由平均PWM和差分PWM计算得到右轮PWM

				/*PWM限幅*/
				/*上式计算后，LeftPWM和RightPWM可能会超出电机允许的PWM范围，此处将PWM值范围限制在-100~100之内*/
				if (LeftPWM > 100) {LeftPWM = 100;} else if (LeftPWM < -100) {LeftPWM = -100;}
				if (RightPWM > 100) {RightPWM = 100;} else if (RightPWM < -100) {RightPWM = -100;}

				/*PWM输出给电机*/
				Motor_SetPWM(1, LeftPWM);		//LeftPWM输出给左轮电机
				Motor_SetPWM(2, RightPWM);		//RightPWM输出给右轮电机
			}
			else		//RunFlag为0时，停止PID程序
			{
				/*左右电机PWM均设置为0*/
				Motor_SetPWM(1, 0);
				Motor_SetPWM(2, 0);
			}
		}

		/*计次分频*/
		Count1 ++;				//计次自增
		if (Count1 >= 50)		//如果计次50次，则if成立，即if每隔50ms进一次
		{
			Count1 = 0;			//计次清零，便于下次计次

			/*获取编码器的计次值增量，并计算电机旋转速度*/
			/*ENCODER_TO_SPEED = 1 / (44 * 0.05 * 9.27666)，四个常量预计算合并*/
			/*编码器磁铁每圈 44 个计数，读取间隔 50ms，减速比 9.27666*/
			LeftSpeed = Encoder_Get(1) * ENCODER_TO_SPEED;
			RightSpeed = Encoder_Get(2) * ENCODER_TO_SPEED;

			/*信号量转换*/
			AveSpeed = (LeftSpeed + RightSpeed) / 2.0;	//由左轮速度和右轮速度计算得到平均速度
			DifSpeed = LeftSpeed - RightSpeed;			//由左轮速度和右轮速度计算得到差分速度

			/*执行PID调控程序*/
			if (RunFlag)					//RunFlag非0时，启动PID程序
			{
				/*速度环PID控制*/
				SpeedPID.Actual = AveSpeed;			//速度环实际值为AveSpeed
				PID_Update(&SpeedPID);				//调用封装好的函数，一步完成PID计算和更新
				AnglePID.Target = SpeedPID.Out;		//速度环的输出值给到角度环的目标值，构成串级PID

				/*转向环PID控制*/
				TurnPID.Actual = DifSpeed;			//转向环实际值为DifSpeed
				PID_Update(&TurnPID);				//调用封装好的函数，一步完成PID计算和更新
				DifPWM = TurnPID.Out;				//转向环的输出值给到电机差分PWM，用于控制左右转弯
			}
		}

		/*中断函数退出前，再次检查标志位*/
		if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
		{
			/*标志位又置1了，说明中断函数执行时间超过了定时时间（1ms）*/
			/*置TimerErrorFlag为1，表示定时中断错误*/
			TimerErrorFlag = 1;

			/*清标志位，避免中断连续触发，导致主函数完全无法执行*/
			TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		}

		/*中断函数退出前，读取计数器的值，此值可用于测量中断函数的具体执行时间*/
		TimerCount = TIM_GetCounter(TIM1);
	}
}

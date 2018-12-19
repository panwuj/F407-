#include "main.h"
#include "stdio.h"

#define FLIGHTCONTROL_TASK_PRIO	1
#define FLIGHTCONTROL_TASK_STACK_SIZE 	1024
TaskHandle_t FlightControl_Task_Handler;

#define	BARO_TASK_PRIO	2
#define BARO_STK_SIZE		128
TaskHandle_t ReadBarometer_Task_Handler;

#define FUN_TASK_PRIO	4
#define FUN_STK_SIZE	128
TaskHandle_t FUNTask_Handler;

#define PORT_TASK_PRIO	3
#define PORT_STK_SIZE	128
TaskHandle_t PORTTask_Handler;

extern QueueHandle_t recorder_queue;
extern TaskHandle_t RecorderTask_Handler;

uint8_t SD_mounted=0;

void SysTaskCreate(void);
void FlightControl_Task(void *pvParameters);
void ReadBarometer_Task(void *pvParameters);
void FunSelect_task(void *pvParameters);
void Port_task(void *pvParameters);
void SDcard_Mount(void);

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); 
	DelayInit(168);
	LED_Init();
	BEEP_Init(); 
	TIM5_Init(0xffffffff,84-1);
	SPI1_Init();
	i2cdevInit();
	MS5611_Init();
	MPU6050_Init();
	HMC5883L_Init();
	USART1_Init(100000);
	USART2_Init(115200);
	USART3_Init(9600); 
	UART4_Init(115200);
//	UART5_Init(115200);
//	SDcard_Mount();
	global_params_Init();
	Motor_Init();

	SysTaskCreate();
  vTaskStartScheduler();          //开启任务调度
}
//开始任务任务函数
void SysTaskCreate(void)
{
    taskENTER_CRITICAL();           //进入临界区
	
//		if(SD_mounted)
//			recorder_queue=xQueueCreate(REC_MSG_NUM,sizeof(_flight_recoder_ *));
//		else
//			recorder_queue=NULL;
		
//		if(recorder_queue!=NULL)
//				xTaskCreate(FlightDataRecord_task,
//										"REC_task",
//										RECORD_STK_SIZE,
//										NULL,RECORD_TASK_PRIO,
//									  RecorderTask_Handler);
		
		xTaskCreate(FlightControl_Task,
								"FlightCtrl",
								FLIGHTCONTROL_TASK_STACK_SIZE,
								NULL,FLIGHTCONTROL_TASK_PRIO,
								&FlightControl_Task_Handler);
	
		xTaskCreate(ReadBarometer_Task,
								"ReadBaro",
								BARO_STK_SIZE,
								NULL,BARO_TASK_PRIO,
								&ReadBarometer_Task_Handler);
								
		xTaskCreate((TaskFunction_t )FunSelect_task,     	
                (const char*    )"fun_task",   	
                (uint16_t       )FUN_STK_SIZE, 
                (void*          )NULL,				
                (UBaseType_t    )FUN_TASK_PRIO,	
                (TaskHandle_t*  )&FUNTask_Handler);        

    xTaskCreate((TaskFunction_t )Port_task,     
                (const char*    )"Port_task",   
                (uint16_t       )PORT_STK_SIZE, 
                (void*          )NULL,
                (UBaseType_t    )PORT_TASK_PRIO,
                (TaskHandle_t*  )&PORTTask_Handler); 
								
    taskEXIT_CRITICAL();            //退出临界区
								
}

void FlightControl_Task(void *pvParameters)
{
	LED3=0;
	RemoteControl_Init();
	controllerInit();
	FlightCtrlSysInit();
	LED2=0;
	while(true)
	{
		GetAttitude();
		GetSensorsData();
		ReadRemoteControl();
		FlightControl();
	}
}
void ReadBarometer_Task(void *pvParameters)
{
	while(true)
	{
		if(alt_ground_set)
		{
			altitude_baro=MS5611_get_height()-altitude_baro_ground;
		}
		vTaskDelay(10);
	}
}

void FunSelect_task(void *pvParameters)
{
	vTaskDelay(3000);
	while(true)
	{
		if(flight_function==2)
		{
			flight_task_2();
//			flight_function=0;
		}
		if(flight_function==3)
		{
			flight_task_3();
//			flight_function=0;
		}
		if(flight_function==4)
		{
			flight_task_4();
			flight_function=0;
		}
		vTaskDelay(10);
	}
}

void Port_task(void *pvParameters)
{
	u8 flag=1;
	while(true)
	{
		GetSystemInput();
		pid_setting++;
		if(pid_setting>10000000)
		{
			pid_setting=10000000;
			
//			ANO_Send_To_GroundStation();
//		RAPOO_data_send();
//			if(flag)
//			{
//				flag=0;
//				ANO_Send_Status();
//			}
//			else
//			{
//				flag=1;
//				ANO_Send_Sensors();
//			}
//			ANO_Send_Status();
//			ANO_Send_UserData();
			vTaskDelay(10);
		}
	}
}


void SDcard_Mount(void)
{
	FRESULT fs_err;
	FATFS fs;
	uint8_t count=0;
	while(disk_initialize(0)) //0 stand for SD_CARD
	{
		count++;
		if(count==20)
		{
			SD_mounted=0;
			return;
		}
		printf("sdcard check error!");
		LED0=!LED0;
		delay_ms(250);
		disk_initialize(0);
	}
	fs_err=f_mount(&fs,"0",1);
	if(fs_err==FR_OK)
		SD_mounted=1;
	else
		SD_mounted=0;
}





































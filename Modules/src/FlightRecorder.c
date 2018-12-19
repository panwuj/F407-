//#include "FlightRecorder.h"
//#include "stdio.h"

//TaskHandle_t RecorderTask_Handler;

//QueueHandle_t recorder_queue;
//_flight_recoder_ recoder_data[REC_MSG_NUM];

//char str[1024*4];
//uint16_t len=0;
//uint16_t len_rec_test=0;
//extern u8 ass_locked;
//extern u8 use_locked;
///**************      1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16  17   18   19   20   21   22   23  24 25  26   27   28   29   30   31   32   33   34   35  36 37 38 39 40 41 42 43 44 45*****/
////const char format[]="%d %d %d %d %d %d %d %d %d %f %f %f %f %f %f %f %.2f %.2f %.2f %.2f %.2f %.2f %.2f %f %f %.2f %.2f %.2f %.2f %.3f %.3f %.3f %.3f %.3f %.3f %d %d %d %d %d %d %d %d %d %d\r\n";                                                

//void DataConvertToData(_flight_recoder_ *rec_data,uint16_t *len);
//void FlightDataRecord_task(void *pvParameters)
//{
//	FRESULT fs_err;
//	static FIL file;
//	static u8 i=0;
//	_flight_recoder_ *rec_data;
//	//creat a new filght record.
//	fs_err=f_unlink("0:\\fligt_record4.txt");
//	fs_err=f_open(&file,"0:\\fligt_record4.txt",FA_CREATE_NEW|FA_WRITE);
//	if(fs_err!=FR_OK)
//	{
//		vTaskDelete(RecorderTask_Handler);
//		while(1);
//	}		
//	while(true)
//	{
////		if(i<2){
////			if(xQueueReceive(recorder_queue,&rec_data,0))
////			{
////				taskENTER_CRITICAL();
////				DataConvertToData(rec_data,&len);
////				taskEXIT_CRITICAL();
////				i++;
////			}
////		}
////		if(i==2)
////		{
////			str[len]=0;
//////			fs_err=f_puts(str,&file);
//////			fs_err=f_sync(&file);//save the file in time.
////			LED1=!LED1;
////			i=0;
////			len_rec_test++;
////		}
////		vTaskDelay(100);
//		
//		if(!ass_locked)
//		{
//			use_locked=1;
//			DataConvertToData(&recoder_data[0],&len);
//			use_locked=0;
//			str[len]=0;
//			fs_err=f_puts(str,&file);
//			fs_err=f_sync(&file);//save the file in time.
//			LED1=!LED1;
//			len=0;
//		}
//		vTaskDelay(1);
//	}
//}

//void RecorderDataAssign(_flight_recoder_ *rec_data)
//{
//	rec_data->acc_raw[0]=mpu6050.ax;
//	rec_data->acc_raw[1]=mpu6050.ay;
//	rec_data->acc_raw[2]=mpu6050.az;
//	rec_data->gyro_raw[0]=mpu6050.gx;
//	rec_data->gyro_raw[1]=mpu6050.gy;
//	rec_data->gyro_raw[2]=mpu6050.gz;
//	rec_data->magn_raw[0]=hmc5883l.x;
//	rec_data->magn_raw[1]=hmc5883l.y;
//	rec_data->magn_raw[2]=hmc5883l.z;
//	rec_data->imu_update_dt=imu_update_dt;
//	rec_data->alt_ctrl_dt=alt_update_dt;
//	rec_data->vel_ctrl_dt=vel_ctrl_dt;
//	rec_data->q[0]=q[0];
//	rec_data->q[1]=q[1];
//	rec_data->q[2]=q[2];
//	rec_data->q[3]=q[3];
//	rec_data->att_actual[0]=Roll;
//	rec_data->att_actual[1]=Pitch;
//	rec_data->att_actual[2]=Yaw;
//	rec_data->att_desired[0]=global_params.AttitudeDesired.roll;
//	rec_data->att_desired[1]=global_params.AttitudeDesired.pitch;
//	rec_data->att_desired[2]=global_params.AttitudeDesired.yaw;
//	rec_data->yaw_rate_desired=yawRateDesired;
//	rec_data->longitude=gps.longitude;
//	rec_data->latitude=gps.latitude;
//	rec_data->pos_est[0]=EST_X[0];
//	rec_data->pos_est[1]=EST_Y[0];
//	rec_data->pos_desired[0]=posDesired[0];
//	rec_data->pos_desired[1]=posDesired[1];
//	rec_data->altitude_baro=altitude_baro;
//	rec_data->altitude_est=global_params.AltitudeActual;
//	rec_data->altitude_desired=global_params.AltitudeDesired;
//	rec_data->vel_actual[0]=EST_X[1];
//	rec_data->vel_actual[1]=EST_Y[1];
//	rec_data->vel_actual[2]=VerVelEst;
//	rec_data->motor[0]=motor1;
//	rec_data->motor[1]=motor2;
//	rec_data->motor[2]=motor3;
//	rec_data->motor[3]=motor4;
//	rec_data->ctrl_mode=global_params.ControlMode;
//	rec_data->rc_value[0]=global_params.rc[ROLL].value;
//	rec_data->rc_value[1]=global_params.rc[PITCH].value;
//	rec_data->rc_value[2]=global_params.rc[THROTTLE].value;
//	rec_data->rc_value[3]=global_params.rc[YAW].value;
//	rec_data->rc_value[4]=global_params.rc[MODE].value;
//}

//void DataConvertToData(_flight_recoder_ *rec_data,uint16_t *len)
//{
//	*len += sprintf(str+(*len),format,
//							rec_data->acc_raw[0],
//							rec_data->acc_raw[1],
//							rec_data->acc_raw[2],
//							rec_data->gyro_raw[0],
//							rec_data->gyro_raw[1],
//							rec_data->gyro_raw[2],
//							rec_data->magn_raw[0],
//							rec_data->magn_raw[1],
//							rec_data->magn_raw[2],
//							rec_data->imu_update_dt,
//							rec_data->alt_ctrl_dt,
//							rec_data->vel_ctrl_dt,
//							rec_data->q[0],
//							rec_data->q[1],
//							rec_data->q[2],
//							rec_data->q[3],
//							rec_data->att_actual[0],
//							rec_data->att_actual[1],
//							rec_data->att_actual[2],
//							rec_data->att_desired[0],
//							rec_data->att_desired[1],
//							rec_data->att_desired[2],
//							rec_data->yaw_rate_desired,
//							rec_data->longitude,
//							rec_data->latitude,
//							rec_data->pos_est[0],
//							rec_data->pos_est[1],
//							rec_data->pos_desired[0],
//							rec_data->pos_desired[1],
//							rec_data->altitude_baro,
//							rec_data->altitude_est,
//							rec_data->altitude_desired,
//							rec_data->vel_actual[0],
//							rec_data->vel_actual[1],
//							rec_data->vel_actual[2],
//							rec_data->motor[0],
//							rec_data->motor[1],
//							rec_data->motor[2],
//							rec_data->motor[3],
//							rec_data->ctrl_mode,
//							rec_data->rc_value[0],
//							rec_data->rc_value[1],
//							rec_data->rc_value[2],
//							rec_data->rc_value[3],
//							rec_data->rc_value[4]);
//}

//void FlightDataEnQueue(void)
//{
//	static uint8_t count=0;
//	_flight_recoder_ *rec_data;
//	BaseType_t err;
//	if(recorder_queue==NULL)
//		return;
//	if(uxQueueSpacesAvailable(recorder_queue)>=2)
//	{
//		rec_data=recoder_data+count;
//		RecorderDataAssign(rec_data);
//		err=xQueueSend(recorder_queue,&rec_data,0);
//		if(err!=errQUEUE_FULL)
//		{
//			count++;
//			if(count==REC_MSG_NUM)
//				count=0;
//		}
//	}
//}
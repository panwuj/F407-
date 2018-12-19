#include "GPS.h"
#include "string.h"
#include "stdlib.h"

#define GPS_SCALE 6371000

gps_struct gps;
uint8_t NMEA_comma_pos(u8 *buf,u8 cn)
{
	u8 *p=buf;
	while(cn)
	{
		if(*buf=='*'||*buf<' '||*buf>'z')
			return 0xff;		//遇到'*'或者非法字符,则不存在第cn个逗号
		if(*buf==',')	cn--;
		buf++;
	}
	return buf-p;
}
//return 
//0:available positioning 
//1:invalid positioning
//2:other error.
uint8_t NMEA_GNRMC_Analysis(gps_struct *gps,u8 *buf)
{
	u8 *p;
	u8 pos;
	uint8_t res;
	p=(u8*)strstr((const char*)buf,"$GNRMC");
	//check is positioning available?
	pos=NMEA_comma_pos(p,2);
	if(pos==0xff)	return 2;
	if(*(p+pos)=='A')	res=0;
	else if(*(p+pos)=='V') return 1;
	else return 2;
	//get latitude
	pos=NMEA_comma_pos(p,3);
	if(pos==0xff) return 2;
	gps->latitude=atof((const char*)(p+pos));
	//location in north hemisphere or south hemisphere. 
	pos=NMEA_comma_pos(p,4);
	if(pos==0xff)	return 2;
	gps->NShemisphere=*(p+pos);
	//get longitude
	pos=NMEA_comma_pos(p,5);
	if(pos==0xff)	return 2;
	gps->longitude=atof((const char*)(p+pos));
	//location in east or west.
	pos=NMEA_comma_pos(p,6);
	if(pos==0xff) return 2;
	gps->EWhemisphere=*(p+pos);
	//get speed
	pos=NMEA_comma_pos(p,7);
	if(pos==0xff)	return 2;
	gps->speed=atof((const char*)(p+pos));
	gps->speed *=0.51444f;//unit convert to m/s
	
	return res;
}

uint8_t NMEA_GNGGA_Analysis(gps_struct *gps,u8 *buf)
{
	u8 *p;
	u8 pos;
	p=(u8*)strstr((const char*)buf,"$GNGGA");
	pos=NMEA_comma_pos(p,7);
	if(pos==0xff) return 2;
	gps->svnum=atoi((const char*)(p+pos));
	pos=NMEA_comma_pos(p,9);
	if(pos==0xff) return 2;
	gps->altitude=atof((const char*)(p+pos));
	return 0;
}
void GPS_ConvertToRadian(double *lat)
{
	int16_t degree=*lat/100;
	double minute=*lat-(float)degree*100;
	*lat=degree+minute/60;
	*lat *=3.141592653;
	*lat /=180;
}

void GPS_Monitor(gps_struct *gps,uint8_t res1,uint8_t res2)
{
	static double longitude_last;
	static double latitude_last;
//	static uint8_t fun_first_run=1;
//	if(fun_first_run)
//	{
//		
//		fun_first_run=0;
//		
//	}
	if(gps->svnum>=7&&res1==0&&res2==0)
	{
		gps->available=1;
		gps->update=1;
	}
	else
	{
		gps->available=0;
		gps->update=0;
		gps->HomeSet=0;
	}
}

void GPS_Analysis(u8 *buf)
{
	uint8_t res1,res2;
	res1=NMEA_GNRMC_Analysis(&gps,buf);
	res2=NMEA_GNGGA_Analysis(&gps,buf);
	GPS_ConvertToRadian(&gps.latitude);
	GPS_ConvertToRadian(&gps.longitude);
	GPS_Monitor(&gps,res1,res2);
	if(gps.HomeSet)
	{
		gps.pos_x=(gps.latitude-global_params.HomePoint.latitude)*GPS_SCALE;
		gps.pos_y=-(gps.longitude-global_params.HomePoint.longitude)*GPS_SCALE;
	}
	else
	{
		gps.pos_x=0;
		gps.pos_y=0;
	}
}
void GPS_Update(void)
{
//	if(usart3.update)
//	{
//		usart3.update=0;
//		GPS_Analysis(usart3.rx_buf);
//	}
	if(usart2.update)
	{
		usart2.update=0;
		GPS_Analysis(usart2.rx_buf);
	}
}
void GPS_HomeReset(void)
{
	if(gps.available)
	{
		global_params.HomePoint.longitude=gps.longitude;
		global_params.HomePoint.latitude=gps.latitude;
		global_params.HomePoint.altitude=0;
		gps.pos_x=0;
		gps.pos_y=0;
		position_estimate_reset();
		gps.HomeSet=1;
	}
}
void Location_Init(void)
{
	gps.longitude=0;
	gps.latitude=0;
	gps.altitude=0;
	gps.speed=0;
	gps.pos_x=0;
	gps.pos_y=0;
	gps.NShemisphere='N';
	gps.EWhemisphere='E';
	gps.svnum=0;
	gps.available=0;
	gps.update=0;
	gps.HomeSet=0;
}
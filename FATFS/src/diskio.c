/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2013        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control module to the FatFs module with a defined API.        */
/*-----------------------------------------------------------------------*/

#include "diskio.h"		/* FatFs lower layer API */
#include "sdio_sdcard.h"
//#include "malloc.h"
#include "FreeRTOS.h"

#define SD_CARD	 0  //SD卡,卷标为0
#define EX_FLASH 1	//外部flash,卷标为1

#define FLASH_SECTOR_SIZE 	512			  
//对于W25Q128
//前12M字节给fatfs用,12M字节后,用于存放字库,字库占用3.09M.	剩余部分,给客户自己用	 			    
u16	    FLASH_SECTOR_COUNT=2048*12;	//W25Q1218,前12M字节给FATFS占用
#define FLASH_BLOCK_SIZE   	8     	//每个BLOCK有8个扇区

//初始化磁盘
DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber (0..) */
)
{
	u8 err=0;	    
	switch(pdrv)
	{
		case SD_CARD:       //SDcard initialize
			err=SD_Init();    
  			break;
		default:
			err=1; 
	}		 
	if(err)
		return  STA_NOINIT;
	else 
		return 0; //initialize complete
}  

//获得磁盘状态
DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber (0..) */
)
{ 
	return 0;
} 

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address (LBA) */
	UINT count		/* Number of sectors to read (1..128) */
)
{
	u8 err=0;
	u8 i=0;
  if (!count)
		return RES_PARERR;	 	 
	switch(pdrv)
	{
		case SD_CARD://SD卡
			err=SDIO_ReadDisk(buff,sector,count);	 
			while(err)
			{
				if(i==20)
					break;
				SD_Init();	
				err=SDIO_ReadDisk(buff,sector,count);
				i++;
			}
			break;
		default:
			err=1; 
	}
  if(err==0x00)
		return RES_OK;	 
  else 
		return RES_ERROR;	   
}

#if _USE_WRITE
DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber (0..) */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address (LBA) */
	UINT count			/* Number of sectors to write (1..128) */
)
{
	u8 err=0;  
  if (!count)
		return RES_PARERR;//count不能等于0，否则返回参数错误		 	 
	switch(pdrv)
	{
		case SD_CARD://SD卡
			err=SDIO_WriteDisk((u8*)buff,sector,count);
			while(err)//写出错
			{
				SD_Init();	//重新初始化SD卡
				err=SDIO_WriteDisk((u8*)buff,sector,count);	
				//printf("sd wr error:%d\r\n",res);
			}
			break;
		default:
			err=1; 
	}
    //处理返回值，将SPI_SD_driver.c的返回值转成ff.c的返回值
    if(err == 0x00)
			return RES_OK;	 
    else 
			return RES_ERROR;	
}
#endif

#if _USE_IOCTL
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		  /* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT err;						  			     
	if(pdrv==SD_CARD)//SD卡
	{
	    switch(cmd)
	    {
		    case CTRL_SYNC:
				err = RES_OK; 
		        break;	 
		    case GET_SECTOR_SIZE:
				*(DWORD*)buff = 512; 
		        err = RES_OK;
		        break;	 
		    case GET_BLOCK_SIZE:
				*(WORD*)buff = SDCardInfo.CardBlockSize;
		        err = RES_OK;
		        break;	 
		    case GET_SECTOR_COUNT:
		        *(DWORD*)buff = SDCardInfo.CardCapacity/512;
		        err = RES_OK;
		        break;
		    default:
		        err = RES_PARERR;
		        break;
	    }
	}
	else 
		err=RES_ERROR;//其他的不支持
  return err;
}
#endif

//User defined function to give a current time to fatfs module      */
//31-25: Year(0-127 org.1980), 24-21: Month(1-12), 20-16: Day(1-31) */                                                                                                                                                                                                                                          
//15-11: Hour(0-23), 10-5: Minute(0-59), 4-0: Second(0-29 *2) */                                                                                                                                                                                                                                                
DWORD get_fattime (void)
{				 
	return 0;
}			 


void *ff_memalloc (UINT size)			
{
//	return (void*)mymalloc(SRAMIN,size);
	return pvPortMalloc((size_t)size);
}

void ff_memfree (void* mf)		 
{
//	myfree(SRAMIN,mf);
	vPortFree(mf);
}
















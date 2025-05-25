#ifndef __BSP_SD_H_
#define __BSP_SD_H_

#include "sdcard.h"

extern sd_card_info_struct sd_cardinfo;                            /* information of SD card */

void sd_nvic_config(void);
sd_error_enum sd_io_init(void);
void card_info_get(void);
uint8_t  FatFs_FileTest(void);	//文件创建和写入测试

#endif


 


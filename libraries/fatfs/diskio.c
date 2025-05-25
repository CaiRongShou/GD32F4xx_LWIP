/*-----------------------------------------------------------------------*/
/* Low level disk I/O module SKELETON for FatFs     (C)ChaN, 2019        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "ff.h"			/* Obtains integer types */
#include "diskio.h"		/* Declarations of disk functions */

/* Definitions of physical drive number for each drive */
#define SD_CARD 0
#define SPI_FLASH 1
#define INTER_FLASH 2
#define FLASH_SECTOR_COUNT 512 /*SPI_Flash SECTOR number*/
#define FLASH_SECTOR_SIZE 4096 /*SPI_Flash SECTOR size*/
#define FLASH_BLOCK_SIZE 1 /*smallest unit of erased sector*/
#define SD_CARD_BLOCK_SIZE 1
#define FMC_WRITE_START_ADDR ((uint32_t)0x08000000U)

#include "bsp_sd.h"
#include "string.h"


/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
	switch(pdrv) {
		case SD_CARD :
			return 0;
		case SPI_FLASH :
//			if(spi_flash_ID_read() == 0xC84015) {
//			stat = 0; //initialization normal
//			} else {
//			stat = STA_NOINIT; //initialize not normal
//			}
			return stat;
		case INTER_FLASH:
			stat = 0;
			return stat;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/
DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
	switch(pdrv) {
		case SD_CARD :
			stat &= ~STA_NOINIT;
			return 0;
		case SPI_FLASH :
//			spi_flash_config();
//			return disk_status(SPI_FLASH);
		case INTER_FLASH:
			stat = 0;
			return stat;
	}
	return STA_NOINIT;
}

/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	LBA_t sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	uint32_t *ptrd, *btrd;
	DRESULT res;
	sd_error_enum SD_stat = SD_OK;
	switch(pdrv) {
		case SD_CARD :
			if(count > 1) 
				SD_stat = sd_multiblocks_read((uint32_t *)buff, sector * sd_cardinfo.card_blocksize, sd_cardinfo.card_blocksize, count);
			else 
				SD_stat = sd_block_read((uint32_t *)buff, sector * sd_cardinfo.card_blocksize, sd_cardinfo.card_blocksize);
			
			if(SD_stat == SD_OK)
				res = RES_OK ;
			else 
				res = RES_ERROR ;
			
			return res;
		case SPI_FLASH :
//			spi_flash_buffer_read((uint8_t *)buff, sector * FLASH_SECTOR_SIZE, count * 
//			FLASH_SECTOR_SIZE);
//			res = RES_OK;
			return res;
		case INTER_FLASH:
//			btrd = (uint32_t *)buff;
//			for(ptrd = (uint32_t *)(FMC_WRITE_START_ADDR + (sector + 47) * 2048);ptrd < (uint32_t 
//			*)(FMC_WRITE_START_ADDR + ((sector + 47) * 2048) + (count * 2048)); ptrd++) {
//			*btrd = *ptrd;
//			btrd++;
//			}
//			res = RES_OK;
			return res;
	}
	return RES_PARERR;
}


/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	LBA_t sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	DRESULT res;
	sd_error_enum SD_stat = SD_OK;
	uint32_t address;
	uint32_t erase_counter;
	switch(pdrv) {
		case SD_CARD :
		if(count > 1) 
			SD_stat = sd_multiblocks_write((uint32_t *)buff, sector * sd_cardinfo.card_blocksize, sd_cardinfo.card_blocksize, count);
		else 
			SD_stat = sd_block_write((uint32_t *)buff, sector * sd_cardinfo.card_blocksize,sd_cardinfo.card_blocksize);
		
		if(SD_stat == SD_OK) 
			res = RES_OK ;
		else 
			res = RES_ERROR ;
		
		return res;
		case SPI_FLASH:
			/*first erase then write*/
//			spi_flash_sector_erase(sector * FLASH_SECTOR_SIZE);
//			spi_flash_buffer_write((uint8_t *)buff, sector * FLASH_SECTOR_SIZE, count * FLASH_SECTOR_SIZE);
			res = RES_OK;
			return res;
		case INTER_FLASH:
//			fmc_unlock();
//			fmc_flag_clear(FMC_FLAG_BANK0_END);
//			fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
//			fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
//			/* erase the flash pages */
//			for(erase_counter = 0; erase_counter < count; erase_counter++) 
//			{
//				fmc_page_erase(FMC_WRITE_START_ADDR + ((sector + 47) * 2048) + (2048 * 
//				erase_counter));
//				fmc_flag_clear(FMC_FLAG_BANK0_END);
//				fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
//				fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
//			}
//			address = (sector + 47) * 2048 + FMC_WRITE_START_ADDR;
//			while(address < (((sector + 47) * 2048 + FMC_WRITE_START_ADDR) + count * 2048)) 
//			{
//				fmc_word_program(address, *(uint32_t *)buff);
//				address += 4;
//				buff += 4;
//				fmc_flag_clear(FMC_FLAG_BANK0_END);
//				fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
//				fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
//			}
//			fmc_lock();
			res = RES_OK;
		return res;
	}
	return RES_PARERR;
}
#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/
DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res;
	switch(pdrv) 
	{
		case SD_CARD :
			switch(cmd) 
			{
				/*return sector number*/
				case GET_SECTOR_COUNT:
				*(DWORD *)buff = sd_cardinfo.card_capacity / (sd_cardinfo.card_blocksize);
				break;
				/*return each sector size*/
				case GET_SECTOR_SIZE:
				*(WORD *)buff = sd_cardinfo.card_blocksize;
				break;
				/*Returns the smallest unit of erased sector (unit 1)*/
				case GET_BLOCK_SIZE:
				*(DWORD *)buff = SD_CARD_BLOCK_SIZE;
				break;
			}
			res = RES_OK;
			return res;
		case SPI_FLASH :
//			switch(cmd) 
//			{
//				/*return sector number*/
//				case GET_SECTOR_COUNT:
//				*(DWORD *)buff = FLASH_SECTOR_COUNT;
//				break;
//				/*return each sector size*/
//				case GET_SECTOR_SIZE:
//				*(WORD *)buff = FLASH_SECTOR_SIZE;
//				break;
//				/*Returns the smallest unit of erased sector (unit 1)*/
//				case GET_BLOCK_SIZE:
//				*(DWORD *)buff = FLASH_BLOCK_SIZE;
//				break;
//			}
			res = RES_OK;
			return res;
		case INTER_FLASH:
//			switch(cmd) 
//			{
//				/*return sector number*/
//				case GET_SECTOR_COUNT:
//				*(DWORD *)buff = 128;
//				break;
//				/*return each sector size*/
//				case GET_SECTOR_SIZE:
//				*(WORD *)buff = 2048;
//				break;
//				/*Returns the smallest unit of erased sector (unit 1)*/
//				case GET_BLOCK_SIZE:
//				*(DWORD *)buff = 1;
//				break;
//			}
			res = RES_OK;
			return res;
	}
	return RES_PARERR;
} 
 
 
 
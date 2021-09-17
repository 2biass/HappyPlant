#include "hp_myfs.h"
#include "hp_sdcard.h"

static FATFS* myfs;
static const char *TAG = "HappyPlant myfs";

/*-----------------------------------------------------------------------*/
/* Initialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS myfs_initialize(BYTE pdrv)
{
    /* Initialise the SD driver */
    if(SD_init() != 0)
    {
        return RES_ERROR;
    }
    
    return RES_OK;
}

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS myfs_disk_status(BYTE pdrv)
{
    /* This feature is not implemented in this project. */
    DRESULT res = RES_OK;
    return res;
}

/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT myfs_disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	unsigned char state;
	for(unsigned int i = 0; i < count; i++)
	{
		state = SD_readSingleBlock(sector+i, buff);
		if(state != 0)
		{
			return RES_ERROR;
		}
		buff+=512;
	}
	
	return RES_OK;
}

/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

DRESULT myfs_disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	unsigned char state;
	for(unsigned int i = 0; i < count; i++)
	{
		state = SD_writeSingleBlock(sector+i, buff);
		if(state != 0)
		{
			return RES_ERROR;
		}
		buff+=512;
	}
	
	return RES_OK;
}

/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT myfs_disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
    switch(cmd) {
        case CTRL_SYNC:
            return RES_OK;
        case GET_SECTOR_COUNT:
            //*((DWORD*) buff) = card->csd.capacity; 
            /* The sector count should be read from the SD cards CSD-register. 
             * But we're not worried about running out of memory */
            return RES_OK;
        case GET_SECTOR_SIZE:
            *((WORD*) buff) = 512;
            return RES_OK;
        case GET_BLOCK_SIZE:
            return RES_ERROR;
    }
    return RES_ERROR;
}


unsigned char myfs_init(const char* base_path)
{
    esp_err_t err = ESP_OK;
    
    /* Connecting the SD driver to FATFS */
    /* Phystical drive number to identify the drive */
    BYTE pdrv = 0xFF;
    if (ff_diskio_get_drive(&pdrv) != ESP_OK || pdrv == 0xFF) {
        ESP_LOGE(TAG, "the maximum count of volumes is already mounted");
        return ESP_ERR_NO_MEM;
    }

    /* Register diskio driver */
    static const ff_diskio_impl_t sddriver_functions = {
    .init = &myfs_initialize,
    .status = &myfs_disk_status,
    .read = &myfs_disk_read,
    .write = &myfs_disk_write,
    .ioctl = &myfs_disk_ioctl,
    };
    ff_diskio_register(pdrv, &sddriver_functions);
    
    /* Connect FATFS to VFS */
    char drv[3] = {(char)('0' + pdrv), ':', 0};
    err = esp_vfs_fat_register(base_path, drv, 5, &myfs);
    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to connect FATFS to VFS");
        return err;
    }

    /* Mount partition */
    FRESULT res = f_mount(myfs, drv, 1);
    if(res != FR_OK)
    {
        err = ESP_FAIL;
        ESP_LOGE(TAG, "Failed to mount sd: %i", res);
    }

    return err;
}

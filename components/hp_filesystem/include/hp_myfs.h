#ifndef _HP_MYFS
#define _HP_MYFS

#include "esp_vfs_fat.h"
#include "diskio_impl.h"
#include "ffconf.h"
#include "ff.h"


unsigned char myfs_init(const char* base_path);

DSTATUS myfs_initialize(BYTE pdrv);
DSTATUS myfs_disk_status(BYTE pdrv);

DRESULT myfs_disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
);

DRESULT myfs_disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
);

DRESULT myfs_disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
);

#endif
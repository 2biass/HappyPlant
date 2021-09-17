#ifndef _HP_SDCARD
#define _HP_SDCARD

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <esp_log.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

/* We use SPI-3 (VSPI_HOST) as the SD-host and DMA-channel is 1 */
#define SD_HOST VSPI_HOST
#define DMA_CHAN 1

/* SPI pins (IO_MUX pins) */
#define SPI3_MISO 19
#define SPI3_MOSI 23
#define SPI3_CLK  18
#define SD_CS     5

/* SD commands, many of these are not used here */
#define GO_IDLE_STATE            0
#define SEND_OP_COND             1
#define SEND_IF_COND			 8
#define SEND_CSD                 9
#define STOP_TRANSMISSION        12
#define SEND_STATUS              13
#define SET_BLOCK_LEN            16
#define READ_SINGLE_BLOCK        17
#define READ_MULTIPLE_BLOCKS     18
#define WRITE_SINGLE_BLOCK       24
#define WRITE_MULTIPLE_BLOCKS    25
#define ERASE_BLOCK_START_ADDR   32
#define ERASE_BLOCK_END_ADDR     33
#define ERASE_SELECTED_BLOCKS    38
#define SD_SEND_OP_COND			 41   /* For application specific commands */
#define APP_CMD					 55
#define READ_OCR				 58
#define CRC_ON_OFF               59

#define ON     1
#define OFF    0

volatile unsigned long startBlock, totalBlocks;
volatile unsigned char SDHC_flag, cardType;

/* SPI specific */
unsigned char SPI_init();
void SPI_Chip_Select();
void SPI_Chip_Deselect();

/* SD specific */
unsigned char SD_init(void);
unsigned char SD_sendCommand(unsigned char cmd, unsigned long arg);
unsigned char SD_readSingleBlock(unsigned long startBlock, unsigned char* ptr);
unsigned char SD_writeSingleBlock(unsigned long startBlock, const unsigned char* ptr);
unsigned char SD_erase (unsigned long startBlock, unsigned long totalBlocks);

#endif
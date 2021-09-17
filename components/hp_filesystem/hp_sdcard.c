#include "hp_sdcard.h"
static spi_device_handle_t sd = NULL;

/* We need these buffers to be able to transmit data using the DMA.
 * Since they are allocated on the heap this should be done on init. */

static uint8_t* dma_data = NULL;
static uint8_t* dma_command = NULL;

static const char *TAG = "HappyPlant SD-driver";

unsigned char SPI_init()
{
    esp_err_t ret;

    spi_bus_config_t buscfg = {
        .miso_io_num = SPI3_MISO,
        .mosi_io_num = SPI3_MOSI,
        .sclk_io_num = SPI3_CLK,
        .quadwp_io_num = -1,    /* not using write protect or 4-bit transfers */
        .quadhd_io_num = -1,    /* not using hold data or 4-bit transfers */
        .max_transfer_sz = 1024 /* (SPI_MAX_DMA_LEN+4)/ 4 */
    };

    /* Setup SPI: Master mode, MSB first, SCK phase low, SCK idle low, f_SCK = 250 kHz */
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 250 * 1000,   /* Clock out at 250 kHz */
        .mode = 0,                      /* SPI mode 0 */
        .spics_io_num = -1,             /* We want to be able to control CS manually. */
        .queue_size = 2,                /* We should only need 1, but we dont want this to be a point of failure. */
    };

    /* Initialising the spi host */
    ret = spi_bus_initialize(SD_HOST, &buscfg, DMA_CHAN);

    /* Register the SD-card as a spi device */
    ret = spi_bus_add_device(SD_HOST, &devcfg, &sd);
    if(ret != ESP_OK)
    {
        ESP_LOGE(TAG, "ESP failed to initialize SPI-driver");
        return 1;
    }
    /* Enable internal pullup? This should be done for older MMC cards (we're not using that) */
    //gpio_pullup_en(SPI3_MISO);

    /* Initialize CS-pin */
    gpio_set_direction(SD_CS, GPIO_MODE_OUTPUT);
    SPI_Chip_Deselect();

    ESP_LOGI(TAG, "SPI-driver initialised");
    return 0;
}

void SPI_Chip_Select()
{
    gpio_set_level(SD_CS, 0); /* Active low */
}
void SPI_Chip_Deselect()
{
    gpio_set_level(SD_CS, 1); /* Active high */
}

unsigned char SD_init(void)
{
    unsigned char response, SD_version;
	unsigned int retry = 0;

    SPI_init(sd);
    gpio_set_direction(GPIO_NUM_16, GPIO_MODE_OUTPUT);
    /* Even though time has passed since power-up, make sure 1ms is passed before accessing SD-card */
    vTaskDelay(1 / portTICK_PERIOD_MS);

    /* Using DMA requires 32-bit alligned and dma-reachable memory 
     * "Statically" allocating memory used for larger transfers 
     * (instead of allocating/deallocating every time they are used). */
    dma_data = heap_caps_malloc(512, MALLOC_CAP_DMA | MALLOC_CAP_32BIT);
    dma_command = heap_caps_malloc(6, MALLOC_CAP_DMA | MALLOC_CAP_32BIT);

    if(dma_data == NULL || dma_command == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate dma buffers");
        free(dma_data);
        free(dma_command);
        return 1;
    }

    /* 80 clock pulses before sending the first command (Only needs 76, but we just do
     * 80 to be sure). This will make the card enter the native operation "SD"-mode. */
    uint8_t *dma_dummybits = heap_caps_malloc(10, MALLOC_CAP_DMA | MALLOC_CAP_32BIT);

    if(dma_dummybits == NULL)
    {
        ESP_LOGE(TAG, "Error allocating buffer for 80 clock pulses");
        free(dma_data);
        free(dma_command);
        return 1;
    }

    memset(dma_dummybits, 0xff, 10);
    spi_transaction_t t_payload = {
        .length = 8*10,
        .tx_buffer = dma_dummybits,
        .rx_buffer = dma_dummybits,
    };

    spi_device_transmit(sd, &t_payload);

    spi_transaction_t t_byte = {
        .length = 8,
    };
    t_byte.flags |= (SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA);
    t_byte.tx_data[0] = 0xff;

    /* To initialise in SPI mode, we have to pull CS low and send CMD0. This will effectively reset the card. */
    SPI_Chip_Select();
    do
	{
		response = SD_sendCommand(GO_IDLE_STATE, 0); /* Send 'reset & go idle' command (= CMD0) */
		retry++;
		if(retry > 0x20)
        {
            ESP_LOGE(TAG, "Time out, card not detected");
            free(dma_dummybits);
            free(dma_data);
            free(dma_command);
            return 1;
        }
	} while(response != 0x01); /* Repeat until SD is in IDLE state (0x01), that is, until it is in SPI mode. */
    SPI_Chip_Deselect();

    spi_device_transmit(sd, &t_byte);
    spi_device_transmit(sd, &t_byte);

    retry = 0;
    SD_version = 2; /* Default set to SD compliance with ver2.x. this may change after checking the next command */
	do
	{
		response = SD_sendCommand(SEND_IF_COND, 0x000001AA); /* Check power supply status, mandatory for SDHC card (= CMD8) */
		retry++;
		if(retry > 0xfe || response == 0x05)
		{
			SD_version = 1;
			cardType = 1;
            ESP_LOGE(TAG, "Time out or 0x05, indicating Ver.1 or MMC");
			break;
		} /* time out */
	} while(response != 0x01); /* If accepted (expect 0x01 + 32 bit return value) */

    retry = 0;
    do
	{
        response = SD_sendCommand(APP_CMD, 0); /* CMD55, must be sent before sending any ACMD command */   
        if(response == 0x01) /* Make sure SD accepted the APP_CMD before sending ACMD41 */
        {
            response = SD_sendCommand(SD_SEND_OP_COND, 0x40000000); /* ACMD41 */
        }

		if(retry++ > 0xfe)
        {
            ESP_LOGE(TAG, "Time out, APP_CMD+OP_COND");
            free(dma_dummybits);
            free(dma_data);
            free(dma_command);
            return 2;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    } while(response != 0x00); /* A change from 0x01 to 0x00 indicates that initialization is over. */

	retry = 0;
	SDHC_flag = 0;
	if (SD_version == 2)
	{
		do
		{
			response = SD_sendCommand(READ_OCR, 0); /* (=CMD58) */
			retry++;
			if(retry > 0xfe)
			{
				cardType = 0;
				break;
			} /* time out */
		} while(response != 0x00);
		
		if(SDHC_flag == 1)
			cardType = 2;
		else
			cardType = 3;
	}

    response = SD_sendCommand(CRC_ON_OFF, OFF); /* Disable CRC; default - CRC disabled in SPI mode */
    response = SD_sendCommand(SET_BLOCK_LEN, 512); /* Set block size to 512 (default size is 512) */

    free(dma_dummybits);
    ESP_LOGI(TAG, "SD-card initialized");

    return 0;
}

unsigned char SD_sendCommand(unsigned char cmd, unsigned long arg)
{

    if(dma_command == NULL)
    {
        ESP_LOGE(TAG, "dma_command buffer not allocated. Initialise the SD-driver");
        return 1;
    }

    unsigned char retry = 0;
    unsigned char response, status;

    if(SDHC_flag == 0)
	{
		if(cmd == READ_SINGLE_BLOCK      ||
		   cmd == READ_MULTIPLE_BLOCKS   ||
		   cmd == WRITE_SINGLE_BLOCK     ||
		   cmd == WRITE_MULTIPLE_BLOCKS  ||
		   cmd == ERASE_BLOCK_START_ADDR ||
		   cmd == ERASE_BLOCK_END_ADDR)
		   {
			   arg = arg << 9;
		   }
    }
    /* Since we're transmitting 6 bytes, we can't just use the internal 4-byte tx_data buffer.
     * Therefore we use the DMA buffer for transfer. */
    dma_command[0] = cmd | 0b01000000;
    dma_command[1] = arg >> 24;
    dma_command[2] = arg >> 16;
    dma_command[3] = arg >> 8;
    dma_command[4] = arg;

    /* It is compulsory to send correct CRC for CMD8 (CRC=0x87) & CMD0 (CRC=0x95)
     * That is, it is mandatory before the card is initialised in SPI-mode */
    if(cmd == SEND_IF_COND)
		dma_command[5] = 0x87;
	else 
        dma_command[5] = 0x95;
    
    spi_transaction_t t_payload = {
        .length = 8*6,
        .tx_buffer = dma_command,
        .rx_buffer = NULL, /* "Skip" read phase on this */ 
    };
    spi_transaction_t t_byte = {
        .length = 8,
    };
    t_byte.flags |= (SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA);
    t_byte.tx_data[0] = 0xff;

    spi_device_transmit(sd, &t_byte);
    SPI_Chip_Select();
    spi_device_transmit(sd, &t_payload);

    /* Keep sending 1 byte to receive response until response is not 0xFF */
    do
    {
        spi_device_transmit(sd, &t_byte);
        response = t_byte.rx_data[0];
        if(retry++ > 254)
        {
			break; /* Timed out */
        }
    } while ( response == 0xFF ); /* Wait for the response. */
    
    if(response == 0x00 && cmd == READ_OCR) /* Checking response of CMD58 */
	{
        spi_device_transmit(sd, &t_byte);
        status = t_byte.rx_data[0] & 0x40; /* First byte of the OCR register (bit 31:24) */
        if(status == 0x40)
			SDHC_flag = 1; /* We need it to verify SDHC card */
		else
			SDHC_flag = 0;

        /* remaining 3 bytes of the OCR register are ignored here
         * one can use these bytes to check power supply limits of SD */
        spi_device_transmit(sd, &t_byte);
        spi_device_transmit(sd, &t_byte);
        spi_device_transmit(sd, &t_byte);
	}

    if (cmd == ERASE_SELECTED_BLOCKS)
	{
        do
        {
            spi_device_transmit(sd, &t_byte);
            status = t_byte.rx_data[0];
        } while ( status == 0 );
	}

    /* Extra 8 CLK to clock out R1B or R2 response (This could be used if needed) */
    spi_device_transmit(sd, &t_byte);
    SPI_Chip_Deselect();

    return response;
}

unsigned char SD_readSingleBlock(unsigned long startBlock, unsigned char* ptr)
{

    if(dma_data == NULL)
    {
        ESP_LOGE(TAG, "dma_data buffer not allocated. Initialise the SD-driver");
        return 1;
    }

    unsigned char response = 0;
    unsigned char status = 0;
	unsigned int retry = 0;
    
	response = SD_sendCommand(READ_SINGLE_BLOCK, startBlock);
	if(response != 0x00) /* If the response is non-zero, an error occurred */
    {
        ESP_LOGE(TAG, "Non-zero response to READ_SINGLE_BLOCK: %u", response);
        return response;
    }

    memset(dma_data, 0xff, 512);

    /* Transaction descript for the 512 bytes */
    spi_transaction_t t_payload = {
        .length = 512*8,
        .tx_buffer = dma_data, /* Even though we want to "skip" the write phase, we have to clock out 0xff to receive correct data. */
        .rx_buffer = dma_data,
    };

    /* Transaction descriptor for single byte-length transmissions */
    spi_transaction_t t_byte = {
        .length = 8,
    };
    t_byte.flags |= (SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA ); /* Now we want to use the rx_data buffer */
    t_byte.tx_data[0] = 0xff;

    SPI_Chip_Select();

    /* Wait for single block token */
	do 
	{
        spi_device_transmit(sd, &t_byte);
        status = t_byte.rx_data[0];
		if(retry++ > 65000)
		{
            ESP_LOGE(TAG, "Timed out, waiting for read token");
			SPI_Chip_Deselect();
			return 1;
		}
	} while (status != 0xFE );

    /* Receive the 512 bytes */
    spi_device_transmit(sd, &t_payload);

    /* Ignoring CRC (16 bit) */
	spi_device_transmit(sd, &t_byte);
	spi_device_transmit(sd, &t_byte);
	
	/* Wait 8 clk pulses more */
	spi_device_transmit(sd, &t_byte);
	SPI_Chip_Deselect();

    /* Copy the data from RAM into ptr. */
    memcpy(ptr, dma_data, 512);

    return 0;
}

unsigned char SD_writeSingleBlock(unsigned long startBlock, const unsigned char* ptr)
{
    if(dma_data == NULL)
    {
        ESP_LOGE(TAG, "dma_data buffer not allocated. Initialise the SD-driver");
        return 1;
    }

    unsigned char response = 0;
	unsigned char status = 0;
	unsigned int retry = 0;

	response = SD_sendCommand(WRITE_SINGLE_BLOCK, startBlock);
    if(response != 0x00) /* If the response is non-zero, an error occurred */
    {
        ESP_LOGE(TAG, "Non-zero response to WRITE_SINGLE_BLOCK: %u", response);
        return response;
    }

    memcpy(dma_data, ptr, 512); /* Copy the data from ptr into RAM */

    /* Transaction descript for the 512 bytes */
    spi_transaction_t t_payload = {
        .length = 512*8,
        .tx_buffer = dma_data,
        .rx_buffer = NULL, /* We want to skip the read phase here */
    };

    /* Transaction descriptor for single byte-length transmissions */
    spi_transaction_t t_byte = {
        .length = 8,
    };
    t_byte.flags |= (SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA ); /* Now we want to use the rx_data buffer */

	SPI_Chip_Select();

    /* Send start block token (0xFE) and data block */
    t_byte.tx_data[0] = 0xFE; /* Start block token */
    spi_device_transmit(sd, &t_byte);
	
    spi_device_transmit(sd, &t_payload);

    t_byte.tx_data[0] = 0xFF;
    /* Transmit dummy CRC (ignored here) */
    spi_device_transmit(sd, &t_byte);
    spi_device_transmit(sd, &t_byte);
    
    /* Get the response */
	do 
	{
        spi_device_transmit(sd, &t_byte);
        status = t_byte.rx_data[0];
        /* As this can take some time, free the cpu for 10ms at the time. */
        vTaskDelay(10 / portTICK_PERIOD_MS);
	} while (status == 0xFF);

    if( ((status >> 1) & 0x07) == 0x06 ) /* Write error */
    {
        ESP_LOGE(TAG, "Write error during WRITE_SINGLE_BLOCK");
        SPI_Chip_Deselect();
        return status;
    }
    else if( ((status >> 1) & 0x07) == 0x05 )
    {
        ESP_LOGE(TAG, "CRC error during WRITE_SINGLE_BLOCK"); /* This should never occur since CRC is disabled */
        SPI_Chip_Deselect();
        return status;
    }
    /* Wait for SD card to complete writing and get idle */
	do
	{
		spi_device_transmit(sd, &t_byte);
        status = t_byte.rx_data[0];
		if(retry++ > 65000)
		{
            ESP_LOGW(TAG, "Timed out waiting for write and get idle.");
			SPI_Chip_Deselect();
			return status;
		}
	} while (status == 0);

    SPI_Chip_Deselect();

    spi_device_transmit(sd, &t_byte);

    return 0;
}

unsigned char SD_erase (unsigned long startBlock, unsigned long totalBlocks)
{
    unsigned char response;
	response = SD_sendCommand(ERASE_BLOCK_START_ADDR, startBlock); /* Send starting block address */
	if(response != 0x00) /* Check for SD status: 0x00 - OK (No flags set) */
		return response;

	response = SD_sendCommand(ERASE_BLOCK_END_ADDR, (startBlock + totalBlocks - 1)); /* Send end block address */
	if(response != 0x00) /* Check for SD status: 0x00 - OK (No flags set) */
		return response;

	response = SD_sendCommand(ERASE_SELECTED_BLOCKS, 0); /* Erase all selected blocks */
	if(response != 0x00) /* Check for SD status: 0x00 - OK (No flags set) */
		return response;

    return 0;
}
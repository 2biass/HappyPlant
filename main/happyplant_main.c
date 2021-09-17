#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "nvs_flash.h"

#include "hp_softap.h"
#include "hp_webserver.h"
#include "hp_sdcard.h"
#include "hp_myfs.h"
#include "hp_sensors.h"

#ifdef CONFIG_IDF_TARGET_ESP32
#define CHIP_NAME "ESP32"
#endif

#ifdef CONFIG_IDF_TARGET_ESP32S2BETA
#define CHIP_NAME "ESP32-S2 Beta"
#endif


/* Things to be done: 
 * Find out what session is in HTTP server */

void app_main(void)
{
    /* Initialize NVS flash for the WiFi phy to use */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU cores, WiFi%s%s, ",
            CHIP_NAME,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);
    
    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    
    /* myfs_init will initialise the SPI sd-card driver and mount it with espressifs VFS and FATFS. */
    myfs_init("/sdcard");

    /* Initialising i2c driver and sensors */
    init_i2c_driver();
    i2c_GY21_init();
    i2c_TSL2561_init();

    /* Setting up esp as a software defined accesspoint */
    wifi_init_softap();

    /* Start the http-server and websocket (port 80) with the base-path to the virtual file-system where we store html, logo, scripts etc. */
    start_webserver("/sdcard");
}
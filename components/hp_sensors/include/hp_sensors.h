#ifndef _HP_SENSORS
#define _HP_SENSORS

#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_log.h>
#include "math.h"

/* Master defines */
#define I2C_MASTER_SDA_IO 22
#define I2C_MASTER_SCL_IO 21
#define I2C_MASTER_FREQ_HZ 100000  
#define I2C_PORT_NUM I2C_NUM_1
#define I2C_TX_BUF_DISABLE 0              
#define I2C_RX_BUF_DISABLE 0

#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */
#define DATA_WR 0x0 /* Data direction bit write access */ 
#define DATA_RD 0x1 /* Data read */

/* GY21 defines */
#define GY21_SLAVE_ADDR 0x40 /* device address */
#define GY21_TEMP_REG 0xF3 /* Tempereture register 'no hold' */
#define GY21_HUM_REG 0xF5 /* Humidity register 'no hold' */
#define SOFT_RES 0xFE /* Soft reset */

/* TSL2561 defines */
#define TSL2561_SLAVE_ADDR 0x39 /* device address (floating addr pin) */
#define TSL2561_CMD 0x80
#define TSL2561_CMD_WORD 0xA0
#define TSL2561_CONTROL_REG 0x00
#define TSL2561_POWERUP 0x3
#define TSL2561_DATA0LOW_REG 0xC
#define TSL2561_DATA1LOW_REG 0xE

esp_err_t init_i2c_driver();

/* Functions for GY21 */
esp_err_t i2c_GY21_init();
esp_err_t i2c_GY21_write(uint8_t slave_add, uint8_t reg_add, uint8_t data);
esp_err_t i2c_GY21_read(uint8_t slave_add, uint8_t reg_add, uint16_t *meas);
esp_err_t i2c_GY21_read_temp(float *temperature);
esp_err_t i2c_GY21_read_humid(float *humididty, float temperature);

/* Functions for TSL2561 */
esp_err_t i2c_TSL2561_init();
esp_err_t i2c_TSL2561_read_lux(float *lux);

#endif
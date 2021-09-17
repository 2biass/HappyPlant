#include "hp_sensors.h"

static const char *TAG = "HappyPlant sensors";


esp_err_t i2c_GY21_write(uint8_t slave_add, uint8_t reg_add, uint8_t data)
{
    esp_err_t res = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    res |= i2c_master_start(cmd);
    res |= i2c_master_write_byte(cmd, (slave_add << 1), ACK_CHECK_EN);
    res |= i2c_master_write_byte(cmd, reg_add, ACK_CHECK_EN); 
    if(data != 0)
    {
        res |= i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    }
    res |= i2c_master_stop(cmd);
    res |= i2c_master_cmd_begin(I2C_PORT_NUM, cmd, 20 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    return res; 
}

esp_err_t i2c_GY21_read(uint8_t slave_add, uint8_t reg_add, uint16_t *meas)
{
    esp_err_t res = ESP_OK; 
    uint8_t data_msb;
    uint8_t data_lsb;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_add<<1), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_add, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    res = i2c_master_cmd_begin(I2C_PORT_NUM, cmd, 20 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if(res != ESP_OK)
    {
        i2c_cmd_link_delete(cmd);
        ESP_LOGE(TAG, "Read command NACK");
        return ESP_FAIL;
    }
    vTaskDelay(60/portTICK_RATE_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (slave_add<<1) | DATA_RD, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data_msb, ACK_VAL);
    i2c_master_read_byte(cmd, &data_lsb, NACK_VAL);
    i2c_master_stop(cmd);
    res = i2c_master_cmd_begin(I2C_PORT_NUM, cmd, 20 / portTICK_RATE_MS);
    if(res != ESP_OK)
    {
        ESP_LOGE(TAG, "Reading temperature failed");
        i2c_cmd_link_delete(cmd);
        return ESP_FAIL;
    }
    i2c_cmd_link_delete(cmd);

    *meas = (((uint16_t)data_msb<<8)|(uint16_t)data_lsb);
    
    return res; 
}

esp_err_t init_i2c_driver()
{
    esp_err_t res = ESP_OK;

    int i2c_master_port = I2C_PORT_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,        
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,         
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,  
    };

    res |= i2c_param_config(i2c_master_port, &conf);

    res |= i2c_driver_install(i2c_master_port, conf.mode,
    		I2C_RX_BUF_DISABLE, I2C_TX_BUF_DISABLE, 0);

    return res; 
}

/* Setup I2C slave (GY21) */ 
esp_err_t i2c_GY21_init()
{

    esp_err_t res = ESP_OK;

    vTaskDelay(15/portTICK_RATE_MS);
    res = i2c_GY21_write(GY21_SLAVE_ADDR, SOFT_RES, 0);
    vTaskDelay(15/portTICK_RATE_MS);

    return res; 
}

esp_err_t i2c_GY21_read_temp(float *temperature)
{
    esp_err_t err = ESP_OK;
    uint16_t sTemp;
    err = i2c_GY21_read(GY21_SLAVE_ADDR, GY21_TEMP_REG, &sTemp);
    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read temperature");
        return err;
    }
    *temperature = (175.72 * sTemp / 65536.0) - 46.85;
    return err;
}

esp_err_t i2c_GY21_read_humid(float *humidity, float temperature)
{
    esp_err_t err = ESP_OK;
    uint16_t sTemp;
    float relavtive_humidity;
    err = i2c_GY21_read(GY21_SLAVE_ADDR, GY21_HUM_REG, &sTemp);
    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read humidity");
        return err;
    }
    relavtive_humidity = (125.0 * sTemp / 65536.0) - 6.0;
    *humidity = relavtive_humidity + (25 - temperature) * relavtive_humidity / temperature;
    return err;
}

esp_err_t i2c_TSL2561_init()
{
    /* After Vdd we have to issue power up command 0x03 */
    esp_err_t res = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TSL2561_SLAVE_ADDR << 1), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, TSL2561_CMD | TSL2561_CONTROL_REG, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, TSL2561_POWERUP, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    res = i2c_master_cmd_begin(I2C_PORT_NUM, cmd, 10 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if(res != ESP_OK )
    {
        ESP_LOGE(TAG, "Failed to power up TSL2561");
        return res;
    }
    /* It takes 400ms for a integration conversion */
    vTaskDelay(400/portTICK_RATE_MS);
    return res;
}

esp_err_t i2c_TSL2561_read_lux(float *lux)
{
    uint8_t data_msb=0;
    uint8_t data_lsb=0;
    uint16_t temp = 0;

    float channel0 = 0;
    float channel1 = 0;

    esp_err_t res = ESP_OK;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TSL2561_SLAVE_ADDR << 1), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, TSL2561_CMD_WORD | TSL2561_DATA0LOW_REG, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TSL2561_SLAVE_ADDR << 1) | DATA_RD, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data_lsb, ACK_VAL);
    i2c_master_read_byte(cmd, &data_msb, ACK_VAL);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_PORT_NUM, cmd, 10 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    temp = (((uint16_t)data_msb<<8)|(uint16_t)data_lsb);
    if(!temp) /* Make sure no division zero */
    {
        *lux = 0;
        return res;
    }

    channel0 = (float)temp;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TSL2561_SLAVE_ADDR << 1), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, TSL2561_CMD_WORD | TSL2561_DATA1LOW_REG, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (TSL2561_SLAVE_ADDR << 1) | DATA_RD, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data_lsb, ACK_VAL);
    i2c_master_read_byte(cmd, &data_msb, ACK_VAL);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_PORT_NUM, cmd, 10 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    channel1 = (float)(((uint16_t)data_msb<<8)|(uint16_t)data_lsb);

    float ratio = channel1 / channel0;
    if( ratio <= 0.50 )
        *lux = ( 0.0304 * channel0 ) - ( 0.062 * channel0 * pow(ratio, 1.4));
    else if( ratio > 0.50 && ratio <= 0.61 )
        *lux = 0.0224 * channel0 - 0.031 * channel1;
    else if( ratio > 0.61 && ratio <= 0.80 )
        *lux = 0.0128 * channel0 - 0.0153 * channel1;
    else if( ratio > 0.80 && ratio <= 1.30 )
        *lux = 0.00146 * channel0 - 0.00112 * channel1;
    else
        *lux = 0;

    return res;
}
/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * SCCB (I2C like) driver.
 *
 */

#include <stdbool.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sccb.h"
#include "sensor.h"
#include <stdio.h>
#include "sdkconfig.h"
#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
static const char* TAG = "sccb";
#endif

#define LITTLETOBIG(x)          ((x<<8)|(x>>8))

#include "driver/i2c.h"
#include "i2c_dev.h"
#include "esp_err.h"

// support IDF 5.x
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

#define SCCB_FREQ               CONFIG_SCCB_CLK_FREQ  /*!< I2C master frequency*/
#define WRITE_BIT               I2C_MASTER_WRITE      /*!< I2C master write */
#define READ_BIT                I2C_MASTER_READ       /*!< I2C master read */
#define ACK_CHECK_EN            0x1                   /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS           0x0                   /*!< I2C master will not check ack from slave */
#define ACK_VAL                 0x0                   /*!< I2C ack value */
#define NACK_VAL                0x1                   /*!< I2C nack value */
#if CONFIG_SCCB_HARDWARE_I2C_PORT1
const int SCCB_I2C_PORT_DEFAULT = 1;
#else
const int SCCB_I2C_PORT_DEFAULT = 0;
#endif

static int sccb_i2c_port;
static bool sccb_owns_i2c_port;
static i2c_dev_t i2c_dev;

int SCCB_Init(int pin_sda, int pin_scl)
{
    ESP_LOGI(TAG, "pin_sda %d pin_scl %d", pin_sda, pin_scl);
    i2c_config_t conf;
    esp_err_t ret;

    memset(&conf, 0, sizeof(i2c_config_t));

    sccb_i2c_port = SCCB_I2C_PORT_DEFAULT;
    sccb_owns_i2c_port = true;
    ESP_LOGI(TAG, "sccb_i2c_port=%d", sccb_i2c_port);

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = pin_sda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = pin_scl;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = SCCB_FREQ;

    if ((ret =  i2c_param_config(sccb_i2c_port, &conf)) != ESP_OK) {
        return ret;
    }

    return i2c_driver_install(sccb_i2c_port, conf.mode, 0, 0, 0);
}

int SCCB_Init_Desc(int pin_sda, int pin_scl, int i2c_master_num)
{
    esp_err_t ret = 0;
    i2c_dev.port = i2c_master_num;
    i2c_dev.addr = OV5640_SCCB_ADDR;
    i2c_dev.cfg.sda_io_num = pin_sda;
    i2c_dev.cfg.scl_io_num = pin_scl;
    i2c_dev.cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_dev.cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_dev.cfg.master.clk_speed = SCCB_FREQ;
    i2c_dev.timeout_ticks = 10/portTICK_PERIOD_MS;
    ret = i2c_dev_create_mutex(&i2c_dev);
    return ret;
}

int SCCB_Use_Port(int i2c_num) { // sccb use an already initialized I2C port
    if (sccb_owns_i2c_port) {
        SCCB_Deinit();
    }
    if (i2c_num < 0 || i2c_num > I2C_NUM_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    sccb_i2c_port = i2c_num;
    sccb_owns_i2c_port = false; // in this case, camera doesn't own the i2c port
    return ESP_OK;
}

int SCCB_Deinit(void)
{
    if (!sccb_owns_i2c_port) {
        return ESP_OK;
    }
    sccb_owns_i2c_port = false;
    return i2c_driver_delete(sccb_i2c_port);
}

int SCCB_Probe(uint8_t slv_addr)
{
    i2c_dev.addr = slv_addr;
    esp_err_t ret = i2c_dev_probe(&i2c_dev, I2C_DEV_WRITE);
    return ret == ESP_OK ? 0 : -1;
}

uint8_t SCCB_Read(uint8_t slv_addr, uint8_t reg)
{
    uint8_t data=0;
    ESP_ERROR_CHECK(i2c_dev_take_mutex(&i2c_dev));
    esp_err_t ret = i2c_dev_read_reg(&i2c_dev, reg, &data, 1);
    ESP_ERROR_CHECK(i2c_dev_give_mutex(&i2c_dev));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SCCB_Read Failed addr:0x%02x, reg:0x%02x, data:0x%02x, ret:%d", slv_addr, reg, data, ret);
    }
    return data;
}

int SCCB_Write(uint8_t slv_addr, uint8_t reg, uint8_t data)
{
    ESP_ERROR_CHECK(i2c_dev_take_mutex(&i2c_dev));
    esp_err_t ret = i2c_dev_write_reg(&i2c_dev, reg, &data, 1);
    ESP_ERROR_CHECK(i2c_dev_give_mutex(&i2c_dev));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SCCB_Write Failed addr:0x%02x, reg:0x%02x, data:0x%02x, ret:%d", slv_addr, reg, data, ret);
    }
    return ret == ESP_OK ? 0 : -1;
}

uint8_t SCCB_Read16(uint8_t slv_addr, uint16_t reg)
{
    uint8_t data=0;
    ESP_ERROR_CHECK(i2c_dev_take_mutex(&i2c_dev));
    esp_err_t ret = i2c_dev_read_reg_16_byte(&i2c_dev, reg, &data);
    ESP_ERROR_CHECK(i2c_dev_give_mutex(&i2c_dev));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SCCB_Read16 Failed addr:0x%02x, reg:0x%04x, data:0x%02x", 
                 slv_addr, reg, data);
    }
    return data;
}

int SCCB_Write16(uint8_t slv_addr, uint16_t reg, uint8_t data)
{
    ESP_ERROR_CHECK(i2c_dev_take_mutex(&i2c_dev));
    esp_err_t ret = i2c_dev_write_reg_16_byte(&i2c_dev, reg, data);
    ESP_ERROR_CHECK(i2c_dev_give_mutex(&i2c_dev));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SCCB_Write16 Failed addr:0x%02x, reg:0x%04x, data:0x%02x", 
                 slv_addr, reg, data);
    }
    return ret == ESP_OK ? 0 : -1;
}

uint16_t SCCB_Read_Addr16_Val16(uint8_t slv_addr, uint16_t reg)
{
    uint16_t data = 0;
    ESP_ERROR_CHECK(i2c_dev_take_mutex(&i2c_dev));
    esp_err_t ret = i2c_dev_read_reg_16_val_16(&i2c_dev, reg, &data);
    ESP_ERROR_CHECK(i2c_dev_give_mutex(&i2c_dev));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SCCB_Read_Addr16_Val16 Failed addr:0x%02x, reg:0x%04x, data:0x%04x", 
                 slv_addr, reg, data);
    }
    return data;
}

int SCCB_Write_Addr16_Val16(uint8_t slv_addr, uint16_t reg, uint16_t data)
{
    ESP_ERROR_CHECK(i2c_dev_take_mutex(&i2c_dev));
    esp_err_t ret = i2c_dev_write_reg_16_val_16(&i2c_dev, reg, data);
    ESP_ERROR_CHECK(i2c_dev_give_mutex(&i2c_dev));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SCCB_Write_Addr16_Val16 Failed addr:0x%02x, reg:0x%04x, data:0x%04x", 
                 slv_addr, reg, data);
    }
    return ret == ESP_OK ? 0 : -1;
}

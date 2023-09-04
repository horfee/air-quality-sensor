/**
 * @file sgp41.c
 *
 * ESP-IDF driver for SGP40 Indoor Air Quality Sensor for VOC Measurements
 *
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#include <esp_err.h>
#include <esp_idf_lib_helpers.h>
#include <esp_log.h>
#include "sgp41.h"
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ets_sys.h>

#define I2C_FREQ_HZ 400000

static const char *TAG = "sgp41";

#define CMD_SOFT_RESET           0x0006
#define CMD_FEATURESET           0x202f
#define CMD_MEASURE_RAW          0x2619 // invalid initial : 0x260f
#define CMD_EXECUTE_CONDITIONING 0x2612
#define CMD_SELF_TEST            0x280e // OK
#define CMD_SERIAL               0x3682 // OK
#define CMD_HEATER_OFF           0x3615 // OK

#define TIME_SOFT_RESET  1
#define TIME_FEATURESET  1
#define TIME_MEASURE_RAW 75
#define TIME_EXECUTE_CONDITIONING 75
#define TIME_SELF_TEST   320
#define TIME_SERIAL      1
#define TIME_HEATER_OFF  1

#define SGP41_ADDRESS    0x59
#define SELF_TEST_OK     0xd400

#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)
#define CHECK_ARG(ARG) do { if (!(ARG)) return ESP_ERR_INVALID_ARG; } while (0)

static uint8_t crc8(const uint8_t *data, size_t count)
{
    uint8_t res = 0xff;

    for (size_t i = 0; i < count; ++i)
    {
        res ^= data[i];
        for (uint8_t bit = 8; bit > 0; --bit)
        {
            if (res & 0x80)
                res = (res << 1) ^ 0x31;
            else
                res = (res << 1);
        }
    }
    return res;
}

static inline uint16_t swap(uint16_t v)
{
    return (v << 8) | (v >> 8);
}

static esp_err_t send_cmd(i2c_dev_t *dev, uint16_t cmd, uint16_t *data, size_t words)
{
    uint8_t buf[2 + words * 3];
    // add command
    *(uint16_t *)buf = swap(cmd);
    if (data && words)
        // add arguments
        for (size_t i = 0; i < words; i++)
        {
            uint8_t *p = buf + 2 + i * 3;
            *(uint16_t *)p = swap(data[i]);
            *(p + 2) = crc8(p, 2);
        }

    ESP_LOGV(TAG, "Sending buffer:");
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, buf, sizeof(buf), ESP_LOG_VERBOSE);

    return i2c_dev_write(dev, NULL, 0, buf, sizeof(buf));
}

static esp_err_t read_resp(i2c_dev_t *dev, uint16_t *data, size_t words)
{
    uint8_t buf[words * 3];
    CHECK(i2c_dev_read(dev, NULL, 0, buf, sizeof(buf)));

    ESP_LOGV(TAG, "Received buffer:");
    ESP_LOG_BUFFER_HEX_LEVEL(TAG, buf, sizeof(buf), ESP_LOG_VERBOSE);

    for (size_t i = 0; i < words; i++)
    {
        uint8_t *p = buf + i * 3;
        uint8_t crc = crc8(p, 2);
        if (crc != *(p + 2))
        {
            ESP_LOGE(TAG, "Invalid CRC 0x%02x, expected 0x%02x", crc, *(p + 2));
            return ESP_ERR_INVALID_CRC;
        }
        data[i] = swap(*(uint16_t *)p);
    }
    return ESP_OK;
}

static esp_err_t execute_cmd(sgp41_t *dev, uint16_t cmd, uint32_t timeout_ms,
        uint16_t *out_data, size_t out_words, uint16_t *in_data, size_t in_words)
{
    CHECK_ARG(dev);

    I2C_DEV_TAKE_MUTEX(&dev->i2c_dev);
    I2C_DEV_CHECK(&dev->i2c_dev, send_cmd(&dev->i2c_dev, cmd, out_data, out_words));
    if (timeout_ms)
    {
        if (timeout_ms > 10)
            vTaskDelay(pdMS_TO_TICKS(timeout_ms));
        else
            ets_delay_us(timeout_ms * 1000);
    }
    if (in_data && in_words)
        I2C_DEV_CHECK(&dev->i2c_dev, read_resp(&dev->i2c_dev, in_data, in_words));
    I2C_DEV_GIVE_MUTEX(&dev->i2c_dev);

    return ESP_OK;
}

////////////////////////////////////////////////////////////////////////////////

esp_err_t sgp41_init_desc(sgp41_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);

    dev->i2c_dev.port = port;
    dev->i2c_dev.addr = SGP41_ADDRESS;
    dev->i2c_dev.cfg.sda_io_num = sda_gpio;
    dev->i2c_dev.cfg.scl_io_num = scl_gpio;
#if HELPER_TARGET_IS_ESP32
    dev->i2c_dev.cfg.master.clk_speed = I2C_FREQ_HZ;
#endif
    return i2c_dev_create_mutex(&dev->i2c_dev);
}

esp_err_t sgp41_free_desc(sgp41_t *dev)
{
    CHECK_ARG(dev);

    return i2c_dev_delete_mutex(&dev->i2c_dev);
}

esp_err_t sgp41_init(sgp41_t *dev)
{
    CHECK_ARG(dev);

    CHECK(execute_cmd(dev, CMD_SERIAL, TIME_SERIAL, NULL, 0, dev->serial, 3));
    CHECK(execute_cmd(dev, CMD_FEATURESET, TIME_FEATURESET, NULL, 0, &dev->featureset, 1));

    ESP_LOGD(TAG, "Device found. S/N: 0x%04x%04x%04x, featureset 0x%04x",
            dev->serial[0], dev->serial[1], dev->serial[2], dev->featureset);

    //VocAlgorithm_init(&dev->voc);

    return ESP_OK;
}

esp_err_t sgp41_soft_reset(sgp41_t *dev)
{
    CHECK_ARG(dev);

    return execute_cmd(dev, CMD_SOFT_RESET, TIME_SOFT_RESET, NULL, 0, NULL, 0);
}

esp_err_t sgp41_self_test(sgp41_t *dev)
{
    CHECK_ARG(dev);

    uint16_t res;
    CHECK(execute_cmd(dev, CMD_SELF_TEST, TIME_SELF_TEST, NULL, 0, &res, 1));

    return res == SELF_TEST_OK ? ESP_OK : ESP_FAIL;
}

esp_err_t sgp41_execute_conditioning(sgp41_t *dev, uint16_t default_rh, uint16_t default_t, uint16_t* sraw_voc) {
    CHECK_ARG(dev);
    CHECK_ARG(sraw_voc);

    uint16_t params[2];
    params[0] = default_rh;
    params[1] = default_t;

    return execute_cmd(dev, CMD_EXECUTE_CONDITIONING, TIME_EXECUTE_CONDITIONING, params, 2, sraw_voc, 1);

}

esp_err_t sgp41_heater_off(sgp41_t *dev)
{
    CHECK_ARG(dev);

    return execute_cmd(dev, CMD_HEATER_OFF, TIME_HEATER_OFF, NULL, 0, NULL, 0);
}

esp_err_t sgp41_measure_raw(sgp41_t *dev, float humidity, float temperature, uint16_t* sraw_voc, uint16_t* sraw_nox)
{
    CHECK_ARG(dev);
    CHECK_ARG(sraw_voc);
    CHECK_ARG(sraw_nox);

    uint16_t params[2];
    if (isnan(humidity) || isnan(temperature))
    {
        params[0] = 0x8000;
        params[1] = 0x6666;
        ESP_LOGW(TAG, "Uncompensated measurement");
    }
    else
    {
        if (humidity < 0)
            humidity = 0;
        else if (humidity > 100)
            humidity = 100;

        if (temperature < -45)
            temperature = -45;
        else if (temperature > 129.76)
            temperature = 129.76;

        params[0] = (uint16_t)(humidity / 100.0 * 65536);
        params[1] = (uint16_t)((temperature + 45) / 175.0 * 65535);
    }
    

    uint16_t raw[2];
    esp_err_t result = execute_cmd(dev, CMD_MEASURE_RAW, TIME_MEASURE_RAW, params, 2, raw, 2);

    *sraw_voc = raw[0];
    *sraw_nox = raw[1];
    return result;
    /*
    int16_t error;
    uint8_t buffer[8];
    uint16_t offset = 0;
    offset = sensirion_i2c_add_command_to_buffer(&buffer[0], offset, 0x2619);

    offset = sensirion_i2c_add_uint16_t_to_buffer(&buffer[0], offset,
                                                  relative_humidity);
    offset =
        sensirion_i2c_add_uint16_t_to_buffer(&buffer[0], offset, temperature);

    error = sensirion_i2c_write_data(SGP41_I2C_ADDRESS, &buffer[0], offset);
    if (error) {
        return error;
    }

    sensirion_i2c_hal_sleep_usec(50000);

    error = sensirion_i2c_read_data_inplace(SGP41_I2C_ADDRESS, &buffer[0], 4);
    if (error) {
        return error;
    }
    *sraw_voc = sensirion_common_bytes_to_uint16_t(&buffer[0]);
    *sraw_nox = sensirion_common_bytes_to_uint16_t(&buffer[2]);
    return NO_ERROR;
    */
    //return execute_cmd(dev, CMD_MEASURE_RAW, TIME_MEASURE_RAW, params, 2, raw, 1);
}

esp_err_t sgp41_measure_index(sgp41_t *dev, float humidity, float temperature, int8_t *voc_index, int8_t *nox_index)
{
    CHECK_ARG(dev);
    CHECK_ARG(voc_index);
    CHECK_ARG(nox_index);

    uint16_t voc_raw, nox_raw;
    CHECK(sgp41_measure_raw(dev, humidity, temperature, &voc_raw, &nox_raw));
    //VocAlgorithm_process(&dev->voc, raw, voc_index);

    return ESP_OK;
}

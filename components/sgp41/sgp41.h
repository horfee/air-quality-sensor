/**
 * @file sgp41.h
 * @defgroup sgp41 sgp41
 * @{
 *
 * ESP-IDF driver for SGP40 Indoor Air Quality Sensor for VOC Measurements
 *
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
 *
 * BSD Licensed as described in the file LICENSE
 */
#ifndef __SGP40_H__
#define __SGP40_H__

#include <stdbool.h>
#include <time.h>
#include <i2cdev.h>
#include <esp_err.h>

// #include <VOCGasIndexAlgorithm.h>
// #include <NOxGasIndexAlgorithm.h>
#include "algorithm/sensirion_gas_index_algorithm.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Device descriptor
 */
typedef struct
{
    i2c_dev_t i2c_dev;
    uint16_t serial[3];
    uint16_t featureset;
    GasIndexAlgorithmParams vocParams;
    GasIndexAlgorithmParams noxParams;
} sgp41_t;

/**
 * @brief Initialize device descriptor
 *
 * @param dev Device descriptor
 * @param port I2C port
 * @param sda_gpio SDA GPIO
 * @param scl_gpio SCL GPIO
 * @return `ESP_OK` on success
 */
esp_err_t sgp41_init_desc(sgp41_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);

/**
 * @brief Free device descriptor
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t sgp41_free_desc(sgp41_t *dev);

/**
 * @brief Read device information, initialize the VOC algorithm
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t sgp41_init(sgp41_t *dev);

/**
 * @brief Reset device, than put it to idle mode
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t sgp41_soft_reset(sgp41_t *dev);

/**
 * sgp41_execute_conditioning() - This command starts the conditioning, i.e.,
 * the VOC pixel will be operated at the same temperature as it is by calling
 * the sgp41_measure_raw command while the NOx pixel will be operated at a
 * different temperature for conditioning. This command returns only the
 * measured raw signal of the VOC pixel SRAW_VOC as 2 bytes (+ 1 CRC byte).
 *
 * @param dev Device descriptor
 * @param default_rh Default conditions for relative humidty.
 *
 * @param default_t Default conditions for temperature.
 *
 * @param sraw_voc u16 unsigned integer directly provides the raw signal
 * SRAW_VOC in ticks which is proportional to the logarithm of the resistance of
 * the sensing element.
 *
 * @return 0 on success, an error code otherwise
 */
esp_err_t sgp41_execute_conditioning(sgp41_t *dev, uint16_t default_rh, uint16_t default_t, uint16_t* sraw_voc);

/**
 * @brief Perform a self-test
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t sgp41_self_test(sgp41_t *dev);

/**
 * @brief Turn hotplate off, stop measurement and put device to idle mode
 *
 * @param dev Device descriptor
 * @return `ESP_OK` on success
 */
esp_err_t sgp41_heater_off(sgp41_t *dev);

/**
 * @brief Perform a measurement
 *
 * @param dev Device descriptor
 * @param humidity Relative humidity, percents. Use NaN if
 *                 you want uncompensated measurement
 * @param temperature Temperature, degrees Celsius. Use NaN if
 *                    you want uncompensated measurement
 * @param[out] raw Raw value, proportional to the logarithm
 *                 of the resistance of the sensing element
 * @return `ESP_OK` on success
 */
esp_err_t sgp41_measure_raw(sgp41_t *dev, float humidity, float temperature, uint16_t* sraw_voc, uint16_t* sraw_nox);

/**
 * @brief Perform a measurement and update VOC index
 *
 * @param dev Device descriptor
 * @param humidity Relative humidity, percents. Use NaN if
 *                 you want uncompensated measurement
 * @param temperature Temperature, degrees Celsius. Use NaN if
 *                    you want uncompensated measurement
 * @param[out] voc_index Calculated VOC index
 * @return
 */
esp_err_t sgp41_measure_index(sgp41_t *dev, float humidity, float temperature, int32_t *voc_index, int32_t *nox_index);

esp_err_t sgp41_convert_raw(sgp41_t *dev, uint16_t sraw_voc, uint16_t sraw_nox, int32_t *voc_index, int32_t *nox_index);

#ifdef __cplusplus
}
#endif

/**@}*/

#endif /* __SGP41_H__ */

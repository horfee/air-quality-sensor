/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS products only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

/* HomeKit Lightbulb Example
*/

#include <stdio.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>

#include <hap.h>
#include <hap_apple_servs.h>
#include <hap_apple_chars.h>
#include <hap_fw_upgrade.h>

#include <iot_button.h>

#include <app_wifi.h>
#include <app_hap_setup_payload.h>

#include "lightbulb.h"
#include <sht3x.h>
#include <sgp41.h>



/* Comment out the below line to disable Firmware Upgrades */
#define CONFIG_FIRMWARE_SERVICE
#define CONFIG_I2C_MASTER_SDA   21
#define CONFIG_I2C_MASTER_SCL   22

static const char *TAG = "CUSTOM";

static sht3x_t temperatureAndHumdityDevice;
static sgp41_t airqualityDevice;
static hap_serv_t *temperatureService;
static hap_serv_t *humidityService;
static hap_serv_t *airQualityService;

#define HOMEKIT_TASK_PRIORITY  1
#define HOMEKIT_TASK_STACKSIZE 4 * 1024
#define HOMEKIT_TASK_NAME      "homekit"

#define TEMPHUM_TASK_PRIORITY   1
#define TEMPHUM_TASK_STACKSIZE  4 * 1024
#define TEMPHUM_TASK_NAME       "temp_humidity"


#define AIRQUAL_TASK_NAME       "air_quality"
#define AIRQUAL_TASK_STACKSIZE  4 * 1024
#define AIRQUAL_TASK_PRIORITY   1


#define DEFAULT_COMPENSATION_TEMPERATURE    0x6666
#define DEFAULT_COMPENSATION_RHUMIDITY      0x8000

/* Reset network credentials if button is pressed for more than 3 seconds and then released */
#define RESET_NETWORK_BUTTON_TIMEOUT        3

/* Reset to factory if button is pressed and held for more than 10 seconds */
#define RESET_TO_FACTORY_BUTTON_TIMEOUT     10

/* The button "Boot" will be used as the Reset button for the example */
#define RESET_GPIO  GPIO_NUM_0
/**
 * @brief The network reset button callback handler.
 * Useful for testing the Wi-Fi re-configuration feature of WAC2
 */
static void reset_network_handler(void* arg)
{
    hap_reset_network();
}
/**
 * @brief The factory reset button callback handler.
 */
static void reset_to_factory_handler(void* arg)
{
    hap_reset_to_factory();
}

/**
 * The Reset button  GPIO initialisation function.
 * Same button will be used for resetting Wi-Fi network as well as for reset to factory based on
 * the time for which the button is pressed.
 */
static void reset_key_init(gpio_num_t key_gpio_pin)
{
    button_handle_t handle = iot_button_create(key_gpio_pin, BUTTON_ACTIVE_LOW);
    iot_button_add_on_release_cb(handle, RESET_NETWORK_BUTTON_TIMEOUT, reset_network_handler, NULL);
    iot_button_add_on_press_cb(handle, RESET_TO_FACTORY_BUTTON_TIMEOUT, reset_to_factory_handler, NULL);
}

/* Mandatory identify routine for the accessory.
 * In a real accessory, something like LED blink should be implemented
 * got visual identification
 */
static int device_identify(hap_acc_t *ha)
{
    ESP_LOGI(TAG, "Accessory identified");
    // TODO : make blinking a led
    return HAP_SUCCESS;
}

// /* Callback for handling writes on the Light Bulb Service
//  */
// static int lightbulb_write(hap_write_data_t write_data[], int count,
//         void *serv_priv, void *write_priv)
// {
//     int i, ret = HAP_SUCCESS;
//     hap_write_data_t *write;
//     for (i = 0; i < count; i++) {
//         write = &write_data[i];
//         /* Setting a default error value */
//         *(write->status) = HAP_STATUS_VAL_INVALID;
//         if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_ON)) {
//             ESP_LOGI(TAG, "Received Write for Light %s", write->val.b ? "On" : "Off");
//             if (lightbulb_set_on(write->val.b) == 0) {
//                 *(write->status) = HAP_STATUS_SUCCESS;
//             }
//         } else if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_BRIGHTNESS)) {
//             ESP_LOGI(TAG, "Received Write for Light Brightness %d", write->val.i);
//             if (lightbulb_set_brightness(write->val.i) == 0) {
//                 *(write->status) = HAP_STATUS_SUCCESS;
//             }
//         } else if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_HUE)) {
//             ESP_LOGI(TAG, "Received Write for Light Hue %f", write->val.f);
//             if (lightbulb_set_hue(write->val.f) == 0) {
//                 *(write->status) = HAP_STATUS_SUCCESS;
//             }
//         } else if (!strcmp(hap_char_get_type_uuid(write->hc), HAP_CHAR_UUID_SATURATION)) {
//             ESP_LOGI(TAG, "Received Write for Light Saturation %f", write->val.f);
//             if (lightbulb_set_saturation(write->val.f) == 0) {
//                 *(write->status) = HAP_STATUS_SUCCESS;
//             }
//         } else {
//             *(write->status) = HAP_STATUS_RES_ABSENT;
//         }
//         /* If the characteristic write was successful, update it in hap core
//          */
//         if (*(write->status) == HAP_STATUS_SUCCESS) {
//             hap_char_update_val(write->hc, &(write->val));
//         } else {
//             /* Else, set the return value appropriately to report error */
//             ret = HAP_FAIL;
//         }
//     }
//     return ret;
// }

static int8_t resolveAirQualityIndex(int32_t index) {
    if (index <= 1) return 0;   // undetermined;
    if ( index <= 150 ) return 1; // Excellent;
    if ( index <= 250 ) return 2; // Good
    if ( index <= 325 ) return 3; // Fair
    if ( index <= 400 ) return 4; // Inferior
    return 5; // Poor;
}

static void measureAirQuality(void *pvParameters) {


    float temperature, humidity;
    u_int16_t raw_voc, raw_nox;
    esp_err_t err;
    int conditioning = 10;

    uint16_t temperatureCompensation, rHumidityCompensation;

    vTaskDelay(pdMS_TO_TICKS(250));
    TickType_t last_wakeup = xTaskGetTickCount();

    while(1) {    
        err = sht3x_get_results(&temperatureAndHumdityDevice, &temperature, &humidity);
        ESP_ERROR_CHECK(err);
        if ( err != ESP_OK ) {
            temperatureCompensation = DEFAULT_COMPENSATION_TEMPERATURE;
            rHumidityCompensation = DEFAULT_COMPENSATION_RHUMIDITY;
        } else {
            temperatureCompensation = (temperature + 45) * 65535 / 175;
            rHumidityCompensation = (humidity * 65535 / 100);

            //hap_char_t *hap_serv_get_first_char(hap_serv_t *hs);
            hap_char_t* temperatureChar = hap_serv_get_first_char(temperatureService);
            hap_char_t* humidityChar = hap_serv_get_first_char(humidityService);

            hap_val_t tempVal, humidityVal;
            tempVal.f = temperature;
            humidityVal.f = humidity;
            hap_char_update_val(temperatureChar, &tempVal);
            hap_char_update_val(humidityChar, &humidityVal);

            printf("SHT3x Sensor: %.2f °C, %.2f %%\n", temperature, humidity);
        }


        hap_char_t* airQualityChar = hap_serv_get_first_char(airQualityService);
        if ( conditioning > 0 ) {
            err = sgp41_execute_conditioning(&airqualityDevice, temperatureCompensation, rHumidityCompensation, &raw_voc);
            if ( err == ESP_OK ) {
                conditioning--;
                //hap_char_update_val()
                int32_t vocIndex;
                if ( ESP_OK == sgp41_convert_raw(&airqualityDevice, raw_voc, 0, &vocIndex, NULL)) {
                    hap_val_t vocVal;
                    vocVal.i = resolveAirQualityIndex(vocIndex);
                    hap_char_update_val(airQualityChar, &vocVal);
                }
            } else {
                ESP_LOGI(TAG, "Failed executing conditioning...");
            }
        } else {
            err = sgp41_measure_raw(&airqualityDevice, rHumidityCompensation, temperatureCompensation, &raw_voc, &raw_nox);
            if ( err == ESP_OK ) {
                int32_t vocIndex;
                int32_t noxIndex;
                if ( ESP_OK == sgp41_convert_raw(&airqualityDevice, raw_voc, raw_nox, &vocIndex, &noxIndex)) {
                    hap_val_t vocVal;
                    vocVal.i = resolveAirQualityIndex(vocIndex);
                    hap_char_update_val(airQualityChar, &vocVal);
                    //TODO : alter index for VOC
                }
            } else {
                ESP_LOGI(TAG, "Failed measuring VOC and NOX");
            } 
        }
        // Feed it to SGP40
        ESP_LOGI(TAG, "%.2f °C, %.2f %%, VOC raw: %i, NOX raw : %i", temperature, humidity, raw_voc, raw_nox);//, voc_index);//, voc_index_name(voc_index));

        // Wait until 1 seconds (VOC cycle time) are over.
        vTaskDelayUntil(&last_wakeup, pdMS_TO_TICKS(1000));
    }

}

/*The main thread for handling the Light Bulb Accessory */
static void homekit_thread_entry(void *arg)
{
    hap_acc_t *accessory;

    /* Initialize the HAP core */
    hap_init(HAP_TRANSPORT_WIFI);

    /* Initialise the mandatory parameters for Accessory which will be added as
     * the mandatory services internally
     */
    hap_acc_cfg_t cfg = {
        .name = "AirQualitySensor",
        .manufacturer = "Horfee Inc.",
        .model = "AIQS01",
        .serial_num = "abcdefg",
        .fw_rev = "0.9.0",
        .hw_rev = "1.0",
        .pv = "1.0.0",
        .identify_routine = device_identify,
        .cid = HAP_CID_SENSOR,
    };

    /* Create accessory object */
    accessory = hap_acc_create(&cfg);
    if (!accessory) {
        ESP_LOGE(TAG, "Failed to create accessory");
        goto device_err;
    }

    /* Add a dummy Product Data */
    uint8_t product_data[] = {'E','S','P','3','2','H','A','P'};
    hap_acc_add_product_data(accessory, product_data, sizeof(product_data));

    /* Add Wi-Fi Transport service required for HAP Spec R16 */
    hap_acc_add_wifi_transport_service(accessory, 0);

    /* Create the Light Bulb Service. Include the "name" since this is a user visible service  */
    airQualityService = hap_serv_air_quality_sensor_create(0);
    if (!airQualityService) {
        ESP_LOGE(TAG, "Failed to create Air Quality Service");
        goto device_err;
    }

    /* Add the optional characteristic to the Light Bulb Service */
    int ret = hap_serv_add_char(airQualityService, hap_char_name_create("Air Quality"));
    //ret |= hap_serv_add_char(service, hap_char_current_relative_humidity_create(-1));
    //ret |= hap_serv_add_char(service, hap_char_current_temperature_create(-1));
    //ret |= hap_serv_add_char(service, hap_char_brightness_create(50));
    //ret |= hap_serv_add_char(service, hap_char_hue_create(180));
    //ret |= hap_serv_add_char(service, hap_char_saturation_create(100));
    
    if (ret != HAP_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add optional characteristics to Air Quality");
        goto device_err;
    }
    /* Set the write callback for the service */
    //hap_serv_set_write_cb(service, lightbulb_write);
    

    temperatureService = hap_serv_temperature_sensor_create(0);
    if (!temperatureService) {
        ESP_LOGE(TAG, "Failed to create Temperature Service");
        goto device_err;
    }

    ret = hap_serv_add_char(temperatureService, hap_char_name_create("Temperature"));
    if (ret != HAP_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add optional characteristics to Temperature");
        goto device_err;
    }

    humidityService = hap_serv_humidity_sensor_create(0);
    if (!humidityService) {
        ESP_LOGE(TAG, "Failed to create Relative Humidity Service");
        goto device_err;
    }

    ret = hap_serv_add_char(humidityService, hap_char_name_create("Humidity"));
    if (ret != HAP_SUCCESS) {
        ESP_LOGE(TAG, "Failed to add optional characteristics to Humidity");
        goto device_err;
    }
    /* Add the Air Quality Service to the Accessory Object */
    hap_acc_add_serv(accessory, airQualityService);
    hap_acc_add_serv(accessory, temperatureService);
    hap_acc_add_serv(accessory, humidityService);

    hap_serv_link_serv(airQualityService,temperatureService);
    hap_serv_link_serv(airQualityService,humidityService);


#ifdef CONFIG_FIRMWARE_SERVICE
    /*  Required for server verification during OTA, PEM format as string  */
    static char server_cert[] = {};
    hap_fw_upgrade_config_t ota_config = {
        .server_cert_pem =  server_cert,
    };
    /* Create and add the Firmware Upgrade Service, if enabled.
     * Please refer the FW Upgrade documentation under components/homekit/extras/include/hap_fw_upgrade.h
     * and the top level README for more information.
     */
    hap_serv_t *service = hap_serv_fw_upgrade_create(&ota_config);
    if (!service) {
        ESP_LOGE(TAG, "Failed to create Firmware Upgrade Service");
        goto device_err;
    }
    hap_acc_add_serv(accessory, service);
#endif

    /* Add the Accessory to the HomeKit Database */
    hap_add_accessory(accessory);

    /* Initialize the Light Bulb Hardware */
    //lightbulb_init();

    /* Register a common button for reset Wi-Fi network and reset to factory.
     */
    reset_key_init(RESET_GPIO);

    /* TODO: Do the actual hardware initialization here */

    /* For production accessories, the setup code shouldn't be programmed on to
     * the device. Instead, the setup info, derived from the setup code must
     * be used. Use the factory_nvs_gen utility to generate this data and then
     * flash it into the factory NVS partition.
     *
     * By default, the setup ID and setup info will be read from the factory_nvs
     * Flash partition and so, is not required to set here explicitly.
     *
     * However, for testing purpose, this can be overridden by using hap_set_setup_code()
     * and hap_set_setup_id() APIs, as has been done here.
     */
#ifdef CONFIG_EXAMPLE_USE_HARDCODED_SETUP_CODE
    /* Unique Setup code of the format xxx-xx-xxx. Default: 111-22-333 */
    hap_set_setup_code(CONFIG_EXAMPLE_SETUP_CODE);
    /* Unique four character Setup Id. Default: ES32 */
    hap_set_setup_id(CONFIG_EXAMPLE_SETUP_ID);
#ifdef CONFIG_APP_WIFI_USE_WAC_PROVISIONING
    app_hap_setup_payload(CONFIG_EXAMPLE_SETUP_CODE, CONFIG_EXAMPLE_SETUP_ID, true, cfg.cid);
#else
    app_hap_setup_payload(CONFIG_EXAMPLE_SETUP_CODE, CONFIG_EXAMPLE_SETUP_ID, false, cfg.cid);
#endif
#endif

    /* Enable Hardware MFi authentication (applicable only for MFi variant of SDK) */
    hap_enable_mfi_auth(HAP_MFI_AUTH_HW);

    /* Initialize Wi-Fi */
    app_wifi_init();

    /* After all the initializations are done, start the HAP core */
    hap_start();

    /* Start Wi-Fi */
    app_wifi_start(portMAX_DELAY);

    /* The task ends here. The read/write callbacks will be invoked by the HAP Framework */
    vTaskDelete(NULL);

device_err:
    hap_acc_delete(accessory);
    vTaskDelete(NULL);
}

void app_main()
{
    ESP_LOGI(TAG, "Initializing i2c");
    ESP_ERROR_CHECK(i2cdev_init());

    // setup SHT3x
    ESP_LOGI(TAG, "Initializing SHT31 device");
    memset(&temperatureAndHumdityDevice, 0, sizeof(temperatureAndHumdityDevice));
    ESP_LOGI(TAG, "memset ok");
    ESP_ERROR_CHECK(sht3x_init_desc(&temperatureAndHumdityDevice, SHT3X_I2C_ADDR_GND /*SHT3X_I2C_ADDR_VDD*/, 0, CONFIG_I2C_MASTER_SDA, CONFIG_I2C_MASTER_SCL));
    ESP_LOGI(TAG, "init desc ok");
    ESP_ERROR_CHECK(sht3x_init(&temperatureAndHumdityDevice));
    ESP_LOGI(TAG, "init ok");
    // Start periodic measurements with 2 measurements per second.
    ESP_ERROR_CHECK(sht3x_start_measurement(&temperatureAndHumdityDevice, SHT3X_PERIODIC_1MPS, SHT3X_HIGH));
    ESP_LOGI(TAG, "Humidity sensor initilalized");

    // setup sgp41
    ESP_LOGI(TAG, "Initializing SGP41 device");
    memset(&airqualityDevice, 0, sizeof(airqualityDevice));
    ESP_ERROR_CHECK(sgp41_init_desc(&airqualityDevice, 0, CONFIG_I2C_MASTER_SDA, CONFIG_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(sgp41_init(&airqualityDevice));
    ESP_LOGI(TAG, "SGP40 initilalized. Serial: 0x%04x%04x%04x", airqualityDevice.serial[0], airqualityDevice.serial[1], airqualityDevice.serial[2]);

    xTaskCreate(homekit_thread_entry, HOMEKIT_TASK_NAME, HOMEKIT_TASK_STACKSIZE, NULL, HOMEKIT_TASK_PRIORITY, NULL);
    //xTaskCreate(measureTemperatureAndHumidityTask, TEMPHUM_TASK_NAME, TEMPHUM_TASK_STACKSIZE, NULL, TEMPHUM_TASK_PRIORITY, NULL);
    xTaskCreate(measureAirQuality, AIRQUAL_TASK_NAME, AIRQUAL_TASK_STACKSIZE, NULL, AIRQUAL_TASK_PRIORITY, NULL);
    ESP_LOGI(TAG, "Initialized successfully");
}

#include <stdio.h>
#include <string.h>
#include <string>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>
#include <mpu6050.h>
#include <math.h>
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_now.h"

#include "nvs_flash.h" //TOSO . replace with littlefs

#define ADDR MPU6050_I2C_ADDRESS_LOW
#define LEFT_DEVICE 0
#define RIGHT_DEVICE 1
#define SDA_PIN 6 //D4 on XIAO ESP32C3
#define SCL_PIN 7 //D5 on XIAO ESP32C3
#define PMK "DFGTR98756123456"
#define LMK "ALSFEASMDKREQWSD"

static const char *TAG = "Mpu6050";
unsigned char mac_base[6] = {0};
unsigned char mac_target[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
unsigned char mac_right[6] = {0xE4, 0xB3, 0x23, 0xC5, 0x91, 0x7C};
unsigned char mac_left[6] = {0xE4, 0xB3, 0x23, 0xC4, 0x75, 0xD0};

static uint8_t device_type = LEFT_DEVICE;
esp_now_peer_info_t peerInfo;

typedef struct mpudata_t {
    float roll;
    float pitch;
    float temp;
} mpudata_t;


void mpu6050(void *pvParameters)
{
    mpu6050_dev_t dev = { (i2c_port_t)0 };

    ESP_ERROR_CHECK(mpu6050_init_desc(&dev, ADDR, (i2c_port_t)0, (gpio_num_t)SDA_PIN, (gpio_num_t)SCL_PIN));

    while (1)
    {
        esp_err_t res = i2c_dev_probe(&dev.i2c_dev, I2C_DEV_WRITE);
        if (res == ESP_OK)
        {
            ESP_LOGI(TAG, "Found MPU60x0 device");
            break;
        }
        ESP_LOGE(TAG, "MPU60x0 not found");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_ERROR_CHECK(mpu6050_init(&dev));

    ESP_LOGI(TAG, "Accel range: %d", dev.ranges.accel);
    ESP_LOGI(TAG, "Gyro range:  %d", dev.ranges.gyro);

    // Gyro calibration
    float gyro_x_offset = 0;
    float gyro_y_offset = 0;
    float gyro_z_offset = 0;

    ESP_LOGI(TAG, "Calibrating gyroscope, keep the device still...");
    for (int i = 0; i < 500; i++)
    {
        mpu6050_rotation_t rotation;
        ESP_ERROR_CHECK(mpu6050_get_rotation(&dev, &rotation));
        gyro_x_offset += rotation.x;
        gyro_y_offset += rotation.y;
        gyro_z_offset += rotation.z;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    gyro_x_offset /= 500;
    gyro_y_offset /= 500;
    gyro_z_offset /= 500;
    ESP_LOGI(TAG, "Calibration complete.");

    while (1)
    {
        mpudata_t data = {0, 0, 0};
        mpu6050_acceleration_t accel = {};
        mpu6050_rotation_t rotation = {};

        ESP_ERROR_CHECK(mpu6050_get_temperature(&dev, &data.temp));
        ESP_ERROR_CHECK(mpu6050_get_motion(&dev, &accel, &rotation));

        // Calibrate gyroscope readings
        rotation.x -= gyro_x_offset;
        rotation.y -= gyro_y_offset;

        // Calculate unfiltered roll and pitch from accelerometer
        data.roll = atan2(accel.y, accel.z) * 180.0 / M_PI;
        data.pitch = atan2(-accel.x, sqrt(accel.y * accel.y + accel.z * accel.z)) * 180.0 / M_PI;

        ESP_LOGI(TAG, "Roll: %.2f, Pitch: %.2f", data.roll, data.pitch);
        if(device_type == LEFT_DEVICE)
        {
            // Send data via ESP-NOW
            esp_err_t result = esp_now_send(mac_target, (uint8_t *)&data, sizeof(data));
            if (result == ESP_OK) {
                ESP_LOGI(TAG, "Sent data to RIGHT_DEVICE");
            } else {
                ESP_LOGE(TAG, "Error sending data: %s", esp_err_to_name(result));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}


static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    ESP_LOGI(TAG, "Last Packet Send Status: %s", status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Updated callback function with robust MAC address printing
void OnDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
    mpudata_t received_data;
    if (len != sizeof(received_data)) {
        ESP_LOGE(TAG, "Received data length mismatch: expected %d, got %d", sizeof(received_data), len);
        return;
    }

    memcpy(&received_data, incomingData, sizeof(received_data));
    ESP_LOGI(TAG, "Roll: %.2f, Pitch: %.2f, Temp: %.2f", received_data.roll, received_data.pitch, received_data.temp);
}

void init_esp_now() {
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(OnDataSent));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(OnDataRecv));
}


extern "C" void app_main()
{
    //init_nvs(); //TODO: will come with littleFS
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );


    wifi_init();
    init_esp_now();
    esp_now_set_pmk((uint8_t *)PMK);

    esp_efuse_mac_get_default(mac_base);
    ESP_LOGI(TAG, "MAC address: %02X:%02X:%02X:%02X:%02X:%02X", mac_base[0], mac_base[1], mac_base[2], mac_base[3], mac_base[4], mac_base[5]);
    if(mac_base[5] == 124) //0x7C
    {
        ESP_LOGI(TAG, "Using RIGHT_DEVICE configuration");
        device_type = RIGHT_DEVICE;
        memcpy(mac_target, mac_left, sizeof(mac_target));
    }
    else
    {
        ESP_LOGI(TAG, "Using LEFT_DEVICE configuration");
        device_type = LEFT_DEVICE;
        memcpy(mac_target, mac_right, sizeof(mac_target));
    }
    //Log if device_type is LEFT_DEVICE
    ESP_LOGI(TAG, "This device is set as: %s", device_type == LEFT_DEVICE ? "LEFT_DEVICE" : "RIGHT_DEVICE");

    memcpy(peerInfo.peer_addr, mac_target, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = true;
    memcpy(peerInfo.lmk, LMK, 16);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add peer");
        return;
    }

    if(device_type == LEFT_DEVICE)
    {
        ESP_ERROR_CHECK(i2cdev_init());
        xTaskCreate(mpu6050, "mpu6050", configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL);
        return;
    }
    else
    {
        while (1)
        {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }

}
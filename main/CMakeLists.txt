idf_component_register(SRCS "main.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES "mpu6050" nvs_flash esp_event esp_netif esp_wifi)

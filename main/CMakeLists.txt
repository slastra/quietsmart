idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES nvs_flash esp_wifi esp_event mqtt mdns esp_driver_gpio esp_driver_ledc esp_driver_pcnt driver)
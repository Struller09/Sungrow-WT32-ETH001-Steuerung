idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES
        esp_http_server
        esp_eth
        esp_wifi
        esp_netif
        esp_event
        nvs_flash
        app_update
        esp_timer
        mbedtls
        driver
        esp_driver_gpio
		lwip
)
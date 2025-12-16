idf_component_register(
    SRCS "MINIVMACESP32.c"
	 "TCA9554PWR.c"
	 "ESP32API.c"
	 "OSGLUESP32.c"
    INCLUDE_DIRS "." "../components/minivmac_allarchs"
    PRIV_REQUIRES spiffs esp_timer esp_lcd)

spiffs_create_partition_image(spiffs ${CMAKE_CURRENT_LIST_DIR}/../spiffs FLASH_IN_PROJECT)

idf_component_get_property(lvgl_lib lvgl__lvgl COMPONENT_LIB)

target_compile_options(${lvgl_lib} PRIVATE -Wno-format)

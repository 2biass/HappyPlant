idf_component_register( SRCS "happyplant_main.c"
    INCLUDE_DIRS "."
	REQUIRES hp_filesystem hp_softap hp_webserver hp_sensors spi_flash nvs_flash)
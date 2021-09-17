
idf_component_register(SRCS "hp_webserver.c"
             	INCLUDE_DIRS "include"
				 REQUIRES hp_sensors esp_http_server json)
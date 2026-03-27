idf_component_register(SRCS "iis_audio.c"
                      INCLUDE_DIRS "."
                      REQUIRES esp_driver_i2s)
component_compile_options(-ffast-math -O3 -Wno-error=format -Wno-format)
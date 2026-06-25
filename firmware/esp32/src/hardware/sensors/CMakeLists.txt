idf_component_register(
    SRCS
        "adc.cpp"
        "bmi088.cpp"

        "wheel-encoder.cpp"
        "lighthouse.cpp"

        "bosch_sensortec/bmi08g.c"
        "bosch_sensortec/bmi08a.c"
    INCLUDE_DIRS
        "include"
        "bosch_sensortec"
    REQUIRES
        main
        control-types
        interfaces
        spi
        uart
        util
        "esp_adc"
)

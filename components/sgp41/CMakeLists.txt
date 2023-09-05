idf_component_register(
    SRCS sgp41.c algorithm/sensirion_gas_index_algorithm.c
    INCLUDE_DIRS .
    REQUIRES i2cdev log esp_idf_lib_helpers
)

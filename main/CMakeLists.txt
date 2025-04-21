idf_component_register(SRCS
    $<$<BOOL:${CONFIG_SENDER}>:sender.c>
    $<$<BOOL:${CONFIG_RECEIVER}>:receiver.c>
    INCLUDE_DIRS ".")

# Ensure the component includes the custom Kconfig
set(COMPONENT_ADD_INCLUDEDIRS ".")
set(COMPONENT_REQUIRES "esp_timer" "esp_wifi" "esp_now" "esp_event" 
                    "nvs_flash" "esp_idf_lib_helpers" "i2cdev" "ads111x")
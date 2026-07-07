idf_component_register(
    SRCS audio_sr.cpp
    INCLUDE_DIRS .
    REQUIRES espressif__esp-sr espressif__esp_codec_dev ei_model driver)

# Suppress third-party warnings from Edge Impulse SDK headers
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-error" "-Wno-maybe-uninitialized" "-Wno-uninitialized" "-Wno-deprecated-declarations")

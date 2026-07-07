cmake_minimum_required(VERSION 3.13.1)

idf_component_register(SRCS
        "main.cpp"
        "camera_capture.c"
        "video_dev.c"
        "rtsp_service.c"
        "app_ethernet.c"
    INCLUDE_DIRS
        "."
        ".."
    PRIV_REQUIRES
        espressif__esp-dl
        esp_driver_gpio
        esp_driver_spi
        esp_driver_uart
        esp_eth
        esp_event
        esp_http_server
        esp_mm
        esp_netif
        esp_system
        esp_timer
        espressif__media_lib_sal
        lwip)

set(cmake_dir "${CMAKE_CURRENT_LIST_DIR}/../managed_components/espressif__esp-dl/fbs_loader/cmake")
include("${cmake_dir}/utilities.cmake")
target_add_aligned_binary_data(${COMPONENT_LIB}
    "gesture_cnn_96_gray_esp32p4.espdl"
    BINARY
    16)

target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++17)

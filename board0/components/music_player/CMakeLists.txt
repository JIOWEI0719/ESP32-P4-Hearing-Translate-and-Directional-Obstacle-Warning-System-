idf_component_register(
    SRCS music_player.c
    INCLUDE_DIRS .
    REQUIRES bsp_extra espressif__esp_codec_dev chmorgan__esp-audio-player espressif__esp32_p4_function_ev_board_noglib
)

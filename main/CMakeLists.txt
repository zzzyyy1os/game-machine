idf_component_register(SRCS "hw_test.c" "lcd.c" "sdcard.c" "mp4play.c" "mp3play.c" "max98357a.c" "rocker.c" "button.c"
                    PRIV_REQUIRES driver fatfs sdmmc esp_timer esp_video_codec esp_extractor esp_driver_i2s esp_adc espressif__esp_audio_codec
                    INCLUDE_DIRS ".")

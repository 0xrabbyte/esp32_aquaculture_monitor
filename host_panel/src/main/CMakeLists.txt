#Add sources from ui directory
file(GLOB_RECURSE SRC_UI ${CMAKE_SOURCE_DIR} "ui/*.c")
file(GLOB_RECURSE SRC_HTTPS ${CMAKE_SOURCE_DIR} "https_mbedtls/*.c")
file(GLOB_RECURSE SRC_SR ${CMAKE_SOURCE_DIR} "app_sr/*.c")

idf_component_register(SRCS "bsp_board.c" "main.c" ${SRC_UI} ${SRC_HTTPS} ${SRC_SR}
                    INCLUDE_DIRS "." "ui" "https_mbedtls" "app_sr")
                    
spiffs_create_partition_image(storage ../spiffs FLASH_IN_PROJECT)

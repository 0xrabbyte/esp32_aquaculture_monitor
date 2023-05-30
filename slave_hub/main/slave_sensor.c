#include <stdio.h>
#include <math.h>
#include <ctype.h>

#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_err.h"

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
#include "esp_mac.h"
#endif

#include "espnow.h"
#include "espnow_ctrl.h"
#include "espnow_utils.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "driver/gpio.h"

#include "nvs_flash.h"
#include "esp_camera.h"

#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "sdkconfig.h"

#include "camera_pin.h"

#define I2C_MASTER_NUM 0              /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 100000     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0   /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0   /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

#define FLASHLIGHT_PIN    GPIO_NUM_4

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

const char *TAG = "slave_sensor";
const uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
uint8_t master_mac[ESP_NOW_ETH_ALEN] = {0};
#ifdef CONFIG_USE_MASTER_MAC
char mac_addr[] = CONFIG_MASTER_MAC_ADDR;

static inline int get_16bit(char ch) {
    return isdigit(ch) ? ch - '0' : tolower(ch) - 'a' + 10;
}

static inline void get_master_mac() {
    master_mac[0] = get_16bit(mac_addr[0]) << 4 | get_16bit(mac_addr[1]);
    master_mac[1] = get_16bit(mac_addr[3]) << 4 | get_16bit(mac_addr[4]);
    master_mac[2] = get_16bit(mac_addr[6]) << 4 | get_16bit(mac_addr[7]);
    master_mac[3] = get_16bit(mac_addr[9]) << 4 | get_16bit(mac_addr[10]);
    master_mac[4] = get_16bit(mac_addr[12]) << 4 | get_16bit(mac_addr[13]);
    master_mac[5] = get_16bit(mac_addr[15]) << 4 | get_16bit(mac_addr[16]);
}
#endif

static inline uint8_t* get_cast_mac() {
    #ifdef CONFIG_USE_MASTER_MAC
    return master_mac;
    #else
    return broadcast_mac;
    #endif
}

bool sd_mounted = false;
int photo_count = 0;
uint32_t image_size, total_packages;
uint8_t buf[33], image_buf[25000];

struct _aqua_data {
    float TDS_value;
    float pH_value;
    float NTH_value;
    float water_level_value;
    float water_temperature;
    bool is_pump;
} aqua_data;

struct _control_data {
    bool is_pump;
    bool is_servo;
    bool open_camera;
} control_data;

static struct _motor_data {
    bool is_pump;
    bool is_servo;
} motor_data;

void memory_monitor()
{
    static char buffer[128];    /* Make sure buffer is enough for `sprintf` */
    if (1) {
        sprintf(buffer, "   Biggest /     Free /    Total\n"
                "\t  SRAM : [%8d / %8d / %8d]\n"
                "\t PSRAM : [%8d / %8d / %8d]",
                heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
                heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                heap_caps_get_total_size(MALLOC_CAP_INTERNAL),
                heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM),
                heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                heap_caps_get_total_size(MALLOC_CAP_SPIRAM));
        ESP_LOGI("MEM", "%s", buffer);
    }
    int currentMem = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    static int LastMem = 0;
    static int originMem = 0;
    static int total_dec = 0;
    if (currentMem ^ LastMem) {
        if (LastMem) {
            total_dec = (currentMem - originMem);
        } else {
            originMem = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        }
        ESP_LOGI("MEM Lost", "%d, %d", currentMem - LastMem, total_dec);
        LastMem = currentMem;
    }
}

static void app_wifi_init() {
    esp_event_loop_create_default();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void app_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    uint8_t *mac_addr = recv_info->src_addr;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

    if (len != sizeof(struct _control_data)) {
        ESP_LOGE(TAG, "Receive cb arg len error, len: %d", len);
        return;
    }

    memcpy(&control_data, data, sizeof(struct _control_data));

    ESP_LOGI(TAG, "Receive cb, from: " MACSTR ", is_pump: %d, open_camera: %d, is_servo: %d",
             MAC2STR(mac_addr), control_data.is_pump, control_data.open_camera, control_data.is_servo);

    aqua_data.is_pump = control_data.is_pump;
    motor_data.is_pump = control_data.is_pump;
    motor_data.is_servo = control_data.is_servo;
    i2c_master_write_to_device(I2C_MASTER_NUM, CONFIG_ARDUINO_SLAVE_ADDR, (uint8_t *)&motor_data, sizeof(struct _motor_data), I2C_MASTER_TIMEOUT_MS);
}

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = CONFIG_I2C_MASTER_SDA_IO,
        .scl_io_num = CONFIG_I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void camera_init() {
    camera_config_t config = {
        .ledc_channel = LEDC_CHANNEL_0,
        .ledc_timer = LEDC_TIMER_0,
        .pin_d0 = Y2_GPIO_NUM,
        .pin_d1 = Y3_GPIO_NUM,
        .pin_d2 = Y4_GPIO_NUM,
        .pin_d3 = Y5_GPIO_NUM,
        .pin_d4 = Y6_GPIO_NUM,
        .pin_d5 = Y7_GPIO_NUM,
        .pin_d6 = Y8_GPIO_NUM,
        .pin_d7 = Y9_GPIO_NUM,
        .pin_xclk = XCLK_GPIO_NUM,
        .pin_pclk = PCLK_GPIO_NUM,
        .pin_vsync = VSYNC_GPIO_NUM,
        .pin_href = HREF_GPIO_NUM,
        .pin_sscb_sda = SIOD_GPIO_NUM,
        .pin_sscb_scl = SIOC_GPIO_NUM,
        .pin_pwdn = PWDN_GPIO_NUM,
        .pin_reset = RESET_GPIO_NUM,
        .xclk_freq_hz = 20000000,
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size = FRAMESIZE_CIF,
        .jpeg_quality = 2,
        .fb_count = 2,
        .fb_location = CAMERA_FB_IN_PSRAM,
    };

    // Init Camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return;
    }
}

void sd_init() {
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_card_t *card;
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card VFAT filesystem. Error: %s", esp_err_to_name(ret));
        return;
    }
    sd_mounted = true;
}

void app_main(void) {
    ESP_ERROR_CHECK(i2c_master_init());
    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    gpio_reset_pin(FLASHLIGHT_PIN);
    gpio_set_direction(FLASHLIGHT_PIN, GPIO_MODE_OUTPUT);

    memory_monitor();
    camera_init();
    sd_init();
    memory_monitor();

    app_wifi_init();
    espnow_config_t espnow_config = ESPNOW_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(espnow_init(&espnow_config));

    ESP_ERROR_CHECK(esp_now_register_recv_cb(app_espnow_recv_cb));
    
    #ifdef CONFIG_USE_MASTER_MAC
    get_master_mac();
    esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
    if (peer == NULL) {
        ESP_LOGE(TAG, "Malloc peer information fail");
		ESP_ERROR_CHECK(espnow_deinit());
        vTaskDelete(NULL);
    }
    memset(peer, 0, sizeof(esp_now_peer_info_t));
    memcpy(peer->peer_addr, master_mac, ESP_NOW_ETH_ALEN);
    ESP_ERROR_CHECK(esp_now_add_peer(peer));
    free(peer);
    #endif

    aqua_data.NTH_value = -1;         // NTH sensor
    aqua_data.pH_value = -1;          // pH sensor
    aqua_data.TDS_value = -1;         // TDS sensor
    aqua_data.water_level_value = -1; // water level sensor
    aqua_data.water_temperature = -1; // water temperature sensor
    aqua_data.is_pump = false;        // pump status

    while (true) {
        if (control_data.open_camera) {
            ESP_LOGI(TAG, "Begin Camera Image Transferring");
            #ifdef CONFIG_USE_FLASHLIGHT
            gpio_set_level(FLASHLIGHT_PIN, 1);
            #endif
            camera_fb_t *fb = esp_camera_fb_get();
            if (!fb) {
                ESP_LOGE(TAG, "Camera capture failed");
                control_data.open_camera = false;
                esp_camera_fb_return(fb);
                #ifdef CONFIG_USE_FLASHLIGHT
                gpio_set_level(FLASHLIGHT_PIN, 1);
                #endif
                continue;
            }
            ESP_LOGI(TAG, "Camera capture OK");
            image_size = fb->len;
            memcpy(&image_buf, fb->buf, fb->len);
            if (sd_mounted) {
                char *filename = malloc(15);
                sprintf(filename, "/sdcard/%d.jpg", photo_count++);
                FILE *file = fopen(filename, "w");
                if (!file) {
                    ESP_LOGE(TAG, "Failed to open file for writing");
                }
                else {
                    fwrite(image_buf, image_size, 1, file);
                    ESP_LOGI(TAG, "File written to %s", filename);
                }
                fclose(file);
                free(filename);
            }
            ESP_LOGI(TAG, "Camera buffer size: %d", (int)image_size);
            total_packages = ceil(image_size / CONFIG_ONE_SINGLE_PACKAGE_SIZE);
            uint64_t first_package = ((uint64_t)total_packages << 32) | image_size;
            esp_now_send(get_cast_mac(), (uint8_t*)&first_package, sizeof(first_package));
            vTaskDelay(CONFIG_IMAGE_DELAY_PERIOD_MS / portTICK_PERIOD_MS);
            ESP_LOGI(TAG, "Sending first package: %llu = %lu + %lu", first_package, total_packages, image_size);
            memory_monitor();
            for (int i = 0, p = 0; i < total_packages; i++, p += CONFIG_ONE_SINGLE_PACKAGE_SIZE) {
                size_t package_size = CONFIG_ONE_SINGLE_PACKAGE_SIZE;
                ESP_LOGI(TAG, "Sending package %d/%d", i + 1, (int)total_packages);
                if (p + CONFIG_ONE_SINGLE_PACKAGE_SIZE > image_size)
                    package_size = image_size - p;
                memcpy(buf, image_buf + p, package_size);
                buf[package_size] = i;
                esp_now_send(get_cast_mac(), buf, package_size + 1);
                vTaskDelay(CONFIG_IMAGE_DELAY_PERIOD_MS / portTICK_PERIOD_MS);
            }
            ESP_LOGI(TAG, "Image Transferring Done");
            control_data.open_camera = false;
            #ifdef CONFIG_USE_FLASHLIGHT
            gpio_set_level(FLASHLIGHT_PIN, 0);
            #endif
            esp_camera_fb_return(fb);
        }
        else {
            i2c_master_read_from_device(I2C_MASTER_NUM, CONFIG_ARDUINO_SLAVE_ADDR, (uint8_t *)&aqua_data, sizeof(struct _aqua_data), I2C_MASTER_TIMEOUT_MS);
            ESP_LOGI(TAG, "TDS: %f, pH: %f, NTH: %f, water_level: %f, water_temperature: %f, is_pump: %d", aqua_data.TDS_value, aqua_data.pH_value, aqua_data.NTH_value, aqua_data.water_level_value, aqua_data.water_temperature, aqua_data.is_pump);
            ESP_ERROR_CHECK(esp_now_send(get_cast_mac(), &aqua_data, sizeof(struct _aqua_data)));
            vTaskDelay(CONFIG_DATA_UPDATE_PERIOD_MS / portTICK_PERIOD_MS);
        }
    }
}

/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "esp_log.h"
#include "espnow.h"
#include "espnow_ctrl.h"
#include "espnow_utils.h"
#include "bsp/esp32_s3_lcd_ev_board.h"
#include "lvgl.h"
#include "ui/ui.h"
#include "ui/ui_events.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_wpa2.h"
#include "esp_timer.h"
#include "freeRTOS/FreeRTOS.h"
#include "freeRTOS/event_groups.h"
#include "freertos/task.h"
#include "esp_smartconfig.h"
#include "esp_spiffs.h"
#include "sdkconfig.h"
#include "https_mbedtls/https_mbedtls.h"
#include "esp_sntp.h"
#include "sys/time.h"
#include "esp_system.h"
#include "bsp_board.h"
#include "audio_player.h"
#include "app_sr/app_sr.h"


#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
#include "esp_mac.h"
#endif


#define TAG "AQUACULTURE_PANEL"

bool is_connected = false;

static const int CONNECTED_BIT = BIT0;
static const int ESPTOUCH_DONE_BIT = BIT1;

/* FreeRTOS event group to signal when we are connected & ready to make a request */
EventGroupHandle_t s_wifi_event_group;

static void smartconfig_example_task(void * parm);

static void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification of a time synchronization event, sec=%lu", tv->tv_sec);
    settimeofday(tv, NULL);
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        xTaskCreate(smartconfig_example_task, "smartconfig_example_task", 4096, NULL, 3, NULL);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        wifi_config_t wifi_config;
        ESP_ERROR_CHECK( esp_wifi_get_config(ESP_IF_WIFI_STA, &wifi_config));
        #ifdef CONFIG_ENABLE_SEU_LOGIN
        if (strcmp((char*)wifi_config.sta.ssid, "SEU-WLAN") == 0) {
            ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
            char * ip_addr_buf = (char *)malloc(20);
            sprintf(ip_addr_buf, IPSTR, IP2STR(&event->ip_info.ip));
            https_client_config_t config;
	        strcpy(config.server, "w.seu.edu.cn"); strcpy(config.port, "802");
            strcpy(config.url,"https://w.seu.edu.cn:802/eportal/?c=Portal&a=login&callback=dr1004&login_method=1&user_account="CONFIG_SEU_LOGIN_USERNAME"&user_password="CONFIG_SEU_LOGIN_PASSWORD"&wlan_user_ip=");
            strcat(config.url, ip_addr_buf);
            xTaskCreate(&https_get_task, "https_get_task", 8192, &config, 5, NULL);
        }
        #endif
        is_connected = true;
        const char * data = CONFIG_DATA_URL;
        lv_qrcode_update(ui_QR_Code_Image, data, strlen(data));
        lv_obj_clear_flag(ui_UpdateDropdown, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(ui_Wifi_Button, LV_OBJ_FLAG_HIDDEN);
        const esp_timer_create_args_t periodic_timer_args = {
            .callback = &periodic_timer_callback,
            /* name is optional, but may help identify the timer when debugging */
            .name = "periodic_upload_the_data",
        };
        esp_timer_handle_t periodic_timer;
        ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
        ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 60ll * 1e6));
        setenv("TZ", "CST-8", 1);
        tzset();
        ESP_LOGI(TAG, "Initializing SNTP");
        sntp_setoperatingmode(SNTP_OPMODE_POLL);
        sntp_setservername(0, "ntp.tencent.com");
        sntp_setservername(1, "time.asia.apple.com");
        sntp_setservername(2, "pool.ntp.org");
        sntp_set_time_sync_notification_cb(time_sync_notification_cb);
        sntp_init();

        // wait for time to be set
        time_t now = 0;
        struct tm timeinfo = {0};
        int retry = 0;
        const int retry_count = 10;
        while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
            ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
        while (sntp_get_sync_status() == SNTP_SYNC_STATUS_IN_PROGRESS) {
            struct timeval outdelta;
            adjtime(NULL, &outdelta);
            ESP_LOGI(TAG, "Waiting for adjusting time ... outdelta = %li sec: %li ms: %li us",
                        (long)outdelta.tv_sec,
                        outdelta.tv_usec/1000,
                        outdelta.tv_usec%1000);
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }
        time(&now);
        localtime_r(&now, &timeinfo);
        ESP_LOGI(TAG, "Time is set, yy: %d, mm: %d, dd: %d, hh: %d, mm: %d, ss: %d",
                    timeinfo.tm_year, timeinfo.tm_mon, timeinfo.tm_mday,
                    timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
        FILE* fp = fopen("/spiffs/wifi.mp3", "rb");
        audio_player_play(fp);
        bsp_led_set_rgb(0, 0, 100, 0);
        xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SCAN_DONE) {
        ESP_LOGI(TAG, "Scan done");
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_FOUND_CHANNEL) {
        ESP_LOGI(TAG, "Found channel");
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD) {
        ESP_LOGI(TAG, "Got SSID and password");

        smartconfig_event_got_ssid_pswd_t *evt = (smartconfig_event_got_ssid_pswd_t *)event_data;
        wifi_config_t wifi_config;
        uint8_t ssid[33] = { 0 };
        uint8_t password[65] = { 0 };
        uint8_t rvd_data[33] = { 0 };

        bzero(&wifi_config, sizeof(wifi_config_t));
        memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
        memcpy(wifi_config.sta.password, evt->password, sizeof(wifi_config.sta.password));
        wifi_config.sta.bssid_set = evt->bssid_set;
        if (wifi_config.sta.bssid_set == true) {
            memcpy(wifi_config.sta.bssid, evt->bssid, sizeof(wifi_config.sta.bssid));
        }

        memcpy(ssid, evt->ssid, sizeof(evt->ssid));
        memcpy(password, evt->password, sizeof(evt->password));
        ESP_LOGI(TAG, "SSID:%s", ssid);
        ESP_LOGI(TAG, "PASSWORD:%s", password);
        if (evt->type == SC_TYPE_ESPTOUCH_V2) {
            ESP_ERROR_CHECK( esp_smartconfig_get_rvd_data(rvd_data, sizeof(rvd_data)) );
            ESP_LOGI(TAG, "RVD_DATA:");
            for (int i=0; i<33; i++) {
                printf("%02x ", rvd_data[i]);
            }
            printf("\n");
        }

        ESP_ERROR_CHECK( esp_wifi_disconnect() );
        ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
        esp_wifi_connect();
    } else if (event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE) {
        xEventGroupSetBits(s_wifi_event_group, ESPTOUCH_DONE_BIT);
    }
}

static void smartconfig_example_task(void * parm)
{
    EventBits_t uxBits;
    ESP_ERROR_CHECK( esp_smartconfig_set_type(SC_TYPE_ESPTOUCH) );
    smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_smartconfig_start(&cfg) );
    while (1) {
        uxBits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY);
        if(uxBits & CONNECTED_BIT) {
            ESP_LOGI(TAG, "WiFi Connected to ap");
        }
        if(uxBits & ESPTOUCH_DONE_BIT) {
            ESP_LOGI(TAG, "smartconfig over");
            esp_smartconfig_stop();
            vTaskDelete(NULL);
        }
    }
}

esp_err_t bsp_spiffs_mount(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = CONFIG_BSP_SPIFFS_MOUNT_POINT,
        .partition_label = CONFIG_BSP_SPIFFS_PARTITION_LABEL,
        .max_files = CONFIG_BSP_SPIFFS_MAX_FILES,
#ifdef CONFIG_BSP_SPIFFS_FORMAT_ON_MOUNT_FAIL
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif
    };

    esp_err_t ret_val = esp_vfs_spiffs_register(&conf);

    ESP_ERROR_CHECK(ret_val);

    size_t total = 0, used = 0;
    ret_val = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret_val != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret_val));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    return ret_val;
}

esp_err_t bsp_spiffs_unmount(void)
{
    return esp_vfs_spiffs_unregister(CONFIG_BSP_SPIFFS_PARTITION_LABEL);
}

static void app_wifi_init()
{
    ESP_ERROR_CHECK(esp_netif_init());
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    #ifdef CONFIG_ENABLE_SEU_LOGIN
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "SEU-WLAN"
        },
        .threshold.authmode = WIFI_AUTH_OPEN,
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    esp_wifi_connect();
    #endif

    ESP_ERROR_CHECK( esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL) );
    ESP_ERROR_CHECK( esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL) );

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void app_esp_now_init()
{
	espnow_storage_init();

	app_wifi_init();

	espnow_config_t espnow_config = ESPNOW_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(espnow_init(&espnow_config));

	ESP_ERROR_CHECK(esp_now_register_recv_cb(app_espnow_recv_cb));
}

/*******************************************************************************
* Private functions
*******************************************************************************/

static esp_err_t audio_mute_function(AUDIO_PLAYER_MUTE_SETTING setting)
{
    // Volume saved when muting and restored when unmuting. Restoring volume is necessary
    // as es8311_set_voice_mute(true) results in voice volume (REG32) being set to zero.
    static int last_volume;
    bsp_codec_config_t *codec_handle = bsp_board_get_codec_handle();

    codec_handle->mute_set_fn(setting == AUDIO_PLAYER_MUTE ? true : false);

    // restore the voice volume upon unmuting
    if (setting == AUDIO_PLAYER_UNMUTE) {
        codec_handle->volume_set_fn(80, NULL);
    }

    ESP_LOGI(TAG, "mute setting %d, volume:%d", setting, last_volume);

    return ESP_OK;
}

void audio_player_init(void)
{
    bsp_codec_config_t *codec_handle = bsp_board_get_codec_handle();
    audio_player_config_t config = { .mute_fn = audio_mute_function,
                                     .write_fn = codec_handle->i2s_write_fn,
                                     .clk_set_fn = codec_handle->i2s_reconfig_clk_fn,
                                     .priority = 5
                                   };
    ESP_ERROR_CHECK(audio_player_new(config));
}

void app_main(void)
{
    /* Initialize I2C (for touch and audio) */
    bsp_i2c_init();

    bsp_board_init();

    /* Initialize SPIFFS */
    bsp_spiffs_mount();

    /* Initialize audio player */
    audio_player_init();

    /* Initialize wifi functions */
    app_esp_now_init();

    bsp_led_set_rgb(0, 100, 0, 0);

    /* Initialize SR */
    app_sr_init();

    audio_mute_function(AUDIO_PLAYER_UNMUTE);

    /* Initialize display and LVGL */
    bsp_display_start();

    /* Turn on display backlight */
    bsp_display_backlight_on();

    bsp_display_lock(0);

    ui_init();

    bsp_display_unlock();

    memory_monitor();

    ESP_LOGI(TAG, "UI initialization done.");
}
// *INDENT-ON*

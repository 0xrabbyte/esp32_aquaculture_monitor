#include <stdbool.h>

#include "sdkconfig.h"
#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "model_path.h"
#include "esp_log.h"
#include "../bsp_board.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_process_sdkconfig.h"
#include "esp_afe_config.h"
#include "../ui/ui.h"
#include "../ui/ui_events.h"

#define TAG "app_sr"

#define I2S_CHANNEL_NUM     (2)

extern bool is_connected;
extern int device_cnt;
extern struct aqua_data{
	float TDS_value;
	float pH_value;
	float NTH_value;
	float water_level_value;
	float water_temperature;
	bool is_pump;
}aqua_datas[CONFIG_MAX_DEVICE];
extern uint8_t** mac_addrs;

extern struct {
	bool is_pump;
	bool is_servo;
	bool open_camera;
}control_data;


static srmodel_list_t *models = NULL;
const esp_afe_sr_iface_t *afe_handle = &ESP_AFE_SR_HANDLE;

QueueHandle_t result_que;

static void feed_Task(void *arg)
{
    size_t bytes_read = 0;
    esp_afe_sr_data_t *afe_data = (esp_afe_sr_data_t *) arg;
    int audio_chunksize = afe_handle->get_feed_chunksize(afe_data);
    int feed_channel = 3;
    ESP_LOGI(TAG, "audio_chunksize=%d, feed_channel=%d", audio_chunksize, feed_channel);

    /* Allocate audio buffer and check for result */
    int16_t *audio_buffer = heap_caps_malloc(audio_chunksize * sizeof(int16_t) * feed_channel, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (NULL == audio_buffer) {
        esp_system_abort("No mem for audio buffer");
    }

    while (true) {
        /* Read audio data from I2S bus */
        bsp_codec_config_t *codec_handle = bsp_board_get_codec_handle();
        codec_handle->i2s_read_fn((char *)audio_buffer, audio_chunksize * I2S_CHANNEL_NUM * sizeof(int16_t), &bytes_read, portMAX_DELAY);

        /* Save audio data to file if record enabled */
        // if (g_sr_data->b_record_en && (NULL != g_sr_data->fp)) {
        //     fwrite(audio_buffer, 1, audio_chunksize * I2S_CHANNEL_NUM * sizeof(int16_t), g_sr_data->fp);
        // }

        /* Channel Adjust */
        for (int  i = audio_chunksize - 1; i >= 0; i--) {
            audio_buffer[i * 3 + 2] = 0;
            audio_buffer[i * 3 + 1] = audio_buffer[i * 2 + 1];
            audio_buffer[i * 3 + 0] = audio_buffer[i * 2 + 0];
        }

        /* Feed samples of an audio stream to the AFE_SR */
        afe_handle->feed(afe_data, audio_buffer);
    }
    
    afe_handle->destroy(afe_data);
    vTaskDelete(NULL);
}

static void detect_Task(void *arg)
{
    bool detect_flag = false;
	esp_afe_sr_data_t *afe_data = arg;
    int afe_chunksize = afe_handle->get_fetch_chunksize(afe_data);
    int nch = afe_handle->get_channel_num(afe_data);
    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_CHINESE);
    ESP_LOGI(TAG, "mn_name: %s", mn_name);
    esp_mn_iface_t *multinet = esp_mn_handle_from_name(mn_name);
    model_iface_data_t *model_data = multinet->create(mn_name, 5760);
    esp_mn_commands_update_from_sdkconfig(multinet, model_data);
    int mu_chunksize = multinet->get_samp_chunksize(model_data);
    int chunk_num = multinet->get_samp_chunknum(model_data);
    assert(mu_chunksize == afe_chunksize);
    ESP_LOGI(TAG, "ch: %d, afe_chunksize: %d, mu_chunksize: %d, chunk_num: %d", nch, afe_chunksize, mu_chunksize, chunk_num);
    while (1) {
        afe_fetch_result_t* res = afe_handle->fetch(afe_data);
        if (!res || res->ret_value == ESP_FAIL) {
            ESP_LOGE(TAG, "afe fetch failed");
            break;
        }

        if (res->wakeup_state == WAKENET_DETECTED) {                          
            ESP_LOGI(TAG, "WAKEWORD_DETECTED");
        } else if (res->wakeup_state == WAKENET_CHANNEL_VERIFIED) {
            detect_flag = true;
            ESP_LOGI(TAG, "AFE_FETCH_CHANNEL_VERIFIED, channel index: %d", res->trigger_channel_id);
            bsp_led_set_rgb(0, 0, 0, 100);
        }

        if (detect_flag) {
            esp_mn_state_t mn_state = multinet->detect(model_data, res->data);

            if (mn_state == ESP_MN_STATE_DETECTING) {
                continue;
            }

            if (mn_state == ESP_MN_STATE_TIMEOUT) {
                ESP_LOGW(TAG, "Time out");
                detect_flag = false;
                if (is_connected) bsp_led_set_rgb(0, 0, 100, 0);
                else bsp_led_set_rgb(0, 100, 0, 0);
                continue;
            }

            if (mn_state == ESP_MN_STATE_DETECTED) {
                esp_mn_results_t *mn_result = multinet->get_results(model_data);
                for (int i = 0; i < mn_result->num; i++) {
                    ESP_LOGI(TAG, "TOP %d, command_id: %d, phrase_id: %d, prob: %f", 
                    i+1, mn_result->command_id[i], mn_result->phrase_id[i], mn_result->prob[i]);
                }
                int sr_command_id = mn_result->command_id[0];
                ESP_LOGI(TAG, "Deteted command : %d", sr_command_id);
                xQueueSend(result_que, &sr_command_id, 0);
                ESP_LOGI(TAG, "listening...");
            }
        }
    }
    afe_handle->destroy(afe_data);
    vTaskDelete(NULL);
}

void handler_Task(void *arg) {
    while (true) {
        int result;
        xQueueReceive(result_que, &result, portMAX_DELAY);
        int ind = lv_dropdown_get_selected(ui_Slave_Dropdown);
        if (ind >= device_cnt) continue;
        switch (result)
        {
            case 0:
                ESP_LOGI(TAG, "Turn on the pump");
                if (!aqua_datas[ind].is_pump) {
                    lv_obj_add_state(ui_Pump_Switch, LV_STATE_CHECKED);
                    pump_status_switched(NULL);
                }
                break;
            case 1:
                ESP_LOGI(TAG, "Turn off the pump");
                if (aqua_datas[ind].is_pump) {
                    lv_obj_add_state(ui_Pump_Switch, LV_STATE_DEFAULT);
                    pump_status_switched(NULL);
                }
                break;
            case 2:
                ESP_LOGI(TAG, "Turn on the servo");
                feed_triggered(NULL);
                break;
        }
    }
    vTaskDelete(NULL);
}

void app_sr_init() {
    result_que = xQueueCreate(1, sizeof(int));

    models = esp_srmodel_init("model");
                   
    afe_config_t afe_config = AFE_CONFIG_DEFAULT();
    
    afe_config.aec_init = false;

    afe_config.wakenet_model_name = esp_srmodel_filter(models, ESP_WN_PREFIX, NULL);
    esp_afe_sr_data_t *afe_data = afe_handle->create_from_config(&afe_config);

    xTaskCreatePinnedToCore(&feed_Task, "feed", 4 * 1024, (void*)afe_data, 5, NULL, 0);
    xTaskCreatePinnedToCore(&detect_Task, "detect", 8 * 1024, (void*)afe_data, 5, NULL, 1);
    xTaskCreatePinnedToCore(&handler_Task, "handler", 4 * 1024, NULL, 5, NULL, 0);
}
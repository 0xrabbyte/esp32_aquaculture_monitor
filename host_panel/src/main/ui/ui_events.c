// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.3.0
// LVGL version: 8.3.3
// Project name: panel

#include "ui.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "https_mbedtls/https_mbedtls.h"
#include "esp_timer.h"
#include "sys/time.h"
#include "jpeg_decoder.h"
#include "sdkconfig.h"
#include "audio_player.h"
extern bool is_connected;

#define TAG "ui_events"

struct aqua_data{
	float TDS_value;
	float pH_value;
	float NTH_value;
	float water_level_value;
	float water_temperature;
	bool is_pump;
}aqua_datas[CONFIG_MAX_DEVICE];

struct {
	bool is_pump;
	bool is_servo;
	bool open_camera;
}control_data;

uint8_t mac_addrs[CONFIG_MAX_DEVICE][ESP_NOW_ETH_ALEN];

esp_timer_handle_t feed_periodic_timers[CONFIG_MAX_DEVICE];
int * ind_ptr[CONFIG_MAX_DEVICE];

int device_cnt = 0;

static int update_cnt = 0;
const int dropdown_peroid_list[] = {1, 5, 10};
const int dropdown_feed_period_list[] = {0, 1, 6, 12};
static bool is_first_package = true;
static int package_cnt = 0;
static uint32_t packages_cnt, total_packages;
static size_t image_size_left, image_size;
static uint8_t *image_data, *image_data_ptr;

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

static void update_labels(uint16_t ind)
{
	if (ind >= device_cnt)
		return;
	char str[10];
	sprintf(str, "%.2fppm", aqua_datas[ind].TDS_value);
	if (aqua_datas[ind].TDS_value < 0)
		lv_label_set_text(ui_TDS_Value, "N/A");
	else
		lv_label_set_text(ui_TDS_Value, str);
	sprintf(str, "%.2f", aqua_datas[ind].pH_value);
	if (aqua_datas[ind].pH_value < 0)
		lv_label_set_text(ui_pH_Value, "N/A");
	else
		lv_label_set_text(ui_pH_Value, str);
	sprintf(str, "%.2f", aqua_datas[ind].NTH_value);
	if (aqua_datas[ind].NTH_value < 0)
		lv_label_set_text(ui_NTH_Value, "N/A");
	else
		lv_label_set_text(ui_NTH_Value, str);
	sprintf(str, "%.2fcm", aqua_datas[ind].water_level_value / 160);
	if (aqua_datas[ind].water_level_value < 0)
		lv_label_set_text(ui_Water_Level_Value, "N/A");
	else
		lv_label_set_text(ui_Water_Level_Value, str);
	sprintf(str, "%.2f C", aqua_datas[ind].water_temperature);
	if (aqua_datas[ind].water_temperature < 0)
		lv_label_set_text(ui_Water_Temperature_Value, "N/A");
	else
		lv_label_set_text(ui_Water_Temperature_Value, str);
	lv_obj_add_state(ui_Pump_Switch, aqua_datas[ind].is_pump ? LV_STATE_CHECKED : LV_STATE_DEFAULT);
}

static size_t out_img_buf_size;
static uint8_t * out_img_buf;
static lv_img_dsc_t * jpeg_img_dsc;

void update_camera_image() {
    lv_obj_set_width(ui_Camera_Image, CONFIG_CAMERA_IMAGE_WIDTH);
    lv_obj_set_height(ui_Camera_Image, CONFIG_CAMERA_IMAGE_HEIGHT);

	out_img_buf_size = 3 * CONFIG_CAMERA_IMAGE_WIDTH * CONFIG_CAMERA_IMAGE_HEIGHT;
    if (out_img_buf != NULL) free(out_img_buf);
    out_img_buf = heap_caps_malloc(out_img_buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

	esp_jpeg_image_cfg_t jpeg_cfg = {
        .indata = image_data,
        .indata_size = image_size,
        .outbuf = out_img_buf,
        .outbuf_size = out_img_buf_size,
    };
    esp_jpeg_image_output_t outimg;

	esp_jpeg_decode(&jpeg_cfg, &outimg);
    ESP_LOGI("Image Load", "Image size: %d x %d", outimg.width, outimg.height);

	if (outimg.width != CONFIG_CAMERA_IMAGE_WIDTH || outimg.height != CONFIG_CAMERA_IMAGE_HEIGHT) {
		ESP_LOGE(TAG, "Received image size incorrect");
		return ;
	}

	if (jpeg_img_dsc != NULL) lv_img_buf_free(jpeg_img_dsc);
    jpeg_img_dsc = lv_img_buf_alloc(CONFIG_CAMERA_IMAGE_WIDTH, CONFIG_CAMERA_IMAGE_HEIGHT, LV_IMG_CF_TRUE_COLOR);
    for (int i = 0; i < CONFIG_CAMERA_IMAGE_HEIGHT; i++) {
        for (int j = 0; j < CONFIG_CAMERA_IMAGE_WIDTH; j++) {
            uint8_t red = out_img_buf[3 * (i * CONFIG_CAMERA_IMAGE_WIDTH + j)];
            uint8_t green = out_img_buf[3 * (i * CONFIG_CAMERA_IMAGE_WIDTH + j) + 1];
            uint8_t blue = out_img_buf[3 * (i * CONFIG_CAMERA_IMAGE_WIDTH + j) + 2];
            lv_img_buf_set_px_color(jpeg_img_dsc, j, i, lv_color_make(red, green, blue));
        }
    }
    lv_img_set_src(ui_Camera_Image, jpeg_img_dsc);
}

void app_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
	uint8_t * mac_addr = recv_info->src_addr;

	if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
        return;
    }

	int ind = -1;
	for (int i = 0; i < device_cnt; i++) {
		if (memcmp(mac_addr, mac_addrs[i], ESP_NOW_ETH_ALEN) == 0) {
			ind = i;
			break;
		}
	}

	if (ind == -1) {
		if (device_cnt >= CONFIG_MAX_DEVICE) {
			ESP_LOGE(TAG, "Device count is over");
			return;
		}
		memcpy(mac_addrs[device_cnt], mac_addr, ESP_NOW_ETH_ALEN);
		esp_now_peer_info_t *peer = malloc(sizeof(esp_now_peer_info_t));
        if (peer == NULL) {
            ESP_LOGE(TAG, "Malloc peer information fail");
			ESP_ERROR_CHECK(espnow_deinit());
            vTaskDelete(NULL);
        }
        memset(peer, 0, sizeof(esp_now_peer_info_t));
        memcpy(peer->peer_addr, mac_addr, ESP_NOW_ETH_ALEN);
        ESP_ERROR_CHECK(esp_now_add_peer(peer));
        free(peer);
		ind = device_cnt;
		device_cnt++;
		char str[20];
		sprintf(str, MACSTR, MAC2STR(mac_addr));
		lv_dropdown_add_option(ui_Slave_Dropdown, str, LV_DROPDOWN_POS_LAST );
	}

	if (control_data.open_camera) {
		if (is_first_package) {
			uint64_t first_package;
			if (len != sizeof(uint64_t)) {
				ESP_LOGE(TAG, "Receive image metadata package length error");
				control_data.open_camera= false;
				return;
			}
			if (image_data != NULL) free(image_data);
			memcpy(&first_package, data, sizeof(uint64_t));
			total_packages = first_package >> 32;
			packages_cnt = total_packages;
			image_size = first_package | ((1ll << 32) - 1);
			image_size_left = image_size;
			image_data = heap_caps_malloc(image_size_left, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
			image_data_ptr = image_data;
			is_first_package = false;
			return ;
		}
		package_cnt--;
		size_t package_size = CONFIG_ONE_SINGLE_PACKAGE_SIZE;
		if (!package_cnt) package_size = image_size_left;
		if (len != package_size + 1) {
			ESP_LOGE(TAG, "Receive image package length error");
			control_data.open_camera= false;
			return;
		}
		uint8_t package_index = data[package_size];
		ESP_LOGI(TAG, "Receive image package index: %d", package_index);
		if (package_index + package_cnt + 1 != total_packages) {
			ESP_LOGE(TAG, "Receive image package index error");
			control_data.open_camera= false;
			return;
		}
		memcpy(image_data_ptr, data, package_size);
		image_size_left -= package_size;
		image_data_ptr += package_size;
		if (!image_size_left) {update_camera_image(); control_data.open_camera= false;}
	}
	else {
		if (len != sizeof(struct aqua_data)) {
			ESP_LOGE(TAG, "Receive aqua data package length error");
			return;
		}

		memcpy(&aqua_datas[ind], data, sizeof(struct aqua_data));

		ESP_LOGI(TAG, "Receive cb from: "MACSTR", index: %d, TDS: %f, pH: %f, NTH: %f, water_level: %f, water_temperature: %f, is_pump: %d",
				MAC2STR(mac_addr), ind, aqua_datas[ind].TDS_value, aqua_datas[ind].pH_value, aqua_datas[ind].NTH_value, aqua_datas[ind].water_level_value, aqua_datas[ind].water_temperature, aqua_datas[ind].is_pump);

		if (lv_dropdown_get_selected(ui_Slave_Dropdown) == ind)
			update_labels(ind);
	}
}

void device_selecting_triggered(lv_event_t * e)
{
	uint16_t ind = lv_dropdown_get_selected(ui_Slave_Dropdown);
	update_labels(ind);
}

void pump_status_switched(lv_event_t * e)
{
	int ind = lv_dropdown_get_selected(ui_Slave_Dropdown);
	if (ind >= device_cnt)
		return;
	FILE* fp = fopen("/spiffs/bubble.mp3", "rb");
	audio_player_play(fp);
	aqua_datas[ind].is_pump = lv_obj_has_state(ui_Pump_Switch, LV_STATE_CHECKED);
	control_data.is_pump = aqua_datas[ind].is_pump; control_data.open_camera = false; control_data.is_servo = false;
	ESP_ERROR_CHECK(esp_now_send(mac_addrs[ind], (uint8_t*)&control_data, sizeof(control_data)));
}

void feed_triggered(lv_event_t * e)
{
	int ind = lv_dropdown_get_selected(ui_Slave_Dropdown);
	if (ind >= device_cnt)
		return;
	FILE* fp = fopen("/spiffs/bubble.mp3", "rb");
	audio_player_play(fp);
	control_data.is_pump = aqua_datas[ind].is_pump; control_data.open_camera = false; control_data.is_servo = true;
	ESP_ERROR_CHECK(esp_now_send(mac_addrs[ind], (uint8_t*)&control_data, sizeof(control_data)));
}

void periodic_timer_callback(void* arg)
{
	update_cnt++;
	if (update_cnt >= dropdown_peroid_list[lv_dropdown_get_selected(ui_UpdateDropdown)]) {
		update_cnt = 0;
		https_client_config_t config;
		strcpy(config.server, CONFIG_SERVER_NAME); strcpy(config.port, CONFIG_SERVER_PORT);
		for (int i = 0; i < device_cnt; i++) {
			time_t rawtime;
			struct tm info;
			char buffer[80];
			time(&rawtime);
			localtime_r(&rawtime, &info);
			strftime(buffer, 80, "%Y-%m-%d-%H:%M:%S", &info);
			sprintf(config.url, CONFIG_SERVER_URL, buffer, i, aqua_datas[i].TDS_value, aqua_datas[i].pH_value,\
					aqua_datas[i].NTH_value, aqua_datas[i].water_level_value, aqua_datas[i].water_temperature);
			xTaskCreate(&https_get_task, "https_get_task", 8192, &config, 5, NULL);
		}
	}
}

void get_camera_image(lv_event_t * e)
{
	int ind = lv_dropdown_get_selected(ui_Slave_Dropdown);
	if (ind >= device_cnt)
		return;
	control_data.is_pump = aqua_datas[ind].is_pump; control_data.open_camera = true; control_data.is_servo = false;
	is_first_package = true;
	ESP_ERROR_CHECK(esp_now_send(mac_addrs[ind], (uint8_t*)&control_data, sizeof(control_data)));
}

void feed_periodic_timers_callback(void *arg)
{
	int ind = *((int*)arg);
	control_data.is_pump = aqua_datas[ind].is_pump; control_data.open_camera = false; control_data.is_servo = true;
	ESP_ERROR_CHECK(esp_now_send(mac_addrs[ind], (uint8_t*)&control_data, sizeof(control_data)));
}

void update_feed_period(lv_event_t * e)
{
	int ind = lv_dropdown_get_selected(ui_Slave_Dropdown);
	if (ind >= device_cnt)
		return;
	free(ind_ptr[ind]);
	esp_timer_stop(feed_periodic_timers[ind]); esp_timer_delete(feed_periodic_timers[ind]);
	int period = dropdown_feed_period_list[lv_dropdown_get_selected(ui_Feed_Dropdown)];
	if (period) {
		ind_ptr[ind] = malloc(sizeof(int)); *ind_ptr[ind] = ind;
		const esp_timer_create_args_t periodic_timer_args = {
			.callback = &feed_periodic_timers_callback,
			.arg = (void*)ind_ptr,
			/* name is optional, but may help identify the timer when debugging */
			.name = "periodic_feed",
		};
		ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &feed_periodic_timers[ind]));
		ESP_ERROR_CHECK(esp_timer_start_periodic(feed_periodic_timers[ind], period * 60ll * 60ll * 1e6));
	}
}

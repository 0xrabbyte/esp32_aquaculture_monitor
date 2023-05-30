#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include "camera_pin.h"
#include "esp_camera.h"
#include "Arduino.h"
#include "FS.h"                // SD Card ESP32
#include "SD_MMC.h"            // SD Card ESP32
#include "soc/soc.h"           // Disable brownour problems
#include "soc/rtc_cntl_reg.h"  // Disable brownour problems
#include "driver/rtc_io.h"

#define FLASHLIGHT_PIN    4

#define ONE_SINGLE_PACKAGE_SIZE 240

const bool use_flashlight = false;
const int DATA_UPDATE_PERIOD_MS = 1000;
const uint8_t broadcast_mac[ESP_NOW_ETH_ALEN] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
bool sd_mounted = false;
int pictureNumber = 0;

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

void app_espnow_recv_cb(const uint8_t * mac_addr, const uint8_t *data, int len) {
    if (mac_addr == NULL || data == NULL || len <= 0 || len != sizeof(struct _control_data)) {
        Serial.println("Receive cb arg error");
        return ;
    }
    memcpy(&control_data, data, sizeof(struct _control_data));
    char * msg;
    asprintf(&msg, "Receive cb, from: " MACSTR ", is_pump: %d, open_camera: %d, is_servo: %d", \
            MAC2STR(mac_addr), control_data.is_pump, control_data.open_camera, control_data.is_servo);
    Serial.println(msg);
    free(msg);

    aqua_data.is_pump = control_data.is_pump;
    struct _motor_data {
        bool is_pump;
        bool is_servo;
    } motor_data;
    motor_data.is_pump = control_data.is_pump;
    motor_data.is_servo = control_data.is_servo;
    Wire.beginTransmission(8);
    Wire.write((uint8_t*)&motor_data, sizeof(_motor_data));
    Wire.endTransmission();
}

void camera_init() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size = FRAMESIZE_CIF;

    Serial.println("psramFound() = " + String(psramFound()));

    if (psramFound()) {
        config.jpeg_quality = 2;
        config.fb_count = 2;
    } else {
        config.jpeg_quality = 12;
        config.fb_count = 1;
    }

    // Init Camera
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        return;
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting SD Card");
    if (SD_MMC.begin("/sdcard", true)) {
        uint8_t cardType = SD_MMC.cardType();
        if (cardType != CARD_NONE) {
            sd_mounted = true;
        }
        else Serial.println("No SD Card attached");
    }
    else Serial.println("SD Card Mount Failed");
    
    Wire.begin(13, 12);
    WiFi.mode(WIFI_MODE_STA);
    Serial.println(WiFi.macAddress());
    camera_init();

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return ;
    }

    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, broadcast_mac, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
          
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Failed to add peer");
        return;
    }

    esp_now_register_recv_cb(app_espnow_recv_cb);

    pinMode(FLASHLIGHT_PIN, OUTPUT);
}

void loop() {
    if (!control_data.open_camera) {
        Wire.requestFrom(8, sizeof(_aqua_data));
        char buf[sizeof(_aqua_data)]; int i = 0;
        while (Wire.available()) {
            char c = Wire.read();
            buf[i++] = c;
        }
        memcpy(&aqua_data, buf, sizeof(_aqua_data));
        char * msg;
        asprintf(&msg, "TDS: %f, pH: %f, NTH: %f, water_level: %f, water_temperature: %f, is_pump: %d",\
                aqua_data.TDS_value, aqua_data.pH_value, aqua_data.NTH_value, aqua_data.water_level_value, aqua_data.water_temperature, aqua_data.is_pump);
        Serial.println(msg);
        free(msg);
        esp_now_send(broadcast_mac, (uint8_t*)&aqua_data, sizeof(_aqua_data));
    }
    else {
        Serial.println("Begin Camera Image Transferring");
        if (use_flashlight) digitalWrite(FLASHLIGHT_PIN, HIGH);
        camera_fb_t * fb = NULL;

        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("Camera capture failed");
            control_data.open_camera = false;
            digitalWrite(FLASHLIGHT_PIN, LOW);
            return;
        }
        if (sd_mounted) {
            String path = "/picture" + String(pictureNumber) + ".jpg";
            fs::FS &fs = SD_MMC;
            Serial.printf("Picture file name: %s\n", path.c_str());

            fs.remove(path.c_str());

            File file = fs.open(path.c_str(), FILE_WRITE);
            if (!file) {
              Serial.println("Failed to open file in writing mode");
            }
            else {
              file.write(fb->buf, fb->len); // payload (image), payload length
              Serial.printf("Saved file to path: %s\n", path.c_str());
            }
            file.close();
        }
        Serial.println(fb->len);
        uint32_t image_size = fb->len;
        uint32_t total_packages = ceil(image_size / ONE_SINGLE_PACKAGE_SIZE);
        uint64_t first_package = (total_packages << 32) | image_size;
        esp_now_send(broadcast_mac, (uint8_t*)&first_package, sizeof(first_package));
        uint8_t buf[32];
        for (int i = 0, p = 0; i < total_packages; i++, p += ONE_SINGLE_PACKAGE_SIZE) {
            size_t package_size = ONE_SINGLE_PACKAGE_SIZE;
            Serial.println(String(i) + "th package is being transferred");
            if (p + ONE_SINGLE_PACKAGE_SIZE > image_size)
                package_size = image_size - p;
            memcpy(buf, fb->buf + p, package_size);
            esp_now_send(broadcast_mac, buf, package_size);
        }
        Serial.println("Image Transferred!");
        pictureNumber++;
        esp_camera_fb_return(fb);
        control_data.open_camera = false;
        digitalWrite(FLASHLIGHT_PIN, LOW);
    }
    delay(DATA_UPDATE_PERIOD_MS);
}
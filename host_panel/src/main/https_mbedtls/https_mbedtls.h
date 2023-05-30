void https_get_task(void *pvParameters);
typedef struct {
    char server[64];
    char port[6];
    char url[256];
} https_client_config_t;
#include "WiFi.h"
#include "config.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "string.h"

static const char *TAG = "WiFi";

// ==================== 状态 ====================
static bool              g_connected = false;
static int               g_sock      = -1;
static struct sockaddr_in g_remote_addr = {};
static SemaphoreHandle_t g_rc_mutex  = NULL;
static RCCommand_t       g_last_rc   = {};

// ==================== WiFi 事件回调 ====================
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        g_connected = false;
        ESP_LOGW(TAG, "Disconnected, reconnecting...");
        esp_wifi_connect();
    } else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *evt = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&evt->ip_info.ip));
        g_connected = true;
    }
}

// ==================== UDP 接收任务 ====================
static void udp_rx_task(void *pvParameters)
{
    g_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (g_sock < 0) {
        ESP_LOGE(TAG, "socket create failed");
        vTaskDelete(NULL);
        return;
    }

    struct sockaddr_in local = {};
    local.sin_family      = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port        = htons(UDP_LOCAL_PORT);

    if (bind(g_sock, (struct sockaddr *)&local, sizeof(local)) < 0) {
        ESP_LOGE(TAG, "bind failed");
        close(g_sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "UDP listening on port %d", UDP_LOCAL_PORT);

    uint8_t buf[128];
    while (1) {
        struct sockaddr_in from;
        socklen_t from_len = sizeof(from);
        int len = recvfrom(g_sock, buf, sizeof(buf), 0,
                           (struct sockaddr *)&from, &from_len);
        if (len <= 0) continue;

        // 记住地面站地址（用于回传遥测）
        g_remote_addr = from;
        g_remote_addr.sin_port = htons(UDP_REMOTE_PORT);

        // 解析遥控指令（简单按结构体拷贝）
        if (len >= (int)sizeof(RCCommand_t)) {
            xSemaphoreTake(g_rc_mutex, portMAX_DELAY);
            memcpy(&g_last_rc, buf, sizeof(RCCommand_t));
            xSemaphoreGive(g_rc_mutex);
        }
    }
}

// ==================== 对外接口 ====================

void WiFi_Init(void)
{
    // 1. NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    // 2. TCP/IP 栈 + 默认事件循环
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    // 3. WiFi 驱动
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    // 4. 注册事件
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                        &wifi_event_handler, NULL, NULL);

    // 5. STA 配置 + 启动
    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.sta.ssid,     WIFI_SSID,     sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char *)wifi_config.sta.password, WIFI_PASSWORD, sizeof(wifi_config.sta.password) - 1);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();

    // 6. 创建 RC 互斥锁和 UDP 任务
    g_rc_mutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(udp_rx_task, "udp_rx", 4096, NULL, 5, NULL, 0); // ← 固定在 Core 0

    ESP_LOGI(TAG, "WiFi init done");
}

bool WiFi_Is_Connected(void)
{
    return g_connected;
}

bool WiFi_GetRCCommand(RCCommand_t *out)
{
    if (!out) return false;
    xSemaphoreTake(g_rc_mutex, portMAX_DELAY);
    *out = g_last_rc;
    xSemaphoreGive(g_rc_mutex);
    return true;
}

void WiFi_SendTelemetry(const Telemetry_t *tlm)
{
    if (!g_connected || g_sock < 0 || !tlm) return;
    if (g_remote_addr.sin_addr.s_addr == 0) return;   // 还没收到过地面站地址

    sendto(g_sock, tlm, sizeof(Telemetry_t), 0,
           (struct sockaddr *)&g_remote_addr, sizeof(g_remote_addr));
}
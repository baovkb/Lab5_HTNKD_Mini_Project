#include <stdio.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "ssd1306.h"
#include "sensor.h"
#include "Wifi.h"
#include "mqtt_client.h"
#include <cJSON.h>

#define I2C_MASTER_SCL_IO 22        /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 21        /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_1    /*!< I2C port number for master dev */

const char* TAG = "main";
const char* ssid = "Loading...!";
const char* password = "29061971";
const char* broker_url = "mqtt://mqtt.flespi.io:1883";
const char* token = "zlkEsnlaGffnr2sKaubQaQhT2kkw7gr4kOCBksUWNFzapx5LBro0YRAt9mEIfKiD";
esp_mqtt_client_handle_t client;

QueueHandle_t sensorQueue;

esp_err_t i2c_init();
void sendSensor();
static void mqtt_app_start();
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
void connectTask();

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    initRainSensor();

    sensorQueue = xQueueCreate(1, sizeof(SensorData));
    if (sensorQueue == NULL) {
        printf("init queue for sensor error");
        exit(-1);
    }

    if (i2c_init() == ESP_OK) {
        ssd1306_init();
        task_ssd1306_display_clear();
    } else {
        printf("faild to config oled");
    }

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    if (connectWifi(ssid, password)) {
        mqtt_app_start();
    }

    xTaskCreate(readSensorTask, "readSensorTask", 2048, NULL, 1, NULL);
    xTaskCreate(sendSensor, "readSensor", 2048, NULL, 1, NULL);
}

esp_err_t i2c_init() {
    i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = I2C_MASTER_SDA_IO,
		.scl_io_num = I2C_MASTER_SCL_IO,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = I2C_MASTER_FREQ_HZ
	};
	i2c_param_config(I2C_MASTER_NUM, &i2c_config);
	return i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
}

void sendSensor()
{
    SensorData ssRevData;
    char strPrint[50];
    while (1)
    {
        if (xQueueReceive(sensorQueue, &ssRevData, portMAX_DELAY) == pdPASS) {
            strPrint[0] = '\0';
            sprintf(strPrint, "Humidity: %.1f%%\nTemperature: %.0fC\nRain: %d", ssRevData.humidity, ssRevData.temperature, ssRevData.rain);
            task_ssd1306_display_clear();
            vTaskDelay(1);
	        task_ssd1306_display_text(strPrint);

            cJSON *json = cJSON_CreateObject();
            cJSON_AddNumberToObject(json, "temperature", ssRevData.temperature);
            cJSON_AddNumberToObject(json, "humidity", ssRevData.humidity);
            cJSON_AddNumberToObject(json, "rain", ssRevData.rain);
            char *js = cJSON_PrintUnformatted(json);
            cJSON_Delete(json);
            esp_mqtt_client_publish(client, "sensor", js, 0, 0, 1);
        }
    }

    vTaskDelete(NULL);
}

static void mqtt_app_start()
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = broker_url,
        .credentials.username = token,
    };
    client = esp_mqtt_client_init(&mqtt_cfg);
    
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%" PRIi32 "", base, event_id);
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
        break;
    case MQTT_EVENT_SUBSCRIBED:

        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
        break;
    case MQTT_EVENT_DATA:
        ESP_LOGI(TAG, "MQTT_EVENT_DATA");
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            ESP_LOGI(TAG, "Last errno string (%s)", strerror(event->error_handle->esp_transport_sock_errno));
        }
        break;
    default:
        ESP_LOGI(TAG, "Other event id:%d", event->event_id);
        break;
    }
}
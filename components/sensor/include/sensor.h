#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/queue.h>
#include "esp_log.h"
#include "dht.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define I2C_MASTER_FREQ_HZ 100000   /*!< I2C master clock frequency */
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate

static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;
static esp_adc_cal_characteristics_t *adc_chars;

typedef struct {
    float humidity;
    float temperature;
    uint16_t rain;
} SensorData;

static dht_sensor_type_t dht11 = DHT_TYPE_DHT11;
static gpio_num_t dht11_pin = GPIO_NUM_4;

void readSensorTask();
void initRainSensor();
uint16_t readRainSensor();
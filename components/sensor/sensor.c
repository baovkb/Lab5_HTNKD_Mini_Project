#include "sensor.h"
extern QueueHandle_t sensorQueue;

void readSensorTask()
{
    SensorData ssData;
    while (1)
    {
        if (dht_read_float_data(dht11, dht11_pin, &ssData.humidity, &ssData.temperature) == ESP_OK) {
            ssData.rain = readRainSensor();
            xQueueSend(sensorQueue, &ssData, 0);    
        }
        else
            printf("Could not read data from sensor\n");
            
        vTaskDelay(pdMS_TO_TICKS(10000));
    }

    vTaskDelete(NULL);
}

void initRainSensor() {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(channel, atten);
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);

}

uint16_t readRainSensor() {
    uint32_t adc_reading = 0;
    //Multisampling
    for (int i = 0; i < 64; i++) {
        adc_reading += adc1_get_raw((adc1_channel_t)channel);
    }
    adc_reading /= 64;
    //Convert adc_reading to voltage in mV
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    printf("Raw: %lu\tVoltage: %lumV\n", adc_reading, voltage);
    
    uint16_t heigh = (voltage - 142) * 40 / 4095;
    return heigh;
}
/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "freertos/queue.h"
#include "esp_spi_flash.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "soc/mcpwm_periph.h"
#include <math.h>
#include "driver/timer.h"
#include "AutoBottlegerGPIO.h"
#include "PowBoardPeriph.h"
#include "PageSystem.h"



#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_SENS1) | (1ULL<<GPIO_INPUT_SENS2) | (1ULL<<GPIO_INPUT_SENS3) | (1ULL<<GPIO_INPUT_SENSG) | (1ULL<<GPIO_INPUT_SENSW))

#define ESP_INTR_FLAG_DEFAULT 0


#ifdef CONFIG_IDF_TARGET_ESP32
#define CHIP_NAME "ESP32"
#endif

#ifdef CONFIG_IDF_TARGET_ESP32S2BETA
#define CHIP_NAME "ESP32-S2 Beta"
#endif



xQueueHandle timer_queue;

typedef struct _audio_info
{
	uint32_t sampleRate;
	uint32_t dataLength;
	const uint8_t *data;
} audio_info_t;

const audio_info_t sound_wav_info =
{
	11025, // sampleRate
	111667, // dataLength
	0 // data
};

typedef struct {
    int type;  // the type of timer's event
    int timer_group;
    int timer_idx;
    uint64_t timer_counter_value;
} timer_event_t;

static void IRAM_ATTR timer0_ISR(void* arg)
{
   timer_spinlock_take(TIMER_GROUP_0);
    uint64_t timer_counter_value = timer_group_get_counter_value_in_isr(TIMER_GROUP_0, static_cast<timer_idx_t>(0));
	// Очистить флаги прерываний
	timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
	// Перезапустить прерывание Alarm
	timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);
    
    timer_event_t evt;
    evt.timer_group = 0;
    evt.timer_idx = 1;
    evt.timer_counter_value = timer_counter_value;
    xQueueSendFromISR(timer_queue, &evt, NULL);
    timer_spinlock_give(TIMER_GROUP_0);
}

static void timerInit()
{
	timer_config_t config;
    
	config.divider = 8; // Предделитель
	config.counter_dir = TIMER_COUNT_UP; // Считать вверх
	config.counter_en = TIMER_PAUSE; // Состояние - пауза
	config.alarm_en = TIMER_ALARM_EN; // Включить прерывание Alarm
	config.intr_type = TIMER_INTR_LEVEL; // Прерывание по уровню
	config.auto_reload = static_cast<timer_autoreload_t>(1); // Автоматически перезапускать счет
	
	ESP_ERROR_CHECK(timer_init(TIMER_GROUP_0, TIMER_0, &config));
	// Установить начальное значение счетчика
	ESP_ERROR_CHECK(timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL));
	// Установить значение счетчика для срабатывания прерывания Alarm
	ESP_ERROR_CHECK(timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_BASE_CLK / config.divider / 1));	
	timer_isr_register(TIMER_GROUP_0, TIMER_0, timer0_ISR, (void *)&sound_wav_info, ESP_INTR_FLAG_IRAM, NULL);
	ESP_ERROR_CHECK(timer_enable_intr(TIMER_GROUP_0, TIMER_0));    

    ESP_ERROR_CHECK(timer_init(TIMER_GROUP_0, TIMER_1, &config));
    ESP_ERROR_CHECK(timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0x00000000ULL));
    ESP_ERROR_CHECK(timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_BASE_CLK / config.divider / 200));
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer0_ISR, (void *)&sound_wav_info, ESP_INTR_FLAG_IRAM, NULL);
	ESP_ERROR_CHECK(timer_enable_intr(TIMER_GROUP_0, TIMER_0));

	timer_start(TIMER_GROUP_0, TIMER_0);
    timer_start(TIMER_GROUP_0, TIMER_1);
}






static void mcpwm_example_config(void *arg)
{
    Periph ABPeriph;
    timerInit();
    vTaskDelete(NULL);
}
/*
static void test_valves_control(void *arg)
{
    float duty_cycle = 50;
    float test_time = 0;
    while(1){
        vTaskDelay(10 / portTICK_PERIOD_MS);
        test_time += 0.01;
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, 20 + 20*sin(2*3.14*2*test_time));
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, 40 + 20*sin(2*3.14*2*test_time));
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, 60 + 20*sin(2*3.14*2*test_time));                
        mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B, 80 + 20*sin(2*3.14*2*test_time));
    }    
}

static void test_relay2(void *arg)
{    
    while(1){
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_OUTPUT_RL2, 1);
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_OUTPUT_RL2, 0);
    }    
}*/

static void main_task(void *arg){

    Periph ABPeriph;
    PageSystem PageSys(&ABPeriph);
    timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    timerInit();

    while (1) {
        timer_event_t evt;
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);
        
        printf("------- EVENT TIME --------\n");
        printf("Counter: 0x%08x%08x  themp: %.3f\n", (uint32_t) (evt.timer_counter_value >> 32),
           (uint32_t) (evt.timer_counter_value), temper1);

     }
}


extern "C" void app_main(void)
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU cores, WiFi%s%s, ",
            CHIP_NAME,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    //xTaskCreate(mcpwm_example_config, "mcpwm_example_config", 4096, NULL, 5, NULL);
       
}

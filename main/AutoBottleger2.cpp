/* 
   This example code is ...
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
#include "Transmitter.h"
#include "esp_log.h"
#include "esp_event.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "nvs_flash.h"



#define ESP_INTR_FLAG_DEFAULT 0

#ifdef CONFIG_IDF_TARGET_ESP32
#define CHIP_NAME "ESP32"
#endif

#ifdef CONFIG_IDF_TARGET_ESP32S2BETA
#define CHIP_NAME "ESP32-S2 Beta"
#endif

TaskHandle_t mainTaskHandle;

xQueueHandle timer_queue;
xQueueHandle uart_msg_queue;
xQueueHandle udp_msg_queue;
xQueueHandle udp_msg_queue_out;
xQueueHandle udp_connection_queue;



extern void uart_event_task(void *pvParameters);
extern void udp_client_task(void *pvParameters);

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
    timer_event_t evt;
    evt.timer_group = 0;
    evt.timer_idx = 0;
    evt.timer_counter_value = timer_counter_value;
	timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_0);
    timer_counter_value += (uint64_t) (TIMER_BASE_CLK / 8);
	timer_group_set_alarm_value_in_isr(TIMER_GROUP_0, TIMER_0, timer_counter_value);
    timer_group_enable_alarm_in_isr(TIMER_GROUP_0, TIMER_0);
    
    xQueueSendFromISR(timer_queue, &evt, NULL);
    xTaskResumeFromISR( mainTaskHandle);
    timer_spinlock_give(TIMER_GROUP_0);
}

static void IRAM_ATTR timer1_ISR(void* arg)
{
   timer_spinlock_take(TIMER_GROUP_0);    
	// Очистить флаги прерываний
	timer_group_clr_intr_status_in_isr(TIMER_GROUP_0, TIMER_1);
    PageSystem* Sys = (PageSystem*)arg;    
    Sys->perform_fast_loop(5);    
    timer_spinlock_give(TIMER_GROUP_0);
}

static void timerInit(PageSystem *PageSys)
{
	timer_config_t config;    
	config.divider = 8; // Предделитель
	config.counter_dir = TIMER_COUNT_UP; // Считать вверх
	config.counter_en = TIMER_PAUSE; // Состояние - пауза
	config.alarm_en = TIMER_ALARM_EN; // Включить прерывание Alarm
	config.intr_type = TIMER_INTR_LEVEL; // Прерывание по уровню
	config.auto_reload = static_cast<timer_autoreload_t>(0); // Автоматически перезапускать счет
	
	ESP_ERROR_CHECK(timer_init(TIMER_GROUP_0, TIMER_0, &config));
	ESP_ERROR_CHECK(timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL));
	// Установить значение счетчика для срабатывания прерывания Alarm
	ESP_ERROR_CHECK(timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_BASE_CLK / config.divider / 1));	
	timer_isr_register(TIMER_GROUP_0, TIMER_0, timer0_ISR, (void *)NULL, ESP_INTR_FLAG_IRAM, NULL);
	ESP_ERROR_CHECK(timer_enable_intr(TIMER_GROUP_0, TIMER_0));
     printf("t1 %f, ",0.0);

    config.divider = 8; // Предделитель
	config.counter_dir = TIMER_COUNT_UP; // Считать вверх
	config.counter_en = TIMER_PAUSE; // Состояние - пауза
	config.alarm_en = TIMER_ALARM_EN; // Включить прерывание Alarm
	config.intr_type = TIMER_INTR_LEVEL; // Прерывание по уровню
	config.auto_reload = static_cast<timer_autoreload_t>(1); // Автоматически перезапускать счет
    
    ESP_ERROR_CHECK(timer_init(TIMER_GROUP_0, TIMER_1, &config));
    ESP_ERROR_CHECK(timer_set_counter_value(TIMER_GROUP_0, TIMER_1, 0x00000000ULL));
    ESP_ERROR_CHECK(timer_set_alarm_value(TIMER_GROUP_0, TIMER_1, TIMER_BASE_CLK / config.divider / 200));
    timer_isr_register(TIMER_GROUP_0, TIMER_1, timer1_ISR, (void *)PageSys, ESP_INTR_FLAG_IRAM, NULL);
    ESP_ERROR_CHECK(timer_enable_intr(TIMER_GROUP_0, TIMER_1));
    timer_start(TIMER_GROUP_0, TIMER_1);
	timer_start(TIMER_GROUP_0, TIMER_0);
    
}


static void main_task(void *arg){

    AutoBottlegerUDPClient* ABUdp = (AutoBottlegerUDPClient*)arg;

    Periph ABPeriph;    
    printf("Peripherial initialized\n");
    
    PageSystem PageSys(&ABPeriph);
    printf("Page system initialized\n");
    
    timerInit(&PageSys);
    printf("Timers initialized\n");

    AutoBottlegerUart ABUart;
    xTaskCreate(uart_event_task, "uart_event_task", 4096, &ABUart, 12, NULL);

    PageState msg;
    PageState new_state;
    timer_event_t evt;
    
    while (1) {
        
        if(xQueueReceive(timer_queue, &evt, portMAX_DELAY)){
            PageSys.perform_slow_loop(evt.timer_counter_value);
            msg = PageSys.get_state();
            ABUart.send_msg_uart(&msg);
            ABUdp->send_msg_wifi(&msg);          
        }

        if(xQueueReceive(udp_msg_queue, &new_state, 0)){
            PageSys.update_page(new_state); 
        }
        
        if(xQueueReceive(uart_msg_queue, &new_state, 0)){
            PageSys.update_page(new_state); 
        }
               
     }
}

extern "C" void app_main(void)
{
    
    printf("Start!\n");

    AutoBottlegerUDPClient ABUdp;
    timer_queue = xQueueCreate(10, sizeof(timer_event_t));    
    uart_msg_queue = xQueueCreate(10, sizeof(PageState));    
    udp_msg_queue = xQueueCreate(10, sizeof(PageState));
    udp_msg_queue_out = xQueueCreate(10, sizeof(PageState));
    udp_connection_queue = xQueueCreate(10, sizeof(SystemState));

    xTaskCreate(main_task, "main_task", 8192, &ABUdp, 12, &mainTaskHandle);
    vTaskDelay(7000 / portTICK_PERIOD_MS);    
    xTaskCreate(udp_client_task, "udp_event_task", 8096, &ABUdp, 6, NULL);

}

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
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_event.h"
#include "freertos/event_groups.h"
#include "protocol_examples_common.h"
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

#define EX_UART_NUM UART_NUM_0
#define PATTERN_CHR_NUM    (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)

# define bCONFIG_EXAMPLE_IPV4
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR

#define EXAMPLE_ESP_WIFI_SSID      "AsusRT-N11_Wi-fi"
#define EXAMPLE_ESP_WIFI_PASS      "123456780"
#define EXAMPLE_ESP_MAXIMUM_RETRY  5

#define PORT CONFIG_EXAMPLE_PORT

static const char *TAG = "example";
static const char *payload = "Message from ESP32 ";

static const char *TAGuart = "uart_events";

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAGwifi = "wifi station";

static int s_retry_num = 0;

xQueueHandle timer_queue;
static QueueHandle_t uart0_queue;
xQueueHandle msg_queue;

typedef struct {
    int type;  // the type of timer's event
    int timer_group;
    int timer_idx;
    uint64_t timer_counter_value;
} timer_event_t;

static void send_msg_uart(PageState *msg){
    
    printf("AAAA %3.2f %3.2f %3.2f %3.2f %3.2f %3.2f %3.2f %3d %1d %1d %1d %ju %2d %1d\n",
        msg->temperature_1,
        msg->temperature_2,
        msg->temperature_3,
        msg->temperature_4,
        msg->temper_control_val_1,
        msg->temper_control_val_2,
        msg->temper_control_val_3,
        msg->triac_percent_open,
        msg->valve_1_state,
        msg->valve_2_state,
        msg->valve_3_state,
        msg->page_time,
        msg->next_page_num,
        msg->error_code);
}


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


extern "C" void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:{
                    //ESP_LOGI(TAGuart, "[UART DATA]: %d", event.size);
                    //uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
                    //ESP_LOGI(TAGuart, "[DATA EVT]:");
                    //uart_write_bytes(EX_UART_NUM, (const char*) dtmp, event.size);

                    PageState p;
                    memcpy(&p.temperature_1, dtmp, sizeof(float));
                    memcpy(&p.temperature_2, dtmp + 4, sizeof(float));
                    memcpy(&p.temperature_3, dtmp + 8, sizeof(float));
                    memcpy(&p.temperature_4, dtmp + 12, sizeof(float));
                    memcpy(&p.temper_control_val_1, dtmp + 16, sizeof(float));
                    memcpy(&p.temper_control_val_2, dtmp + 20, sizeof(float));
                    memcpy(&p.temper_control_val_3, dtmp + 24, sizeof(float));
                    memcpy(&p.triac_percent_open, dtmp + 28, sizeof(int));
                    memcpy(&p.valve_1_state, dtmp + 32, sizeof(bool));
                    memcpy(&p.valve_2_state, dtmp + 33, sizeof(bool));
                    memcpy(&p.valve_3_state, dtmp + 34, sizeof(bool));
                    memcpy(&p.page_time, dtmp + 35, sizeof(uint64_t));
                    memcpy(&p.next_page_num, dtmp + 43, sizeof(int));
                    memcpy(&p.error_code, dtmp + 51, sizeof(int));

                    uart_flush_input(EX_UART_NUM);}
                    break;
                //Event of HW FIFO overflow detected
                 default:{
                    ESP_LOGI(TAGuart, "uart event type: %d", event.type);}
                    break;
                //Others
                
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));


    wifi_config_t wifi_config; 
        wifi_config.ssid = EXAMPLE_ESP_WIFI_SSID;
        wifi_config.password = EXAMPLE_ESP_WIFI_PASS;
	    wifi_config.threshold.authmode = WIFI_AUTH_WPA2_PSK;
        wifi_config.pmf_cfg.capable = true;
        ifi_config.pmf_cfg.required = false;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAGwifi, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAGwifi, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAGwifi, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAGwifi, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

static void udp_client_task(void *pvParameters)
{
    char rx_buffer[128];
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;

    while (1) {

        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

        while (1) {

            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
            ESP_LOGI(TAG, "Message sent");

            struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
                ESP_LOGI(TAG, "%s", rx_buffer);
                if (strncmp(rx_buffer, "OK: ", 4) == 0) {
                    ESP_LOGI(TAG, "Received expected message, reconnecting");
                    break;
                }
            }

            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}



static void main_task(void *arg){

    Periph ABPeriph;    
    printf("Single device optimisations disenabled\n");
    
    PageSystem PageSys(&ABPeriph);
    timer_queue = xQueueCreate(10, sizeof(timer_event_t));
    timerInit(&PageSys);
     printf("Timer disenabled\n");
    PageState msg;

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);

    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //Set uart pattern detect function.
    uart_enable_pattern_det_baud_intr(EX_UART_NUM, '+', PATTERN_CHR_NUM, 8, 0, 0);
    //Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(EX_UART_NUM, 20);

    msg_queue = xQueueCreate(5, sizeof(PageState));

    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 4096, NULL, 12, NULL);/**/

    while (1) {
        timer_event_t evt;
        if(xQueueReceive(timer_queue, &evt, portMAX_DELAY)){
            PageSys.perform_slow_loop(evt.timer_counter_value);
            msg = PageSys.get_state();
            send_msg_uart(&msg);            
        }
        if(xQueueReceive(msg_queue, &newState, portMAX_DELAY)){
            PageSys.update_page(newState); 
        }            
        
     }
}


extern "C" void app_main(void)
{
    printf("Start!\n");

    xTaskCreate(main_task, "main_task", 8192, NULL, 5, NULL);

    
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAGwifi, "ESP_WIFI_MODE_STA");
    wifi_init_sta();

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(example_connect());
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);


}

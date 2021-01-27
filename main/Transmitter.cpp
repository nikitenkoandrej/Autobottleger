#include "Transmitter.h"

#define bCONFIG_EXAMPLE_IPV4

#define EXAMPLE_ESP_WIFI_SSID      "AsusRT-N11_Wi-fi"
#define EXAMPLE_ESP_WIFI_PASS      "123456780"
static char HOST_IP_ADDR[32] = "192.168.1.10";
static int PORT = 3333;
#define EXAMPLE_ESP_MAXIMUM_RETRY  5

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;

static const char *TAG = "event";

uint8_t wifi_ssid[32] = {'A','s','u','s','R','T','-','N','1','1','_','W','i','-','f','i'};
uint8_t wifi_pass[64] = {'1','2','3','4','5','6','7','8','0'};

extern xQueueHandle uart_msg_queue;
extern xQueueHandle udp_msg_queue;
extern xQueueHandle udp_connection_queue;
extern xQueueHandle udp_msg_queue_out;

int sock;
bool UDPconncted = 0;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

static QueueHandle_t uart0_queue;

wifi_config_t wifi_config; 
        

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG,"retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG,"got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(bool &ok)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));


    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG,"wifi_init_sta finished.");

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
        ESP_LOGI(TAG,"connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG,"Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGI(TAG,"UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
    vEventGroupDelete(s_wifi_event_group);
    ok = 1;
}



extern "C" void uart_event_task(void *pvParameters)
{
    AutoBottlegerUart* ABUart = (AutoBottlegerUart*)pvParameters;
    uart_event_t event;
    size_t buffered_size;
    uint8_t* dtmp = (uint8_t*) malloc(ABUart->RD_BUF_SIZE);
    PageState PState_temp;
    SystemState SState_temp;
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, ABUart->RD_BUF_SIZE);

            switch(event.type) {                
                case UART_DATA:{

                    if(event.size < 3){
                        uart_flush_input(UART_NUM_0);
                        break;
                    }

                    if(event.size == 47){
                        uart_read_bytes(UART_NUM_0, dtmp, event.size, portMAX_DELAY);
                        uart_flush_input(UART_NUM_0);
                        PState_temp = ABUart->parse_msg_from_byte_array(dtmp);                       
                        xQueueSend(uart_msg_queue, &PState_temp, NULL);
                    } 

                    if(event.size == 69){
                        uart_read_bytes(UART_NUM_0, dtmp, event.size, portMAX_DELAY);
                        uart_flush_input(UART_NUM_0);
                        SState_temp = ABUart->parse_state_from_byte_array(dtmp);                        
                        xQueueSend(udp_connection_queue, &SState_temp, NULL);
                    }

                    uart_flush_input(UART_NUM_0);
                }
                break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                {
                    ESP_LOGI(TAG,"hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM_0);
                    xQueueReset(uart0_queue);
                }    
                break;                
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                {
                    ESP_LOGI(TAG,"ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_NUM_0);
                    xQueueReset(uart0_queue);
                }    
                break;                
                //Event of UART RX break detected
                case UART_BREAK:
                {
                    ESP_LOGI(TAG,"uart rx break");
                }    
                break;                
                //Event of UART parity check error
                case UART_PARITY_ERR:
                {
                    ESP_LOGI(TAG,"uart parity error");
                }    
                break;                
                //Event of UART frame error
                case UART_FRAME_ERR:
                {
                    ESP_LOGI(TAG,"uart frame error");
                }    
                break;
                
                default:
                {
                    uart_flush_input(UART_NUM_0);
                }
                break;
                //Others
                
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

extern "C" void udp_client_task(void *pvParameters)
{
    PageState PState_temp;
    AutoBottlegerUDPClient* ABUdp = (AutoBottlegerUDPClient*)pvParameters;
    char rx_buffer[128];
    char addr_str[128];
    int addr_family = 0;
    int ip_protocol = 0;
    char host_ip[32] = {'1','9','2','.','1','6','8','.','1','.','1','0'}; 
    bool udp_init = 0;    
    bool wifi_ok = 0;

    SystemState st;
 std::string msg_buf;
    for(int i = 0; i < 32; ++i){
        wifi_config.sta.ssid[i] = wifi_ssid[i];
    }
        
    for(int i = 0; i < 64; ++i){
        wifi_config.sta.password[i] = wifi_pass[i];
    }
        

	wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    while(!wifi_ok){
/*
         while (!xQueueReceive(udp_connection_queue, (void * )&st, (portTickType)portMAX_DELAY)){
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }*/
        
        for(int i = 0; i < 32; ++i)
            wifi_config.sta.ssid[i] = wifi_ssid[i];
        for(int i = 0; i < 64; ++i)
            wifi_config.sta.password[i] = wifi_pass[i];

        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            if(nvs_flash_erase() != ESP_OK){
                continue;
            }
            ret = nvs_flash_init();
        }
        if(ret != ESP_OK){
          continue;
        }
        ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
        wifi_init_sta(wifi_ok);
        ESP_LOGI(TAG,"Bizarre EVENT %d",wifi_ok);
    }

    while(!udp_init)
    {
        if(nvs_flash_init() != ESP_OK)
        {
            ESP_LOGI(TAG,"Bizarre EVENT1 %d",1);
            continue;
        }
        if(esp_netif_init() != ESP_OK){
            ESP_LOGI(TAG,"Bizarre EVENT2 %d",2);
            continue;
        }
        if(esp_event_loop_create_default() == ESP_ERR_NO_MEM){

            ESP_LOGI(TAG,"Bizarre EVENT3 %d",3);
            
            continue;
        }
        if(esp_event_loop_create_default() == ESP_FAIL){

            ESP_LOGI(TAG,"Bizarre EVENT4 %d",4);
            
            continue;
        }


        UDPconncted = 1;
        udp_init = 1; 
    }
    
    struct sockaddr_in dest_addr;
PageState msg;
    while (1) {        
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);

        sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", 0);
            break;
        }
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

         

        while (1) { 
             if(xQueueReceive(udp_msg_queue_out, &msg, 0)){


                 msg_buf.clear();
    msg_buf += std::to_string(msg.temperature_1);
    msg_buf.push_back(' ');
    msg_buf += std::to_string(msg.temperature_2);
    msg_buf.push_back(' ');
    msg_buf += std::to_string(msg.temperature_3);
    msg_buf.push_back(' ');
    msg_buf += std::to_string(msg.temperature_4);
    msg_buf.push_back(' ');
    msg_buf += std::to_string(msg.temper_control_val_1);
    msg_buf.push_back(' ');
    msg_buf += std::to_string(msg.temper_control_val_2);
    msg_buf.push_back(' ');
    msg_buf += std::to_string(msg.temper_control_val_3);
    msg_buf.push_back(' ');
    msg_buf += std::to_string(msg.triac_percent_open);
    msg_buf.push_back(' ');
    msg_buf += std::to_string(msg.valve_1_state);
    msg_buf.push_back(' ');
    msg_buf += std::to_string(msg.valve_2_state);
    msg_buf.push_back(' ');
    msg_buf += std::to_string(msg.valve_3_state);
    msg_buf.push_back(' ');
    msg_buf += std::to_string(msg.page_time);
    msg_buf.push_back(' ');
    msg_buf += std::to_string(msg.next_page_num);
    msg_buf.push_back(' ');
    msg_buf += std::to_string(msg.error_code);
            int err = sendto(sock, msg_buf.data(), 68, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", 0);
                } 
             }/*
            struct sockaddr_storage source_addr; 
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            if (len > 0) {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG,"Received %d bytes from %s:", len, host_ip);
                ESP_LOGI(TAG,"%s", rx_buffer);
                //PState_temp = ABUdp->parse_msg_from_byte_array(rx_buffer);
                xQueueSend(udp_msg_queue, &PState_temp, NULL);
            }*/

            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGI(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}


AutoBottlegerUart::AutoBottlegerUart(){
    
    uart_config_t uart_config; 
        uart_config.baud_rate = 115200;
        uart_config.data_bits = UART_DATA_8_BITS;
        uart_config.parity = UART_PARITY_DISABLE;
        uart_config.stop_bits = UART_STOP_BITS_1;
        uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        uart_config.source_clk = UART_SCLK_APB;

    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    uart_param_config(UART_NUM_0, &uart_config);

    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //Set uart pattern detect function.
    //uart_enable_pattern_det_baud_intr(UART_NUM_0, '+', PATTERN_CHR_NUM, 8, 0, 0);
    //Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(UART_NUM_0, 20);

   
}

void AutoBottlegerUart::send_msg_uart(PageState *msg){
    
    printf("%3.2f %3.2f %3.2f %3.2f %3.2f %3.2f %3.2f %3d %1d %1d %1d %5d %2d %1d\n",
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

PageState AutoBottlegerUart::parse_msg_from_byte_array(uint8_t *arr)
{
    PageState p;
    memcpy(&p.temperature_1, arr, sizeof(float));
    memcpy(&p.temperature_2, arr + 4, sizeof(float));
    memcpy(&p.temperature_3, arr + 8, sizeof(float));
    memcpy(&p.temperature_4, arr + 12, sizeof(float));
    memcpy(&p.temper_control_val_1, arr + 16, sizeof(float));
    memcpy(&p.temper_control_val_2, arr + 20, sizeof(float));
    memcpy(&p.temper_control_val_3, arr + 24, sizeof(float));
    memcpy(&p.triac_percent_open, arr + 28, sizeof(int));
    memcpy(&p.valve_1_state, arr + 32, sizeof(bool));
    memcpy(&p.valve_2_state, arr + 33, sizeof(bool));
    memcpy(&p.valve_3_state, arr + 34, sizeof(bool));
    memcpy(&p.page_time, arr + 35, sizeof(int));
    memcpy(&p.next_page_num, arr + 39, sizeof(int));
    memcpy(&p.error_code, arr + 43, sizeof(int));

    return p;
}

SystemState AutoBottlegerUart::parse_state_from_byte_array(uint8_t *arr)
{
    SystemState s;
    memcpy(&s.ssid, arr, 32);
    memcpy(&s.pass, arr + 32, 32);
    memcpy(&s.main_mode, arr + 64, 1);
    memcpy(&s.time_s, arr + 65, 8);

    return s;
}

AutoBottlegerUDPClient::AutoBottlegerUDPClient(){
    rdy_to_send = 0;
}


void AutoBottlegerUDPClient::send_msg_wifi(PageState * msg){
if(!UDPconncted){
    return;
}
else{
    xQueueSend(udp_msg_queue_out, msg, NULL);
}
    /*
    msg_buf.clear();
    msg_buf += std::to_string(msg->temperature_1);
    msg_buf.push_back(' ');
    msg_buf += std::to_string(msg->temperature_2);
    msg_buf.push_back(' ');
    msg_buf += std::to_string(msg->temperature_3);
    msg_buf.push_back(' ');
    msg_buf += std::to_string(msg->temperature_4);
    msg_buf.push_back(' ');
    msg_buf += std::to_string(msg->temper_control_val_1);
    msg_buf.push_back(' ');
    msg_buf += std::to_string(msg->temper_control_val_2);
    msg_buf.push_back(' ');
    msg_buf += std::to_string(msg->temper_control_val_3);
    msg_buf.push_back(' ');
    msg_buf += std::to_string(msg->triac_percent_open);
    msg_buf.push_back(' ');
    msg_buf += std::to_string(msg->valve_1_state);
    msg_buf.push_back(' ');
    msg_buf += std::to_string(msg->valve_2_state);
    msg_buf.push_back(' ');
    msg_buf += std::to_string(msg->valve_3_state);
    msg_buf.push_back(' ');
    msg_buf += std::to_string(msg->page_time);
    msg_buf.push_back(' ');
    msg_buf += std::to_string(msg->next_page_num);
    msg_buf.push_back(' ');
    msg_buf += std::to_string(msg->error_code);

    int err = sendto(sock, msg_buf.data(), msg_buf.length(), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", 0);
            }     */     
    
}

PageState AutoBottlegerUDPClient::parse_msg_from_byte_array(char *arr)
{
    PageState p;
    memcpy(&p.temperature_1, arr, sizeof(float));
    memcpy(&p.temperature_2, arr + 4, sizeof(float));
    memcpy(&p.temperature_3, arr + 8, sizeof(float));
    memcpy(&p.temperature_4, arr + 12, sizeof(float));
    memcpy(&p.temper_control_val_1, arr + 16, sizeof(float));
    memcpy(&p.temper_control_val_2, arr + 20, sizeof(float));
    memcpy(&p.temper_control_val_3, arr + 24, sizeof(float));
    memcpy(&p.triac_percent_open, arr + 28, sizeof(int));
    memcpy(&p.valve_1_state, arr + 32, sizeof(bool));
    memcpy(&p.valve_2_state, arr + 33, sizeof(bool));
    memcpy(&p.valve_3_state, arr + 34, sizeof(bool));
    memcpy(&p.page_time, arr + 35, sizeof(int));
    memcpy(&p.next_page_num, arr + 39, sizeof(int));
    memcpy(&p.error_code, arr + 43, sizeof(int));

    return p;
}


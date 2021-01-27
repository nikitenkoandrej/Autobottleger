#ifndef TRANSMITTER_H
#define TRANSMITTER_H

#include <string>
#include "driver/uart.h"
#include "PageSystem.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_event.h"
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



extern "C" void uart_event_task(void *pvParameters);
extern "C" void udp_client_task(void *pvParameters);

class AutoBottlegerUart{
    //private:
    
    public:
    const int BUF_SIZE = 1024;
    const int RD_BUF_SIZE = 1024;    
    AutoBottlegerUart();
    PageState parse_msg_from_byte_array(uint8_t *arr);
    SystemState parse_state_from_byte_array(uint8_t *arr);
    void send_msg_uart(PageState * msg);
};

class AutoBottlegerUDPClient{
    private:
    std::string msg_buf; 

    public:
    int rdy_to_send;

    struct sockaddr_in dest_addr;

    AutoBottlegerUDPClient();
    PageState parse_msg_from_byte_array(char *arr);
    void send_msg_wifi(PageState * msg);
};


#endif  

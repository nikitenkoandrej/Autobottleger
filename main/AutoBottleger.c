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
#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"
#include "freertos/queue.h"
#include "esp_spi_flash.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"
#include "soc/mcpwm_periph.h"
#include <math.h>

#define GPIO_PWM0A_OUT 33   //Set GPIO 33 as PWM0A
#define GPIO_PWM0B_OUT 27   //Set GPIO 27 as PWM0B
#define GPIO_PWM1A_OUT 25   //Set GPIO 25 as PWM1A
#define GPIO_PWM1B_OUT 26   //Set GPIO 26 as PWM1B

#define GPIO_OUTPUT_TRIAC 4
#define GPIO_OUTPUT_RL1 16
#define GPIO_OUTPUT_RL2 32
#define GPIO_OUTPUT_OPTORL 2
#define GPIO_OUTPUT_TEST 2
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_TRIAC) | (1ULL<<GPIO_OUTPUT_RL1) | (1ULL<<GPIO_OUTPUT_RL2) | (1ULL<<GPIO_OUTPUT_OPTORL) | (1ULL<<GPIO_OUTPUT_TEST))

#define GPIO_INPUT_SENS1 23
#define GPIO_INPUT_SENS2 22
#define GPIO_INPUT_SENS3 21
#define GPIO_INPUT_SENSW 34
#define GPIO_INPUT_SENSG 35

#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_SENS1) | (1ULL<<GPIO_INPUT_SENS2) | (1ULL<<GPIO_INPUT_SENS3) | (1ULL<<GPIO_INPUT_SENSG) | (1ULL<<GPIO_INPUT_SENSW))

#define ESP_INTR_FLAG_DEFAULT 0

static xQueueHandle gpio_evt_queue = NULL;

#ifdef CONFIG_IDF_TARGET_ESP32
#define CHIP_NAME "ESP32"
#endif

#ifdef CONFIG_IDF_TARGET_ESP32S2BETA
#define CHIP_NAME "ESP32-S2 Beta"
#endif



#define GPIO_DS18B20_0       (CONFIG_ONE_WIRE_GPIO)
#define MAX_DEVICES          (8)
#define DS18B20_RESOLUTION   (DS18B20_RESOLUTION_11_BIT)
#define SAMPLE_PERIOD        (1000)   // milliseconds

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}

static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm gpio...\n");

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_PWM1A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, GPIO_PWM1B_OUT);
}

static void mcpwm_example_config(void *arg)
{
    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initialize mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    //frequency = 1000Hz
    pwm_config.cmpr_a = 50.0;       //duty cycle of PWMxA = 50.0%
    pwm_config.cmpr_b = 50.0;       //duty cycle of PWMxb = 50.0%
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);   //Configure PWM0A & PWM0B with above settings
    pwm_config.frequency = 1000;     //frequency = 500Hz
    pwm_config.cmpr_a = 50.0;       //duty cycle of PWMxA = 50.0%
    pwm_config.cmpr_b = 50.0;    //duty cycle of PWMxb = 50.0%
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);   //Configure PWM1A & PWM1B with above settings
    /*pwm_config.frequency = 400;     //frequency = 400Hz
    pwm_config.cmpr_a = 23.2;       //duty cycle of PWMxA = 23.2%
    pwm_config.cmpr_b = 97.0;       //duty cycle of PWMxb = 97.0%
    pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER; //frequency is half when up down count mode is set i.e. SYMMETRIC PWM
    pwm_config.duty_mode = MCPWM_DUTY_MODE_1;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_2, &pwm_config);   //Configure PWM2A & PWM2B with above settings
    */

    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
/*
      //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

   
    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //start gpio task
    xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_SENS1, gpio_isr_handler, (void*) GPIO_INPUT_SENS1);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_SENS2, gpio_isr_handler, (void*) GPIO_INPUT_SENS2);
    gpio_isr_handler_add(GPIO_INPUT_SENS3, gpio_isr_handler, (void*) GPIO_INPUT_SENS3);
    gpio_isr_handler_add(GPIO_INPUT_SENSG, gpio_isr_handler, (void*) GPIO_INPUT_SENSG);
    gpio_isr_handler_add(GPIO_INPUT_SENSW, gpio_isr_handler, (void*) GPIO_INPUT_SENSW);
*/
    vTaskDelete(NULL);
}

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

static void test_triac_control(void *arg)
{
    float duty_cycle = 50;
    float test_time = 0;
    float test_period_time = 0;
    float prec_x = 0;
    float x_quant = 0;
    float eps = 0;
    float power = 50;
    while(1){
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
        power = 50 + 40*sin(2*3.14*0.2*test_time);
        while(test_period_time < 1){
            vTaskDelay(10 / portTICK_PERIOD_MS);
            test_period_time += 0.01;
            prec_x = power*test_period_time;
            eps = prec_x - x_quant;
            if(eps > 0.5){
                x_quant += 1;
                gpio_set_level(GPIO_OUTPUT_TRIAC, 1);
            }else{
                gpio_set_level(GPIO_OUTPUT_TRIAC, 0);
            }
        }
        test_period_time = 0.0;
        x_quant = 0.0;
        test_time += 1;
    }
        

}

static void test_optorelay(void *arg)
{
    while(1){
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_OUTPUT_OPTORL, 1);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_OUTPUT_OPTORL, 0);
    }    
}

static void test_relay1(void *arg)
{
    
    while(1){
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_OUTPUT_TEST, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(GPIO_OUTPUT_TEST, 0);
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
}

void app_main(void)
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

    OneWireBus * owb, * owb1;

    owb_rmt_driver_info rmt_driver_info;
owb_rmt_driver_info rmt_driver_info1;

    owb = owb_rmt_initialize(&rmt_driver_info, 17, RMT_CHANNEL_1, RMT_CHANNEL_0);
    owb1 = owb_rmt_initialize(&rmt_driver_info1, 5, RMT_CHANNEL_2, RMT_CHANNEL_3);

    owb_use_crc(owb, true);  // enable CRC check for ROM code 7001191c12a38128
    owb_use_crc(owb1, true);  // enable CRC check for ROM code 7001191c12a38128
    int num_devices = 1;

    OneWireBus_ROMCode known_device = {
            .fields.family = { 0x28 },
            .fields.serial_number = { 0x81, 0xa3, 0x12, 0x1c, 0x19, 0x01 },
            .fields.crc = { 0x70 },
        };
OneWireBus_ROMCode known_device1 = {
            .fields.family = { 0x28 },
            .fields.serial_number = { 0x81, 0xa3, 0x12, 0x1c, 0x19, 0x01 },
            .fields.crc = { 0x70 },
        };


        char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
        char rom_code_s1[OWB_ROM_CODE_STRING_LENGTH];

        owb_string_from_rom_code(known_device, rom_code_s, sizeof(rom_code_s));
        owb_string_from_rom_code(known_device1, rom_code_s1, sizeof(rom_code_s1));
        bool is_present = false;

        owb_status search_status = owb_verify_rom(owb, known_device, &is_present);

        if (search_status == OWB_STATUS_OK)
        {
            printf("Device %s is %s\n", rom_code_s, is_present ? "present" : "not present");
        }
        else
        {
            printf("An error occurred searching for known device: %d", search_status);
        }
        DS18B20_Info * devices[MAX_DEVICES] = {0};

        is_present = false;

        owb_status search_status1 = owb_verify_rom(owb1, known_device1, &is_present);

        if (search_status1 == OWB_STATUS_OK)
        {
            printf("Device %s is %s\n", rom_code_s1, is_present ? "present" : "not present");
        }
        else
        {
            printf("An error occurred searching for known device: %d", search_status1);
        }
        DS18B20_Info * devices1[MAX_DEVICES] = {0};/**/

    for (int i = 0; i < num_devices; ++i)
    {
        DS18B20_Info * ds18b20_info = ds18b20_malloc();  // heap allocation
        devices[i] = ds18b20_info;

        if (num_devices == 1)
        {
            printf("Single device optimisations enabled\n");
            ds18b20_init_solo(ds18b20_info, owb);          // only one device on bus
        }
        
        ds18b20_use_crc(ds18b20_info, true);           // enable CRC check on all reads
        ds18b20_set_resolution(ds18b20_info, DS18B20_RESOLUTION);
    }

for (int i = 0; i < num_devices; ++i)
    {
        DS18B20_Info * ds18b20_info1 = ds18b20_malloc();  // heap allocation
        devices1[i] = ds18b20_info1;

        if (num_devices == 1)
        {
            printf("Single device optimisations enabled\n");
            ds18b20_init_solo(ds18b20_info1, owb1);          // only one device on bus
        }
        
        ds18b20_use_crc(ds18b20_info1, true);           // enable CRC check on all reads
        ds18b20_set_resolution(ds18b20_info1, DS18B20_RESOLUTION);
    }/**/

    int errors_count[MAX_DEVICES] = {0};
    int errors_count1[MAX_DEVICES] = {0};
    int sample_count = 0;
    if (num_devices > 0)
    {
        TickType_t last_wake_time = xTaskGetTickCount();

        while (1)
        {
            ds18b20_convert_all(owb);

            // In this application all devices use the same resolution,
            // so use the first device to determine the delay
            ds18b20_wait_for_conversion(devices[0]);

            // Read the results immediately after conversion otherwise it may fail
            // (using printf before reading may take too long)
            float readings[MAX_DEVICES] = { 0 };
            DS18B20_ERROR errors[MAX_DEVICES] = { 0 };

            for (int i = 0; i < num_devices; ++i)
            {
                errors[i] = ds18b20_read_temp(devices[i], &readings[i]);
            }

            // Print results in a separate loop, after all have been read
            printf("\nTemperature readings (degrees C): sample %d\n", ++sample_count);
            for (int i = 0; i < num_devices; ++i)
            {
                if (errors[i] != DS18B20_OK)
                {
                    ++errors_count[i];
                }

                printf("  %d: %.1f    %d errors\n", i, readings[i], errors_count[i]);
            }

            //vTaskDelayUntil(&last_wake_time, SAMPLE_PERIOD / portTICK_PERIOD_MS);

            ds18b20_convert_all(owb1);

            // In this application all devices use the same resolution,
            // so use the first device to determine the delay
            ds18b20_wait_for_conversion(devices1[0]);

            // Read the results immediately after conversion otherwise it may fail
            // (using printf before reading may take too long)
            float readings1[MAX_DEVICES] = { 0 };
            DS18B20_ERROR errors1[MAX_DEVICES] = { 0 };

            for (int i = 0; i < num_devices; ++i)
            {
                errors1[i] = ds18b20_read_temp(devices1[i], &readings1[i]);
            }

            // Print results in a separate loop, after all have been read
            printf("\nTemperature readings (degrees C): sample %d\n", ++sample_count);
            for (int i = 0; i < num_devices; ++i)
            {
                if (errors1[i] != DS18B20_OK)
                {
                    ++errors_count1[i];
                }

                printf("  %d: %.1f    %d errors\n", i, readings1[i], errors_count1[i]);
            }/**/

            vTaskDelayUntil(&last_wake_time, SAMPLE_PERIOD / portTICK_PERIOD_MS);

        }
    }
    else
    {
        printf("\nNo DS18B20 devices detected!\n");
    }/**/

/*
    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();*/
    //xTaskCreate(mcpwm_example_config, "mcpwm_example_config", 4096, NULL, 5, NULL);
    //xTaskCreate(test_valves_control, "test_valves_control", 4096, NULL, 2, NULL);
    //gpio_set_level(GPIO_OUTPUT_TRIAC, 1);
    //xTaskCreate(test_triac_control, "test_triac_control", 4096, NULL, 2, NULL);
    //xTaskCreate(test_relay1, "test_relay2", 4096, NULL, 2, NULL);
    //xTaskCreate(test_optorelay, "test_optorelay", 4096, NULL, 5, NULL);
}

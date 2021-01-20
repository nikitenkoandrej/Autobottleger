/*
 *
 *
 */

/**
 * 
 *
 *
 * 
 */

#ifndef AB_GPIO_H
#define AB_GPIO_H

#define GPIO_OUTPUT_TRIAC static_cast<gpio_num_t>(4)
#define GPIO_VALVE1_OUT static_cast<gpio_num_t>(33)   
#define GPIO_VALVE2_OUT static_cast<gpio_num_t>(27)   
#define GPIO_VALVE3_OUT static_cast<gpio_num_t>(25)   
#define GPIO_VALVE4_OUT static_cast<gpio_num_t>(26)   

#define GPIO_OUTPUT_RELAY12V 16
#define GPIO_OUTPUT_RELAY220V 32
#define GPIO_OUTPUT_FAN 2

#define GPIO_DS18B20_0       (CONFIG_ONE_WIRE_GPIO)
#define MAX_DEVICES          (8)
#define SAMPLE_PERIOD        (1000)   // milliseconds

#define GPIO_INPUT_THERMO1 17
#define GPIO_INPUT_THERMO2 5
#define GPIO_INPUT_THERMO3 18
#define GPIO_INPUT_THERMO4 19

#define GPIO_INPUT_SENS1 23
#define GPIO_INPUT_SENS2 22
#define GPIO_INPUT_SENS3 21
#define GPIO_INPUT_SENSW 34
#define GPIO_INPUT_SENSG 35



#endif  // PB_GPIO_H

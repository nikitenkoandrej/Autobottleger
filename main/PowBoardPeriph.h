/*
 * MIT License
 *
 * 
 */

/**
 * @file
 * @brief 
 */

#ifndef PB_PERIPH_H
#define PB_PERIPH_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "AutoBottlegerGPIO.h"
#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"


class TriacUnit{

private:
    const int ESP32_IO_pin; 
    float state_percents_open;    
    float brezenghem_x_quant;

public:
    TriacUnit(int TRIAC_GPIO_OUTPUT_NUM);
    const int getOutPin() const {return ESP32_IO_pin;}
    bool setState(float percent_open);
    void brezenghem(int time_ms);
    
};

struct Valve_pwm_settings{
    mcpwm_io_signals_t PWM_ch;
    mcpwm_timer_t PWM_TIMERi;
    mcpwm_generator_t PWM_gen;
};

class Valve{

private:
    const int ESP32_IO_pin;
    float percent_duty_cycle;
    Valve_pwm_settings PWM_SETTINGS;
        
public:
    const int getOutPin() const {return ESP32_IO_pin;}
    bool setState(float percent_duty_cycle);
    Valve(int VALVE_GPIO_OUTPUT_NUM, Valve_pwm_settings VPWM_PARAM);
};

class FanUnit{

private:
    const int ESP32_IO_pin; 
    bool fan_is_on;

public:
    const int getOutPin() const {return ESP32_IO_pin;}
    bool setState(bool is_on);
    FanUnit(int FAN_GPIO_OUTPUT_NUM);
};

class RelayUnit{

private:
    const int ESP32_IO_pin; 
    bool relay_is_open;

public:
    const int getOutPin() const {return ESP32_IO_pin;}
    bool setState(bool is_open);
    RelayUnit(int RELAY_OUTPUT_NUM);
};

class ThermoSensor{
private:
    const int ESP32_IO_pin;
    int num_devices = 0;
    int errors_count = 0;
    DS18B20_Info * devices[MAX_DEVICES];
    OneWireBus * owb;
    owb_rmt_driver_info rmt_driver_info;

public:
    ThermoSensor(int THERM_SENS_INPUT, rmt_channel_id_t RMT_CH_RX, rmt_channel_id_t RMT_CH_TX);
    float get_temperature();
};

class AlarmSensor{
    private:
    const int ESP32_IO_pin;
    bool is_triggered;
    QueueHandle_t gpio_evt_queue;

public:
    ThermoSensor(int THERM_SENS_INPUT, rmt_channel_id_t RMT_CH_RX, rmt_channel_id_t RMT_CH_TX);
    float check_alarm();
}


class Periph{
private:
    void check_alarm();
public:
    TriacUnit* Triac;
    Valve *Valve1,
          *Valve2,
          *Valve3,
          *Valve4;
    RelayUnit *Relay12V, *Relay220V;
    FanUnit *Fan;
    ThermoSensor *ThermoSensor1,
                 *ThermoSensor2,
                 *ThermoSensor3,
                 *ThermoSensor4;
    AlarmSensor *GasSensor,
                *WaterSensor;             
    Periph();

};

#endif  // PB_PERIPH_H

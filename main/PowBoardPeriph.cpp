/* 
*
*/
#include "PowBoardPeriph.h"


//****************************peripherial ISR***********************************


static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}


//******************************************************************************

TriacUnit::TriacUnit(int TRIAC_GPIO_OUTPUT_NUM):ESP32_IO_pin(TRIAC_GPIO_OUTPUT_NUM){
    brezenghem_x_quant = 0;
    this->state_percents_open = 0;

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<ESP32_IO_pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
}

void TriacUnit::brezenghem(int current_time){    
        
    float precise_x = this->state_percents_open*static_cast<float>(current_time)/1000.0;
    
    float eps = 0;           
            eps = precise_x - brezenghem_x_quant;
            if(eps > 0.5){
                brezenghem_x_quant += 1.0;
                gpio_set_level(GPIO_OUTPUT_TRIAC, 1);
            }else{
                gpio_set_level(GPIO_OUTPUT_TRIAC, 0);
            } 
    if(current_time > 995.0){        
        brezenghem_x_quant = 0;
    }
}

 bool TriacUnit::setState(float percent_open){
     this->state_percents_open = percent_open;
     return true;
 }


Valve::Valve(int VALVE_GPIO_OUTPUT_NUM, Valve_pwm_settings VPWM_SETTINGS):ESP32_IO_pin(VALVE_GPIO_OUTPUT_NUM)
{
    this->percent_duty_cycle = 0;
    this->PWM_SETTINGS.PWM_ch = VPWM_SETTINGS.PWM_ch;
    this->PWM_SETTINGS.PWM_TIMERi = VPWM_SETTINGS.PWM_TIMERi;
    this->PWM_SETTINGS.PWM_gen = VPWM_SETTINGS.PWM_gen;

    mcpwm_gpio_init(MCPWM_UNIT_0, this->PWM_SETTINGS.PWM_ch, this->ESP32_IO_pin);

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<ESP32_IO_pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 1000;    
    pwm_config.cmpr_a = 0;       
    pwm_config.cmpr_b = 0;       
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, this->PWM_SETTINGS.PWM_TIMERi, &pwm_config);   
}

 bool Valve::setState(float percent){
     this->percent_duty_cycle = percent;
     mcpwm_set_duty(MCPWM_UNIT_0, this->PWM_SETTINGS.PWM_TIMERi, this->PWM_SETTINGS.PWM_gen, percent);
     return true;
 }

 FanUnit::FanUnit(int FAN_GPIO_OUTPUT_NUM):ESP32_IO_pin(FAN_GPIO_OUTPUT_NUM){
     this->fan_is_on = 0;

     gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<ESP32_IO_pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
 }

  bool FanUnit::setState(bool is_on){
     this->fan_is_on = is_on;
     gpio_set_level(static_cast<gpio_num_t>(ESP32_IO_pin), static_cast<gpio_num_t>(is_on));
     return true;
 }

RelayUnit::RelayUnit(int RELAY_OUTPUT_NUM):ESP32_IO_pin(RELAY_OUTPUT_NUM){
     this->relay_is_open = 0;

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<ESP32_IO_pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
 }

  bool RelayUnit::setState(bool is_open){
     this->relay_is_open = is_open;
     gpio_set_level(static_cast<gpio_num_t>(ESP32_IO_pin), is_open);
     return true;
 }

 ThermoSensor::ThermoSensor(int THERM_SENS_INPUT, rmt_channel_id_t RMT_CH_RX, rmt_channel_id_t RMT_CH_TX):ESP32_IO_pin(THERM_SENS_INPUT){
     owb = owb_rmt_initialize(&rmt_driver_info, static_cast<gpio_num_t>(ESP32_IO_pin), RMT_CH_RX, RMT_CH_TX);
     owb_use_crc(owb, true);  // enable CRC check for ROM code
     OneWireBus_ROMCode known_device = {/*
            { 0x28 },
            { 0x81, 0xa3, 0x12, 0x1c, 0x19, 0x01 },
            { 0x70 },*/
        };
     char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
     owb_string_from_rom_code(known_device, rom_code_s, sizeof(rom_code_s));
     bool is_present = false;

     owb_status search_status = owb_verify_rom(owb, known_device, &is_present);
     if (search_status == OWB_STATUS_OK)
        {
            num_devices = 1;
            printf("Device %s is %s\n", rom_code_s, is_present ? "present" : "not present");
        }
    else
        {
            num_devices = 0;
            printf("An error occurred searching for known device: %d", search_status);
        }
    
    
    DS18B20_Info * ds18b20_info = ds18b20_malloc();  // heap allocation
    devices[0] = ds18b20_info;
    printf("Single device optimisations enabled\n");
    ds18b20_init_solo(ds18b20_info, owb);          // only one device on bus
    ds18b20_use_crc(ds18b20_info, true);           // enable CRC check on all reads
    ds18b20_set_resolution(ds18b20_info, DS18B20_RESOLUTION_11_BIT);    
 }


float ThermoSensor::get_temperature(){
    if (num_devices > 0)
    {
      ds18b20_convert_all(owb);
      ds18b20_wait_for_conversion(devices[0]);
      float readings = 0;
      DS18B20_ERROR errors = DS18B20_OK;
      errors = ds18b20_read_temp(devices[0], &readings);
      
      if (errors != DS18B20_OK)
      {
            ++errors_count;
      }
      return readings;
    }
    return -1000.0;
}



void AlarmSensor::AlarmSensor(int SENSOR_INPUT_NUM):ESP32_IO_pin(SENSOR_INPUT_NUM){
    is_triggered = FALSE
    io_conf.intr_type = GPIO_INTR_HIGH_LEVEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL<<ESP32_IO_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(ESP32_IO_PIN, gpio_isr_handler, (void*) ESP32_IO_PIN);
    gpio_isr_handler_add(, gpio_isr_handler, (void*) GPIO_INPUT_SENSW);
}

bool AlarmSensor::check_alarm(){
    if(xQueueReceive(gpio_evt_queue, &io_num, 0)){
            is_triggered = true;
    }
    return is_triggered;
}


Periph::Periph(){

    Triac = new TriacUnit(GPIO_OUTPUT_TRIAC);

    Valve_pwm_settings V1_PWM_SETTINGS{MCPWM0A, MCPWM_TIMER_0, MCPWM_GEN_A};
    Valve1 = new Valve(GPIO_VALVE1_OUT, V1_PWM_SETTINGS);

    Valve_pwm_settings V2_PWM_SETTINGS{MCPWM0B, MCPWM_TIMER_0, MCPWM_GEN_B};
    Valve2 = new Valve(GPIO_VALVE2_OUT, V2_PWM_SETTINGS);

    Valve_pwm_settings V3_PWM_SETTINGS{MCPWM0A, MCPWM_TIMER_1, MCPWM_GEN_A};
    Valve3 = new Valve(GPIO_VALVE3_OUT, V3_PWM_SETTINGS);

    Valve_pwm_settings V4_PWM_SETTINGS{MCPWM0B, MCPWM_TIMER_1, MCPWM_GEN_B};
    Valve4 = new Valve(GPIO_VALVE4_OUT, V4_PWM_SETTINGS);

    Relay12V = new RelayUnit(GPIO_OUTPUT_RELAY12V);

    Relay220V = new RelayUnit(GPIO_OUTPUT_RELAY220V);

    Fan = new FanUnit(GPIO_OUTPUT_FAN);

    ThermoSensor1 = new ThermoSensor(GPIO_INPUT_THERMO1, RMT_CHANNEL_1, RMT_CHANNEL_0);
    ThermoSensor2 = new ThermoSensor(GPIO_INPUT_THERMO2, RMT_CHANNEL_3, RMT_CHANNEL_2);
    ThermoSensor3 = new ThermoSensor(GPIO_INPUT_THERMO3, RMT_CHANNEL_5, RMT_CHANNEL_4);
    ThermoSensor4 = new ThermoSensor(GPIO_INPUT_THERMO4, RMT_CHANNEL_7, RMT_CHANNEL_6);

    GasSensor = new AlarmSensor(GPIO_INPUT_SENSG);
    
    WaterSensor = new AlarmSensor(GPIO_INPUT_SENSW);
}

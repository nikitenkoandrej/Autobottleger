#ifndef P_DATA_H
#define P_DATA_H

#include <string>

    struct SystemState{
        bool main_mode;
        uint8_t ssid[32];
        uint8_t pass[64];
        int wi_fi_err_code;
        int time_s;
        bool error;
    };

    typedef struct PageState{

        float temperature_1;
        float temperature_2;
        float temperature_3;
        float temperature_4;

        float temper_control_val_1;
        float temper_control_val_2;
        float temper_control_val_3;

        int triac_percent_open;

        bool valve_1_state;
        bool valve_2_state;
        bool valve_3_state;

        int page_time;
        int next_page_num; 
        int error_code;
    };
//*********************PAGE_1*******************************
extern PageState page_1_initializers;        
bool page_1_to_2_condition(PageState &current_state);    
//*********************PAGE_2*******************************
extern PageState page_2_initializers;
bool page_2_to_0_condition(PageState &current_state);
//*********************PAGE_0(ALARM_PAGE)********************
extern PageState page_0_initializers;
#endif  // PB_GPIO_H

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

#ifndef P_DATA_H
#define P_DATA_H

    struct SystemState{
        bool main_mode;
        std::string ssid;
        std::string pass;
        int wi_fi_err_code;
        uint64_t time_s;
        bool error;
    };

    struct PageState{

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

        uint64_t page_time;
        int next_page_num; 
        int error_code;
    };
//*********************PAGE_1*******************************
    PageState page_1_initializers;
        page1.temper_control_val_1 = 0.0;
        page1.temper_control_val_2 = 0.0;
        page1.temper_control_val_3 = 0.0;
        page1.triac_percent_open = 100;
        page1.valve_1_state = 0;
        page1.valve_2_state = 0;
        page1.valve_3_state = 0;
        page1.page_time = 0;
        page1.next_page_num = 2;
        page1.error_code = 0;

    bool page_1_to_2_condition(PageState &current_state){
        return current_state.temperature_2 > 70.0;
    }  
//*********************PAGE_2*******************************
    PageState page_2_initializers;
        page1.temper_control_val_1 = 0.0;
        page1.temper_control_val_2 = 0.0;
        page1.temper_control_val_3 = 0.0;
        page1.triac_percent_open = 80;
        page1.valve_1_state = 1;
        page1.valve_2_state = 0;
        page1.valve_3_state = 0;
        page1.page_time = 0;
        page1.next_page_num = 0;
        page1.error_code = 0;

    bool page_2_to_0_condition(PageState &current_state){
        return current_state.page_time > 100;
    }
//*********************PAGE_0(ALARM_PAGE)********************
    const PageState page_0_initializers;
        page1.temper_control_val_1 = 0.0;
        page1.temper_control_val_2 = 0.0;
        page1.temper_control_val_3 = 0.0;
        page1.triac_percent_open = 0;
        page1.valve_1_state = 0;
        page1.valve_2_state = 0;
        page1.valve_3_state = 0;
        page1.page_time = 0;
        page1.next_page_num = 0;












#endif  // PB_GPIO_H

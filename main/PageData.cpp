#include "PageData.h"

//*********************PAGE_1*******************************
PageState page_1_initializers = {
    0.0,
    0.0,
    0.0,
    0.0,

    0.0,
    0.0,
    0.0,
    
    100,
    
    0,
    0,
    0,
    
    0,
    2,
    0};        

    bool page_1_to_2_condition(PageState &current_state){
        return current_state.temperature_2 > 70.0;
    }  
//*********************PAGE_2*******************************
    PageState page_2_initializers = {
    0.0,
    0.0,
    0.0,
    0.0,

    0.0,
    0.0,
    0.0,
    
    80,
    
    1,
    0,
    0,
    
    0,
    0,
    0};

    bool page_2_to_0_condition(PageState &current_state){
        return current_state.page_time > 100;
    }
//*********************PAGE_0(ALARM_PAGE)********************
    PageState page_0_initializers = {
    0.0,
    0.0,
    0.0,
    0.0,

    0.0,
    0.0,
    0.0,
    
    0,
    
    0,
    0,
    0,
    
    0,
    0,
    0};

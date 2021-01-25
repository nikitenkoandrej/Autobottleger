#pragma once
#ifndef PAGE_SYS_H
#define PAGE_SYS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "AutoBottlegerGPIO.h"
#include <string.h>
#include "PowBoardPeriph.h"
#include "PageData.h"
#include <vector>

class Page{
private:
    PageState page_state;
    uint64_t page_start_time;    
          

public:
    void update_state(PageState &new_state); 
    void set_page_start_time(uint64_t time);
    uint64_t get_page_start_time(){return page_start_time;}
    bool (*check_condition_ptr)(PageState &current_state);
    Page(PageState &initializer, bool (*condition_ptr)(PageState &current_state));
    PageState& get_state() {return page_state;}
    void set_err(int code);
};

class PageSystem{
private:
    SystemState state;
    Periph* ABPeriph_ptr;
    std::vector<Page> Page_table;    
    int current_page_num;
    bool check_condition();
    bool check_alarm();
    bool next_page_transfer;
    //bool update_state();       

public:    
    void perform_slow_loop(uint64_t time);
    void perform_fast_loop(int time_ms);
    PageSystem(Periph* ABPeriph);        
    void update_page(PageState &new_state);
    PageState& get_state();  
};




#endif  // PB_PERIPH_H

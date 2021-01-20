/*
 * MIT License
 *
 * Copyright (c) 2017 David Antliff
 * Copyright (c) 2017 Chris Morgan <chmorgan@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file
 * @brief Interface definitions for the 1-Wire bus component.
 *
 * This component provides structures and functions that are useful for communicating
 * with devices connected to a Maxim Integrated 1-Wire® bus via a single GPIO.
 *
 * Currently only externally powered devices are supported. Parasitic power is not supported.
 */

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


class Page{
private:
    PageState page_state;
    uint64_t page_start_time;    
    void update_state(PageState &new_state);       

public:
    void set_page_start_time(uint64_t time);
    uint64_t get_page_start_time(){return page_start_time;}
    bool (*check_condition_ptr)(PageState &current_state);
    Page(PageState &initializer, bool (*condition_ptr)(PageState &current_state));
    PageState& get_state() const {return page_state}; 
};

class PageSystem{
private:
    SystemState state;
    Periph* ABPeriph_ptr;
    std::vector<Page> Page_table;    
    int current_page_num;
    void perform_slow_loop(uint64_t time);
    void perform_fast_loop();
    //bool update_state();       

public:
    bool check_condition();
    bool check_alarm();
    PageSystem(Periph* ABPeriph);        
    void update_page(PageState &new_state);
    PageState& get_state() const;  
};




#endif  // PB_PERIPH_H

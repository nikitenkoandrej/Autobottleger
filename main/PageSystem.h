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

class Page{

private:

    Periph*
    
    struct SystemState{
        bool main_mode;
        std::string ssid;
        std::string pass;
        int wi_fi_err_code;
        uint64_t time_s;
        bool error;
    } state;

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

        uint64_t time_s;
        bool error;
    } page_state;


    bool perform_slow_loop();
    //bool perform_fast_loop();
    //bool (*check_ptr)();
    //bool update_state();       

public:
    bool check_condition;
    Page();
};


#endif  // PB_PERIPH_H

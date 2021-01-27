/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "PageSystem.h"
#include <vector>



Page::Page(PageState &initializer, bool (*condition_ptr)(PageState &current_state)){
    check_condition_ptr = condition_ptr;
    
    this->page_state.temper_control_val_1 = initializer.temper_control_val_1;
    this->page_state.temper_control_val_2 = initializer.temper_control_val_2;
    this->page_state.temper_control_val_3 = initializer.temper_control_val_3;
    this->page_state.triac_percent_open = initializer.triac_percent_open;
    this->page_state.valve_1_state = initializer.valve_1_state;
    this->page_state.valve_2_state = initializer.valve_2_state;
    this->page_state.valve_3_state = initializer.valve_3_state;
    this->page_state.page_time = initializer.page_time;
    this->page_state.next_page_num = initializer.next_page_num;
    this->page_state.error_code = initializer.error_code;

}

void Page::update_state(PageState &new_state){
    this->page_state.temperature_1 = new_state.temperature_1;
    this->page_state.temperature_2 = new_state.temperature_2;
    this->page_state.temperature_3 = new_state.temperature_3;
    this->page_state.temperature_4 = new_state.temperature_4;
    this->page_state.temper_control_val_1 = new_state.temper_control_val_1;
    this->page_state.temper_control_val_2 = new_state.temper_control_val_2;
    this->page_state.temper_control_val_3 = new_state.temper_control_val_3;
    this->page_state.triac_percent_open = new_state.triac_percent_open;
    this->page_state.valve_1_state = new_state.valve_1_state;
    this->page_state.valve_2_state = new_state.valve_2_state;
    this->page_state.valve_3_state = new_state.valve_3_state;
    this->page_state.page_time = new_state.page_time;
    this->page_state.next_page_num = new_state.next_page_num;
    this->page_state.error_code = new_state.error_code;
}

void Page::set_page_start_time(int time){
    this->page_start_time = time;
}

void Page::set_err(int code){
    this->page_state.error_code = code;
    this->page_state.next_page_num = 0;
}

bool PageSystem::check_condition(){
    return Page_table[current_page_num].check_condition_ptr(this->get_state());
}

bool PageSystem::check_alarm(){
    if(ABPeriph_ptr->GasSensor->check_alarm()){
        Page_table[current_page_num].set_err(1);
        return 1;
    }
    if(ABPeriph_ptr->WaterSensor->check_alarm()){
        Page_table[current_page_num].set_err(2);
        return 1;
    }
    return 0;
}

void PageSystem::perform_slow_loop(uint64_t time){

    this->state.time_s = int(time/10000000);

    PageState p = Page_table[current_page_num].get_state();

    p.temperature_1 = ABPeriph_ptr->ThermoSensor1->get_temperature();
    p.temperature_2 = ABPeriph_ptr->ThermoSensor2->get_temperature();
    p.temperature_3 = ABPeriph_ptr->ThermoSensor3->get_temperature();
    p.temperature_4 = ABPeriph_ptr->ThermoSensor4->get_temperature();
    p.page_time = int(time/10000000) - Page_table[current_page_num].get_page_start_time();

    if(this->check_alarm()){
        next_page_transfer =1;
        this->state.error = 1;
    }

    if(this->check_condition()){
        next_page_transfer = 1;
    }

    if(next_page_transfer){
        next_page_transfer = 0;
        current_page_num = p.next_page_num;        
        Page_table[current_page_num].set_page_start_time(time);           
    }

    Page_table[current_page_num].update_state(p);  
}

PageSystem::PageSystem(Periph* ABPeriph):ABPeriph_ptr(ABPeriph){
    printf("Start ps init\n");
    current_page_num = 1;
    next_page_transfer = 0;
    Page_table.push_back(Page(page_0_initializers, NULL));
    Page_table.push_back(Page(page_1_initializers, page_1_to_2_condition));
    Page_table[1].set_page_start_time(0);
    Page_table.push_back(Page(page_2_initializers, page_2_to_0_condition));
    
    
    printf("End ps init\n");

}

void PageSystem::update_page(PageState &new_state){
    Page_table[current_page_num].update_state(new_state);
}

PageState& PageSystem::get_state(){
    return Page_table[current_page_num].get_state();
}  

void PageSystem::perform_fast_loop(int time_ms){
    
    ABPeriph_ptr->Triac->brezenghem(time_ms);
}


/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <PageSystem.h>
#include <vector.h>



Page::Page(PageState &initializer, bool (*condition_ptr)(PageState &current_state)){
    next_page_transfer = 0;
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

void Page::set_page_start_time(uint64_t time){
    this->page_start_time = time;
}

bool PageSystem::check_condition(){
    return Page_table[current_page_num].(check_condition_ptr)(this->page_state);
}

bool PageSystem::check_alarm(){
    if(ABPeriph_ptr->GasSensor.check_alarm()){
        Page_table[current_page_num].page_state.error_code = 1;
    }
    if(ABPeriph_ptr->WaterSensor.check_alarm()){
        Page_table[current_page_num].page_state.error_code = 2;
    }
    if(this->page_state.error_code > 0){
        Page_table[current_page_num].page_state.next_page_num = 0;
        return TRUE;
    }
    return FALSE;
}

void PageSystem::perform_slow_loop(uint64_t time){

    this->state.time_s = time;

    PageState *p = Page_table[current_page_num].get_state();
    p->temperature_1 = ABPeriph_ptr->ThermoSensor1->get_temperature();
    p->temperature_2 = ABPeriph_ptr->ThermoSensor2->get_temperature();
    p->temperature_3 = ABPeriph_ptr->ThermoSensor3->get_temperature();
    p->temperature_4 = ABPeriph_ptr->ThermoSensor4->get_temperature();
    p->page_time = time - Page_table[current_page_num].get_page_start_time();

    if(this->check_alarm()){
        next_page_transfer = TRUE;
        this->state.error = 1;
    }

    if(Page_table[current_page_num].check_condition()){
        next_page_transfer = TRUE;
    }

    if(next_page_transfer){
        next_page_transfer = 0;
        current_page_num = Page_table[current_page_num].page_state.next_page_num;
        Page_table[current_page_num].page_start_time =             
    }    
}

PageSystem::PageSystem(Periph* ABPeriph):ABPeriph_ptr:(ABPeriph){
    current_page_num = 1;

    Page_table.append(new Page(page_1_initializers, page_1_to_2_condition));
    Page_table.append(new Page(page_2_initializers, page_2_to_0_condition));
    Page_table.append(new Page(page_0_initializers, NULL));
}

void PageSystem::update_page(){
    Page_table[current_page_num].update_state(PageState &new_state);
}

PageState& PageSystem::get_state() const{
    return Page_table[current_page_num].get_state();
}  

void PageSystem::perform_fast_loop(uint64_t time_ms){
    ABPeriph_ptr->Triac.brezenghem(static_cast<int>(time));
}
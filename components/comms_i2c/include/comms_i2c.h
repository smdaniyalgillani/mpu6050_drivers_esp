#pragma once
#include "common_includes.h"

typedef struct i2c_bus_controller{
    i2c_master_bus_handle_t i2c_bus_handle;

}i2c_bus_controller;


void i2c_bus_init(i2c_bus_controller *i2c_control,  i2c_master_bus_handle_t i2c_bus_handle, int i2c_port_, gpio_num_t scl_, gpio_num_t sda_, bool PULLUP_EN);

void i2c_add_new_device(i2c_bus_controller *i2c_control, i2c_master_dev_handle_t *dev_handle, uint16_t dev_addr, uint32_t dev_speed_hz);
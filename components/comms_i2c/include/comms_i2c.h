#pragma once
#include "common_includes.h"

void i2c_bus_init(int i2c_port_, gpio_num_t scl_, gpio_num_t sda_, bool PULLUP_EN);

void i2c_add_new_device(uint16_t dev_addr, uint32_t dev_speed_hz);
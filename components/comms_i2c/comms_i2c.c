#include "comms_i2c.h"

void i2c_bus_init(i2c_bus_controller *i2c_control, i2c_master_bus_handle_t i2c_bus_handle, int i2c_port_, gpio_num_t scl_, gpio_num_t sda_, bool PULLUP_EN)
{
    i2c_control->i2c_bus_handle = i2c_bus_handle;
    i2c_master_bus_config_t i2c_bus_conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = i2c_port_,
        .scl_io_num = scl_,
        .sda_io_num = sda_,
        // .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = PULLUP_EN,    
        };

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_conf, &i2c_control->i2c_bus_handle));
}

void i2c_add_new_device(i2c_bus_controller *i2c_control, i2c_master_dev_handle_t dev_handle, uint16_t dev_addr, uint32_t dev_speed_hz)
{
    i2c_device_config_t device_config = {
        .device_address = dev_addr,
        .scl_speed_hz = dev_speed_hz,
    };
    

    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_control->i2c_bus_handle, &device_config, &dev_handle));

}
#ifndef MAX30101_H
#define MAX30101_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <stdint.h>
#include <stdbool.h>

// Define the I2C address for MAX30101
#define MAX30101_ADDRESS 0x57
// #define MAX_30101_EXPECTEDPARTID 0x15

// Define storage size for FIFO buffer
#define STORAGE_SIZE 4

#define I2C_BUFFER_LENGTH 32
// Structure for holding sensor data
typedef struct {
    uint32_t red[STORAGE_SIZE];
    uint32_t ir[STORAGE_SIZE];
    uint32_t green[STORAGE_SIZE];
    uint8_t head;
    uint8_t tail;
} max30101_sense_struct;

// MAX30101 device structure
typedef struct{
    const struct dev *i2c_dev;  // Pointer to I2C device
    uint8_t i2c_addr;           // I2C address of MAX30101
    uint8_t active_leds;           // Number of active LEDs
    uint8_t revision_id;           // Revision ID of the sensor
    max30101_sense_struct sense;   // Circular buffer for storing sensor data

}max30101_t;

// Initialization and basic operation
void max30101_init (max30101_t *dev, const struct device *i2c_dev);
bool max30101_safe_check(max30101_t *dev, uint8_t max_time_to_check);
uint32_t max30101_get_red(max30101_t *dev);
uint32_t max30101_get_ir(max30101_t *dev);
uint32_t max30101_get_green(max30101_t *dev);


// Configuration
void max30101_soft_reset(max30101_t *dev);
void max30101_shut_down(max30101_t *dev);
void max30101_wake_up(max30101_t *dev);
void max30101_set_led_mode(max30101_t *dev, uint8_t mode);
void max30101_set_adc_range(max30101_t *dev, uint8_t adc_range);
void max30101_set_sample_rate(max30101_t *dev, uint8_t sample_rate);
void max30101_set_pulse_width(max30101_t *dev, uint8_t pulse_width);
void max30101_set_pulse_amplitude_red(max30101_t *dev, uint8_t value);
void max30101_set_pulse_amplitude_ir(max30101_t *dev, uint8_t value);
void max30101_set_pulse_amplitude_green(max30101_t *dev, uint8_t value);
// void max30101_set_pulse_amplitude_proximity(max30101_t *dev, uint8_t value);
// void max30101_set_proximity_threshold(max30101_t *dev, uint8_t thresh_msb);

// Multi-LED configuration
void max30101_enable_slot(max30101_t *dev, uint8_t slot_number, uint8_t device);
void max30101_disable_slots(max30101_t *dev);

// Interrupts
uint8_t max30101_get_int1(max30101_t *dev);
uint8_t max30101_get_int2(max30101_t *dev);
void max30101_enable_afull(max30101_t *dev);
void max30101_disable_afull(max30101_t *dev);
void max30101_enable_ppgrdy(max30101_t *dev);
void max30101_disable_ppgrdy(max30101_t *dev);
void max30101_enable_alcovf(max30101_t *dev);
void max30101_disable_alcovf(max30101_t *dev);
// void max30101_enable_proxint(max30101_t *dev);
// void max30101_disable_proxint(max30101_t *dev);
void max30101_enable_dietemprdy(max30101_t *dev);
void max30101_disable_dietemprdy(max30101_t *dev);

// FIFO configuration and reading
void max30101_set_fifo_average(max30101_t *dev, uint8_t samples);
void max30101_enable_fifo_rollover(max30101_t *dev);
void max30101_disable_fifo_rollover(max30101_t *dev);
void max30101_set_fifo_almost_full(max30101_t *dev, uint8_t samples);
uint16_t max30101_check(max30101_t *dev);
uint8_t max30101_available(max30101_t *dev);
void max30101_next_sample(max30101_t *dev);
uint32_t max30101_get_fifo_red(max30101_t *dev);
uint32_t max30101_get_fifo_ir(max30101_t *dev);
uint32_t max30101_get_fifo_green(max30101_t *dev);
uint8_t max30101_get_write_pointer(max30101_t *dev);
uint8_t max30101_get_read_pointer(max30101_t *dev);
void max30101_clear_fifo(max30101_t *dev);

// Die Temperature
float max30101_read_temperature(max30101_t *dev);
float max30101_read_temperature_f(max30101_t *dev);

// Detecting ID/Revision
uint8_t max30101_get_revision_id(max30101_t *dev);
uint8_t max30101_read_part_id(max30101_t *dev);

// Setup the IC with user-selectable settings
void max30101_setup(max30101_t *dev, uint8_t power_level, uint8_t sample_average, uint8_t led_mode, int sample_rate, int pulse_width, int adc_range);

// Low-level I2C communication
uint8_t max30101_read_register8(max30101_t *dev, uint8_t reg);
void max30101_write_register8(max30101_t *dev, uint8_t address, uint8_t reg, uint8_t value);


void max30101_bitmask(max30101_t *dev, uint8_t reg, uint8_t mask, uint8_t thing);
void max30101_read_revision_id(max30101_t *dev);

#endif // MAX30105_H

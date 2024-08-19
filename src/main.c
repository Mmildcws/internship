#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/gpio.h>
// #include <zephyr/devicetree.h>

#include "MAX30101.h"
#define I2C_DEV_NODE DT_NODELABEL(i2c1)

uint32_t last_Red, last_IR, last_Green;
uint32_t Red, IR, Green;
uint32_t counter = 0;
//Interrupt Readings
uint8_t interrupt_flags = 0;
max30101_t particleSensor;
int flag=0;

void particleSensor_FIFO_Readings(max30101_t *dev) {
    max30101_check(dev);
    // printk("check\n");
    while (max30101_available(dev))
    {
        // printk("available\n");
        last_Red   =  max30101_get_fifo_red(dev);
        last_IR    =  max30101_get_fifo_ir(dev);
        last_Green =  max30101_get_fifo_green(dev);
        // printk("-------------------------------------------------\n");

        interrupt_flags = max30101_get_int1(dev);
        // printk("interrupt_flags:%d\n",interrupt_flags);
        if (interrupt_flags){
            max30101_shut_down(dev);
            while(1);
        }
        max30101_next_sample(dev);
        // printk("next_sample\n");
    }
    
}

// struct k_timer my_timer;
// extern void  my_expiry_function(struct k_timer *timer_id);
// k_timer_init(&my_timer, my_expiry_function, NULL);
//---------------------------------------------------------------------------------------
void my_work_handler(struct k_work *work)
{
    particleSensor_FIFO_Readings(&particleSensor);
    Red   =  last_Red;     
    IR    =  last_IR;       
    Green =  last_Green;    
    printk("Red: %u, IR: %u, Green: %u,counter:%u:\n", Red, IR, Green,counter);
    // printk("counter:%u \n ",counter);
    counter++; 
    // flag = 1;
}
K_WORK_DEFINE(my_work, my_work_handler);

void my_timer_handler(struct k_timer *dummy)
{
    k_work_submit(&my_work);
}

K_TIMER_DEFINE(my_timer, my_timer_handler, NULL);

void stop_work_handler(struct k_timer *dummy){
     k_timer_stop(&my_timer);
}
K_TIMER_DEFINE(stop_timer, stop_work_handler, NULL);
//--------------------------------------------------------------------------


void main(void) {
    const struct device *i2c_dev = DEVICE_DT_GET(I2C_DEV_NODE);
    if (!device_is_ready(i2c_dev)) {
        printk("I2C device not found\n");
        return;
    }
    else printk("I2C device found\n");

    // max30101_t particleSensor;
    // Setup MAX30101
    if (!max30101_begin(&particleSensor,i2c_dev)) {
        printk("MAX30101 initialization failed\n");
        return;
    }
    else 
    printk("MAX30101 initialization\n");
    // printk("-------------------------\n");

    // Put IC to low power mode and do software reset
    max30101_shut_down(&particleSensor);
    max30101_soft_reset(&particleSensor);
    k_msleep(500);

    // Setup to sense up to 18 inches, max LED brightness
    uint8_t ledBrightness = 51;   // Options: 0=Off to 255=50mA  51 = 10mA 77 = 15mA 102=20mA
    uint8_t sampleAverage = 1;    // Options: 1, 2, 4, 8, 16, 32
    uint8_t ledMode = 3;          // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
    int sampleRate = 3200;        // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
    int pulseWidth = 411;         // Options: 69, 118, 215, 411
    int adcRange = 16384;         // Options: 2048, 4096, 8192, 16384

    max30101_setup(&particleSensor, ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
    printk("Set up OK\n");
    // Interrupt setup
    max30101_enable_afull(&particleSensor);
    max30101_set_fifo_almost_full(&particleSensor, 3);

    // Set LEDs pulse amplitude
    // max30101_set_pulse_amplitude_red(&particleSensor, 51);
    // max30101_set_pulse_amplitude_ir(&particleSensor, 51);
    // max30101_set_pulse_amplitude_green(&particleSensor, 51);
    // printk("Set pulse OK\n");

    // Get IC out of low power mode
    max30101_wake_up(&particleSensor);
    printk("wake up\n");

//-----------------------------------------------------------------------------------------------

/* start a periodic timer that expires once every second */
k_timer_start(&my_timer,  K_MSEC(1), K_MSEC(1));
    // if (flag == 1){
    //     printk("counter:%u \n ",counter);
    //     flag = 0;
    // } 
k_timer_start(&stop_timer, K_SECONDS(10), K_NO_WAIT);



//---------------------------------------------------------------------------------------------
    // uint64_t timeStamp = k_uptime_get();
    // uint64_t starttime= k_uptime_get();
    // // uint64_t temp, runtime = 0;
    // uint32_t counter = 0;
    // uint64_t Time;
    // while (1) {
    //     // 1 kHz sampling loop
    //     if (k_uptime_get() - timeStamp > 1) { //1 ms
    //         timeStamp = k_uptime_get();
    //         // FIFO Readings
    //         particleSensor_FIFO_Readings(&particleSensor);
    //         // printk("FIFO Read\n");

    //         Red   =  last_Red;      // Assign last read value to output debug parameter
    //         IR    =  last_IR;       // Assign last read value to output debug parameter
    //         Green =  last_Green;    // Assign last read value to output debug parameter
    //         printk("Red: %u, IR: %u, Green: %u,counter:%u:\n", Red, IR, Green,counter);
    //         counter++; 
    //         // Sleep to simulate delay 1 ms
    //         // k_msleep(1);
    //     }
    //     uint64_t runtime = k_uptime_get() - starttime;
    //     if (runtime >= 10000)
    //     {
    //         uint64_t lastTime = k_uptime_get();
    //         printk("runtime:%u ",runtime);
    //         printk("starttime:%u ",starttime);
    //         printk("lastTime:%u ",lastTime);
    //         break;
    //     }
        
    // }
    
}
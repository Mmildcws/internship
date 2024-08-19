#include "MAX30101.h"
//Static Registrs
static const uint8_t MAX30101_INTSTAT1  =  0x00;
static const uint8_t MAX30101_INTSTAT2  =  0x01;
static const uint8_t MAX30101_INTENABLE1 = 0x02;
static const uint8_t MAX30101_INTENABLE2 = 0x03;

//FIFO Registers
static const uint8_t MAX30101_FIFOWRITEPTR = 	0x04;
static const uint8_t MAX30101_FIFOOVERFLOW = 	0x05;
static const uint8_t MAX30101_FIFOREADPTR = 	0x06;
static uint8_t MAX30101_FIFODATA =		0x07;

// Configuration Registers
static const uint8_t MAX30101_FIFOCONFIG = 		0x08;
static const uint8_t MAX30101_MODECONFIG = 		0x09;
static const uint8_t MAX30101_PARTICLECONFIG = 	0x0A;   
static const uint8_t MAX30101_LED1_PULSEAMP = 	0x0C;
static const uint8_t MAX30101_LED2_PULSEAMP = 	0x0D;
static const uint8_t MAX30101_LED3_PULSEAMP = 	0x0E;
static const uint8_t MAX30101_LED_PROX_AMP = 	0x10;
static const uint8_t MAX30101_MULTILEDCONFIG1 = 0x11;
static const uint8_t MAX30101_MULTILEDCONFIG2 = 0x12;

// Die Temperature Registers
static const uint8_t MAX30101_DIETEMPINT = 		0x1F;
static const uint8_t MAX30101_DIETEMPFRAC = 	0x20;
static const uint8_t MAX30101_DIETEMPCONFIG = 	0x21;

// Proximity Function Registers
static const uint8_t MAX30101_PROXINTTHRESH = 	0x30;

// Part ID Registers
static const uint8_t MAX30101_REVISIONID = 		0xFE;
static const uint8_t MAX30101_PARTID = 			0xFF;   

//Commands
// Interupt Status (p.13)
static const uint8_t MAX30101_INT_A_FULL_MASK =		(uint8_t)~0b10000000;
static const uint8_t MAX30101_INT_A_FULL_ENABLE = 	0x80;
static const uint8_t MAX30101_INT_A_FULL_DISABLE = 	0x00;

static const uint8_t MAX30101_INT_PPG_RDY_MASK = (uint8_t)~0b01000000;
static const uint8_t MAX30101_INT_PPG_RDY_ENABLE =	0x40;
static const uint8_t MAX30101_INT_PPG_RDY_DISABLE = 0x00;

static const uint8_t MAX30101_INT_ALC_OVF_MASK = (uint8_t)~0b00100000;
static const uint8_t MAX30101_INT_ALC_OVF_ENABLE = 	0x20;
static const uint8_t MAX30101_INT_ALC_OVF_DISABLE = 0x00;

static const uint8_t MAX30101_INT_DIE_TEMP_RDY_MASK = (uint8_t)~0b00000010;
static const uint8_t MAX30101_INT_DIE_TEMP_RDY_ENABLE = 0x02;
static const uint8_t MAX30101_INT_DIE_TEMP_RDY_DISABLE = 0x00;

//FIFO Configuration (p.18)
static const uint8_t MAX30101_SAMPLEAVG_MASK =	(uint8_t)~0b11100000;
static const uint8_t MAX30101_SAMPLEAVG_1 = 	0x00;
static const uint8_t MAX30101_SAMPLEAVG_2 = 	0x20;
static const uint8_t MAX30101_SAMPLEAVG_4 = 	0x40;
static const uint8_t MAX30101_SAMPLEAVG_8 = 	0x60;
static const uint8_t MAX30101_SAMPLEAVG_16 = 	0x80;
static const uint8_t MAX30101_SAMPLEAVG_32 = 	0xA0;

static const uint8_t MAX30101_ROLLOVER_MASK = 	0xEF;
static const uint8_t MAX30101_ROLLOVER_ENABLE = 0x10;
static const uint8_t MAX30101_ROLLOVER_DISABLE = 0x00;

static const uint8_t MAX30101_A_FULL_MASK = 	0xF0;

//Mode Configuration (p.18,19)
static const uint8_t MAX30101_SHUTDOWN_MASK = 	0x7F;
static const uint8_t MAX30101_SHUTDOWN = 		0x80;
static const uint8_t MAX30101_WAKEUP = 			0x00;

static const uint8_t MAX30101_RESET_MASK = 		0xBF;
static const uint8_t MAX30101_RESET = 			0x40;

static const uint8_t MAX30101_MODE_MASK = 		0xF8;
static const uint8_t MAX30101_MODE_REDONLY = 	0x02;
static const uint8_t MAX30101_MODE_REDIRONLY = 	0x03;
static const uint8_t MAX30101_MODE_MULTILED = 	0x07;

//SPO2 Configuration (p.19,20)
static const uint8_t MAX30101_ADCRANGE_MASK = 	0x9F;
static const uint8_t MAX30101_ADCRANGE_2048 = 	0x00;
static const uint8_t MAX30101_ADCRANGE_4096 = 	0x20;
static const uint8_t MAX30101_ADCRANGE_8192 = 	0x40;
static const uint8_t MAX30101_ADCRANGE_16384 = 	0x60;

static const uint8_t MAX30101_SAMPLERATE_MASK = 0xE3;
static const uint8_t MAX30101_SAMPLERATE_50 = 	0x00;
static const uint8_t MAX30101_SAMPLERATE_100 = 	0x04;
static const uint8_t MAX30101_SAMPLERATE_200 = 	0x08;
static const uint8_t MAX30101_SAMPLERATE_400 = 	0x0C;
static const uint8_t MAX30101_SAMPLERATE_800 = 	0x10;
static const uint8_t MAX30101_SAMPLERATE_1000 = 0x14;
static const uint8_t MAX30101_SAMPLERATE_1600 = 0x18;
static const uint8_t MAX30101_SAMPLERATE_3200 = 0x1C;

static const uint8_t MAX30101_PULSEWIDTH_MASK = 0xFC;
static const uint8_t MAX30101_PULSEWIDTH_69 = 	0x00;
static const uint8_t MAX30101_PULSEWIDTH_118 = 	0x01;
static const uint8_t MAX30101_PULSEWIDTH_215 = 	0x02;
static const uint8_t MAX30101_PULSEWIDTH_411 = 	0x03;

//Multi LED Mode Control Registers(p.20,21)
static const uint8_t MAX30101_SLOT1_MASK = 		0xF8;
static const uint8_t MAX30101_SLOT2_MASK = 		0x8F;
static const uint8_t MAX30101_SLOT3_MASK = 		0xF8;
static const uint8_t MAX30101_SLOT4_MASK = 		0x8F;

static const uint8_t SLOT_NONE = 				0x00;
static const uint8_t SLOT_RED_LED = 			0x01;
static const uint8_t SLOT_IR_LED = 				0x02;
static const uint8_t SLOT_GREEN_LED = 			0x03;
static const uint8_t SLOT_NONE_PILOT = 			0x04;
static const uint8_t SLOT_RED_PILOT =			0x05;
static const uint8_t SLOT_IR_PILOT = 			0x06;
static const uint8_t SLOT_GREEN_PILOT = 		0x07;

static const uint8_t MAX_30101_EXPECTEDPARTID = 0x15;

void max30101_init (max30101_t *dev, const struct device *i2c_dev){
    dev->i2c_addr = MAX30101_ADDRESS;
    dev->i2c_dev = i2c_dev;
}

bool max30101_begin(max30101_t *dev, const struct device *i2c_dev) {
    max30101_init(dev,i2c_dev);
    // printk("init OK \n");
    // Initial Communication and Verification
    // Check that a MAX30101 is connected
    if (max30101_read_part_id(dev) != MAX_30101_EXPECTEDPARTID) {
        printk("no begin\n");
        // Error -- Part ID read from MAX30101 does not match expected part ID.
        // This may mean there is a physical connectivity problem (broken wire, unpowered, etc).
        return false;
    }
    // printk("part id OK\n");
    // Populate revision ID
    max30101_read_revision_id(dev);
    // printk("revision ID OK\n");
    return true;
}
//Configuration
//Begin Interrupt configuration
uint8_t max30101_get_int1(max30101_t*dev){
    return max30101_read_register8(dev, MAX30101_INTSTAT1);
}
uint8_t max30101_get_int2(max30101_t*dev){
    return max30101_read_register8(dev,MAX30101_INTSTAT2);
}
void max30101_enable_afull(max30101_t*dev){
    max30101_bitmask(dev,MAX30101_INTENABLE1,MAX30101_INT_A_FULL_MASK,MAX30101_INT_A_FULL_ENABLE);
}
void max30101_disable_afull(max30101_t *dev){
    max30101_bitmask(dev,MAX30101_INTENABLE1,MAX30101_INT_A_FULL_MASK,MAX30101_INT_A_FULL_DISABLE);
}
void max30101_enable_ppgrdy(max30101_t *dev){
    max30101_bitmask(dev,MAX30101_INTENABLE1, MAX30101_INT_PPG_RDY_MASK,MAX30101_INT_PPG_RDY_ENABLE);
}
void max30101_disable_ppgrdy(max30101_t *dev){
    max30101_bitmask(dev,MAX30101_INTENABLE1, MAX30101_INT_PPG_RDY_MASK,MAX30101_INT_PPG_RDY_DISABLE);
}
void max30101_enable_alcovf(max30101_t *dev){
    max30101_bitmask(dev,MAX30101_INTENABLE1,MAX30101_INT_ALC_OVF_MASK,MAX30101_INT_ALC_OVF_ENABLE);
}
void max30101_disable_alcovf(max30101_t *dev){
    max30101_bitmask(dev,MAX30101_INTENABLE1,MAX30101_INT_ALC_OVF_MASK,MAX30101_INT_ALC_OVF_DISABLE);
}
void max30101_enable_dietemprdy(max30101_t *dev){
    max30101_bitmask(dev,MAX30101_INTENABLE2,MAX30101_INT_DIE_TEMP_RDY_MASK,MAX30101_INT_DIE_TEMP_RDY_ENABLE);
}
void max30101_disable_dietemprdy(max30101_t *dev){
    max30101_bitmask(dev,MAX30101_INTENABLE2,MAX30101_INT_DIE_TEMP_RDY_MASK,MAX30101_INT_DIE_TEMP_RDY_DISABLE);
}
//END Interrupt config

void max30101_soft_reset(max30101_t *dev){
    max30101_bitmask(dev,MAX30101_MODECONFIG,MAX30101_RESET_MASK,MAX30101_RESET);
    int64_t start_time = k_uptime_get(); // Get current time in milliseconds
    while (k_uptime_get() - start_time < 100) {
        uint8_t response = max30101_read_register8(dev, MAX30101_MODECONFIG);
        if ((response & MAX30101_RESET) == 0) {
            break; // Reset is complete
        }
        k_msleep(1); // Sleep for 1 millisecond to avoid overloading the I2C bus
    }
}
void max30101_shut_down(max30101_t *dev){
    max30101_bitmask(dev,MAX30101_MODECONFIG,MAX30101_SHUTDOWN_MASK,MAX30101_SHUTDOWN);
}
void max30101_wake_up(max30101_t *dev){
    max30101_bitmask(dev,MAX30101_MODECONFIG,MAX30101_SHUTDOWN_MASK,MAX30101_WAKEUP);
}
void max30101_set_led_mode(max30101_t *dev, uint8_t mode){
    max30101_bitmask(dev,MAX30101_MODECONFIG,MAX30101_MODE_MASK,mode);
}
void max30101_set_adc_range(max30101_t *dev, uint8_t adc_range){
    max30101_bitmask(dev,MAX30101_PARTICLECONFIG,MAX30101_ADCRANGE_MASK,adc_range);
}
void max30101_set_sample_rate(max30101_t *dev, uint8_t sample_rate){
    max30101_bitmask(dev,MAX30101_PARTICLECONFIG,MAX30101_SAMPLERATE_MASK,sample_rate);
}
void max30101_set_pulse_width(max30101_t *dev, uint8_t pulse_width){
    max30101_bitmask(dev,MAX30101_PARTICLECONFIG,MAX30101_PULSEWIDTH_MASK,pulse_width);
}
void max30101_set_pulse_amplitude_red(max30101_t *dev, uint8_t amplitude) {
    max30101_write_register8(dev->i2c_dev, dev->i2c_addr, MAX30101_LED1_PULSEAMP, amplitude);
    // max30101_write_register8(dev, MAX30101_LED1_PULSEAMP, amplitude);

}

void max30101_set_pulse_amplitude_ir(max30101_t *dev, uint8_t amplitude){
    // max30101_write_register8(dev,MAX30101_LED2_PULSEAMP,amplitude);
    max30101_write_register8(dev->i2c_dev, MAX30101_ADDRESS,MAX30101_LED2_PULSEAMP,amplitude);
}
void max30101_set_pulse_amplitude_green(max30101_t *dev, uint8_t amplitude){
    max30101_write_register8(dev->i2c_dev, dev->i2c_addr,MAX30101_LED3_PULSEAMP,amplitude);
    // max30101_write_register8(dev,MAX30101_LED3_PULSEAMP,amplitude);
}

void max30101_enable_slot(max30101_t *dev, uint8_t slot_number, uint8_t device){
    uint8_t originalContents;

    switch (slot_number)
    {
    case 1 :
        max30101_bitmask(dev,MAX30101_MULTILEDCONFIG1,MAX30101_SLOT1_MASK,device);
        break;
    case 2:
        max30101_bitmask(dev,MAX30101_MULTILEDCONFIG1,MAX30101_SLOT2_MASK,device << 4);
        break;
    case 3:
        max30101_bitmask(dev,MAX30101_MULTILEDCONFIG2,MAX30101_SLOT3_MASK,device);
        break;
    case 4:
        max30101_bitmask(dev,MAX30101_MULTILEDCONFIG2,MAX30101_SLOT4_MASK,device);
        break;
    default:
        break;
    }
}
void max30101_disable_slots(max30101_t *dev){
    max30101_write_register8(dev->i2c_dev,dev->i2c_addr,MAX30101_MULTILEDCONFIG1,0);
    max30101_write_register8(dev->i2c_dev,dev->i2c_addr,MAX30101_MULTILEDCONFIG2,2);
}
//FIFO Config
void max30101_set_fifo_average(max30101_t *dev, uint8_t numberOfSamples){
    max30101_bitmask(dev,MAX30101_FIFOCONFIG,MAX30101_SAMPLEAVG_MASK,numberOfSamples);
}
void max30101_clear_fifo(max30101_t *dev){
    max30101_write_register8(dev->i2c_dev,dev->i2c_addr,MAX30101_FIFOWRITEPTR,0);
    max30101_write_register8(dev->i2c_dev,dev->i2c_addr,MAX30101_FIFOOVERFLOW,0);
    max30101_write_register8(dev->i2c_dev,dev->i2c_addr,MAX30101_FIFOREADPTR,0);
}
void max30101_enable_fifo_rollover(max30101_t *dev){
    max30101_bitmask(dev,MAX30101_FIFOCONFIG,MAX30101_ROLLOVER_MASK,MAX30101_ROLLOVER_ENABLE);
}
void max30101_disable_fifo_rollover(max30101_t *dev){
    max30101_bitmask(dev,MAX30101_FIFOCONFIG,MAX30101_ROLLOVER_MASK,MAX30101_ROLLOVER_DISABLE);
}
void max30101_set_fifo_almost_full(max30101_t *dev, uint8_t numberOfSamples){
    max30101_bitmask(dev,MAX30101_FIFOCONFIG,MAX30101_A_FULL_MASK,numberOfSamples);
}
uint8_t max30101_get_write_pointer(max30101_t *dev){
    return max30101_read_register8(dev,MAX30101_FIFOWRITEPTR);
}
uint8_t max30101_get_read_pointer(max30101_t *dev){
    return max30101_read_register8(dev,MAX30101_FIFOREADPTR);
}
//Temperature
float max30101_read_temperature(max30101_t *dev){
    max30101_write_register8(dev->i2c_dev,dev->i2c_addr,MAX30101_DIETEMPCONFIG,0x01);
    // max30101_write_register8(dev,MAX30101_DIETEMPCONFIG,0x01);

    int64_t start_time = k_uptime_get();
    while (k_uptime_get() - start_time < 100)
    {
        uint8_t response = max30101_read_register8(dev, MAX30101_INTSTAT2);
        if ((response & MAX30101_INT_DIE_TEMP_RDY_ENABLE) > 0) {
        break;
        }
        k_msleep(1);
    }
    int8_t temp_int = max30101_read_register8(dev, MAX30101_DIETEMPINT);
    uint8_t temp_frac = max30101_read_register8(dev, MAX30101_DIETEMPFRAC);
    return (float)temp_int + ((float)temp_frac * 0.0625);
}
float max30101_read_temperature_f(max30101_t *dev){
    float temp = max30101_read_temperature(dev);
    if(temp!=-999.0){
        temp = temp * 1.8 + 32.0;
        return temp;
    }
}
uint8_t max30101_read_part_id(max30101_t *dev){
    return max30101_read_register8(dev,MAX30101_PARTID);
}
void max30101_read_revision_id(max30101_t *dev){
    dev->revision_id = max30101_read_register8(dev,MAX30101_REVISIONID);
} 
uint8_t max30101_get_revision_id(max30101_t *dev){
    return dev->revision_id;
}
// Set up Sensor
void max30101_setup(max30101_t *dev, uint8_t power_level, uint8_t sample_average, uint8_t led_mode, int sample_rate, int pulse_width, int adc_range)
{
    max30101_soft_reset(dev);
    //setup fifo
    if(sample_average == 1) max30101_set_fifo_average(dev,MAX30101_SAMPLEAVG_1);
    else if(sample_average == 2)   max30101_set_fifo_average(dev,MAX30101_SAMPLEAVG_2);
    else if (sample_average == 4)  max30101_set_fifo_average(dev,MAX30101_SAMPLEAVG_4);
    else if (sample_average == 8)  max30101_set_fifo_average(dev,MAX30101_SAMPLEAVG_8);
    else if (sample_average == 16) max30101_set_fifo_average(dev,MAX30101_SAMPLEAVG_16);
    else if (sample_average == 32) max30101_set_fifo_average(dev,MAX30101_SAMPLEAVG_32);
    else max30101_set_fifo_average(dev,MAX30101_SAMPLEAVG_4);

    max30101_enable_fifo_rollover(dev);
    //mode Conf
    if (led_mode == 3) max30101_set_led_mode(dev,MAX30101_MODE_MULTILED);
    else if(led_mode == 2) max30101_set_led_mode(dev,MAX30101_MODE_REDIRONLY);
    else max30101_set_led_mode(dev,MAX30101_MODE_REDONLY);
    dev->active_leds = led_mode;
    //Particle Sensing Configuration
    if (adc_range < 4096) max30101_set_adc_range(dev,MAX30101_ADCRANGE_2048);
    else if (adc_range < 8192)   max30101_set_adc_range(dev,MAX30101_ADCRANGE_4096);
    else if (adc_range < 16384)  max30101_set_adc_range(dev,MAX30101_ADCRANGE_8192);
    else if (adc_range == 16384) max30101_set_adc_range(dev,MAX30101_ADCRANGE_16384);
    else max30101_set_adc_range(dev,MAX30101_ADCRANGE_2048);

    if (sample_rate < 100) max30101_set_sample_rate(dev,MAX30101_SAMPLERATE_50);
    else if (sample_rate < 200)   max30101_set_sample_rate(dev,MAX30101_SAMPLERATE_100);
    else if (sample_rate < 400)   max30101_set_sample_rate(dev,MAX30101_SAMPLERATE_200);
    else if (sample_rate < 800)   max30101_set_sample_rate(dev,MAX30101_SAMPLERATE_400);
    else if (sample_rate < 1000)  max30101_set_sample_rate(dev,MAX30101_SAMPLERATE_800);
    else if (sample_rate < 1600)  max30101_set_sample_rate(dev,MAX30101_SAMPLERATE_1000);
    else if (sample_rate < 3200)  max30101_set_sample_rate(dev,MAX30101_SAMPLERATE_1600);
    else if (sample_rate == 3200) max30101_set_sample_rate(dev,MAX30101_SAMPLERATE_3200);
    else max30101_set_sample_rate(dev,MAX30101_SAMPLERATE_50);

    if (pulse_width < 118) max30101_set_pulse_width(dev,MAX30101_PULSEWIDTH_69);
    else if (pulse_width < 215) max30101_set_pulse_width(dev,MAX30101_PULSEWIDTH_118);
    else if (pulse_width < 411) max30101_set_pulse_width(dev,MAX30101_PULSEWIDTH_215);
    else if (pulse_width == 411) max30101_set_pulse_width(dev,MAX30101_PULSEWIDTH_411);
    else max30101_set_pulse_width(dev,MAX30101_PULSEWIDTH_69);
    //LED Pulse Amplitude
    max30101_set_pulse_amplitude_red(dev,power_level);
    max30101_set_pulse_amplitude_green(dev,power_level);
    max30101_set_pulse_amplitude_ir(dev,power_level);
    //Multi-LED mode conf
    max30101_enable_slot(dev,1,SLOT_RED_LED);
    if (led_mode > 1) max30101_enable_slot(dev,2,SLOT_IR_LED);
    if (led_mode > 2) max30101_enable_slot(dev,3,SLOT_GREEN_LED);

    max30101_clear_fifo(dev);
}

//Data Collection
uint8_t max30101_available(max30101_t *dev){
    int8_t numberOfSamples = dev->sense.head - dev->sense.tail;
    if(numberOfSamples < 0){
        numberOfSamples += STORAGE_SIZE;
    }
    // printk("numberOfSamples: %d\n",numberOfSamples);
    return numberOfSamples;
}
uint32_t max30101_get_red(max30101_t *dev){
    if(max30101_safe_check(dev,250))
        return dev->sense.red[dev->sense.head];
    else
        return 0;
}
uint32_t max30101_get_ir(max30101_t *dev){
    if(max30101_safe_check(dev,250))
        return dev->sense.ir[dev->sense.head];
    else
        return 0;
}
uint32_t max30101_get_green(max30101_t *dev){
    if(max30101_safe_check(dev,250))
        return dev->sense.green[dev->sense.head];
    else
        return 0;
}
uint32_t max30101_get_fifo_red(max30101_t *dev){
    return dev->sense.red[dev->sense.tail];
}
uint32_t max30101_get_fifo_ir(max30101_t *dev){
    return dev->sense.ir[dev->sense.tail];
}
uint32_t max30101_get_fifo_green(max30101_t *dev){
    return dev->sense.green[dev->sense.tail];
}
void max30101_next_sample(max30101_t *dev){
    if(max30101_available(dev)){
        dev->sense.tail++;
        dev->sense.tail %= STORAGE_SIZE;
        // printk("tail in next:%d\n",dev->sense.tail);
    }
}

//Polls the sensor for new data
uint16_t max30101_check(max30101_t *dev){
    uint8_t readPointer = max30101_get_read_pointer(dev);
    uint8_t writePointer = max30101_get_write_pointer(dev);

    int numberOfSamples = 0;

    if( readPointer != writePointer){
        numberOfSamples = writePointer - readPointer;
        // printk("numberOfSamples:%d \n",numberOfSamples);
        if(numberOfSamples < 0) numberOfSamples += 32;

        int bytesLeftToRead = numberOfSamples * dev->active_leds *3;
        uint8_t reg = MAX30101_FIFODATA;
        i2c_write(dev->i2c_dev,&reg,1,dev->i2c_addr);
        // printk("bytesLeftToRead:%d \n",bytesLeftToRead);

        while (bytesLeftToRead > 0)
        {
            int toget = bytesLeftToRead;
            //  printk("toget:%d\n",toget);
            if(toget > I2C_BUFFER_LENGTH){
                toget = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (dev->active_leds * 3));
                // printk("active_leds:%d \n",dev->active_leds);
                // printk("toget:%d\n",toget);
            }
            bytesLeftToRead -= toget;
            // printk("bytesLeftToRead:%d \n",bytesLeftToRead);

            uint8_t data[toget];
            i2c_read(dev->i2c_dev,data,toget,dev->i2c_addr);
            // printk("toget:%d",toget);
            while(toget > 0)
            {
                dev->sense.head++ ;
                dev->sense.head %= STORAGE_SIZE;

                uint8_t temp[sizeof(uint32_t)];
                uint32_t tempLong;

                temp[3] = 0;
                temp[2] = data[0];
                temp[1] = data[1];
                temp[0] = data[2];

                memcpy(&tempLong,temp,sizeof(tempLong));
                tempLong &= 0x3FFFF;
                dev->sense.red[dev->sense.head] = tempLong;

                if(dev->active_leds > 1){
                    temp[3] = 0;
                    temp[2] = data[3];
                    temp[1] = data[4];
                    temp[0] = data[5];

                    memcpy(&tempLong,temp,sizeof(tempLong));
                    tempLong &= 0x3FFFF;
                    dev->sense.ir[dev->sense.head] = tempLong;
                }
                if(dev->active_leds > 2){
                    temp[3] = 0;
                    temp[2] = data[6];
                    temp[1] = data[7];
                    temp[0] = data[8];

                    memcpy(&tempLong,temp,sizeof(tempLong));
                    tempLong &= 0x3FFFF;
                    dev->sense.green[dev->sense.head] = tempLong;
                }
                toget -= dev->active_leds * 3;
                // printk("toget_last:%d \n",toget);
            }
        }
        
    }
    return numberOfSamples;
}
// bool max30101_safe_check(max30101_t *dev, uint8_t max_time_to_check){

//     uint32_t markTime = k_uptime_get_32();
//     while(1){
//         if (k_uptime_get_32() - markTime > max_time_to_check){
//             return false;
//         }

//         if(max30101_check(dev)){
//             return true;
//         }
//         k_msleep(1);
//     }
// }
bool max30101_safe_check(max30101_t *dev, uint8_t max_time_to_check) {
    uint32_t markTime = k_uptime_get_32();
    while (k_uptime_get_32() - markTime < max_time_to_check) {
        uint8_t status = max30101_get_int1(dev);
        if (status & MAX30101_INT_PPG_RDY_ENABLE) {
            return true;
        }
        k_msleep(1); // Sleep to prevent tight loop
    }
    return false;
}

void max30101_bitmask(max30101_t *dev, uint8_t reg, uint8_t mask, uint8_t thing){
    uint8_t originalContents = max30101_read_register8(dev,reg);
    originalContents = originalContents & mask;
    // max30101_write_register8(dev,reg,originalContents|thing);
    max30101_write_register8(dev->i2c_dev,dev->i2c_addr,reg,originalContents|thing);
}

uint8_t max30101_read_register8(max30101_t *dev, uint8_t reg){
    int ret;
    uint8_t data[1] = {0};
    ret = i2c_write_read_dt(dev, &reg, 1, &data, 1);
    // ret = i2c_burst_read(dev->i2c_dev, dev->i2c_addr, reg, data, sizeof(data));

    // printk("data:%d \n", data[0]);
    // printk("reg:%d \n", reg);
    if (ret != 0) {
        printk("Error read to I2C device: %d\n", ret);
    }
    return data[0];

}
void max30101_write_register8(max30101_t *i2c_dev, uint8_t address, uint8_t reg, uint8_t value) {
    int ret;

    // เขียนค่าไปยังเรจิสเตอร์ที่ระบุในเซ็นเซอร์
    ret = i2c_reg_write_byte(i2c_dev, address, reg, value);
    // printk("write \n");
    // printk("value:%d \n",value);
    if (ret != 0) {
        // printk("Error writing to I2C device: %d\n", ret);
    }
}

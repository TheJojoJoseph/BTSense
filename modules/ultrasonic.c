/*
 * Driver for ultrasonic sensors
 * Srinivasa Raju Namburi
 * 2/2/2018
 * MIT license
 */
#include "ultrasonic.h"
#include "include/gpio16.h"
#include <stdio.h>
#include "osapi.h"

#define TRIGGER_LOW_DELAY 4
#define TRIGGER_HIGH_DELAY 10
#define PING_TIMEOUT 6000
#define ROUNDTRIP 58
uint32_t start_time, end_time;
int flag=-1;
#define timeout_expired(start, len) ((uint32_t)(system_get_time() - (start)) >= (len))

void ICACHE_FLASH_ATTR pulseCallback(unsigned pin, unsigned level)
{
    if(level == 1){
        start_time = system_get_time();
        flag=0;
    }
    else{
    	end_time = system_get_time();
    	flag=1;
    }

}
void ICACHE_FLASH_ATTR ultrasoinc_init(const ultrasonic_sensor_t *dev)
{
    set_gpio_mode(dev->trigger_pin, GPIO_OUTPUT,GPIO_PULLDOWN);
    set_gpio_mode(dev->echo_pin, GPIO_INT,GPIO_PULLDOWN);
    gpio_intr_init(dev->echo_pin, GPIO_PIN_INTR_ANYEDGE);
    gpio_intr_attach(pulseCallback);
    gpio_write(dev->trigger_pin, 0);
}

int32_t ICACHE_FLASH_ATTR ultrasoinc_measure_cm(const ultrasonic_sensor_t *dev, uint32_t max_distance)
{
    // Ping: Low for 2..4 us, then high 10 us
    gpio_write(dev->trigger_pin, 0);
    os_delay_us(TRIGGER_LOW_DELAY);
    gpio_write(dev->trigger_pin, 1);
    os_delay_us(TRIGGER_HIGH_DELAY);
    gpio_write(dev->trigger_pin, 0);

    while(flag==-1)
    {
    	if (timeout_expired(start_time, PING_TIMEOUT))
    	       return ULTRASONIC_ERROR_PING_TIMEOUT;
    }
    uint32_t meas_timeout   = start_time + max_distance * ROUNDTRIP;
    while(flag ==0)
    {
    	if (timeout_expired(start_time, meas_timeout))
    		return ULTRASONIC_ERROR_ECHO_TIMEOUT;
    }

    return (end_time - start_time) / ROUNDTRIP;
}

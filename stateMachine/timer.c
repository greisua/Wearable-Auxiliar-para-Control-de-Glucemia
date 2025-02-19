#include "sm.h"
#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include "./../config.h"

#define ALARM_CHANNEL_ID 0
struct counter_alarm_cfg alarm_cfg;
#define TIMER DT_NODELABEL(rtc2) 
const struct device *const counter_dev = DEVICE_DT_GET(TIMER);

void setNewAcqPeriod(uint32_t seconds){
	alarm_cfg.ticks = counter_us_to_ticks(counter_dev, seconds*1e6);
}

static void acquisitionTimer_fn(const struct device *counter_dev,uint8_t chan_id, uint32_t ticks,void *user_data)
{
	struct counter_alarm_cfg *config = user_data;
	uint32_t now_ticks;
	uint64_t now_usec;
	int now_sec;
	int err;

	err = counter_get_value(counter_dev, &now_ticks);
	if (err) {
        #ifdef PRINT_ON
		    printk("Failed to read counter value (err %d)", err);
        #endif
		return;
	}

	now_usec = counter_ticks_to_us(counter_dev, now_ticks);
	now_sec = (int)(now_usec / USEC_PER_SEC);

    timer_fired();

	/* Set a new alarm with a double length duration */
	config->ticks = config->ticks;

	err = counter_set_channel_alarm(counter_dev, ALARM_CHANNEL_ID,user_data);
	if (err != 0) {
        #ifdef PRINT_ON
		    printk("Alarm could not be set\n");
        #endif
	}
}

void timer_init(stateMachine_t * system_cfg){
    if (!device_is_ready(counter_dev)) {
        #ifdef PRINT_ON
        	printk("device not ready.\n");
        #endif
		return;
	}

	counter_start(counter_dev);

	alarm_cfg.flags = 0;
	alarm_cfg.ticks = counter_us_to_ticks(counter_dev, system_cfg->acq_period*1e6);
	alarm_cfg.callback = acquisitionTimer_fn;
	alarm_cfg.user_data = &alarm_cfg;

    int err = counter_set_channel_alarm(counter_dev, ALARM_CHANNEL_ID,&alarm_cfg);  
    #ifdef PRINT_ON
        printk("Set alarm in %u sec (%u ticks)\n",(uint32_t)(counter_ticks_to_us(counter_dev,alarm_cfg.ticks) / USEC_PER_SEC),alarm_cfg.ticks);
    #endif
	if (-EINVAL == err) {
        #ifdef PRINT_ON
		    printk("Alarm settings invalid\n");
        #endif
	} else if (-ENOTSUP == err) {
        #ifdef PRINT_ON
		    printk("Alarm setting request not supported\n");
        #endif
	} else if (err != 0) {
        #ifdef PRINT_ON
		    printk("Error\n");
        #endif
	}   
}
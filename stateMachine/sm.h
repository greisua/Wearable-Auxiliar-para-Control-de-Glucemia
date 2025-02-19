#ifndef _SM_GENERAL_H
#define _SM_GENERAL_H

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <soc.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/pm/pm.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/policy.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/settings/settings.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/device.h>

#include <hal/nrf_gpio.h>

typedef enum {
    STATUS_FAIL,
    STATUS_OK, 
    STATUS_BLE_READY,
} status_t;

/* States for the State Machine */
typedef enum {
    DEV_INIT,       // Device went online for the very first time. 
    DEV_BLE_ADV, 
    DEV_IDLE,
    DEV_ACQ,
    DEV_SAVE,
    DEV_BLE,
} state_t;

/* List of events in the State Machine */
typedef enum {
    EVT_INIT_COMPLETE,
    EVT_NO_CFG_BLE_ADV,
    EVT_BLE_DATA_SENT,
    EVT_ACQ_TIMER,
    EVT_ACQ_COMPLETE,
    EVT_AD_COMPLETE,
    EVT_BNO_COMPLETE,
    EVT_BATT_COMPLETE,
    EVT_SAVE_COMPLETE,
    EVT_BLE_ATTEMPT,
    EVT_BLE_COMPLETE,
    EVT_CANCEL_ADV,
    EVT_SLEEP,
} event_t;

typedef struct {
    state_t currState;
    event_t event;
    state_t nextState;
} stateTransMatrixRow_t;

typedef struct {
    const char * name;
    void (*func)(void);
} stateFunctionRow_t;

typedef struct {
    state_t currState;
    status_t status;  
    uint32_t sm_period;     // Sleep period in ms. 
    uint32_t acq_period;    // Acquisition Period in (s)
    uint32_t acq_i;         // Current Acquisition Iteration (i)
    uint32_t ble_period;    // BLE period (How many acqs per sync event).
    uint32_t ble_mtu_size;  // BLE MTU Size. 
    uint32_t ble_adv_i;     // Current State Machine Iterations. 
    uint32_t ble_adv_max;   // Timeout for advertising events. 
    uint32_t dataSaveSize;  // Size per acq event. 
    uint32_t BI_nFreqs;     // Different Frequency Values.
    uint32_t BI_startF;     // Sweep Start Frequency Value. 
    uint32_t BI_stopF;      // Sweep Stop Frequency Value.
    bool BI_valid;          // Flag to indicate BI is active.  
    bool ble_conn;          // Am I connected to BLE?
    bool acq_flag;          // Am I acquiring?
    bool ble_flag;          // TX Performed. 
    struct bt_conn * active_connection;
} stateMachine_t;

/* Public Methods */
void SM_Event(event_t event);       // Receives a new Event and Prioritizes through the Priority 
void SM_Run(void);
void SM_BLE_Event(struct bt_conn * act_con);

void setNewAcqPeriod(uint32_t seconds);
void timer_init(stateMachine_t * system_cfg);
void timer_fired(void);

void createCfg(void);


#endif
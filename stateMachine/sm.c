#include "sm.h"

#include "./../bluetooth/volum_bluetooth.h"
#include "./../bluetooth/volum_service.h"
#include "./../leds/leds.h"
#include "./../sensor/sensor.h"
#include "./../filesystem/filesystem.h"

#include "./../config.h"

void smInit(void);
void smIdle(void);
void smAcquire(void);
void smSave(void);
void smBLE(void);
void smBLEadv(void);
void smSleep(void);

stateMachine_t system_cfg = {
    .currState = DEV_INIT,
    .status = STATUS_OK,
    .sm_period = SM_DEFAULT_PERIOD,
    .acq_period = 0,
    .acq_i = 0,
    .ble_period = BLE_PERIOD,
    .ble_mtu_size = BLE_MTU_SIZE,
    .ble_flag = false,
    .ble_adv_i = 0,
    .ble_adv_max = 10,
    .dataSaveSize = 0,
    .BI_nFreqs = BIOIMPEDANCE_NUM,
    .BI_valid = false,
    .ble_conn = false,
    .acq_flag = false,
    .active_connection = NULL,
};

acqValues_t temp_acq = {
    .timestamp = 0x00,
    .battery = 0x01,
    .BI_N = 0,
    .BI_mag = NULL,
    .BI_phase = NULL,
    .orientation = NULL,
};

static stateTransMatrixRow_t stateTransMatrix[] = {
    // CURR STATE   // EVENT                 // NEXT STATE
    { DEV_INIT,     EVT_INIT_COMPLETE,       DEV_IDLE       },
    { DEV_INIT,     EVT_NO_CFG_BLE_ADV,      DEV_BLE_ADV    },
    { DEV_BLE_ADV,  EVT_BLE_DATA_SENT,       DEV_IDLE       },
    { DEV_IDLE,     EVT_ACQ_TIMER,           DEV_ACQ        },
    { DEV_ACQ,      EVT_ACQ_COMPLETE,        DEV_SAVE       },
    { DEV_SAVE,     EVT_SAVE_COMPLETE,       DEV_IDLE       },
    { DEV_SAVE,     EVT_BLE_ATTEMPT,         DEV_BLE        },
    { DEV_BLE,      EVT_CANCEL_ADV,          DEV_IDLE       },
    { DEV_BLE,      EVT_BLE_COMPLETE,        DEV_IDLE       },
};

static stateFunctionRow_t stateFunctionA[] = {
    // NAME                 // FUNC
    { "DEV_INIT",           &smInit     },        // Idle state. 
    { "DEV_BLE_ADV",        &smBLEadv   },
    { "DEV_IDLE",           &smIdle     },        // Idle state. 
    { "DEV_ACQ",            &smAcquire  },        // Acquisition state. 
    { "DEV_SAVE",           &smSave     },        // Data saving to flash. 
    { "DEV_BLE",            &smBLE      },        // Data saving to flash. 
};

void SM_BLE_Event(struct bt_conn * act_con){
    if(act_con == NULL){
        system_cfg.ble_conn = false;
        ledOff(LED_BLUE);
    }else{
        system_cfg.ble_conn = true;
        ledOn(LED_BLUE);
    }
    system_cfg.active_connection = act_con;
}

void SM_Event(event_t event)
{
    if(event == EVT_BLE_DATA_SENT){
        system_cfg.ble_flag = true;
        if(system_cfg.currState == DEV_BLE){
            return; 
        }        
    }

    for(int i = 0; i < sizeof(stateTransMatrix)/sizeof(stateTransMatrix[0]); i++) {
        if( stateTransMatrix[i].currState == system_cfg.currState ) {
            if( stateTransMatrix[i].event == event ) {
                // Transition to the next state
                system_cfg.currState =  stateTransMatrix[i].nextState;
                break;
            }
        }
    }
}

void SM_Run(void)
{    
    #ifdef PRINT_ON
        printk("state: %s\n", stateFunctionA[system_cfg.currState].name);
    #endif
    (stateFunctionA[system_cfg.currState].func)();       // Call the function associated with state
}

void smInit(void)
{
    bioimpedance_init();
	initializeFileSystem();
    eraseFlash();
    ledInit();
    orientation_init();
    battery_init();
    button_init();    
    volum_bt_init();
    
    #ifdef PRINT_ON
        printk("Peripheral Init Complete\n");
    #endif

    if(hasConfigInFlash()){
        loadConfig(&system_cfg);        
        temp_acq.BI_N = system_cfg.BI_nFreqs;
        SM_Event(EVT_INIT_COMPLETE);
    }
    else{
        volum_start_adv();
        createCfg();
        SM_Event(EVT_NO_CFG_BLE_ADV);
    }

    /* Timer initialization */
    timer_init(&system_cfg);
}

void smBLEadv(void)
{
    uint8_t * configFile = NULL;

    ledOn(LED_BLUE);
    k_msleep(400);
    ledOff(LED_BLUE);
    system_cfg.ble_flag = false;
    if(system_cfg.ble_conn){
        // There's an active connection...
        uint32_t cfgSize = getConfigSize();
        if(configFile != NULL){
            free(configFile);
        }
	    configFile = calloc(cfgSize, sizeof(uint8_t));
        uint32_t err = getConfig(configFile);
        if(err < 0){
            #ifdef PRINT_ON
                printk("getConfig Error\n");
            #endif
            return; 
        }
        volum_data_send(system_cfg.active_connection, configFile, (uint16_t) cfgSize);
    }
}


void timer_fired(void){
    SM_Event(EVT_ACQ_TIMER);
}

void smIdle(void)
{	
    if(system_cfg.ble_flag){
        system_cfg.ble_flag = false;
        if(system_cfg.active_connection != NULL){
            bt_conn_disconnect(system_cfg.active_connection, 0x13);
        }        
    }

    ledOn(system_cfg.status);
    k_msleep(200);
    ledOff(system_cfg.status);
    k_msleep(10e3);   
}


void smAcquire(void)
{
    free(temp_acq.BI_mag);
    free(temp_acq.BI_phase);
    free(temp_acq.BI_mag);

    temp_acq.BI_mag = calloc(temp_acq.BI_N, sizeof(float));
    temp_acq.BI_phase = calloc(temp_acq.BI_N, sizeof(float));
    temp_acq.orientation = calloc(8, sizeof(float));
    int64_t time_s = k_uptime_get()/1000;
    temp_acq.timestamp = time_s;

    bioimpedance_measure(&temp_acq);
    orientation_measure(&temp_acq);
    battery_measure(&temp_acq);
     
    if(!bioimpedance_up()){
        system_cfg.status = STATUS_FAIL;
    }

    SM_Event(EVT_ACQ_COMPLETE);
}

void smSave(void)
{
    saveData(&temp_acq, system_cfg);
    reset_button_count();   
    system_cfg.acq_i++;

    if(system_cfg.acq_i > system_cfg.ble_period){
        volum_start_adv();
        system_cfg.ble_adv_i = 0;
        SM_Event(EVT_BLE_ATTEMPT);
    }else{
        SM_Event(EVT_SAVE_COMPLETE);
    }

    
}

void smBLE(void)
{
    if(system_cfg.ble_conn)
    {
        uint8_t * dataFile;
        openDataFile();
        uint16_t dataSize = getDataSize();
        uint16_t num_lines = dataSize / system_cfg.dataSaveSize;
        uint16_t lines_per_packet = 0;

        while( ((lines_per_packet+1)*system_cfg.dataSaveSize) < system_cfg.ble_mtu_size ){
            lines_per_packet++;
        }
        uint16_t packet_size = lines_per_packet*system_cfg.dataSaveSize;        
        dataFile = calloc(packet_size, sizeof(uint8_t));        
        uint16_t readSize = 0;
        uint16_t i = 0;
        uint16_t p = 0;

        while(i < num_lines){
            memset(dataFile, 0, sizeof(packet_size));           // Reset the array to 0.             
            readSize = getDataPacket(dataFile, p, packet_size);
            system_cfg.ble_flag = false;
            uint16_t op = volum_data_send(system_cfg.active_connection, dataFile, packet_size);
            if(op == 0){ 
                while(!system_cfg.ble_flag){
                    k_msleep(20);
                }
                p++;
                i = i + lines_per_packet;
            }
            else{
                k_msleep(200);
            }
            #ifdef PRINT_ON
                printk("i = %u / N = %u \n", i, num_lines);
            #endif 
        }

        system_cfg.acq_i = 0;
        SM_Event(EVT_BLE_COMPLETE);
    }
    else if(system_cfg.ble_adv_i > system_cfg.ble_adv_max){
        volum_stop_adv();
        SM_Event(EVT_CANCEL_ADV);
    }
    else{
        system_cfg.ble_adv_i++;
    }
}

void createCfg(void){

    cfgValues_t cfgVolum = {
        .volumID = 0x01,
        .tacq = ACQ_PERIOD,
        .BI_nFreqs = BIOIMPEDANCE_NUM,
        .BI_freqs = NULL
    }; 

    cfgVolum.BI_freqs = (float *) calloc (cfgVolum.BI_nFreqs, sizeof(float));

    if(cfgVolum.BI_freqs != NULL){ 

        bioimpedance_getFreqs(cfgVolum.BI_nFreqs, cfgVolum.BI_freqs);
        cfgVolum.lineSize = 2*sizeof(uint32_t) + 2*cfgVolum.BI_nFreqs*sizeof(float) + 8*sizeof(float);
        
        system_cfg.dataSaveSize = cfgVolum.lineSize;       // Update so that the Information is also locally configured. 
        system_cfg.acq_period = cfgVolum.tacq;
        system_cfg.BI_nFreqs = cfgVolum.BI_nFreqs;
        temp_acq.BI_N = system_cfg.BI_nFreqs;
        
        bool test = true; 
        for(int i = 0; i < system_cfg.BI_nFreqs; i++){
            test &= (cfgVolum.BI_freqs[i] != 0); 
        }
        if(test)
        {
            system_cfg.BI_valid = true;
        }

        createConfig(&cfgVolum);
        free(cfgVolum.BI_freqs);
        #ifdef PRINT_ON
            printk("Configuration File Created\n");
            printk("Data file will have %uB columns.\n", cfgVolum.lineSize);
        #endif
    }else{
        #ifdef PRINT_ON
            printk("smBLEadv: Error allocating pBI_freqs.\n");
        #endif
    }    
}
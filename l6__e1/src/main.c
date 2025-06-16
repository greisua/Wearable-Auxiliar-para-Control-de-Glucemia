#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/sys/printk.h>
#include "bno055.h"
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gap.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/addr.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>

//####################################################################################################################################################################################
//                                                                                          DEFINES
//####################################################################################################################################################################################

////DEFINES PARA INICIALIZAR EL NODO I2C
#define I2C_NODE DT_NODELABEL(mysensor) //accedemos al dispositivo I2C
#define I2C_NODE2 DT_NODELABEL(mysensor2)

#define SLEEP_TIME_MS 2500
#define REG_ID 0x00 //registro donde esta el id del sensor

//defines para el MAX30100
#define RATE_SIZE 4
//// MAX30100 registers
#define REG_MODE_CONFIG        0x06
#define REG_SPO2_CONFIG        0x07
#define REG_LED_CONFIG         0x09
#define REG_FIFO_WR_PTR        0x02
#define REG_OVF_COUNTER        0x03
#define REG_FIFO_RD_PTR        0x04
#define REG_FIFO_DATA          0x05

//defines de los servicios de la comunicacion bluetooth
#define BT_UUID_CUSTOM_SERVICE_VAL \
BT_UUID_128_ENCODE(0x12345678, 0x1234, 0xabcd, 0xef00, 0x1234567890ab)

#define BT_UUID_CUSTOM_CHAR_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0xabcd, 0xef01, 0x1234567890ab)

#define ADV_TIMEOUT_MS 5000  // 5 segundos
#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

//###################################################################################################################################################################################
//                                                                                     FIN DEFINES
//###################################################################################################################################################################################



//###################################################################################################################################################################################
//                                                                                   VARIABLES
//###################################################################################################################################################################################
////variables I2C
static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE); //inicializamos el dispositivo I2C-bno055
static const struct i2c_dt_spec dev_i2c2 = I2C_DT_SPEC_GET(I2C_NODE2); //inicializamos el dispositivo I2C-max30100
///aquitermina
float q[4];
//VARIABLES AUXILIARES PARA LA INICIALIZACION DEL SENSOR
s32 comres = BNO055_ERROR;
u8 power_mode = BNO055_INIT_VALUE;
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void BNO055_delay_msek(u32 msek);

struct bno055_quaternion_t quaternion_wxyz;//estructura que alamcena el quaternion
struct bno055_t bno055; //estructura que almacena el sensor BNO055

// Variables para MAX30100
static uint8_t rates[RATE_SIZE];
static uint8_t rate_index = 0;
static int64_t last_beat = 0;
static float bpm = 0;
static int avg_bpm = 0;

//####################################################################################################################################################################################
//FUNCION PARA INICIALIZAR UN SESNOR BNO055
void BOSCH_Init(void)
{
    bno055.bus_write = BNO055_I2C_bus_write;
    bno055.bus_read = BNO055_I2C_bus_read;
    bno055.delay_msec = BNO055_delay_msek;
    bno055.dev_addr = BNO055_I2C_ADDR1;
    comres = bno055_init(&bno055);
    power_mode = BNO055_POWER_MODE_NORMAL;
    comres += bno055_set_power_mode(power_mode);	
    comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
}

//FUNCION PARA LEER LOS QUATERNIONES
void BOSCH_quaternion_f(float * result)
{   
    comres += bno055_read_quaternion_wxyz(&quaternion_wxyz);
    if(quaternion_wxyz.w < 16384)
    {
      result[0] = (float) quaternion_wxyz.w / 16384;
    }
    else
    {
      result[0] = (float) -(16384 - quaternion_wxyz.w % 16384) / 16384 ;
    }

    if(quaternion_wxyz.x < 16384)
    {
      result[1] = (float) quaternion_wxyz.x / 16384;
    }
    else
    {
      result[1] = (float) -(16384 - quaternion_wxyz.x % 16384) / 16384 ;
    }

    if(quaternion_wxyz.y < 16384)
    {
      result[2] = (float) quaternion_wxyz.y / 16384;
    }
    else
    {
      result[2] = (float) -(16384 - quaternion_wxyz.y % 16384) / 16384 ;
    }
 
    if(quaternion_wxyz.z < 16384)
    {
      result[3] = (float) quaternion_wxyz.z / 16384;
    }
    else
    {
      result[3] = (float) -(16384 - quaternion_wxyz.z % 16384) / 16384 ;
    }
}

//Funcion para inicializar el sensor MAX30100 
int setup_max30100() {
  int ret = 0;

  ret |= i2c_reg_write_byte_dt(&dev_i2c2, REG_MODE_CONFIG, 0x40); // Reset
  k_msleep(100);

  ret |= i2c_reg_write_byte_dt(&dev_i2c2, REG_MODE_CONFIG, 0x03); // SpO2 mode
  ret |= i2c_reg_write_byte_dt(&dev_i2c2, REG_SPO2_CONFIG, 0x27); // 16-bit, 100Hz
  ret |= i2c_reg_write_byte_dt(&dev_i2c2, REG_LED_CONFIG, 0x24);  // IR/RED pulse amp

  // Clear FIFO
  ret |= i2c_reg_write_byte_dt(&dev_i2c2, REG_FIFO_WR_PTR, 0);
  ret |= i2c_reg_write_byte_dt(&dev_i2c2, REG_OVF_COUNTER, 0);
  ret |= i2c_reg_write_byte_dt(&dev_i2c2, REG_FIFO_RD_PTR, 0);

  return ret;
}

//FUNCION RELATIVA A MAX30100
bool read_fifo_sample(uint16_t *ir, uint16_t *red) {
  uint8_t data[4];
  int ret = i2c_burst_read_dt(&dev_i2c2, REG_FIFO_DATA, data, 4);
  if (ret != 0) {
      printk("Fallo leyendo FIFO");
      return false;
  }
  *ir  = ((uint16_t)data[0] << 8) | data[1];
  *red = ((uint16_t)data[2] << 8) | data[3]; // Puedes ignorar si no lo usas
  return true;
}

//FUNCION RELATIVA A MAX30100
bool check_for_beat(uint16_t ir_value) {
  static int32_t ir_avg = 0;
  static int16_t prev = 0, curr = 0;
  static int16_t max = 0, min = 0;

  ir_avg += (((int32_t)ir_value << 15) - ir_avg) >> 4;
  curr = ((int32_t)ir_value - (ir_avg >> 15));

  if ((prev < 0) && (curr >= 0)) {
      int16_t amplitude = max - min;
      max = min = 0;
      if (amplitude > 10 && amplitude < 3000) return true;
  }

  if ((prev > 0) && (curr <= 0)) min = 0;
  if (curr > prev && curr > max) max = curr;
  if (curr < prev && curr < min) min = curr;

  prev = curr;
  return false;
}



//FUNCIONES PARA BLUETOOTH
//strucs y funciones de las cosas que necesitamos para el servicio para el bluetooth
static bool indication_in_progress = false;
bool indic_enabled ;
int i=0;
static struct bt_gatt_indicate_params ind_params;
struct bt_conn *my_conn = NULL;
static int failed_indications = 0;
static struct k_work adv_work;
static int indications = 0;
static bool int_adv = false;

static void indication_complete(struct bt_conn *conn,
				struct bt_gatt_indicate_params *params,
				uint8_t err)
{
	printk("Indication %s\n", err ? "failed" : "successful");
  indication_in_progress = false;
}

static ssize_t read_callback(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			     void *buf, uint16_t len, uint16_t offset)
{
	const char *value = attr->user_data;
	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, strlen(value));
}

static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	indic_enabled = (value == BT_GATT_CCC_INDICATE);
	printk("Indications %s\n", indic_enabled ? "enabled" : "disabled");
}

static char my_data[20] = "Hola BLE desde NRF!";

// Definición del servicio GATT
BT_GATT_SERVICE_DEFINE(custom_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(BT_UUID_CUSTOM_SERVICE_VAL)),
	BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(BT_UUID_CUSTOM_CHAR_VAL),
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_INDICATE,
			       BT_GATT_PERM_READ,
			       read_callback, NULL, my_data),
	BT_GATT_CCC(ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);
/////////////////////////////////////////////////////////////////////////////////////
//cosas para mandar una indicacion
LOG_MODULE_REGISTER(Less4_Exer2, LOG_LEVEL_INF);

void send_indication_if_connected(void)
{
	if (!my_conn) {//current_conn
		printk("No client connected\n");
		return;
	}
  if (!indic_enabled) {
		printk("Indications not enabled by client\n");
		return;
	}

	if (indication_in_progress) {
		printk("Waiting for previous indication to complete\n");
		return;
	}

    static uint8_t value[10]; 

    for(int i=0; i<4;i++){
      int16_t q_int = (int16_t)(q[i] * 16384); // Convertir a entero de 16 bits
      sys_put_le16(q_int, &value[i * 2]); // Almacenar en el buffer
    }

    int16_t bpm_int = (int16_t)(bpm * 16384); // Convertir a entero de 16 bits
      sys_put_le16(bpm_int, &value[8]); // Almacenar en el buffer
    ind_params.attr = &custom_svc.attrs[1]; 
    ind_params.func = indication_complete;
    ind_params.data = &value;
    ind_params.len = sizeof(value);

    indication_in_progress = true; // Marcar que la indicación está en curso


	int err = bt_gatt_indicate(my_conn, &ind_params);
	if (err) {
		printk("Indication failed: err %d\n", err);
    failed_indications++;
    if(failed_indications == 4){
      bt_conn_disconnect(my_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
    }
	}else {
    printk("INDICATION SENT: q = [%f,%f,%f,%f]\n",q[0],q[1],q[2],q[3]);
    failed_indications = 0;
  }
}

/*
void send_indication_if_connected(void)
{
	if (!my_conn) {
		printk("No client connected\n");
		return;
	}
  if (!indic_enabled) {
		printk("Indications not enabled by client\n");
		return;
	}
  if (indication_in_progress) {
		printk("Waiting for previous indication to complete\n");
		return;
	}

	// Buffer de 10 bytes: 2 BPM + 4 x 2 bytes quaternion
	static uint8_t payload[10];

	// 1. Añadir el BPM
	uint16_t bpm_u16 = (uint16_t)avg_bpm;
	sys_put_le16(bpm_u16, &payload[0]);

	// 2. Leer quaternion directamente del sensor
	comres += bno055_read_quaternion_wxyz(&quaternion_wxyz);

	sys_put_le16((int16_t)quaternion_wxyz.w, &payload[2]);
	sys_put_le16((int16_t)quaternion_wxyz.x, &payload[4]);
	sys_put_le16((int16_t)quaternion_wxyz.y, &payload[6]);
	sys_put_le16((int16_t)quaternion_wxyz.z, &payload[8]);

	// 3. Configurar la indicación
	ind_params.attr = &custom_svc.attrs[1];  // Característica GATT con INDICATE
	ind_params.func = indication_complete;
	ind_params.data = payload;
	ind_params.len = sizeof(payload);

	int err = bt_gatt_indicate(my_conn, &ind_params);
	if (err) {
		printk("Indication failed: err %d\n", err);
	} else {
		printk("Indication sent: BPM=%d W=%d X=%d Y=%d Z=%d\n", bpm_u16,
		       quaternion_wxyz.w, quaternion_wxyz.x,
		       quaternion_wxyz.y, quaternion_wxyz.z);
	}
}*/

//funcion para hacer saltar el timer
static void indicate_timer_handler(struct k_timer *timer_id)
{
	send_indication_if_connected();  // Tu función que envía la indicación
}
//definicion del timer de la indicacion
K_TIMER_DEFINE(indicate_timer, indicate_timer_handler, NULL);





//FUNCIONES PARA BLUETOOTH




static const struct bt_le_adv_param adv_param = BT_LE_ADV_PARAM_INIT(
	(BT_LE_ADV_OPT_CONNECTABLE |
	 BT_LE_ADV_OPT_USE_IDENTITY), /* Connectable advertising and use identity address */
	 BT_GAP_ADV_FAST_INT_MIN_1, /* Min Advertising Interval 500ms (800*0.625ms) */
   BT_GAP_ADV_FAST_INT_MAX_1, /* Max Advertising Interval 500.625ms (801*0.625ms) */
	 NULL); /* Set to NULL for undirected advertising */




static const struct bt_data ad[] = {
	/* STEP 3.1 - Set the flags and populate the device name in the advertising packet */
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),

};

static const struct bt_data sd[] = {
	/* STEP 3.2.2 - Include the 16-bytes (128-Bits) UUID of the LBS service in the scan response packet */
	BT_DATA_BYTES(BT_DATA_UUID128_ALL,
		      BT_UUID_128_ENCODE(0x00001523, 0x1212, 0xefde, 0x1523, 0x785feabcd123)),
};


//PARA REINICIAR EL ADVERTISING
static void adv_work_handler(struct k_work *work)
{

	int err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

	if (err) {
		LOG_INF("Advertising failed to start (err %d)\n", err);
		return;
	}

  
  
	LOG_INF("Advertising successfully started\n");
  
  indications++;
}

static void advertising_start(void)
{
	k_work_submit(&adv_work);
}


//FUNCIONES DE CALLBACK
void on_connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_INF("Connection error %d", err);
		return;
	}
	LOG_INF("#############Connected");
  
	my_conn = bt_conn_ref(conn);
  

	/* STEP 3.2  Turn the connection status LED on */
}

void on_disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("Disconnected. Reason %d", reason);
	bt_conn_unref(my_conn);
  my_conn=NULL;
  advertising_start();

	/* STEP 3.3  Turn the connection status LED off */
}

void on_recycled(void)
{
}


struct bt_conn_cb connection_callbacks = {
	.connected              = on_connected,
	.disconnected           = on_disconnected,
	.recycled               = on_recycled,
};

/*BT_CONN_CB_DEFINE(conn_callbacks) = {
	.recycled = recycled_cb,
};*/
//FUNCIONES PARA SENSOR MAX30100











int main(void)
{
  int err;
  /*bt_addr_le_t addr;
	err = bt_addr_le_from_str("FF:EE:DD:CC:BB:AA", "random", &addr);
	if (err) {
		printk("Invalid BT address (err %d)\n", err);
	}

	err = bt_id_create(&addr, NULL);
	if (err < 0) {
		printk("Creating new ID failed (err %d)\n", err);
	}
*/

  bt_conn_cb_register(&connection_callbacks);


	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return -1;
	}

	printk("Bluetooth initialized\n");
	/* STEP 5.3 - Start connectable advertising */
  
	k_work_init(&adv_work, adv_work_handler);
	advertising_start();

  k_timer_start(&indicate_timer, K_SECONDS(5), K_SECONDS(5));

	printk("Advertising successfully started\n");



  if (!device_is_ready(dev_i2c2.bus)) {
    printk("I2C no listo");
    return;
}

if (setup_max30100() != 0) {
    printk("Error configurando MAX30100");
    return;
}

printk("Coloca el dedo en el sensor\n");


  

	BOSCH_Init();
  
  

	while (1) {

    if(my_conn){
      indications=0;
    }else if(!my_conn ){
      if(indications < 5){
        indications++;
      }else{
        bt_le_adv_stop();
        printk("No hay ningun cliente conectado, reiniciando advertising\n");
        k_msleep(7000);
        indications = 0;
        advertising_start();
      }
    }

    uint16_t ir = 0, red = 0;
        if (read_fifo_sample(&ir, &red)) {
          printk("Aqui entra1\n");
            //if (check_for_beat(ir)) {
                int64_t now = k_uptime_get();
                int64_t dt = now - last_beat;
                last_beat = now;
                printk("Aqui entra\n");
                bpm = 60.0 / (dt / 1000.0);
                if (bpm > 5 && bpm < 3000) {
                    printk("Aqui tambien\n");
                    rates[rate_index++] = (uint8_t)bpm;
                    rate_index %= RATE_SIZE;
                    avg_bpm = 0;
                    for (int i = 0; i < RATE_SIZE; i++) avg_bpm += rates[i];
                    avg_bpm /= RATE_SIZE;
                    printk("BPM: %.1f\n", avg_bpm);
                }
            //}
        }


		BOSCH_quaternion_f(q);
    
		printk("#### Quaternion #####\n");
		printk("w:%f, x:%f, y:%f, z:%f\n", q[0],q[1],q[2],q[3]);

		k_msleep(SLEEP_TIME_MS);
	}
}


s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
	//printk("Leyendo reg 0x%02X, cnt=%d\n", reg_addr, cnt);
    s32 BNO055_iERROR = BNO055_INIT_VALUE;
	//printk("Resultado: %d - Valor leído: 0x%02X\n", BNO055_iERROR, reg_data[0]);
    u8 array[8] = {BNO055_INIT_VALUE};

    array[BNO055_INIT_VALUE] = reg_addr;    

    BNO055_iERROR = i2c_write_read(dev_i2c.bus, dev_addr, &reg_addr, 1, reg_data, cnt);
    
    k_msleep(50);
    
    return (s8) BNO055_iERROR;
}



s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 BNO055_iERROR = BNO055_INIT_VALUE;
    u8 array[8];
    u8 stringpos = BNO055_INIT_VALUE;

    array[0] = reg_addr;
    for (stringpos = 0; stringpos < cnt; stringpos++) {
        array[stringpos + 1] = reg_data[stringpos];
    }

    BNO055_iERROR = i2c_write(dev_i2c.bus, array, cnt + 1, dev_addr);
    k_msleep(10);  // Redúcelo a 10ms para agilidad

    return (s8)BNO055_iERROR;
}


void BNO055_delay_msek(u32 msek)
{
    k_msleep(msek);
}
/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
/* STEP 3 - Include the header file of the I2C API */
#include <zephyr/drivers/i2c.h>
/* STEP 4.1 - Include the header file of printk() */
#include <zephyr/sys/printk.h>
#include "bno055.h"
#include <zephyr/drivers/sensor.h>/* 1000 msec = 1 sec */

#define SLEEP_TIME_MS 2500

/* STEP 8 - Define the I2C slave device address and the addresses of relevant registers */

#define REG_ID 0x00 //registro donde esta el id del sensor
#define I2C_NODE DT_NODELABEL(mysensor) //accedemos al dispositivo I2C
#define I2C_NODE2 DT_NODELABEL(max) //accedemos al dispositivo I2C

static const struct i2c_dt_spec dev_i2c = I2C_DT_SPEC_GET(I2C_NODE); //inicializamos el dispositivo I2C
//static const struct i2c_dt_spec dev_i2c2 = I2C_DT_SPEC_GET(I2C_NODE2); //inicializamos el dispositivo I2C
const struct device *max30101 = DEVICE_DT_GET(I2C_NODE2);

#define SAMPLE_FREQ_HZ 100                        // Debe coincidir con CONFIG_MAX30101_SR
#define WINDOW_SEC     5
#define BUF_SIZE       (SAMPLE_FREQ_HZ * WINDOW_SEC)

static int32_t ir_buf[BUF_SIZE];
static int      buf_idx       = 0;
static int      last_peak_idx = -1;



float q[4];
//VARIABLES AUXILIARES PARA LA INICIALIZACION DEL SENSOR
s32 comres = BNO055_ERROR;
u8 power_mode = BNO055_INIT_VALUE;
s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void BNO055_delay_msek(u32 msek);

struct bno055_quaternion_t quaternion_wxyz;

struct bno055_t bno055; //estructura que almacena el sensor BNO055

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



int main(void)
{


    
	/*
    uint8_t reg = REG_ID;       // Dirección del registro del ID
    uint8_t id_val = 0x00;    // Aquí guardamos el resultado

    // Paso 1: Escribir dirección del registro
    int ret = i2c_write_dt(&dev_i2c, &reg, 1);
    if (ret != 0) {
        printk("Error escribiendo dirección del registro: %d\n", ret);
        return;
    }

    // Paso 2: Leer el valor del registro
    ret = i2c_read_dt(&dev_i2c, &id_val, 1);
    if (ret != 0) {
        printk("Error leyendo el registro: %d\n", ret);
        return;
    }

    printk("ID del sensor (reg 0x00): 0x%02X\n", id_val);
	*/

	/*uint8_t reg = 0xFF;       // Dirección del registro del ID
    uint8_t id_val = 0x00;    // Aquí guardamos el resultado

    // Paso 1: Escribir dirección del registro
    int ret = i2c_write_dt(&dev_i2c2, &reg, 1);
    if (ret != 0) {
        printk("Error escribiendo dirección del registro: %d\n", ret);
        return;
    }

    // Paso 2: Leer el valor del registro
    ret = i2c_read_dt(&dev_i2c2, &id_val, 1);
    if (ret != 0) {
        printk("Error leyendo el registro: %d\n", ret);
        return;
    }

    printk("ID del sensor (reg 0xFF): 0x%02X\n", id_val);*/

	BOSCH_Init();
  
  if (!device_is_ready(max30101)) {
    printk("ERROR: MAX30101 no está listo (device_is_ready = false)\n");
    //return -ENODEV;
}

	while (1) {

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
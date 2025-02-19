#include "sensor.h"
#include "bno055.h"
#include <zephyr/pm/device_runtime.h>

#define IMU_ENABLE DT_ALIAS(imuenable)
#define IMU_NBOOT DT_ALIAS(imubl)
#define I2C_DEV_NODE DT_NODELABEL(i2c0)

const struct device *const i2c_dev = DEVICE_DT_GET(I2C_DEV_NODE);
const struct gpio_dt_spec imu_enable_pin = GPIO_DT_SPEC_GET(IMU_ENABLE, gpios);
const struct gpio_dt_spec imu_nboot_pin = GPIO_DT_SPEC_GET(IMU_NBOOT, gpios);

uint8_t device_address = 0;
bool device_found = false; 
uint8_t p_data_tx, p_data_rx, p_data_tx2;
uint8_t rxbuf01, rxbuf02;
uint32_t err_code;
int8_t t_raw;
float temperature;
float acc_x, acc_y, acc_z;
float mag_x, mag_y, mag_z;
float temp_acc = 0.0f;
float temp_mag = 0.0f;

static float QUATERNION[4];
static float Q_VARIANCE[4];
static float Q_MEAN[4];

#define BNO_STAT_N 10
static float QUATERNION_ARRAY[BNO_STAT_N*4];

/* BOSCH VARIABLES */
/* Variable used to return value of
communication routine*/
s32 comres = BNO055_ERROR;
/* variable used to set the power mode of the sensor*/
u8 power_mode = BNO055_INIT_VALUE;
/*********read raw accel data***********/
/* variable used to read the accel x data */
s16 accel_datax = BNO055_INIT_VALUE;
 /* variable used to read the accel y data */
s16 accel_datay = BNO055_INIT_VALUE;
/* variable used to read the accel z data */
s16 accel_dataz = BNO055_INIT_VALUE;
/* variable used to read the accel xyz data */
struct bno055_accel_t accel_xyz;

/*********read raw mag data***********/
/* variable used to read the mag x data */
s16 mag_datax  = BNO055_INIT_VALUE;
/* variable used to read the mag y data */
s16 mag_datay  = BNO055_INIT_VALUE;
/* variable used to read the mag z data */
s16 mag_dataz  = BNO055_INIT_VALUE;
/* structure used to read the mag xyz data */
struct bno055_mag_t mag_xyz;

/***********read raw gyro data***********/
/* variable used to read the gyro x data */
s16 gyro_datax = BNO055_INIT_VALUE;
/* variable used to read the gyro y data */
s16 gyro_datay = BNO055_INIT_VALUE;
 /* variable used to read the gyro z data */
s16 gyro_dataz = BNO055_INIT_VALUE;
 /* structure used to read the gyro xyz data */
struct bno055_gyro_t gyro_xyz;

/*************read raw Euler data************/
/* variable used to read the euler h data */
s16 euler_data_h = BNO055_INIT_VALUE;
 /* variable used to read the euler r data */
s16 euler_data_r = BNO055_INIT_VALUE;
/* variable used to read the euler p data */
s16 euler_data_p = BNO055_INIT_VALUE;
/* structure used to read the euler hrp data */
struct bno055_euler_t euler_hrp;

/************read raw quaternion data**************/
/* variable used to read the quaternion w data */
s16 quaternion_data_w = BNO055_INIT_VALUE;
/* variable used to read the quaternion x data */
s16 quaternion_data_x = BNO055_INIT_VALUE;
/* variable used to read the quaternion y data */
s16 quaternion_data_y = BNO055_INIT_VALUE;
/* variable used to read the quaternion z data */
s16 quaternion_data_z = BNO055_INIT_VALUE;
/* structure used to read the quaternion wxyz data */
struct bno055_quaternion_t quaternion_wxyz;

/************read raw linear acceleration data***********/
/* variable used to read the linear accel x data */
s16 linear_accel_data_x = BNO055_INIT_VALUE;
/* variable used to read the linear accel y data */
s16 linear_accel_data_y = BNO055_INIT_VALUE;
/* variable used to read the linear accel z data */
s16 linear_accel_data_z = BNO055_INIT_VALUE;
/* structure used to read the linear accel xyz data */
struct bno055_linear_accel_t linear_acce_xyz;

/*****************read raw gravity sensor data****************/
/* variable used to read the gravity x data */
s16 gravity_data_x = BNO055_INIT_VALUE;
/* variable used to read the gravity y data */
s16 gravity_data_y = BNO055_INIT_VALUE;
/* variable used to read the gravity z data */
s16 gravity_data_z = BNO055_INIT_VALUE;
/* structure used to read the gravity xyz data */
struct bno055_gravity_t gravity_xyz;

/*************read accel converted data***************/
/* variable used to read the accel x data output as m/s2 or mg */
double d_accel_datax = BNO055_INIT_VALUE;
/* variable used to read the accel y data output as m/s2 or mg */
double d_accel_datay = BNO055_INIT_VALUE;
/* variable used to read the accel z data output as m/s2 or mg */
double d_accel_dataz = BNO055_INIT_VALUE;
/* structure used to read the accel xyz data output as m/s2 or mg */
struct bno055_accel_double_t d_accel_xyz;

/******************read mag converted data********************/
/* variable used to read the mag x data output as uT*/
double d_mag_datax = BNO055_INIT_VALUE;
/* variable used to read the mag y data output as uT*/
double d_mag_datay = BNO055_INIT_VALUE;
/* variable used to read the mag z data output as uT*/
double d_mag_dataz = BNO055_INIT_VALUE;
/* structure used to read the mag xyz data output as uT*/
struct bno055_mag_double_t d_mag_xyz;

/*****************read gyro converted data************************/
/* variable used to read the gyro x data output as dps or rps */
double d_gyro_datax = BNO055_INIT_VALUE;
/* variable used to read the gyro y data output as dps or rps */
double d_gyro_datay = BNO055_INIT_VALUE;
/* variable used to read the gyro z data output as dps or rps */
double d_gyro_dataz = BNO055_INIT_VALUE;
/* structure used to read the gyro xyz data output as dps or rps */
struct bno055_gyro_double_t d_gyro_xyz;

/*******************read euler converted data*******************/
/* variable used to read the euler h data output
as degree or radians*/
double d_euler_data_h = BNO055_INIT_VALUE;
/* variable used to read the euler r data output
as degree or radians*/
double d_euler_data_r = BNO055_INIT_VALUE;
/* variable used to read the euler p data output
as degree or radians*/
double d_euler_data_p = BNO055_INIT_VALUE;
/* structure used to read the euler hrp data output
as as degree or radians */
struct bno055_euler_double_t d_euler_hpr;

/*********read linear acceleration converted data**********/
/* variable used to read the linear accel x data output as m/s2*/
double d_linear_accel_datax = BNO055_INIT_VALUE;
/* variable used to read the linear accel y data output as m/s2*/
double d_linear_accel_datay = BNO055_INIT_VALUE;
/* variable used to read the linear accel z data output as m/s2*/
double d_linear_accel_dataz = BNO055_INIT_VALUE;
/* structure used to read the linear accel xyz data output as m/s2*/
struct bno055_linear_accel_double_t d_linear_accel_xyz;

/********************Gravity converted data**********************/
/* variable used to read the gravity sensor x data output as m/s2*/
double d_gravity_data_x = BNO055_INIT_VALUE;
/* variable used to read the gravity sensor y data output as m/s2*/
double d_gravity_data_y = BNO055_INIT_VALUE;
/* variable used to read the gravity sensor z data output as m/s2*/
double d_gravity_data_z = BNO055_INIT_VALUE;
/* structure used to read the gravity xyz data output as m/s2*/
struct bno055_gravity_double_t d_gravity_xyz;		

s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void BNO055_delay_msek(u32 msek);
s32 bno055_data_readout_template(void);

struct bno055_t bno055;


/* =================================== <START> SENSOR Section =================================== */
void BOSCH_Init(void)
{
    bno055.bus_write = BNO055_I2C_bus_write;
    bno055.bus_read = BNO055_I2C_bus_read;
    bno055.delay_msec = BNO055_delay_msek;
    bno055.dev_addr = BNO055_I2C_ADDR1;

    comres = bno055_init(&bno055);
    power_mode = BNO055_POWER_MODE_NORMAL;
    /* set the power mode as NORMAL*/
    comres += bno055_set_power_mode(power_mode);	
    comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
}

void BOSCH_suspend(void)
{
  comres = bno055_init(&bno055);
  comres += bno055_set_power_mode(BNO055_POWER_MODE_SUSPEND);	
  k_msleep(10);

}

void BOSCH_resume(void)
{
  comres = bno055_init(&bno055);
  power_mode = BNO055_POWER_MODE_NORMAL;
  /* set the power mode as NORMAL*/
  comres += bno055_set_power_mode(power_mode);	
  comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);
}

void BOSCH_pause(void)
{
  comres = bno055_init(&bno055);
  comres += bno055_set_power_mode(BNO055_POWER_MODE_SUSPEND);	
}

void BOSCH_quaternion(uint16_t * result)
{   
    comres += bno055_read_quaternion_wxyz(&quaternion_wxyz);
    result[0] = quaternion_wxyz.w;
    result[1] = quaternion_wxyz.x;
    result[2] = quaternion_wxyz.y;
    result[3] = quaternion_wxyz.z;
    
    printf("Quaternion: W = %d, X = %d, Y = %d, Z = %d \n", result[0], result[1], result[2], result[3]);
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

void BOSCH_Wilderness(void)
{
    comres = bno055_init(&bno055);
    power_mode = BNO055_POWER_MODE_NORMAL;
    /* set the power mode as NORMAL*/
    comres += bno055_set_power_mode(power_mode);	
    
    comres += bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);   
    comres += bno055_read_quaternion_wxyz(&quaternion_wxyz);
    power_mode = BNO055_POWER_MODE_SUSPEND;
    /* set the power mode as SUSPEND*/
    comres += bno055_set_power_mode(power_mode);
}

s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 BNO055_iERROR = BNO055_INIT_VALUE;
    u8 array[8] = {BNO055_INIT_VALUE};

    array[BNO055_INIT_VALUE] = reg_addr;    

    BNO055_iERROR = i2c_write_read(i2c_dev, dev_addr, &reg_addr, 1, reg_data, cnt);
    
    k_msleep(50);
    
    return (s8) BNO055_iERROR;
}

s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    s32 BNO055_iERROR = BNO055_INIT_VALUE;
    u8 array[8];
    u8 stringpos = BNO055_INIT_VALUE;

    array[BNO055_INIT_VALUE] = reg_addr;
    for (stringpos = BNO055_INIT_VALUE; stringpos < cnt; stringpos++)
    {
            array[stringpos + 1] = *(reg_data + stringpos);
    }

    BNO055_iERROR = i2c_write(i2c_dev, &array[0], cnt+1, dev_addr);
    k_msleep(50);

    BNO055_iERROR = i2c_write(i2c_dev, &array[0], cnt+1, dev_addr);
    k_msleep(50);
    
    return (s8)BNO055_iERROR;	
}

void BNO055_delay_msek(u32 msek)
{
    k_msleep(msek);
}

s32 bno055_data_readout_template(void)
{
    return (s8)0;	
}

void computeQuaternionStats(void){
  Q_MEAN[0] = 0; 
  Q_MEAN[1] = 0;
  Q_MEAN[2] = 0;
  Q_MEAN[3] = 0;

  Q_VARIANCE[0] = 0;
  Q_VARIANCE[1] = 0;
  Q_VARIANCE[2] = 0;
  Q_VARIANCE[3] = 0;

  int i = 0;
  while(i< BNO_STAT_N)
  {  
    Q_MEAN[0] += QUATERNION_ARRAY[4*i];
    Q_MEAN[1] += QUATERNION_ARRAY[4*i+1];
    Q_MEAN[2] += QUATERNION_ARRAY[4*i+2];
    Q_MEAN[3] += QUATERNION_ARRAY[4*i+3];
    i++;
  }

  Q_MEAN[0] = (Q_MEAN[0] / BNO_STAT_N);
  Q_MEAN[1] = (Q_MEAN[1] / BNO_STAT_N);
  Q_MEAN[2] = (Q_MEAN[2] / BNO_STAT_N);
  Q_MEAN[3] = (Q_MEAN[3] / BNO_STAT_N);

  /* Mean value is stored in QUATERNION, we can compute VAR now. */
  i = 0;
  while(i< BNO_STAT_N)
  {  
    float qv_w = (QUATERNION_ARRAY[4*i+0] - Q_MEAN[0]);
    float qv_x = (QUATERNION_ARRAY[4*i+1] - Q_MEAN[1]);
    float qv_y = (QUATERNION_ARRAY[4*i+2] - Q_MEAN[2]);
    float qv_z = (QUATERNION_ARRAY[4*i+3] - Q_MEAN[3]);
    
    Q_VARIANCE[0] += qv_w*qv_w;
    Q_VARIANCE[1] += qv_x*qv_x;
    Q_VARIANCE[2] += qv_y*qv_y;
    Q_VARIANCE[3] += qv_z*qv_z;
    i++;
  }

  Q_VARIANCE[0] = (Q_VARIANCE[0] / BNO_STAT_N);
  Q_VARIANCE[1] = (Q_VARIANCE[1] / BNO_STAT_N);
  Q_VARIANCE[2] = (Q_VARIANCE[2] / BNO_STAT_N);
  Q_VARIANCE[3] = (Q_VARIANCE[3] / BNO_STAT_N);

}
void BNO055_analysis(void)
/* Initially we will evaluate 5 seconds (10 measurements. */
{       
  int i = 0;
  BOSCH_resume();
  k_msleep(10);
  while(i< BNO_STAT_N)
  {
    BOSCH_quaternion_f( (float *) &QUATERNION);

    memcpy(&QUATERNION_ARRAY[4*i], &QUATERNION, sizeof(QUATERNION));  
    i++;
    // Saltar la espera tras la ltima medida. 
    if(i < BNO_STAT_N)
    {
      k_msleep(100);
    }
  }        

  BOSCH_suspend(); 
  k_msleep(100);

  computeQuaternionStats();
 }

/* BOSCH SENSOR */

static void ORI_powerOn(void){
  int ret = 0;
  
  if (!device_is_ready(imu_enable_pin.port)) {printk("Error IMU Enable Pin not ready... \n");return;}
  ret = gpio_pin_configure_dt(&imu_enable_pin, GPIO_OUTPUT_INACTIVE);
  if (ret < 0) {printk("Error Configuring IMU Enable Pin... \n");return;}
  
  if (!device_is_ready(imu_nboot_pin.port)) {printk("Error IMU BL Pin not ready... \n");return;}
  ret = gpio_pin_configure_dt(&imu_nboot_pin, GPIO_OUTPUT_INACTIVE);
  if (ret < 0) {printk("Error Configuring IMU BL Pin... \n");return;}

  ret = gpio_pin_set_dt(&imu_enable_pin, 1);		
  if (ret < 0) {printk("Error IMU Enable Pin... \n");return;}
  k_msleep(500);

  /* "get" device (increases usage count, resumes device if suspended) */
  ret = pm_device_runtime_get(i2c_dev);
  if (ret < 0) {
      return;
  }
}

static void ORI_powerOff(void){
  int ret = gpio_pin_set_dt(&imu_enable_pin, 0);		
  if (ret < 0) {printk("Error IMU Enable Pin... \n");return;}

  /* Deconfigure GPIO Pin... */
  ret = gpio_pin_configure_dt(&imu_enable_pin, GPIO_DISCONNECTED);
  if (ret < 0) {printk("Error Configuring IMU Enable Pin... \n");return;}
  ret = gpio_pin_configure_dt(&imu_nboot_pin, GPIO_DISCONNECTED);
  if (ret < 0) {printk("Error Configuring IMU BL Pin... \n");return;}
}

void orientation_init(void){
  ORI_powerOn();  
  BOSCH_Init();
  k_msleep(10);
  BOSCH_suspend();
  ORI_powerOff();
}

void orientation_measure(acqValues_t * tempacq){	
  ORI_powerOn();  

  BNO055_analysis();

  tempacq->orientation[0] = QUATERNION[0];
  tempacq->orientation[1] = QUATERNION[1];
  tempacq->orientation[2] = QUATERNION[2];
  tempacq->orientation[3] = QUATERNION[3];

  tempacq->orientation[4] = Q_VARIANCE[0];
  tempacq->orientation[5] = Q_VARIANCE[1];
  tempacq->orientation[6] = Q_VARIANCE[2];
  tempacq->orientation[7] = Q_VARIANCE[3];

  ORI_powerOff();
}

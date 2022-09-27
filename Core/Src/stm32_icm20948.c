/*************************************************************************
  Includes
*************************************************************************/

#include "stm32_icm20948.h"
#include "usart.h"
#include "i2c.h"
#include "dwt_delay.h"

// InvenSense drivers and utils
#include "Icm20948.h"
#include "SensorTypes.h"
#include "Icm20948MPUFifoControl.h"

/*************************************************************************
  Variables
*************************************************************************/

int chipSelectPin = 10;
int spiSpeed = 7000000;

float gyro_x, gyro_y, gyro_z;
inv_bool_t gyro_data_ready = false;

float accel_x, accel_y, accel_z;
inv_bool_t accel_data_ready = false;

float mag_x, mag_y, mag_z;
inv_bool_t mag_data_ready = false;

float quat_w, quat_x, quat_y, quat_z;
inv_bool_t quat_data_ready = false;

/*************************************************************************
  Invensense Variables
*************************************************************************/

static const uint8_t dmp3_image[] = {
#include "icm20948_img.dmp3a.h"
};

/*
* Just a handy variable to handle the icm20948 object
*/
inv_icm20948_t icm_device;

static const uint8_t EXPECTED_WHOAMI[] = { 0xEA }; /* WHOAMI value for ICM20948 or derivative */
static int unscaled_bias[THREE_AXES * 2];

/* FSR configurations */
int32_t cfg_acc_fsr = 4; // Default = +/- 4g. Valid ranges: 2, 4, 8, 16
int32_t cfg_gyr_fsr = 2000; // Default = +/- 2000dps. Valid ranges: 250, 500, 1000, 2000

/*
* Mounting matrix configuration applied for Accel, Gyro and Mag
*/
static const float cfg_mounting_matrix[9]= {
    1.f, 0, 0,
    0, 1.f, 0,
    0, 0, 1.f
};

static uint8_t convert_to_generic_ids[INV_ICM20948_SENSOR_MAX] = {
    INV_SENSOR_TYPE_ACCELEROMETER,
    INV_SENSOR_TYPE_GYROSCOPE,
    INV_SENSOR_TYPE_RAW_ACCELEROMETER,
    INV_SENSOR_TYPE_RAW_GYROSCOPE,
    INV_SENSOR_TYPE_UNCAL_MAGNETOMETER,
    INV_SENSOR_TYPE_UNCAL_GYROSCOPE,
    INV_SENSOR_TYPE_BAC,
    INV_SENSOR_TYPE_STEP_DETECTOR,
    INV_SENSOR_TYPE_STEP_COUNTER,
    INV_SENSOR_TYPE_GAME_ROTATION_VECTOR,
    INV_SENSOR_TYPE_ROTATION_VECTOR,
    INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR,
    INV_SENSOR_TYPE_MAGNETOMETER,
    INV_SENSOR_TYPE_SMD,
    INV_SENSOR_TYPE_PICK_UP_GESTURE,
    INV_SENSOR_TYPE_TILT_DETECTOR,
    INV_SENSOR_TYPE_GRAVITY,
    INV_SENSOR_TYPE_LINEAR_ACCELERATION,
    INV_SENSOR_TYPE_ORIENTATION,
    INV_SENSOR_TYPE_B2S
};

/*************************************************************************
  HAL Functions for STM32
*************************************************************************/

int idd_io_hal_read_reg(void *context, uint8_t reg, uint8_t *rbuffer, uint32_t rlen)
{
    (void)context;

    HAL_I2C_Mem_Read(&ICM20948_I2C, ICM_I2C_ADDR_REVA << 1, reg, I2C_MEMADD_SIZE_8BIT, rbuffer, rlen, HAL_MAX_DELAY);

    return 0;
}

int idd_io_hal_write_reg(void *context, uint8_t reg, const uint8_t *wbuffer, uint32_t wlen)
{
    (void)context;

    HAL_I2C_Mem_Write(&ICM20948_I2C, ICM_I2C_ADDR_REVA << 1, reg, I2C_MEMADD_SIZE_8BIT, wbuffer, wlen, HAL_MAX_DELAY);

    return 0;
}

inv_bool_t interface_is_SPI(void)
{
    return false;
}

/*************************************************************************
  Invensense Functions
*************************************************************************/

void check_rc(int rc, const char *msg_context)
{
    if (rc < 0) {
        HAL_UART_Transmit(&ICM20948_UART, (uint8_t *)"ICM20948 ERROR!", 16, 100);
        while (1) ;
    }
}

void inv_icm20948_sleep(int ms)
{
    HAL_Delay(ms);
}

void inv_icm20948_sleep_us(int us)
{
    DWT_Delay(us);
}

uint64_t inv_icm20948_get_time_us(void)
{
    return HAL_GetTick();
}

int load_dmp3(void)
{
    int rc = 0;
    // printf("Load DMP3 image");
    rc = inv_icm20948_load(&icm_device, dmp3_image, sizeof(dmp3_image));
    return rc;
}

static void icm20948_apply_mounting_matrix(void)
{
    int ii;

    for (ii = 0; ii < INV_ICM20948_SENSOR_MAX; ii++) {
        inv_icm20948_set_matrix(&icm_device, cfg_mounting_matrix, (enum inv_icm20948_sensor)ii);
    }
}

static void icm20948_set_fsr(void)
{
    inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_RAW_ACCELEROMETER, (const void *)&cfg_acc_fsr);
    inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_ACCELEROMETER, (const void *)&cfg_acc_fsr);
    inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_RAW_GYROSCOPE, (const void *)&cfg_gyr_fsr);
    inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_GYROSCOPE, (const void *)&cfg_gyr_fsr);
    inv_icm20948_set_fsr(&icm_device, INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED, (const void *)&cfg_gyr_fsr);
}

int icm20948_sensor_setup(void)
{
    int rc;
    uint8_t i, whoami = 0xff;
    uint8_t UARTbuf[128];

    rc = inv_icm20948_get_whoami(&icm_device, &whoami);
    sprintf((char *)UARTbuf, "ICM20948 WHOAMI value=0x%02x\r\n", whoami);
    HAL_UART_Transmit(&ICM20948_UART, UARTbuf, 29, 100);

    if (whoami != EXPECTED_WHOAMI[0]) {
        sprintf((char *)UARTbuf, "Bad WHOAMI value = 0x%x\r\n", whoami);
        HAL_UART_Transmit(&ICM20948_UART, UARTbuf, 26, 100);
        return rc;
    }

    /* Setup accel and gyro mounting matrix and associated angle for current board */
    inv_icm20948_init_matrix(&icm_device);

    /* set default power mode */
    HAL_UART_Transmit(&ICM20948_UART, "Putting Icm20948 in sleep mode...\r\n", 36, 100);
    rc = inv_icm20948_initialize(&icm_device, dmp3_image, sizeof(dmp3_image));
    if (rc != 0) {
        HAL_UART_Transmit(&ICM20948_UART, (uint8_t *)"Initialization failed. Error loading DMP3...\r\n", 47, 100);
        return rc;
    }

    // Configure and initialize the ICM20948 for normal use
    HAL_UART_Transmit(&ICM20948_UART, (uint8_t *)"Booting up icm20948...\r\n", 25, 100);

    // Initialize auxiliary sensors
    inv_icm20948_register_aux_compass(&icm_device, INV_ICM20948_COMPASS_ID_AK09916, AK0991x_DEFAULT_I2C_ADDR);
    rc = inv_icm20948_initialize_auxiliary(&icm_device);
    if (rc == -1) {
        HAL_UART_Transmit(&ICM20948_UART, (uint8_t *)"Compass not detected...\r\n", 26, 100);
    }
    else {
        HAL_UART_Transmit(&ICM20948_UART, (uint8_t *)"Compass success...\r\n", 21, 100);
    }

    icm20948_apply_mounting_matrix();

    icm20948_set_fsr();

    /* re-initialize base state structure */
    inv_icm20948_init_structure(&icm_device);

    /* we should be good to go ! */
	HAL_UART_Transmit(&ICM20948_UART, (uint8_t *)"We're good to go !\r\n", 21, 100);

    return 0;
}

static uint8_t icm20948_get_grv_accuracy(void)
{
    uint8_t accel_accuracy;
    uint8_t gyro_accuracy;

    accel_accuracy = (uint8_t)inv_icm20948_get_accel_accuracy();
    gyro_accuracy = (uint8_t)inv_icm20948_get_gyro_accuracy();
    return (min(accel_accuracy, gyro_accuracy));
}


void build_sensor_event_data(void * context, enum inv_icm20948_sensor sensortype, uint64_t timestamp, const void * data, const void *arg) 
{
    float raw_bias_data[6];
    inv_sensor_event_t event;
    (void)context;
    uint8_t sensor_id = convert_to_generic_ids[sensortype];

    memset((void *)&event, 0, sizeof(event));
    event.sensor = sensor_id;
    event.timestamp = timestamp;
    switch (sensor_id) 
    {
    case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
        memcpy(raw_bias_data, data, sizeof(raw_bias_data));
        memcpy(event.data.gyr.vect, &raw_bias_data[0], sizeof(event.data.gyr.vect));
        memcpy(event.data.gyr.bias, &raw_bias_data[3], sizeof(event.data.gyr.bias));
        memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
        break;
    case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:
        memcpy(raw_bias_data, data, sizeof(raw_bias_data));
        memcpy(event.data.mag.vect, &raw_bias_data[0], sizeof(event.data.mag.vect));
        memcpy(event.data.mag.bias, &raw_bias_data[3], sizeof(event.data.mag.bias));
        memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));
        break;
    case INV_SENSOR_TYPE_GYROSCOPE:
        memcpy(event.data.gyr.vect, data, sizeof(event.data.gyr.vect));
        memcpy(&(event.data.gyr.accuracy_flag), arg, sizeof(event.data.gyr.accuracy_flag));

        // WE WANT THIS
        gyro_x = event.data.gyr.vect[0];
        gyro_y = event.data.gyr.vect[1];
        gyro_z = event.data.gyr.vect[2];
        gyro_data_ready = true;
        break;
    case INV_SENSOR_TYPE_GRAVITY:
        memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
        event.data.acc.accuracy_flag = inv_icm20948_get_accel_accuracy();
        break;
    case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
    case INV_SENSOR_TYPE_ACCELEROMETER:
        memcpy(event.data.acc.vect, data, sizeof(event.data.acc.vect));
        memcpy(&(event.data.acc.accuracy_flag), arg, sizeof(event.data.acc.accuracy_flag));

        // WE WANT THIS
        accel_x = event.data.acc.vect[0];
        accel_y = event.data.acc.vect[1];
        accel_z = event.data.acc.vect[2];
        accel_data_ready = true;
        break;
    case INV_SENSOR_TYPE_MAGNETOMETER:
        memcpy(event.data.mag.vect, data, sizeof(event.data.mag.vect));
        memcpy(&(event.data.mag.accuracy_flag), arg, sizeof(event.data.mag.accuracy_flag));

        // WE WANT THIS
        mag_x = event.data.mag.vect[0];
        mag_y = event.data.mag.vect[1];
        mag_z = event.data.mag.vect[2];
        mag_data_ready = true;
        break;
    case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
    case INV_SENSOR_TYPE_ROTATION_VECTOR:
        memcpy(&(event.data.quaternion.accuracy), arg, sizeof(event.data.quaternion.accuracy));
        memcpy(event.data.quaternion.quat, data, sizeof(event.data.quaternion.quat));
        break;
    case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
        memcpy(event.data.quaternion.quat, data, sizeof(event.data.quaternion.quat));
        event.data.quaternion.accuracy_flag = icm20948_get_grv_accuracy();

        // WE WANT THIS
        quat_w = event.data.quaternion.quat[0];
        quat_x = event.data.quaternion.quat[1];
        quat_y = event.data.quaternion.quat[2];
        quat_z = event.data.quaternion.quat[3];
        quat_data_ready = true;
        break;
    case INV_SENSOR_TYPE_BAC:
        memcpy(&(event.data.bac.event), data, sizeof(event.data.bac.event));
        break;
    case INV_SENSOR_TYPE_PICK_UP_GESTURE:
    case INV_SENSOR_TYPE_TILT_DETECTOR:
    case INV_SENSOR_TYPE_STEP_DETECTOR:
    case INV_SENSOR_TYPE_SMD:
        event.data.event = true;
        break;
    case INV_SENSOR_TYPE_B2S:
        event.data.event = true;
        memcpy(&(event.data.b2s.direction), data, sizeof(event.data.b2s.direction));
        break;
    case INV_SENSOR_TYPE_STEP_COUNTER:
        memcpy(&(event.data.step.count), data, sizeof(event.data.step.count));
        break;
    case INV_SENSOR_TYPE_ORIENTATION:
        //we just want to copy x,y,z from orientation data
        memcpy(&(event.data.orientation), data, 3 * sizeof(float));
        break;
    case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
    case INV_SENSOR_TYPE_RAW_GYROSCOPE:
        memcpy(event.data.raw3d.vect, data, sizeof(event.data.raw3d.vect));
        break;
    default:
        return;
    }
    // sensor_event(&event, NULL);
}

// TODO: shoule place this function up to build_sensor_event_data ?
// void sensor_event(const inv_sensor_event_t * event, void * arg){
//     /* arg will contained the value provided at init time */
//     (void)arg;

//     /*
//     * Encode sensor event and sent to host over UART through IddWrapper protocol
//     */
//     static DynProtocolEdata_t async_edata; /* static to take on .bss */
//     static uint8_t async_buffer[256]; /* static to take on .bss */
//     uint16_t async_bufferLen;

//     async_edata.sensor_id = event->sensor;
//     async_edata.d.async.sensorEvent.status = DYN_PRO_SENSOR_STATUS_DATA_UPDATED;
//     convert_sensor_event_to_dyn_prot_data(event, &async_edata.d.async.sensorEvent.vdata);

//     if(DynProtocol_encodeAsync(&protocol,
//         DYN_PROTOCOL_EID_NEW_SENSOR_DATA, &async_edata,
//         async_buffer, sizeof(async_buffer), &async_bufferLen) != 0) {
//             goto error_dma_buf;
//     }

//     DynProTransportUart_tx(&transport, async_buffer, async_bufferLen);
//     return;

// error_dma_buf:
//     INV_MSG(INV_MSG_LEVEL_WARNING, "sensor_event_cb: encode error, frame dropped");

//     return;
// }

static enum inv_icm20948_sensor idd_sensortype_conversion(int sensor)
{
    switch (sensor)
    {
    case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
        return INV_ICM20948_SENSOR_RAW_ACCELEROMETER;
    case INV_SENSOR_TYPE_RAW_GYROSCOPE:
        return INV_ICM20948_SENSOR_RAW_GYROSCOPE;
    case INV_SENSOR_TYPE_ACCELEROMETER:
        return INV_ICM20948_SENSOR_ACCELEROMETER;
    case INV_SENSOR_TYPE_GYROSCOPE:
        return INV_ICM20948_SENSOR_GYROSCOPE;
    case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:
        return INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED;
    case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
        return INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED;
    case INV_SENSOR_TYPE_BAC:
        return INV_ICM20948_SENSOR_ACTIVITY_CLASSIFICATON;
    case INV_SENSOR_TYPE_STEP_DETECTOR:
        return INV_ICM20948_SENSOR_STEP_DETECTOR;
    case INV_SENSOR_TYPE_STEP_COUNTER:
        return INV_ICM20948_SENSOR_STEP_COUNTER;
    case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
        return INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR;
    case INV_SENSOR_TYPE_ROTATION_VECTOR:
        return INV_ICM20948_SENSOR_ROTATION_VECTOR;
    case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
        return INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR;
    case INV_SENSOR_TYPE_MAGNETOMETER:
        return INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD;
    case INV_SENSOR_TYPE_SMD:
        return INV_ICM20948_SENSOR_WAKEUP_SIGNIFICANT_MOTION;
    case INV_SENSOR_TYPE_PICK_UP_GESTURE:
        return INV_ICM20948_SENSOR_FLIP_PICKUP;
    case INV_SENSOR_TYPE_TILT_DETECTOR:
        return INV_ICM20948_SENSOR_WAKEUP_TILT_DETECTOR;
    case INV_SENSOR_TYPE_GRAVITY:
        return INV_ICM20948_SENSOR_GRAVITY;
    case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
        return INV_ICM20948_SENSOR_LINEAR_ACCELERATION;
    case INV_SENSOR_TYPE_ORIENTATION:
        return INV_ICM20948_SENSOR_ORIENTATION;
    case INV_SENSOR_TYPE_B2S:
        return INV_ICM20948_SENSOR_B2S;
    default:
        return INV_ICM20948_SENSOR_MAX;
    }
}

// TODO:
// void inv_icm20948_get_st_bias(struct inv_icm20948 * s, int *gyro_bias, int *accel_bias, int * st_bias, int * unscaled);
// int icm20948_run_selftest(void);

/*************************************************************************
  Class Functions
*************************************************************************/

void ICM20948_init(STM32ICM20948Settings settings)
{
    int rc;

    HAL_UART_Transmit(&ICM20948_UART, (uint8_t *)"Initializing ICM-20948 ...\r\n", 29, 100);

    /*
    * Initialize icm20948 serif structure
    */
    struct inv_icm20948_serif icm20948_serif;
    icm20948_serif.context   = 0; /* no need */
    icm20948_serif.read_reg  = idd_io_hal_read_reg;
    icm20948_serif.write_reg = idd_io_hal_write_reg;
    icm20948_serif.max_read  = 1024 * 16; /* maximum number of bytes allowed per serial read */
    icm20948_serif.max_write = 1024 * 16; /* maximum number of bytes allowed per serial write */

    icm20948_serif.is_spi = interface_is_SPI();

    /*
    * Reset icm20948 driver states
    */
    inv_icm20948_reset_states(&icm_device, &icm20948_serif);

    inv_icm20948_register_aux_compass(&icm_device, INV_ICM20948_COMPASS_ID_AK09916, AK0991x_DEFAULT_I2C_ADDR);

    /*
    * Setup the icm20948 device
    */
    rc = icm20948_sensor_setup();

    // TODO: extra feature
    if (icm_device.selftest_done && !icm_device.offset_done) {
        // If we've run self test and not already set the offset.
        inv_icm20948_set_offset(&icm_device, unscaled_bias);
        icm_device.offset_done = 1;
    }

    /*
    * Now that Icm20948 device was initialized, we can proceed with DMP image loading
    * This step is mandatory as DMP image are not store in non volatile memory
    */
    rc += load_dmp3();
    check_rc(rc, "Error sensor_setup/DMP loading.");

    // TODO: --- check the following ---
    // Set mode
    inv_icm20948_set_lowpower_or_highperformance(&icm_device, settings.mode);

    // Set frequency
    rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GAME_ROTATION_VECTOR), 1000 / settings.quaternion_frequency);
    // rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GYROSCOPE), 1000 / settings.gyroscope_frequency);
    // rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_ACCELEROMETER), 1000 / settings.accelerometer_frequency);
    // rc = inv_icm20948_set_sensor_period(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_MAGNETOMETER), 1000 / settings.magnetometer_frequency);

    // Enable / disable
    // rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GYROSCOPE), settings.enable_gyroscope);
    // rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_ACCELEROMETER), settings.enable_accelerometer);
    rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_GAME_ROTATION_VECTOR), settings.enable_quaternion);
    // rc = inv_icm20948_enable_sensor(&icm_device, idd_sensortype_conversion(INV_SENSOR_TYPE_MAGNETOMETER), settings.enable_magnetometer);
}

void ICM20948_task()
{
    inv_icm20948_poll_sensor(&icm_device, (void *)0, build_sensor_event_data);
}

bool ICM20948_gyroDataIsReady()
{
    return gyro_data_ready;
}

bool ICM20948_accelDataIsReady()
{
    return accel_data_ready;
}

bool ICM20948_magDataIsReady()
{
    return mag_data_ready;
}

bool ICM20948_quatDataIsReady()
{
    return quat_data_ready;
}

void ICM20948_readGyroData(float *x, float *y, float *z)
{
    *x = gyro_x;
    *y = gyro_y;
    *z = gyro_z;
    gyro_data_ready = false;
}

void ICM20948_readAccelData(float *x, float *y, float *z)
{
    *x = accel_x;
    *y = accel_y;
    *z = accel_z;
    accel_data_ready = false;
}

void ICM20948_readMagData(float *x, float *y, float *z)
{
    *x = mag_x;
    *y = mag_y;
    *z = mag_z;
    mag_data_ready = false;
}

void ICM20948_readQuatData(float *w, float *x, float *y, float *z)
{
    *w = quat_w;
    *x = quat_x;
    *y = quat_y;
    *z = quat_z;
    quat_data_ready = false;
}

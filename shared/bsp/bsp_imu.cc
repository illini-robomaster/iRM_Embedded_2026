/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2023 RoboMaster.                                          *
 *  Illini RoboMaster @ University of Illinois at Urbana-Champaign          *
 *                                                                          *
 *  This program is free software: you can redistribute it and/or modify    *
 *  it under the terms of the GNU General Public License as published by    *
 *  the Free Software Foundation, either version 3 of the License, or       *
 *  (at your option) any later version.                                     *
 *                                                                          *
 *  This program is distributed in the hope that it will be useful,         *
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of          *
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the           *
 *  GNU General Public License for more details.                            *
 *                                                                          *
 *  You should have received a copy of the GNU General Public License       *
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.    *
 *                                                                          *
 ****************************************************************************/

#include "bsp_imu.h"

#include <cmath>

#include "MahonyAHRS.h"
#include "bsp_error_handler.h"
#include "bsp_mpu6500_reg.h"
#ifndef BOARD_DM_MC_02
#include "bsp_os.h"
#include "dma.h"
#endif

#define MPU6500_DELAY 55  // SPI delay
// configured with initialization sequences
#define MPU6500_ACC_FACTOR 4096.0f
#define MPU6500_GYRO_FACTOR 32.768f

#define GRAVITY_ACC 9.8f
#define DEG2RAD(x) ((x) / 180 * M_PI)

#define MAG_SEN 0.3f  // raw int16 data change to uT unit.

#define IST8310_WHO_AM_I 0x00        // ist8310 "who am I "
#define IST8310_WHO_AM_I_VALUE 0x10  // device ID

#define IST8310_WRITE_REG_NUM 4

// the first column:the registers of IST8310.
// the second column: the value to be writed to the registers.
// the third column: return error value.
static const uint8_t ist8310_write_reg_data_error[IST8310_WRITE_REG_NUM][3] = {
    {0x0B, 0x08, 0x01},   // enalbe interrupt  and low pin polarity.
    {0x41, 0x09, 0x02},   // average 2 times.
    {0x42, 0xC0, 0x03},   // must be 0xC0.
    {0x0A, 0x0B, 0x04}};  // 200Hz output rate.

#define IST8310_IIC_ADDRESS 0x0E  // the I2C address of IST8310

namespace bsp {

// MPU6500 class is not supported on BOARD_DM_MC_02 (uses STM32F4-specific SPI callbacks)
#ifndef BOARD_DM_MC_02

MPU6500* MPU6500::mpu6500 = nullptr;

MPU6500::MPU6500(SPI_HandleTypeDef* hspi, const GPIO& chip_select, uint16_t int_pin)
    : GPIT(int_pin), hspi_(hspi), chip_select_(chip_select) {
  const uint8_t init_len = 7;
  const uint8_t init_data[init_len][2] = {
      {MPU6500_PWR_MGMT_1, 0x03},      // auto select clock source
      {MPU6500_PWR_MGMT_2, 0x00},      // enable acc & gyro
      {MPU6500_CONFIG, 0x02},          // gyro LP bandwidth 92Hz
      {MPU6500_GYRO_CONFIG, 0x10},     // gyro range 1000 dps / 32.8
      {MPU6500_ACCEL_CONFIG, 0x10},    // acc range 8g / 4096
      {MPU6500_ACCEL_CONFIG_2, 0x02},  // acc LP bandwidth 92Hz
      {MPU6500_INT_PIN_CFG, 0x10},     // any read to clear interrupt
  };
  Reset();  // reset all registers and signal paths
  for (size_t i = 0; i < init_len; ++i) WriteReg(init_data[i][0], init_data[i][1]);
  // validate register values
  uint8_t tmp;
  for (size_t i = 0; i < init_len; ++i) {
    ReadReg(init_data[i][0], &tmp);
    if (tmp != init_data[i][1])
      bsp_error_handler(__FUNCTION__, __LINE__, "imu register incorrect initialization");
  }
  // setup interrupt callback
  RM_ASSERT_FALSE(mpu6500, "Repteated intialization of MPU6500");
  mpu6500 = this;
  HAL_SPI_RegisterCallback(hspi, HAL_SPI_TX_RX_COMPLETE_CB_ID, &MPU6500::SPITxRxCpltCallback);
  // initialize magnetometer
  IST8310Init();
  // enable imu interrupt
  WriteReg(MPU6500_INT_ENABLE, 0x01);
}

void MPU6500::IST8310Init() {
  WriteReg(MPU6500_USER_CTRL, 0x30);     // enable I2C master and reset all slaves
  WriteReg(MPU6500_I2C_MST_CTRL, 0x0d);  // 400 kHz I2C clock
  // slave 0 for auto receive
  WriteReg(MPU6500_I2C_SLV0_ADDR, 0x0e | 0x80);  // read from device 0x0e
  WriteReg(MPU6500_I2C_SLV0_REG, 0x03);          // read data from 0x03 reg
  // slave 1 for auto transmit
  WriteReg(MPU6500_I2C_SLV1_ADDR, 0x0e);  // write into device 0x0e
  WriteReg(MPU6500_I2C_SLV1_REG, 0x0a);   // write data into 0x0a reg
  WriteReg(MPU6500_I2C_SLV1_DO, 0x01);    // send measurement command
  // enable slave 0 and 1
  WriteReg(MPU6500_I2C_SLV0_CTRL, 0xd6);  // swap endian + 6 bytes rx
  WriteReg(MPU6500_I2C_SLV1_CTRL, 0x81);  // 1 bytes tx
}

void MPU6500::UpdateData() {
  chip_select_.Low();
  io_buff_[0] = MPU6500_ACCEL_XOUT_H | 0x80;
  HAL_SPI_TransmitReceive_DMA(hspi_, io_buff_, io_buff_, MPU6500_SIZEOF_DATA + 1);
}

void MPU6500::Reset() {
  WriteReg(MPU6500_PWR_MGMT_1, 0x80);
  WriteReg(MPU6500_SIGNAL_PATH_RESET, 0x07);
  WriteReg(MPU6500_USER_CTRL, 0x03);

  HAL_Delay(1);  // seems like signal path reset needs some time
}

void MPU6500::WriteReg(uint8_t reg, uint8_t data) { WriteRegs(reg, &data, 1); }

void MPU6500::WriteRegs(uint8_t reg_start, uint8_t* data, uint8_t len) {
  uint8_t tx = reg_start & 0x7f;

  chip_select_.Low();
  HAL_SPI_Transmit(hspi_, &tx, 1, MPU6500_DELAY);
  HAL_SPI_Transmit(hspi_, data, len, MPU6500_DELAY);
  chip_select_.High();
}

void MPU6500::ReadReg(uint8_t reg, uint8_t* data) { ReadRegs(reg, data, 1); }

void MPU6500::ReadRegs(uint8_t reg_start, uint8_t* data, uint8_t len) {
  chip_select_.Low();
  *data = static_cast<uint8_t>(reg_start | 0x80);
  HAL_SPI_Transmit(hspi_, data, 1, MPU6500_DELAY);
  HAL_SPI_Receive(hspi_, data, len, MPU6500_DELAY);
  chip_select_.High();
}

void MPU6500::SPITxRxCpltCallback() {
  chip_select_.High();
  // NOTE(alvin): per MPU6500 documentation, the first byte of the rx / tx buffer
  //              contains the address of the SPI device
  uint8_t* buff = io_buff_ + 1;
  int16_t* array = reinterpret_cast<int16_t*>(buff);
  // in-place swap endian
  for (size_t i = 0; i < MPU6500_SIZEOF_DATA; i += 2)
    array[i / 2] = (int16_t)(buff[i] << 8 | buff[i + 1]);

  acce.x = (float)array[0] / (MPU6500_ACC_FACTOR / GRAVITY_ACC);
  acce.y = (float)array[1] / (MPU6500_ACC_FACTOR / GRAVITY_ACC);
  acce.z = (float)array[2] / (MPU6500_ACC_FACTOR / GRAVITY_ACC);
  temp = (float)array[3] / MPU6500_TEMP_FACTOR + MPU6500_TEMP_OFFSET;
  gyro.x = DEG2RAD((float)array[4] / MPU6500_GYRO_FACTOR);
  gyro.y = DEG2RAD((float)array[5] / MPU6500_GYRO_FACTOR);
  gyro.z = DEG2RAD((float)array[6] / MPU6500_GYRO_FACTOR);
  mag.x = (float)array[7];
  mag.y = (float)array[8];
  mag.z = (float)array[9];
}

void MPU6500::IntCallback() {
  timestamp = GetHighresTickMicroSec();
  UpdateData();
}

void MPU6500::SPITxRxCpltCallback(SPI_HandleTypeDef* hspi) {
  UNUSED(hspi);
  mpu6500->SPITxRxCpltCallback();
}

#endif  // !BOARD_DM_MC_02 - End of MPU6500 class

// IST8310 class is also not supported on BOARD_DM_MC_02 (I2C dependent)
#ifndef BOARD_DM_MC_02

IST8310::IST8310(IST8310_init_t init, IMU_typeC* imu) : GPIT(init.int_pin) {
  hi2c_ = init.hi2c;
  rst_group_ = init.rst_group;
  rst_pin_ = init.rst_pin;
  imu_ = imu;
  Init();
}

IST8310::IST8310(I2C_HandleTypeDef* hi2c, uint16_t int_pin, GPIO_TypeDef* rst_group,
                 uint16_t rst_pin, IMU_typeC* imu)
    : GPIT(int_pin) {
  hi2c_ = hi2c;
  rst_group_ = rst_group;
  rst_pin_ = rst_pin;
  imu_ = imu;
  Init();
}

bool IST8310::IsReady() {
  return HAL_I2C_IsDeviceReady(hi2c_, IST8310_IIC_ADDRESS << 1, 1, 100) == HAL_OK;
}

uint8_t IST8310::Init() {
  const uint8_t wait_time = 1;
  const uint8_t sleepTime = 50;
  uint8_t res;
  uint8_t writeNum;

  ist8310_RST_L();
  HAL_Delay(sleepTime);
  ist8310_RST_H();
  HAL_Delay(sleepTime);

  res = ist8310_IIC_read_single_reg(IST8310_WHO_AM_I);
  if (res != IST8310_WHO_AM_I_VALUE) return IST8310_NO_SENSOR;

  // set mpu6500 sonsor config and check
  for (writeNum = 0; writeNum < IST8310_WRITE_REG_NUM; writeNum++) {
    ist8310_IIC_write_single_reg(ist8310_write_reg_data_error[writeNum][0],
                                 ist8310_write_reg_data_error[writeNum][1]);
    HAL_Delay(wait_time);
    res = ist8310_IIC_read_single_reg(ist8310_write_reg_data_error[writeNum][0]);
    HAL_Delay(wait_time);
    if (res != ist8310_write_reg_data_error[writeNum][1])
      return ist8310_write_reg_data_error[writeNum][2];
  }
  return IST8310_NO_ERROR;
}

void IST8310::ist8310_read_over(uint8_t* status_buf, IST8310_real_data_t* ist8310_real_data) {
  if (status_buf[0] & 0x01) {
    int16_t temp_ist8310_data = 0;
    ist8310_real_data->status |= 1 << IST8310_DATA_READY_BIT;

    temp_ist8310_data = (int16_t)((status_buf[2] << 8) | status_buf[1]);
    ist8310_real_data->mag[0] = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((status_buf[4] << 8) | status_buf[3]);
    ist8310_real_data->mag[1] = MAG_SEN * temp_ist8310_data;
    temp_ist8310_data = (int16_t)((status_buf[6] << 8) | status_buf[5]);
    ist8310_real_data->mag[2] = MAG_SEN * temp_ist8310_data;
  } else {
    ist8310_real_data->status &= ~(1 << IST8310_DATA_READY_BIT);
  }
}

void IST8310::ist8310_read_mag(float mag_[3]) {
  uint8_t buf[6];
  int16_t temp_ist8310_data = 0;
  // read the "DATAXL" register (0x03)
  ist8310_IIC_read_muli_reg(0x03, buf, 6);

  temp_ist8310_data = (int16_t)((buf[1] << 8) | buf[0]);
  mag_[0] = MAG_SEN * temp_ist8310_data;
  temp_ist8310_data = (int16_t)((buf[3] << 8) | buf[2]);
  mag_[1] = MAG_SEN * temp_ist8310_data;
  temp_ist8310_data = (int16_t)((buf[5] << 8) | buf[4]);
  mag_[2] = MAG_SEN * temp_ist8310_data;
}

void IST8310::IntCallback() {
  if (imu_ && imu_->useMag_) {
    imu_->mag_update_flag |= 1 << IMU_DR_SHIFTS;

    if (imu_->mag_update_flag &= 1 << IMU_DR_SHIFTS) {
      imu_->mag_update_flag &= ~(1 << IMU_DR_SHIFTS);
      imu_->mag_update_flag |= (1 << IMU_SPI_SHIFTS);

      ist8310_read_mag(imu_->IST8310_real_data_.mag);
    }
  } else {
    ist8310_read_mag(mag);
  }
}

void IST8310::ist8310_RST_H() { HAL_GPIO_WritePin(rst_group_, rst_pin_, GPIO_PIN_SET); }

void IST8310::ist8310_RST_L() { HAL_GPIO_WritePin(rst_group_, rst_pin_, GPIO_PIN_RESET); }

uint8_t IST8310::ist8310_IIC_read_single_reg(uint8_t reg) {
  uint8_t res = 0;
  HAL_I2C_Mem_Read(hi2c_, IST8310_IIC_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, &res, 1, 10);
  return res;
}

void IST8310::ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data) {
  HAL_I2C_Mem_Write(hi2c_, IST8310_IIC_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 10);
}

void IST8310::ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len) {
  HAL_I2C_Mem_Read(hi2c_, IST8310_IIC_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 10);
}

void IST8310::ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t* data, uint8_t len) {
  HAL_I2C_Mem_Write(hi2c_, IST8310_IIC_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, data, len, 10);
}

#endif  // !BOARD_DM_MC_02 - End of MPU6500 and IST8310 classes

BMI088::BMI088(BMI088_init_t init) {
  hspi_ = init.hspi;
  CS1_ACCEL_GPIO_Port_ = init.CS_ACCEL_Port;
  CS1_ACCEL_Pin_ = init.CS_ACCEL_Pin;
  CS1_GYRO_GPIO_Port_ = init.CS_GYRO_Port;
  CS1_GYRO_Pin_ = init.CS_GYRO_Pin;
  Init();
}

BMI088::BMI088(SPI_HandleTypeDef* hspi, GPIO_TypeDef* CS_ACCEL_Port, uint16_t CS_ACCEL_Pin,
               GPIO_TypeDef* CS_GYRO_Port, uint16_t CS_GYRO_Pin) {
  hspi_ = hspi;
  CS1_ACCEL_GPIO_Port_ = CS_ACCEL_Port;
  CS1_ACCEL_Pin_ = CS_ACCEL_Pin;
  CS1_GYRO_GPIO_Port_ = CS_GYRO_Port;
  CS1_GYRO_Pin_ = CS_GYRO_Pin;
  Init();
}

bool BMI088::IsReady() { return HAL_SPI_Init(hspi_) == HAL_OK; }

void BMI088::BMI088_ACCEL_NS_L() {
  HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port_, CS1_ACCEL_Pin_, GPIO_PIN_RESET);
}

void BMI088::BMI088_ACCEL_NS_H() {
  HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port_, CS1_ACCEL_Pin_, GPIO_PIN_SET);
}

void BMI088::BMI088_GYRO_NS_L() {
  HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port_, CS1_GYRO_Pin_, GPIO_PIN_RESET);
}

void BMI088::BMI088_GYRO_NS_H() {
  HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port_, CS1_GYRO_Pin_, GPIO_PIN_SET);
}

uint8_t BMI088::BMI088_read_write_byte(uint8_t tx_data) {
  uint8_t rx_data;
  HAL_SPI_TransmitReceive(hspi_, &tx_data, &rx_data, 1, 1000);
  return rx_data;
}

void BMI088::BMI088_write_single_reg(uint8_t reg, uint8_t data) {
  BMI088_read_write_byte(reg);
  BMI088_read_write_byte(data);
}

void BMI088::BMI088_read_single_reg(uint8_t reg, uint8_t* data) {
  BMI088_read_write_byte(reg | 0x80);
  *data = BMI088_read_write_byte(0x55);
}

void BMI088::BMI088_read_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len) {
  BMI088_read_write_byte(reg | 0x80);
  while (len != 0) {
    *buf = BMI088_read_write_byte(0x55);
    buf++;
    len--;
  }
}

void BMI088::BMI088_accel_write_single_reg(uint8_t reg, uint8_t data) {
  BMI088_ACCEL_NS_L();
  BMI088_write_single_reg(reg, data);
  BMI088_ACCEL_NS_H();
}

void BMI088::BMI088_accel_read_single_reg(uint8_t reg, uint8_t* data) {
  BMI088_ACCEL_NS_L();
  BMI088_read_write_byte(reg | 0x80);
  BMI088_read_write_byte(0x55);
  *data = BMI088_read_write_byte(0x55);
  BMI088_ACCEL_NS_H();
}

void BMI088::BMI088_accel_read_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len) {
  BMI088_ACCEL_NS_L();
  BMI088_read_write_byte(reg | 0x80);
  BMI088_read_muli_reg(reg, buf, len);
  BMI088_ACCEL_NS_H();
}

void BMI088::BMI088_gyro_write_single_reg(uint8_t reg, uint8_t data) {
  BMI088_GYRO_NS_L();
  BMI088_write_single_reg(reg, data);
  BMI088_GYRO_NS_H();
}

void BMI088::BMI088_gyro_read_single_reg(uint8_t reg, uint8_t* data) {
  BMI088_GYRO_NS_L();
  BMI088_read_single_reg(reg, data);
  BMI088_GYRO_NS_H();
}

void BMI088::BMI088_gyro_read_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len) {
  BMI088_GYRO_NS_L();
  BMI088_read_muli_reg(reg, buf, len);
  BMI088_GYRO_NS_H();
}

static float BMI088_ACCEL_SEN = BMI088_ACCEL_3G_SEN;
static float BMI088_GYRO_SEN = BMI088_GYRO_2000_SEN;

static uint8_t write_BMI088_accel_reg_data_error[BMI088_WRITE_ACCEL_REG_NUM][3] = {
    {BMI088_ACC_PWR_CTRL, BMI088_ACC_ENABLE_ACC_ON, BMI088_ACC_PWR_CTRL_ERROR},
    {BMI088_ACC_PWR_CONF, BMI088_ACC_PWR_ACTIVE_MODE, BMI088_ACC_PWR_CONF_ERROR},
    {BMI088_ACC_CONF, BMI088_ACC_NORMAL | BMI088_ACC_800_HZ | BMI088_ACC_CONF_MUST_Set,
     BMI088_ACC_CONF_ERROR},
    {BMI088_ACC_RANGE, BMI088_ACC_RANGE_3G, BMI088_ACC_RANGE_ERROR},
    {BMI088_INT1_IO_CTRL,
     BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_LOW,
     BMI088_INT1_IO_CTRL_ERROR},
    {BMI088_INT_MAP_DATA, BMI088_ACC_INT1_DRDY_INTERRUPT, BMI088_INT_MAP_DATA_ERROR}};

static uint8_t write_BMI088_gyro_reg_data_error[BMI088_WRITE_GYRO_REG_NUM][3] = {
    {BMI088_GYRO_RANGE, BMI088_GYRO_2000, BMI088_GYRO_RANGE_ERROR},
    {BMI088_GYRO_BANDWIDTH, BMI088_GYRO_1000_116_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set,
     BMI088_GYRO_BANDWIDTH_ERROR},
    {BMI088_GYRO_LPM1, BMI088_GYRO_NORMAL_MODE, BMI088_GYRO_LPM1_ERROR},
    {BMI088_GYRO_CTRL, BMI088_DRDY_ON, BMI088_GYRO_CTRL_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_CONF, BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_LOW,
     BMI088_GYRO_INT3_INT4_IO_CONF_ERROR},
    {BMI088_GYRO_INT3_INT4_IO_MAP, BMI088_GYRO_DRDY_IO_INT3, BMI088_GYRO_INT3_INT4_IO_MAP_ERROR}};

uint8_t BMI088::Init() {
  uint8_t error = BMI088_NO_ERROR;

  error |= bmi088_accel_init();
  error |= bmi088_gyro_init();

  hspi_->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;

  return error;
}

bool BMI088::bmi088_accel_init() {
  uint8_t res = 0;
  uint8_t write_reg_num;

  // check commiunication
  BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
  HAL_Delay(1);
  BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
  HAL_Delay(1);

  // accel software reset
  BMI088_accel_write_single_reg(BMI088_ACC_SOFTRESET, BMI088_ACC_SOFTRESET_VALUE);
  HAL_Delay(BMI088_LONG_DELAY_TIME);

  // check commiunication is normal after reset
  BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
  HAL_Delay(1);
  BMI088_accel_read_single_reg(BMI088_ACC_CHIP_ID, &res);
  HAL_Delay(1);

  // check the "who am I"
  if (res != BMI088_ACC_CHIP_ID_VALUE) return false;

  // set accel sonsor config and check
  for (write_reg_num = 0; write_reg_num < BMI088_WRITE_ACCEL_REG_NUM; ++write_reg_num) {
    BMI088_accel_write_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0],
                                  write_BMI088_accel_reg_data_error[write_reg_num][1]);
    HAL_Delay(1);
    BMI088_accel_read_single_reg(write_BMI088_accel_reg_data_error[write_reg_num][0], &res);
    HAL_Delay(1);
    if (res != write_BMI088_accel_reg_data_error[write_reg_num][1])
      return write_BMI088_accel_reg_data_error[write_reg_num][2];
  }
  return false;
}

bool BMI088::bmi088_gyro_init() {
  uint8_t write_reg_num;
  uint8_t res = 0;

  // check commiunication
  BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
  HAL_Delay(1);
  BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
  HAL_Delay(1);

  // reset the gyro sensor
  BMI088_gyro_write_single_reg(BMI088_GYRO_SOFTRESET, BMI088_GYRO_SOFTRESET_VALUE);
  HAL_Delay(BMI088_LONG_DELAY_TIME);
  // check commiunication is normal after reset
  BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
  HAL_Delay(1);
  BMI088_gyro_read_single_reg(BMI088_GYRO_CHIP_ID, &res);
  HAL_Delay(1);

  // check the "who am I"
  if (res != BMI088_GYRO_CHIP_ID_VALUE) return false;

  // set gyro sonsor config and check
  for (write_reg_num = 0; write_reg_num < BMI088_WRITE_GYRO_REG_NUM; ++write_reg_num) {
    BMI088_gyro_write_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0],
                                 write_BMI088_gyro_reg_data_error[write_reg_num][1]);
    HAL_Delay(1);
    BMI088_gyro_read_single_reg(write_BMI088_gyro_reg_data_error[write_reg_num][0], &res);
    HAL_Delay(1);
    if (res != write_BMI088_gyro_reg_data_error[write_reg_num][1])
      return write_BMI088_gyro_reg_data_error[write_reg_num][2];
  }

  return false;
}

void BMI088::Read(float* gyro, float* accel, float* temperate) {
  uint8_t buf[8] = {0, 0, 0, 0, 0, 0};
  int16_t bmi088_raw_temp;

  BMI088_accel_read_muli_reg(BMI088_ACCEL_XOUT_L, buf, 6);

  bmi088_raw_temp = (int16_t)((buf[1]) << 8) | buf[0];
  accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
  bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
  accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
  bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
  accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;

  BMI088_gyro_read_muli_reg(BMI088_GYRO_CHIP_ID, buf, 8);
  if (buf[0] == BMI088_GYRO_CHIP_ID_VALUE) {
    bmi088_raw_temp = (int16_t)((buf[3]) << 8) | buf[2];
    gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
    bmi088_raw_temp = (int16_t)((buf[5]) << 8) | buf[4];
    gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
    bmi088_raw_temp = (int16_t)((buf[7]) << 8) | buf[6];
    gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
  }
  BMI088_accel_read_muli_reg(BMI088_TEMP_M, buf, 2);

  bmi088_raw_temp = (int16_t)((buf[0] << 3) | (buf[1] >> 5));

  if (bmi088_raw_temp > 1023) bmi088_raw_temp -= 2048;

  *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

void BMI088::temperature_read_over(uint8_t* rx_buf, float* temperate) {
  int16_t bmi088_raw_temp;
  bmi088_raw_temp = (int16_t)((rx_buf[0] << 3) | (rx_buf[1] >> 5));

  if (bmi088_raw_temp > 1023) bmi088_raw_temp -= 2048;
  *temperate = bmi088_raw_temp * BMI088_TEMP_FACTOR + BMI088_TEMP_OFFSET;
}

void BMI088::accel_read_over(uint8_t* rx_buf, float* accel, float* time) {
  int16_t bmi088_raw_temp;
  uint32_t sensor_time;
  bmi088_raw_temp = (int16_t)((rx_buf[1]) << 8) | rx_buf[0];
  accel[0] = bmi088_raw_temp * BMI088_ACCEL_SEN;
  bmi088_raw_temp = (int16_t)((rx_buf[3]) << 8) | rx_buf[2];
  accel[1] = bmi088_raw_temp * BMI088_ACCEL_SEN;
  bmi088_raw_temp = (int16_t)((rx_buf[5]) << 8) | rx_buf[4];
  accel[2] = bmi088_raw_temp * BMI088_ACCEL_SEN;
  sensor_time = (uint32_t)((rx_buf[8] << 16) | (rx_buf[7] << 8) | rx_buf[6]);
  *time = sensor_time * 39.0625f;
}

void BMI088::gyro_read_over(uint8_t* rx_buf, float* gyro) {
  int16_t bmi088_raw_temp;
  bmi088_raw_temp = (int16_t)((rx_buf[1]) << 8) | rx_buf[0];
  gyro[0] = bmi088_raw_temp * BMI088_GYRO_SEN;
  bmi088_raw_temp = (int16_t)((rx_buf[3]) << 8) | rx_buf[2];
  gyro[1] = bmi088_raw_temp * BMI088_GYRO_SEN;
  bmi088_raw_temp = (int16_t)((rx_buf[5]) << 8) | rx_buf[4];
  gyro[2] = bmi088_raw_temp * BMI088_GYRO_SEN;
}

// Accel_INT, Gyro_INT, IMU_typeC classes use STM32F4-specific DMA registers
#ifndef BOARD_DM_MC_02

Accel_INT::Accel_INT(uint16_t INT_pin, IMU_typeC* imu) : GPIT(INT_pin) { imu_ = imu; }

void Accel_INT::IntCallback() {
  imu_->accel_update_flag |= 1 << IMU_DR_SHIFTS;
  imu_->accel_temp_update_flag |= 1 << IMU_DR_SHIFTS;
  if (imu_->imu_start_dma_flag) imu_->imu_cmd_spi_dma();
}

Gyro_INT::Gyro_INT(uint16_t INT_pin, IMU_typeC* imu) : GPIT(INT_pin) { imu_ = imu; }

void Gyro_INT::IntCallback() {
  imu_->gyro_update_flag |= 1 << IMU_DR_SHIFTS;
  if (imu_->imu_start_dma_flag) imu_->imu_cmd_spi_dma();
}

IMU_typeC::IMU_typeC(IMU_typeC_init_t init, bool useMag)
    : IST8310_(init.IST8310, this),
      BMI088_(init.BMI088),
      heater_(init.heater),
      Accel_INT_(init.Accel_INT_pin_, this),
      Gyro_INT_(init.Gyro_INT_pin_, this) {
  useMag_ = useMag;
  RM_ASSERT_FALSE(FindInstance(init.hspi), "Uart repeated initialization");
  spi_ptr_map[init.hspi] = this;
  IST8310_param_ = init.IST8310;
  BMI088_param_ = init.BMI088;
  heater_param_ = init.heater;
  hspi_ = init.hspi;
  hdma_spi_rx_ = init.hdma_spi_rx;
  hdma_spi_tx_ = init.hdma_spi_tx;
  BMI088_.Read(BMI088_real_data_.gyro, BMI088_real_data_.accel, &BMI088_real_data_.temp);
  accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = BMI088_real_data_.accel[0];
  accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = BMI088_real_data_.accel[1];
  accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = BMI088_real_data_.accel[2];
  AHRS_init(INS_quat, BMI088_real_data_.accel, IST8310_real_data_.mag);
  SPI_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGTH);
  imu_start_dma_flag = 1;
}

void IMU_typeC::Calibrate() { calibrate_ = true; }

bool IMU_typeC::CaliDone() { return calidone_; }

void IMU_typeC::Update() {
  if (gyro_update_flag & (1 << IMU_NOTIFY_SHIFTS)) {
    gyro_update_flag &= ~(1 << IMU_NOTIFY_SHIFTS);
    BMI088_.gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET,
                           BMI088_real_data_.gyro);
  }

  if (accel_update_flag & (1 << IMU_UPDATE_SHIFTS)) {
    accel_update_flag &= ~(1 << IMU_UPDATE_SHIFTS);
    BMI088_.accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET,
                            BMI088_real_data_.accel, &BMI088_real_data_.time);
  }

  if (accel_temp_update_flag & (1 << IMU_UPDATE_SHIFTS)) {
    accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHIFTS);
    BMI088_.temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET,
                                  &BMI088_real_data_.temp);
    Temp = BMI088_real_data_.temp;
    TempPWM = TempControl(BMI088_real_data_.temp);
  }

  if (calibrate_ && Temp > heater_param_.temp - 2) {
    if (!useMag_) {
      if (++count_ < zeroDriftTry) {
        for (int i = 0; i < 3; ++i) {
          zeroDriftTemp[i] += BMI088_real_data_.gyro[i];
        }
        return;
      } else if (count_ == zeroDriftTry) {
        for (int i = 0; i < 3; ++i) {
          zeroDrift[i] = zeroDriftTemp[i] / (float)zeroDriftTry;
        }
        calidone_ = true;
        return;
      }
      for (int i = 0; i < 3; ++i) {
        BMI088_real_data_.gyro[i] -= zeroDrift[i];
      }
    }
    accel_fliter_1[0] = accel_fliter_2[0];
    accel_fliter_2[0] = accel_fliter_3[0];
    accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] +
                        BMI088_real_data_.accel[0] * fliter_num[2];
    accel_fliter_1[1] = accel_fliter_2[1];
    accel_fliter_2[1] = accel_fliter_3[1];
    accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] +
                        BMI088_real_data_.accel[1] * fliter_num[2];
    accel_fliter_1[2] = accel_fliter_2[2];
    accel_fliter_2[2] = accel_fliter_3[2];
    accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] +
                        BMI088_real_data_.accel[2] * fliter_num[2];
    AHRS_update(INS_quat, 0.001f, BMI088_real_data_.gyro, BMI088_real_data_.accel,
                IST8310_real_data_.mag);
    GetAngle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET,
             INS_angle + INS_ROLL_ADDRESS_OFFSET);
  }
}

bool IMU_typeC::DataReady() { return Temp > heater_param_.temp - 2; }

std::map<SPI_HandleTypeDef*, IMU_typeC*> IMU_typeC::spi_ptr_map;

IMU_typeC* IMU_typeC::FindInstance(SPI_HandleTypeDef* hspi) {
  const auto it = spi_ptr_map.find(hspi);
  if (it == spi_ptr_map.end()) {
    return nullptr;
  }

  return it->second;
}

void IMU_typeC::AHRS_init(float* quat, float* accel, float* mag) {
  UNUSED(accel);
  UNUSED(mag);
  quat[0] = 1.0f;
  quat[1] = 0.0f;
  quat[2] = 0.0f;
  quat[3] = 0.0f;
}

void IMU_typeC::AHRS_update(float* quat, float time, float* gyro, float* accel, float* mag) {
  UNUSED(time);

  if (useMag_) {
    MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], mag[0], mag[1],
                     mag[2]);
  } else {
    MahonyAHRSupdateIMU(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
  }
}

void IMU_typeC::GetAngle(float* q, float* yaw, float* pitch, float* roll) {
  *yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
  *pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
  *roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
}

float IMU_typeC::TempControl(float real_temp) { return heater_.Update(real_temp); }

void IMU_typeC::SPI_DMA_init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num) {
  SET_BIT(hspi1.Instance->CR2, SPI_CR2_TXDMAEN);
  SET_BIT(hspi1.Instance->CR2, SPI_CR2_RXDMAEN);

  __HAL_SPI_ENABLE(&hspi1);

  // disable DMA
  __HAL_DMA_DISABLE(hdma_spi_rx_);

  while (hdma_spi_rx_->Instance->CR & DMA_SxCR_EN) {
    __HAL_DMA_DISABLE(hdma_spi_rx_);
  }

  __HAL_DMA_CLEAR_FLAG(hdma_spi_rx_, DMA_LISR_TCIF2);

  hdma_spi_rx_->Instance->PAR = (uint32_t) & (SPI1->DR);
  // memory buffer 1
  hdma_spi_rx_->Instance->M0AR = (uint32_t)(rx_buf);
  // data length
  __HAL_DMA_SET_COUNTER(hdma_spi_rx_, num);

  __HAL_DMA_ENABLE_IT(hdma_spi_rx_, DMA_IT_TC);

  // disable DMA
  __HAL_DMA_DISABLE(hdma_spi_tx_);

  while (hdma_spi_tx_->Instance->CR & DMA_SxCR_EN) {
    __HAL_DMA_DISABLE(hdma_spi_tx_);
  }

  __HAL_DMA_CLEAR_FLAG(hdma_spi_tx_, DMA_LISR_TCIF3);

  hdma_spi_tx_->Instance->PAR = (uint32_t) & (SPI1->DR);
  // memory buffer 1
  hdma_spi_tx_->Instance->M0AR = (uint32_t)(tx_buf);
  // data length
  __HAL_DMA_SET_COUNTER(hdma_spi_tx_, num);
}

void IMU_typeC::SPI_DMA_enable(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr) {
  // disable DMA
  __HAL_DMA_DISABLE(hdma_spi_rx_);
  __HAL_DMA_DISABLE(hdma_spi_tx_);
  while (hdma_spi_rx_->Instance->CR & DMA_SxCR_EN) {
    __HAL_DMA_DISABLE(hdma_spi_rx_);
  }
  while (hdma_spi_tx_->Instance->CR & DMA_SxCR_EN) {
    __HAL_DMA_DISABLE(hdma_spi_tx_);
  }
  // clear flag
  __HAL_DMA_CLEAR_FLAG(hspi_->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi_->hdmarx));
  __HAL_DMA_CLEAR_FLAG(hspi_->hdmarx, __HAL_DMA_GET_HT_FLAG_INDEX(hspi_->hdmarx));
  __HAL_DMA_CLEAR_FLAG(hspi_->hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(hspi_->hdmarx));
  __HAL_DMA_CLEAR_FLAG(hspi_->hdmarx, __HAL_DMA_GET_DME_FLAG_INDEX(hspi_->hdmarx));
  __HAL_DMA_CLEAR_FLAG(hspi_->hdmarx, __HAL_DMA_GET_FE_FLAG_INDEX(hspi_->hdmarx));

  __HAL_DMA_CLEAR_FLAG(hspi_->hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi_->hdmatx));
  __HAL_DMA_CLEAR_FLAG(hspi_->hdmatx, __HAL_DMA_GET_HT_FLAG_INDEX(hspi_->hdmatx));
  __HAL_DMA_CLEAR_FLAG(hspi_->hdmatx, __HAL_DMA_GET_TE_FLAG_INDEX(hspi_->hdmatx));
  __HAL_DMA_CLEAR_FLAG(hspi_->hdmatx, __HAL_DMA_GET_DME_FLAG_INDEX(hspi_->hdmatx));
  __HAL_DMA_CLEAR_FLAG(hspi_->hdmatx, __HAL_DMA_GET_FE_FLAG_INDEX(hspi_->hdmatx));
  // set memory address
  hdma_spi_rx_->Instance->M0AR = rx_buf;
  hdma_spi_tx_->Instance->M0AR = tx_buf;
  // set data length
  __HAL_DMA_SET_COUNTER(hdma_spi_rx_, ndtr);
  __HAL_DMA_SET_COUNTER(hdma_spi_tx_, ndtr);
  // enable DMA
  __HAL_DMA_ENABLE(hdma_spi_rx_);
  __HAL_DMA_ENABLE(hdma_spi_tx_);
}

void IMU_typeC::imu_cmd_spi_dma() {
  UBaseType_t uxSavedInterruptStatus;
  uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

  if ((gyro_update_flag & (1 << IMU_DR_SHIFTS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) &&
      !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(accel_update_flag & (1 << IMU_SPI_SHIFTS)) &&
      !(accel_temp_update_flag & (1 << IMU_SPI_SHIFTS))) {
    gyro_update_flag &= ~(1 << IMU_DR_SHIFTS);
    gyro_update_flag |= (1 << IMU_SPI_SHIFTS);

    HAL_GPIO_WritePin(BMI088_param_.CS_GYRO_Port, BMI088_param_.CS_GYRO_Pin, GPIO_PIN_RESET);
    SPI_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGTH);
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    return;
  }

  if ((accel_update_flag & (1 << IMU_DR_SHIFTS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) &&
      !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(gyro_update_flag & (1 << IMU_SPI_SHIFTS)) &&
      !(accel_temp_update_flag & (1 << IMU_SPI_SHIFTS))) {
    accel_update_flag &= ~(1 << IMU_DR_SHIFTS);
    accel_update_flag |= (1 << IMU_SPI_SHIFTS);

    HAL_GPIO_WritePin(BMI088_param_.CS_ACCEL_Port, BMI088_param_.CS_ACCEL_Pin, GPIO_PIN_RESET);
    SPI_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGTH);
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    return;
  }

  if ((accel_temp_update_flag & (1 << IMU_DR_SHIFTS)) &&
      !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) &&
      !(gyro_update_flag & (1 << IMU_SPI_SHIFTS)) && !(accel_update_flag & (1 << IMU_SPI_SHIFTS))) {
    accel_temp_update_flag &= ~(1 << IMU_DR_SHIFTS);
    accel_temp_update_flag |= (1 << IMU_SPI_SHIFTS);

    HAL_GPIO_WritePin(BMI088_param_.CS_ACCEL_Port, BMI088_param_.CS_ACCEL_Pin, GPIO_PIN_RESET);
    SPI_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf,
                   SPI_DMA_ACCEL_TEMP_LENGTH);
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    return;
  }
  taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

void DMACallbackWrapper(SPI_HandleTypeDef* hspi) {
  if (__HAL_DMA_GET_FLAG(hspi->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi->hdmarx)) != RESET) {
    __HAL_DMA_CLEAR_FLAG(hspi->hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi->hdmarx));

    IMU_typeC* imu = IMU_typeC::FindInstance(hspi);
    if (!imu) return;

    // gyro read over
    if (imu->gyro_update_flag & (1 << IMU_SPI_SHIFTS)) {
      imu->gyro_update_flag &= ~(1 << IMU_SPI_SHIFTS);
      imu->gyro_update_flag |= (1 << IMU_UPDATE_SHIFTS);
      HAL_GPIO_WritePin(imu->BMI088_param_.CS_GYRO_Port, imu->BMI088_param_.CS_GYRO_Pin,
                        GPIO_PIN_SET);
    }
    // accel read over
    if (imu->accel_update_flag & (1 << IMU_SPI_SHIFTS)) {
      imu->accel_update_flag &= ~(1 << IMU_SPI_SHIFTS);
      imu->accel_update_flag |= (1 << IMU_UPDATE_SHIFTS);
      HAL_GPIO_WritePin(imu->BMI088_param_.CS_ACCEL_Port, imu->BMI088_param_.CS_ACCEL_Pin,
                        GPIO_PIN_SET);
    }
    // temperature read over
    if (imu->accel_temp_update_flag & (1 << IMU_SPI_SHIFTS)) {
      imu->accel_temp_update_flag &= ~(1 << IMU_SPI_SHIFTS);
      imu->accel_temp_update_flag |= (1 << IMU_UPDATE_SHIFTS);
      HAL_GPIO_WritePin(imu->BMI088_param_.CS_ACCEL_Port, imu->BMI088_param_.CS_ACCEL_Pin,
                        GPIO_PIN_SET);
    }

    imu->imu_cmd_spi_dma();

    if (imu->gyro_update_flag & (1 << IMU_UPDATE_SHIFTS)) {
      imu->gyro_update_flag &= ~(1 << IMU_UPDATE_SHIFTS);
      imu->gyro_update_flag |= (1 << IMU_NOTIFY_SHIFTS);
      imu->RxCompleteCallback();
    }
  }
}

void IMU_typeC::RxCompleteCallback() {}

#endif  // !BOARD_DM_MC_02

} /* namespace bsp */

#ifndef BOARD_DM_MC_02
void RM_DMA_IRQHandler(SPI_HandleTypeDef* hspi) { bsp::DMACallbackWrapper(hspi); }
#endif

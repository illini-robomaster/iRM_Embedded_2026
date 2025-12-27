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

#pragma once

#include <map>

#include "bsp_gpio.h"
#include "bsp_heater.h"
#include "cmsis_os.h"
#include "spi.h"

// acc (6 bytes) + temp (2 bytes) + gyro (6 bytes) + mag (6 bytes)
#define MPU6500_SIZEOF_DATA 20

#define IST8310_DATA_READY_BIT 2
#define IST8310_NO_ERROR 0x00
#define IST8310_NO_SENSOR 0x40

#define BMI088_ACC_CHIP_ID 0x00  // the register is  " Who am I "
#define BMI088_ACC_CHIP_ID_VALUE 0x1E

#define BMI088_ACC_ERR_REG 0x02
#define BMI088_ACCEL_CONGIF_ERROR_SHFITS 0x2
#define BMI088_ACCEL_CONGIF_ERROR (1 << BMI088_ACCEL_CONGIF_ERROR_SHFITS)
#define BMI088_FATAL_ERROR_SHFITS 0x0
#define BMI088_FATAL_ERROR (1 << BMI088_FATAL_ERROR)

#define BMI088_ACC_STATUS 0x03
#define BMI088_ACCEL_DRDY_SHFITS 0x7
#define BMI088_ACCEL_DRDY (1 << BMI088_ACCEL_DRDY_SHFITS)

#define BMI088_ACCEL_XOUT_L 0x12
#define BMI088_ACCEL_XOUT_M 0x13
#define BMI088_ACCEL_YOUT_L 0x14
#define BMI088_ACCEL_YOUT_M 0x15
#define BMI088_ACCEL_ZOUT_L 0x16
#define BMI088_ACCEL_ZOUT_M 0x17

#define BMI088_SENSORTIME_DATA_L 0x18
#define BMI088_SENSORTIME_DATA_M 0x19
#define BMI088_SENSORTIME_DATA_H 0x1A

#define BMI088_ACC_INT_STAT_1 0x1D
#define BMI088_ACCEL_DRDY_INTERRUPT_SHFITS 0x7
#define BMI088_ACCEL_DRDY_INTERRUPT (1 << BMI088_ACCEL_DRDY_INTERRUPT_SHFITS)

#define BMI088_TEMP_M 0x22

#define BMI088_TEMP_L 0x23

#define BMI088_ACC_CONF 0x40
#define BMI088_ACC_CONF_MUST_Set 0x80
#define BMI088_ACC_BWP_SHFITS 0x4
#define BMI088_ACC_OSR4 (0x0 << BMI088_ACC_BWP_SHFITS)
#define BMI088_ACC_OSR2 (0x1 << BMI088_ACC_BWP_SHFITS)
#define BMI088_ACC_NORMAL (0x2 << BMI088_ACC_BWP_SHFITS)

#define BMI088_ACC_ODR_SHFITS 0x0
#define BMI088_ACC_12_5_HZ (0x5 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_25_HZ (0x6 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_50_HZ (0x7 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_100_HZ (0x8 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_200_HZ (0x9 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_400_HZ (0xA << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_800_HZ (0xB << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_1600_HZ (0xC << BMI088_ACC_ODR_SHFITS)

#define BMI088_ACC_RANGE 0x41

#define BMI088_ACC_RANGE_SHFITS 0x0
#define BMI088_ACC_RANGE_3G (0x0 << BMI088_ACC_RANGE_SHFITS)
#define BMI088_ACC_RANGE_6G (0x1 << BMI088_ACC_RANGE_SHFITS)
#define BMI088_ACC_RANGE_12G (0x2 << BMI088_ACC_RANGE_SHFITS)
#define BMI088_ACC_RANGE_24G (0x3 << BMI088_ACC_RANGE_SHFITS)

#define BMI088_INT1_IO_CTRL 0x53
#define BMI088_ACC_INT1_IO_ENABLE_SHFITS 0x3
#define BMI088_ACC_INT1_IO_ENABLE (0x1 << BMI088_ACC_INT1_IO_ENABLE_SHFITS)
#define BMI088_ACC_INT1_GPIO_MODE_SHFITS 0x2
#define BMI088_ACC_INT1_GPIO_PP (0x0 << BMI088_ACC_INT1_GPIO_MODE_SHFITS)
#define BMI088_ACC_INT1_GPIO_OD (0x1 << BMI088_ACC_INT1_GPIO_MODE_SHFITS)
#define BMI088_ACC_INT1_GPIO_LVL_SHFITS 0x1
#define BMI088_ACC_INT1_GPIO_LOW (0x0 << BMI088_ACC_INT1_GPIO_LVL_SHFITS)
#define BMI088_ACC_INT1_GPIO_HIGH (0x1 << BMI088_ACC_INT1_GPIO_LVL_SHFITS)

#define BMI088_INT2_IO_CTRL 0x54
#define BMI088_ACC_INT2_IO_ENABLE_SHFITS 0x3
#define BMI088_ACC_INT2_IO_ENABLE (0x1 << BMI088_ACC_INT2_IO_ENABLE_SHFITS)
#define BMI088_ACC_INT2_GPIO_MODE_SHFITS 0x2
#define BMI088_ACC_INT2_GPIO_PP (0x0 << BMI088_ACC_INT2_GPIO_MODE_SHFITS)
#define BMI088_ACC_INT2_GPIO_OD (0x1 << BMI088_ACC_INT2_GPIO_MODE_SHFITS)
#define BMI088_ACC_INT2_GPIO_LVL_SHFITS 0x1
#define BMI088_ACC_INT2_GPIO_LOW (0x0 << BMI088_ACC_INT2_GPIO_LVL_SHFITS)
#define BMI088_ACC_INT2_GPIO_HIGH (0x1 << BMI088_ACC_INT2_GPIO_LVL_SHFITS)

#define BMI088_INT_MAP_DATA 0x58
#define BMI088_ACC_INT2_DRDY_INTERRUPT_SHFITS 0x6
#define BMI088_ACC_INT2_DRDY_INTERRUPT (0x1 << BMI088_ACC_INT2_DRDY_INTERRUPT_SHFITS)
#define BMI088_ACC_INT1_DRDY_INTERRUPT_SHFITS 0x2
#define BMI088_ACC_INT1_DRDY_INTERRUPT (0x1 << BMI088_ACC_INT1_DRDY_INTERRUPT_SHFITS)

#define BMI088_ACC_SELF_TEST 0x6D
#define BMI088_ACC_SELF_TEST_OFF 0x00
#define BMI088_ACC_SELF_TEST_POSITIVE_SIGNAL 0x0D
#define BMI088_ACC_SELF_TEST_NEGATIVE_SIGNAL 0x09

#define BMI088_ACC_PWR_CONF 0x7C
#define BMI088_ACC_PWR_SUSPEND_MODE 0x03
#define BMI088_ACC_PWR_ACTIVE_MODE 0x00

#define BMI088_ACC_PWR_CTRL 0x7D
#define BMI088_ACC_ENABLE_ACC_OFF 0x00
#define BMI088_ACC_ENABLE_ACC_ON 0x04

#define BMI088_ACC_SOFTRESET 0x7E
#define BMI088_ACC_SOFTRESET_VALUE 0xB6

#define BMI088_GYRO_CHIP_ID 0x00
#define BMI088_GYRO_CHIP_ID_VALUE 0x0F

#define BMI088_GYRO_X_L 0x02
#define BMI088_GYRO_X_H 0x03
#define BMI088_GYRO_Y_L 0x04
#define BMI088_GYRO_Y_H 0x05
#define BMI088_GYRO_Z_L 0x06
#define BMI088_GYRO_Z_H 0x07

#define BMI088_GYRO_INT_STAT_1 0x0A
#define BMI088_GYRO_DYDR_SHFITS 0x7
#define BMI088_GYRO_DYDR (0x1 << BMI088_GYRO_DYDR_SHFITS)

#define BMI088_GYRO_RANGE 0x0F
#define BMI088_GYRO_RANGE_SHFITS 0x0
#define BMI088_GYRO_2000 (0x0 << BMI088_GYRO_RANGE_SHFITS)
#define BMI088_GYRO_1000 (0x1 << BMI088_GYRO_RANGE_SHFITS)
#define BMI088_GYRO_500 (0x2 << BMI088_GYRO_RANGE_SHFITS)
#define BMI088_GYRO_250 (0x3 << BMI088_GYRO_RANGE_SHFITS)
#define BMI088_GYRO_125 (0x4 << BMI088_GYRO_RANGE_SHFITS)

#define BMI088_GYRO_BANDWIDTH 0x10
// the first num means Output data  rate, the second num means bandwidth
#define BMI088_GYRO_BANDWIDTH_MUST_Set 0x80
#define BMI088_GYRO_2000_532_HZ 0x00
#define BMI088_GYRO_2000_230_HZ 0x01
#define BMI088_GYRO_1000_116_HZ 0x02
#define BMI088_GYRO_400_47_HZ 0x03
#define BMI088_GYRO_200_23_HZ 0x04
#define BMI088_GYRO_100_12_HZ 0x05
#define BMI088_GYRO_200_64_HZ 0x06
#define BMI088_GYRO_100_32_HZ 0x07

#define BMI088_GYRO_LPM1 0x11
#define BMI088_GYRO_NORMAL_MODE 0x00
#define BMI088_GYRO_SUSPEND_MODE 0x80
#define BMI088_GYRO_DEEP_SUSPEND_MODE 0x20

#define BMI088_GYRO_SOFTRESET 0x14
#define BMI088_GYRO_SOFTRESET_VALUE 0xB6

#define BMI088_GYRO_CTRL 0x15
#define BMI088_DRDY_OFF 0x00
#define BMI088_DRDY_ON 0x80

#define BMI088_GYRO_INT3_INT4_IO_CONF 0x16
#define BMI088_GYRO_INT4_GPIO_MODE_SHFITS 0x3
#define BMI088_GYRO_INT4_GPIO_PP (0x0 << BMI088_GYRO_INT4_GPIO_MODE_SHFITS)
#define BMI088_GYRO_INT4_GPIO_OD (0x1 << BMI088_GYRO_INT4_GPIO_MODE_SHFITS)
#define BMI088_GYRO_INT4_GPIO_LVL_SHFITS 0x2
#define BMI088_GYRO_INT4_GPIO_LOW (0x0 << BMI088_GYRO_INT4_GPIO_LVL_SHFITS)
#define BMI088_GYRO_INT4_GPIO_HIGH (0x1 << BMI088_GYRO_INT4_GPIO_LVL_SHFITS)
#define BMI088_GYRO_INT3_GPIO_MODE_SHFITS 0x1
#define BMI088_GYRO_INT3_GPIO_PP (0x0 << BMI088_GYRO_INT3_GPIO_MODE_SHFITS)
#define BMI088_GYRO_INT3_GPIO_OD (0x1 << BMI088_GYRO_INT3_GPIO_MODE_SHFITS)
#define BMI088_GYRO_INT3_GPIO_LVL_SHFITS 0x0
#define BMI088_GYRO_INT3_GPIO_LOW (0x0 << BMI088_GYRO_INT3_GPIO_LVL_SHFITS)
#define BMI088_GYRO_INT3_GPIO_HIGH (0x1 << BMI088_GYRO_INT3_GPIO_LVL_SHFITS)

#define BMI088_GYRO_INT3_INT4_IO_MAP 0x18

#define BMI088_GYRO_DRDY_IO_OFF 0x00
#define BMI088_GYRO_DRDY_IO_INT3 0x01
#define BMI088_GYRO_DRDY_IO_INT4 0x80
#define BMI088_GYRO_DRDY_IO_BOTH (BMI088_GYRO_DRDY_IO_INT3 | BMI088_GYRO_DRDY_IO_INT4)

#define BMI088_GYRO_SELF_TEST 0x3C
#define BMI088_GYRO_RATE_OK_SHFITS 0x4
#define BMI088_GYRO_RATE_OK (0x1 << BMI088_GYRO_RATE_OK_SHFITS)
#define BMI088_GYRO_BIST_FAIL_SHFITS 0x2
#define BMI088_GYRO_BIST_FAIL (0x1 << BMI088_GYRO_BIST_FAIL_SHFITS)
#define BMI088_GYRO_BIST_RDY_SHFITS 0x1
#define BMI088_GYRO_BIST_RDY (0x1 << BMI088_GYRO_BIST_RDY_SHFITS)
#define BMI088_GYRO_TRIG_BIST_SHFITS 0x0
#define BMI088_GYRO_TRIG_BIST (0x1 << BMI088_GYRO_TRIG_BIST_SHFITS)

#define BMI088_TEMP_FACTOR 0.125f
#define BMI088_TEMP_OFFSET 23.0f

#define BMI088_WRITE_ACCEL_REG_NUM 6
#define BMI088_WRITE_GYRO_REG_NUM 6

#define BMI088_GYRO_DATA_READY_BIT 0
#define BMI088_ACCEL_DATA_READY_BIT 1
#define BMI088_ACCEL_TEMP_DATA_READY_BIT 2

#define BMI088_LONG_DELAY_TIME 80
#define BMI088_COM_WAIT_SENSOR_TIME 150

#define BMI088_ACCEL_IIC_ADDRESSE (0x18 << 1)
#define BMI088_GYRO_IIC_ADDRESSE (0x68 << 1)

#define BMI088_ACCEL_RANGE_3G
//#define BMI088_ACCEL_RANGE_6G
//#define BMI088_ACCEL_RANGE_12G
//#define BMI088_ACCEL_RANGE_24G

#define BMI088_GYRO_RANGE_2000
//#define BMI088_GYRO_RANGE_1000
//#define BMI088_GYRO_RANGE_500
//#define BMI088_GYRO_RANGE_250
//#define BMI088_GYRO_RANGE_125

#define BMI088_ACCEL_3G_SEN 0.0008974358974f
#define BMI088_ACCEL_6G_SEN 0.00179443359375f
#define BMI088_ACCEL_12G_SEN 0.0035888671875f
#define BMI088_ACCEL_24G_SEN 0.007177734375f

#define BMI088_GYRO_2000_SEN 0.00106526443603169529841533860381f
#define BMI088_GYRO_1000_SEN 0.00053263221801584764920766930190693f
#define BMI088_GYRO_500_SEN 0.00026631610900792382460383465095346f
#define BMI088_GYRO_250_SEN 0.00013315805450396191230191732547673f
#define BMI088_GYRO_125_SEN 0.000066579027251980956150958662738366f

#define SPI_DMA_GYRO_LENGTH 8
#define SPI_DMA_ACCEL_LENGTH 9
#define SPI_DMA_ACCEL_TEMP_LENGTH 4

#define IMU_DR_SHIFTS 0
#define IMU_SPI_SHIFTS 1
#define IMU_UPDATE_SHIFTS 2
#define IMU_NOTIFY_SHIFTS 3

#define BMI088_GYRO_RX_BUF_DATA_OFFSET 1
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2

#define IST8310_RX_BUF_DATA_OFFSET 16

#define INS_TASK_INIT_TIME 7

#define INS_YAW_ADDRESS_OFFSET 0
#define INS_PITCH_ADDRESS_OFFSET 1
#define INS_ROLL_ADDRESS_OFFSET 2

#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

#define INS_ACCEL_X_ADDRESS_OFFSET 0
#define INS_ACCEL_Y_ADDRESS_OFFSET 1
#define INS_ACCEL_Z_ADDRESS_OFFSET 2

#define INS_MAG_X_ADDRESS_OFFSET 0
#define INS_MAG_Y_ADDRESS_OFFSET 1
#define INS_MAG_Z_ADDRESS_OFFSET 2

namespace bsp {

typedef struct {
  float x;
  float y;
  float z;
} vec3f_t;

// MPU6500 class is not supported on BOARD_DM_MC_02 (uses STM32F4-specific SPI callbacks)
#ifndef BOARD_DM_MC_02

class MPU6500 : public GPIT {
 public:
  /**
   * @brief constructor for a MPU6500 IMU sensor
   *
   * @param hspi         HAL SPI handle associated with the sensor
   * @param chip_select  chip select gpio pin
   * @param int_pin      interrupt pin number
   */
  MPU6500(SPI_HandleTypeDef* hspi, const GPIO& chip_select, uint16_t int_pin);

  /**
   * @brief reset sensor registers
   */
  void Reset();

  // 3-axis accelerometer
  vec3f_t acce;
  // 3-axis gyroscope
  vec3f_t gyro;
  // 3-axis magnetometer
  vec3f_t mag;
  // sensor temperature
  float temp;
  // sensor timestamp
  uint32_t timestamp = 0;

 private:
  /**
   * @brief sample latest sensor data
   */
  void UpdateData();

  void IST8310Init();
  void WriteReg(uint8_t reg, uint8_t data);
  void WriteRegs(uint8_t reg_start, uint8_t* data, uint8_t len);
  void ReadReg(uint8_t reg, uint8_t* data);
  void ReadRegs(uint8_t reg_start, uint8_t* data, uint8_t len);

  void SPITxRxCpltCallback();
  void IntCallback() override final;

  SPI_HandleTypeDef* hspi_;
  GPIO chip_select_;

  uint8_t io_buff_[MPU6500_SIZEOF_DATA + 1];  // spi tx+rx buffer

  // global interrupt wrapper
  // TODO(alvin): try to support multiple instances in the future
  static void SPITxRxCpltCallback(SPI_HandleTypeDef* hspi);
  static MPU6500* mpu6500;
};

typedef struct {
  I2C_HandleTypeDef* hi2c;
  uint16_t int_pin;
  GPIO_TypeDef* rst_group;
  uint16_t rst_pin;
} IST8310_init_t;

typedef struct {
  uint8_t status;
  float mag[3];
} IST8310_real_data_t;

class IMU_typeC;

class IST8310 : public GPIT {
 public:
  IST8310(IST8310_init_t init, IMU_typeC* imu = nullptr);
  IST8310(I2C_HandleTypeDef* hi2c, uint16_t int_pin, GPIO_TypeDef* rst_group, uint16_t rst_pin,
          IMU_typeC* imu = nullptr);
  bool IsReady();
  void ist8310_read_over(uint8_t* status_buf, IST8310_real_data_t* ist8310_real_data);
  float mag[3];

 private:
  uint8_t Init();
  void ist8310_read_mag(float mag_[3]);
  IMU_typeC* imu_;
  void IntCallback() final;

  void ist8310_RST_H();
  void ist8310_RST_L();
  uint8_t ist8310_IIC_read_single_reg(uint8_t reg);
  void ist8310_IIC_write_single_reg(uint8_t reg, uint8_t data);
  void ist8310_IIC_read_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len);
  void ist8310_IIC_write_muli_reg(uint8_t reg, uint8_t* data, uint8_t len);

  I2C_HandleTypeDef* hi2c_;
  GPIO_TypeDef* rst_group_;
  uint16_t rst_pin_;
};

#endif  // !BOARD_DM_MC_02 - End of MPU6500 and IST8310

typedef struct {
  uint8_t status;
  int16_t accel[3];
  int16_t temp;
  int16_t gyro[3];
} __packed BMI088_raw_data_t;

typedef struct {
  uint8_t status;
  float accel[3];
  float temp;
  float gyro[3];
  float time;
} BMI088_real_data_t;

enum {
  BMI088_NO_ERROR = 0x00,
  BMI088_ACC_PWR_CTRL_ERROR = 0x01,
  BMI088_ACC_PWR_CONF_ERROR = 0x02,
  BMI088_ACC_CONF_ERROR = 0x03,
  BMI088_ACC_SELF_TEST_ERROR = 0x04,
  BMI088_ACC_RANGE_ERROR = 0x05,
  BMI088_INT1_IO_CTRL_ERROR = 0x06,
  BMI088_INT_MAP_DATA_ERROR = 0x07,
  BMI088_GYRO_RANGE_ERROR = 0x08,
  BMI088_GYRO_BANDWIDTH_ERROR = 0x09,
  BMI088_GYRO_LPM1_ERROR = 0x0A,
  BMI088_GYRO_CTRL_ERROR = 0x0B,
  BMI088_GYRO_INT3_INT4_IO_CONF_ERROR = 0x0C,
  BMI088_GYRO_INT3_INT4_IO_MAP_ERROR = 0x0D,

  BMI088_SELF_TEST_ACCEL_ERROR = 0x80,
  BMI088_SELF_TEST_GYRO_ERROR = 0x40,
  BMI088_NO_SENSOR = 0xFF,
};

typedef struct {
  SPI_HandleTypeDef* hspi;
  GPIO_TypeDef* CS_ACCEL_Port;
  uint16_t CS_ACCEL_Pin;
  GPIO_TypeDef* CS_GYRO_Port;
  uint16_t CS_GYRO_Pin;
} BMI088_init_t;

class BMI088 {
 public:
  BMI088(BMI088_init_t init);
  BMI088(SPI_HandleTypeDef* hspi, GPIO_TypeDef* CS_ACCEL_Port, uint16_t CS_ACCEL_Pin,
         GPIO_TypeDef* CS_GYRO_Port, uint16_t CS_GYRO_Pin);
  bool IsReady();
  void Read(float gyro[3], float accel[3], float* temperate);
  void temperature_read_over(uint8_t* rx_buf, float* temperate);
  void accel_read_over(uint8_t* rx_buf, float accel[3], float* time);
  void gyro_read_over(uint8_t* rx_buf, float gyro[3]);

 private:
  SPI_HandleTypeDef* hspi_;
  GPIO_TypeDef* CS1_ACCEL_GPIO_Port_;
  uint16_t CS1_ACCEL_Pin_;
  GPIO_TypeDef* CS1_GYRO_GPIO_Port_;
  uint16_t CS1_GYRO_Pin_;

  uint8_t Init();

  bool bmi088_accel_init();
  bool bmi088_gyro_init();

  void BMI088_ACCEL_NS_L();
  void BMI088_ACCEL_NS_H();

  void BMI088_GYRO_NS_L();
  void BMI088_GYRO_NS_H();

  uint8_t BMI088_read_write_byte(uint8_t tx_data);

  void BMI088_write_single_reg(uint8_t reg, uint8_t data);
  void BMI088_read_single_reg(uint8_t reg, uint8_t* data);
  void BMI088_read_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len);

  void BMI088_accel_write_single_reg(uint8_t reg, uint8_t data);
  void BMI088_accel_read_single_reg(uint8_t reg, uint8_t* data);
  void BMI088_accel_read_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len);

  void BMI088_gyro_write_single_reg(uint8_t reg, uint8_t data);
  void BMI088_gyro_read_single_reg(uint8_t reg, uint8_t* data);
  void BMI088_gyro_read_muli_reg(uint8_t reg, uint8_t* buf, uint8_t len);
};

// Accel_INT, Gyro_INT, IMU_typeC classes use STM32F4-specific DMA registers
#ifndef BOARD_DM_MC_02

class Accel_INT : public GPIT {
 public:
  Accel_INT(uint16_t INT_pin, IMU_typeC* imu);

 private:
  IMU_typeC* imu_;
  void IntCallback() final;
};

class Gyro_INT : public GPIT {
 public:
  Gyro_INT(uint16_t INT_pin, IMU_typeC* imu);

 private:
  IMU_typeC* imu_;
  void IntCallback() final;
};

typedef struct {
  IST8310_init_t IST8310;
  BMI088_init_t BMI088;
  heater_init_t heater;

  SPI_HandleTypeDef* hspi;
  DMA_HandleTypeDef* hdma_spi_rx;
  DMA_HandleTypeDef* hdma_spi_tx;

  uint16_t Accel_INT_pin_;
  uint16_t Gyro_INT_pin_;
} IMU_typeC_init_t;

class IMU_typeC {
 public:
  IMU_typeC(IMU_typeC_init_t init, bool useMag = true);
  void Calibrate();
  bool CaliDone();
  void Update();
  bool DataReady();

  float INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
  float INS_angle[3] = {0.0f, 0.0f, 0.0f};
  float Temp = 0;
  float TempPWM = 0;

 protected:
  virtual void RxCompleteCallback();

 private:
  bool useMag_;
  unsigned long count_ = 0;
  bool calibrate_ = false;
  bool calidone_ = false;

  unsigned zeroDriftTry = 1000;
  float zeroDrift[3] = {0, 0, 0};
  float zeroDriftTemp[3] = {0, 0, 0};

  float accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
  float accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
  float accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
  const float fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

  friend class IST8310;

  IST8310 IST8310_;
  BMI088 BMI088_;
  Heater heater_;

  IST8310_init_t IST8310_param_;
  BMI088_init_t BMI088_param_;
  heater_init_t heater_param_;

  friend class Accel_INT;
  friend class Gyro_INT;

  Accel_INT Accel_INT_;
  Gyro_INT Gyro_INT_;

  SPI_HandleTypeDef* hspi_;
  DMA_HandleTypeDef* hdma_spi_rx_;
  DMA_HandleTypeDef* hdma_spi_tx_;

  BMI088_real_data_t BMI088_real_data_;
  IST8310_real_data_t IST8310_real_data_;

  volatile uint8_t gyro_update_flag = 0;
  volatile uint8_t accel_update_flag = 0;
  volatile uint8_t accel_temp_update_flag = 0;
  volatile uint8_t mag_update_flag = 0;
  volatile uint8_t imu_start_dma_flag = 0;

  uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGTH];
  uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGTH] = {0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGTH];
  uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGTH] = {0x92, 0xFF, 0xFF, 0xFF, 0xFF,
                                                    0xFF, 0xFF, 0xFF, 0xFF};

  uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGTH];
  uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGTH] = {0xA2, 0xFF, 0xFF, 0xFF};

  void AHRS_init(float quat[4], float accel[3], float mag[3]);
  void AHRS_update(float quat[4], float time, float gyro[3], float accel[3], float mag[3]);

  void SPI_DMA_init(uint32_t tx_buf, uint32_t rx_buf, uint16_t num);
  void SPI_DMA_enable(uint32_t tx_buf, uint32_t rx_buf, uint16_t ndtr);

  void imu_cmd_spi_dma();

  friend void DMACallbackWrapper(SPI_HandleTypeDef* hspi);

  static std::map<SPI_HandleTypeDef*, IMU_typeC*> spi_ptr_map;
  static IMU_typeC* FindInstance(SPI_HandleTypeDef* hspi);

  float TempControl(float real_temp);
  void GetAngle(float q[4], float* yaw, float* pitch, float* roll);
};

#endif  // !BOARD_DM_MC_02 - End of IMU_typeC

} /* namespace bsp */

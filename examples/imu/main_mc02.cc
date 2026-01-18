/****************************************************************************
 *                                                                          *
 *  Copyright (C) 2024 RoboMaster.                                          *
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

/**
 * @brief BMI088 IMU example for DM_MC_02 board
 * 
 * This example demonstrates reading accelerometer and gyroscope data
 * from the BMI088 IMU via SPI2.
 * 
 * Hardware connections:
 * - SPI2: PB13 (SCK), PC1 (MOSI), PC2_C (MISO)
 * - CS_ACCEL: PC0
 * - CS_GYRO: PC3_C
 * - INT1_ACCEL: PE10 (EXTI10)
 * - INT1_GYRO: PE12 (EXTI12)
 * 
 * Output format:
 * Accel: X=xxx.xx Y=xxx.xx Z=xxx.xx [m/s^2]
 * Gyro: X=xxx.xx Y=xxx.xx Z=xxx.xx [rad/s]
 * Temp: xx.x [°C]
 */

#include "main.h"
#include "spi.h"
#include "bsp_print.h"
#include "bsp_gpio.h"
#include "bsp_imu.h"
#include "MahonyAHRS.h"
#include "cmsis_os.h"
#include <cmath>

static bsp::BMI088* bmi088 = nullptr;
static float gyro[3];
static float accel[3];
static float temp;
static volatile uint32_t imu_update_count = 0;

// Quaternion and Euler angles
static float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
static float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;

// Convert radians to degrees
#define RAD2DEG(x) ((x) * 180.0f / M_PI)
#define DEG2RAD(x) ((x) * M_PI / 180.0f)

/**
 * @brief IMU Gyroscope interrupt handler class
 */
class IMU_GYRO_INT : public bsp::GPIT {
 public:
    IMU_GYRO_INT() : GPIT(INT1_GYRO_Pin) {}
    
    void IntCallback() override {
        imu_update_count++;
    }
};

/**
 * @brief IMU Accelerometer interrupt handler class
 */
class IMU_ACCEL_INT : public bsp::GPIT {
 public:
    IMU_ACCEL_INT() : GPIT(INT1_ACCEL_Pin) {}
    
    void IntCallback() override {
        // Accelerometer data ready
    }
};

// Global interrupt handlers
static IMU_GYRO_INT* gyro_int = nullptr;
static IMU_ACCEL_INT* accel_int = nullptr;

void RM_RTOS_Init(void) {
    // Use UART7 for debug output (PE7=RX, PE8=TX)
    print_use_uart(&huart7);
    
    // Create interrupt handlers
    gyro_int = new IMU_GYRO_INT();
    accel_int = new IMU_ACCEL_INT();
}

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    
    uint32_t last_count = 0;
    uint32_t update_rate = 0;
    
    print("\r\n=== BMI088 IMU Example for MC02 ===\r\n");
    print("Task stack size: %u bytes\r\n\r\n", (unsigned int)(2048 * 4));
    
    // Initialize BMI088
    print("Initializing BMI088...\r\n");
    
    bmi088 = new bsp::BMI088(&hspi2, 
                             CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin,
                             CS1_GYRO_GPIO_Port, CS1_GYRO_Pin);
    
    if (!bmi088->IsReady()) {
        print("BMI088 Init FAILED!\r\n");
        print("\r\nPlease check hardware connections:\r\n");
        print("  SPI2: PB13(SCK), PC1(MOSI), PC2_C(MISO)\r\n");
        print("  CS_ACCEL: PC0, CS_GYRO: PC3_C\r\n");
        
        while (1) {
            osDelay(1000);
            print("IMU Init Error\r\n");
        }
    }
    
    print("BMI088 Init OK!\r\n");
    print("  Accel: +/-3g, 800Hz\r\n");
    print("  Gyro: +/-2000dps, 1000Hz\r\n\r\n");
    
    osDelay(100);
    
    uint32_t tick_start = HAL_GetTick();
    uint32_t print_tick = HAL_GetTick();
    
    while (1) {
        // Read all IMU data using the BMI088 class
        bmi088->Read(gyro, accel, &temp);
        
        // Convert gyro from deg/s to rad/s for MahonyAHRS
        float gx_rad = DEG2RAD(gyro[0]);
        float gy_rad = DEG2RAD(gyro[1]);
        float gz_rad = DEG2RAD(gyro[2]);
        
        // Update AHRS at 1000Hz (matching sampleFreq in MahonyAHRS.c)
        MahonyAHRSupdateIMU(quat, gx_rad, gy_rad, gz_rad, 
                           accel[0], accel[1], accel[2]);
        
        // Calculate Euler angles from quaternion
        // Roll (x-axis rotation)
        float sinr_cosp = 2.0f * (quat[0] * quat[1] + quat[2] * quat[3]);
        float cosr_cosp = 1.0f - 2.0f * (quat[1] * quat[1] + quat[2] * quat[2]);
        roll = atan2f(sinr_cosp, cosr_cosp);
        
        // Pitch (y-axis rotation)
        float sinp = 2.0f * (quat[0] * quat[2] - quat[3] * quat[1]);
        if (fabsf(sinp) >= 1.0f)
            pitch = copysignf(M_PI / 2.0f, sinp);
        else
            pitch = asinf(sinp);
        
        // Yaw (z-axis rotation)
        float siny_cosp = 2.0f * (quat[0] * quat[3] + quat[1] * quat[2]);
        float cosy_cosp = 1.0f - 2.0f * (quat[2] * quat[2] + quat[3] * quat[3]);
        yaw = atan2f(siny_cosp, cosy_cosp);
        
        // Calculate update rate every second
        uint32_t now = HAL_GetTick();
        if (now - tick_start >= 1000) {
            update_rate = imu_update_count - last_count;
            last_count = imu_update_count;
            tick_start = now;
        }
        
        // Print at 10Hz only (every 100ms)
        if (now - print_tick >= 100) {
            print_tick = now;
            
            float accel_mag = sqrtf(accel[0]*accel[0] + accel[1]*accel[1] + accel[2]*accel[2]);
            
            print("\033[H"
                  "=== BMI088 IMU + Mahony Filter ===\r\n\r\n"
                  "Accel [m/s^2]: X:%8.3f Y:%8.3f Z:%8.3f\r\n"
                  "Gyro [deg/s]:  X:%8.3f Y:%8.3f Z:%8.3f\r\n\r\n"
                  "Euler [deg]:   R:%8.2f P:%8.2f Y:%8.2f\r\n\r\n"
                  "Amag: %.2f  Temp: %.1f C  Rate: %lu Hz\r\n",
                  accel[0], accel[1], accel[2],
                  gyro[0], gyro[1], gyro[2],
                  RAD2DEG(roll), RAD2DEG(pitch), RAD2DEG(yaw),
                  accel_mag, temp, update_rate);
        }
        
        osDelay(1);  // 1ms delay = 1000Hz AHRS update rate
    }
}

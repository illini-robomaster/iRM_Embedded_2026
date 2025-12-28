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
/**
 * @file lidar07.h
 * @brief This file contains the declaration of the LIDAR07_UART and LIDAR07_IIC classes and related entities.
 */

#pragma once

#include "bsp_uart.h"
#include "i2c.h"

#define LIDAR07_VERSION 0 ///< Version of the LIDAR07 sensor.
#define LIDAR07_MEASURE 1 ///< Measure command for the LIDAR07 sensor.

/**
 * @namespace distance
 * @brief Namespace for distance or space related sensors and their support classes and functions.
 */
namespace distance {

/**
 * @struct SendPacket_t
 * @brief Structure for sending packets.
 */
    typedef struct {
        uint8_t head; ///< The head of the packet.
        uint8_t command; ///< The command to be sent.
        uint8_t data[4]; ///< The data to be sent.
        uint8_t checkData[4]; ///< The check data for the packet.
    } __packed SendPacket_t;

/**
 * @typedef lidar07_delay_t
 * @brief Function pointer type for delay function.
 */
    typedef void (*lidar07_delay_t)(uint32_t milli);

/**
 * @class LIDAR07_UART
 * @brief Class for LIDAR07 sensor with UART interface.
 */
    class LIDAR07_UART : public bsp::UART {
    public:
        /**
         * @brief Constructor for the LIDAR07_UART class.
         * @param huart Handle to the UART peripheral.
         * @param delay_func Function for delay.
         */
        LIDAR07_UART(
                UART_HandleTypeDef* huart,
                lidar07_delay_t delay_func = [](uint32_t milli) { HAL_Delay(milli); });

        bool begin(); ///< Function to start the sensor.
        bool setMeasureMode(); ///< Function to set the measurement mode.
        bool startMeasure(); ///< Function to start measurement.
        void stopMeasure(); ///< Function to stop measurement.
        bool startFilter(); ///< Function to start filter.
        bool stopFilter(); ///< Function to stop filter.

        uint16_t distance; ///< Measured distance.
        uint16_t amplitude; ///< Measured amplitude.
        uint32_t version; ///< Version of the sensor.

    private:
        lidar07_delay_t delay_func_ = nullptr; ///< Delay function.

        bool crc32Check(uint8_t* buff, uint8_t len); ///< Function to check CRC32.
        void readValue(uint8_t* buff, uint8_t type); ///< Function to read value.

        SendPacket_t readVersionPacket; ///< Packet for reading version.
        SendPacket_t setIntervalPacket; ///< Packet for setting interval.
        SendPacket_t setModePacket; ///< Packet for setting mode.
        SendPacket_t startPacket; ///< Packet for starting.
        SendPacket_t stopPacket; ///< Packet for stopping.
        SendPacket_t startFilterPacket; ///< Packet for starting filter.
        SendPacket_t stopFilterPacket; ///< Packet for stopping filter.
    };

/**
 * @class LIDAR07_IIC
 * @brief Class for LIDAR07 sensor with IIC interface.
 */
    // TODO: i2c is not working currently for lidar07, please fix it in the future. it may due to the
    // 400k i2c frequency required by lidar07
    class LIDAR07_IIC {
    public:
        /**
         * @brief Constructor for the LIDAR07_IIC class.
         * @param hi2c Handle to the I2C peripheral.
         * @param devAddr Device address.
         * @param delay_func Function for delay.
         */
        LIDAR07_IIC(
                I2C_HandleTypeDef* hi2c, uint16_t devAddr,
                lidar07_delay_t delay_func = [](uint32_t milli) { HAL_Delay(milli); });

        bool IsReady(); ///< Function to check if the sensor is ready.
        bool begin(); ///< Function to start the sensor.
        bool setMeasureMode(); ///< Function to set the measurement mode.
        bool startMeasure(); ///< Function to start measurement.
        void stopMeasure(); ///< Function to stop measurement.
        bool startFilter(); ///< Function to start filter.
        bool stopFilter(); ///< Function to stop filter.

        uint16_t distance; ///< Measured distance.
        uint16_t amplitude; ///< Measured amplitude.
        uint32_t version; ///< Version of the sensor.

    private:
        I2C_HandleTypeDef* hi2c_; ///< Handle to the I2C peripheral.
        uint16_t devAddr_; ///< Device address.

        lidar07_delay_t delay_func_ = nullptr; ///< Delay function.

        bool crc32Check(uint8_t* buff, uint8_t len); ///< Function to check CRC32.
        void readValue(uint8_t* buff, uint8_t type); ///< Function to read value.

        SendPacket_t readVersionPacket; ///< Packet for reading version.
        SendPacket_t setIntervalPacket; ///< Packet for setting interval.
        SendPacket_t setModePacket; ///< Packet for setting mode.
        SendPacket_t startPacket; ///< Packet for starting.
        SendPacket_t stopPacket; ///< Packet for stopping.
        SendPacket_t startFilterPacket; ///< Packet for starting filter.
        SendPacket_t stopFilterPacket; ///< Packet for stopping filter.
    };

}  // namespace distance
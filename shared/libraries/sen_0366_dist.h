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
 * @file sen_0366_dist.h
 * @brief This file contains the declaration of the SEN_0366_DIST class and related entities.
 */
#pragma once

#include "bsp_uart.h"
#include "utils.h"
#include "usart.h"

/**
 * @namespace distance
 * @brief Namespace for distance or space related sensors and their support classes and functions.
 */
namespace distance {

    /**
     * @struct SendPacket_4_t
     * @brief Structure for sending 4-byte commands.
     */
    typedef struct{
        uint8_t command[4]; ///< The command to be sent.
    }__packed SendPacket_4_t;

    /**
     * @struct SendPacket_5_t
     * @brief Structure for sending 5-byte commands.
     */
    typedef struct{
        uint8_t command[5]; ///< The command to be sent.
    }__packed SendPacket_5_t;

    /**
     * @struct SendPacket_6_t
     * @brief Structure for sending 6-byte commands.
     */
    typedef struct{
        uint8_t command[6]; ///< The command to be sent.
    }__packed SendPacket_6_t;

    /**
     * @enum command_id_t
     * @brief Enumeration for command IDs for return packet check.
     */
    typedef enum{
        LASER_RET, ///< Laser return command ID.
        MEASURE_RET, ///< Measure return command ID.
        SHUTDOWN_RET, ///< Shutdown return command ID.
        SET_MEASURE_RANGE_RET, ///< Set measure range return command ID.
        SET_FREQUENCY_RET, ///< Set frequency return command ID.
        SET_RESOLUTION_RET, ///< Set resolution return command ID.
        VERSION_RET ///< Read Version return command ID.
    }command_id_t;

    /**
     * @enum resolution_t
     * @brief Enumeration for resolution types for setting different resolution.
     */
    typedef enum{
        RESOLUTION_1_MM, ///< 1 mm resolution.
        RESOLUTION_0_1_MM, ///< 0.1 mm resolution.
    }resolution_t;

    /**
     * @enum range_t
     * @brief Enumeration for range types for setting different distance.
     */
    typedef enum{
        RANGE_5_M, ///< 5 meter range.
        RANGE_10_M, ///< 10 meter range.
        RANGE_30_M, ///< 30 meter range.
        RANGE_50_M, ///< 50 meter range.
        RANGE_80_M ///< 80 meter range.
    }range_t;

    /**
     * @typedef sen_0366_delay_t
     * @brief Function pointer type for delay function.
     */
    typedef void (*sen_0366_delay_t)(uint32_t milli);

    /**
     * @brief Function to configure the UART for the sensor.
     * @param huart Handle to the UART peripheral.
     */
    void sen_0366_config(UART_HandleTypeDef *huart);

    /**
     * @class SEN_0366_DIST
     * @brief Class for SEN_0366_DIST sensor.
     */
    class SEN_0366_DIST : public bsp::UART {
    public:

        /**
         * @brief Constructor for the SEN_0366_DIST class.
         * @param huart Handle to the UART peripheral.
         * @param ADDR Address of the sensor.
         * @param filter Filter selected for smoothing measured distance.
         * TODO: Frankly the filter is not really helpful lmao, disabled
         */
        SEN_0366_DIST(UART_HandleTypeDef *huart, uint8_t ADDR,MovingAverageFilter<double>filter);

        /**
         * @brief Function to initialize the sensor.
         * @param huart Handle to the UART peripheral.
         * @param ADDR Address of the sensor, 0x80 by default.
         * @return Pointer to the initialized sensor.
         */
        static SEN_0366_DIST* init(UART_HandleTypeDef *huart, uint8_t ADDR);

        uint8_t address; ///< Sensor address.

        bool begin(); ///< Function to start the sensor.
        bool singleMeasure(); ///< Function to perform a single measurement.
        bool continuousMeasure(); ///< Function to start continuous measurement.
        bool setMeasureRange(range_t range); ///< Function to set the measurement range.
        bool laserOn(); ///< Function to turn on the laser.
        void readValue(); ///< Function to read the value from the sensor.
        bool setResolution(resolution_t resolution); ///< Function to set the resolution.
        bool shutdown(); ///< Function to shutdown the sensor.
        bool checkReturn(const uint8_t *buffer, command_id_t command_id); ///< Function to check the return value of a command.
        void fetchParameter(); ///< Function to fetch parameters from the sensor.


        double distance_; ///< Measured distance.
        uint8_t *buff_; ///< Buffer for received data.
        bool connection_flag_;

    private:
        sen_0366_delay_t delay_func = [](uint32_t milli) { HAL_Delay(milli); }; ///< Delay function.
        SendPacket_4_t single_measurement_packet; ///< Packet for single measurement command.
        SendPacket_4_t parameter_packet; ///< Packet for parameter command.
        SendPacket_4_t continuous_measurement_packet; ///< Packet for continuous measurement command.
        SendPacket_4_t shutdown_packet; ///< Packet for shutdown command.
        SendPacket_4_t read_version_packet;
        SendPacket_5_t laser_on_packet{}; ///< Packet for laser on command.
        SendPacket_5_t set_measure_distance_packet; ///< Packet for set measure distance command.
        SendPacket_5_t set_frequency_packet; ///< Packet for set frequency command.
        SendPacket_5_t set_resolution_packet; ///< Packet for set resolution command.
        MovingAverageFilter<double>filter_; ///< filter for smoothing data.
    };
};
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

#include "sen_0366_dist.h"



namespace distance{

    void sen_0366_config(UART_HandleTypeDef *huart){
      // uninitialized the current uart for now
      HAL_UART_DeInit(huart);
      // update baud rate
      huart->Init.BaudRate = 9600;
      // reinitialize the uart
      HAL_UART_Init(huart);
    }
    SEN_0366_DIST::SEN_0366_DIST(UART_HandleTypeDef *huart, uint8_t ADDR, MovingAverageFilter<double> filter)
            : bsp::UART(huart), filter_(std::move(filter)) {
      address = ADDR;
      SetupRx(100);
      SetupTx(100);
    }

    SEN_0366_DIST*  SEN_0366_DIST::init(UART_HandleTypeDef *huart, uint8_t ADDR) {
        sen_0366_config(huart);
        size_t window_size = 15;
        auto* ret = new SEN_0366_DIST(huart, ADDR, MovingAverageFilter<double>(window_size));
        return ret;
    }
    bool SEN_0366_DIST::begin() {
        // packet for single_measurement
        single_measurement_packet.command[0] = address;
        single_measurement_packet.command[1] = 0x06;
        single_measurement_packet.command[2] = 0x02;
        single_measurement_packet.command[3] = ~(address +
                single_measurement_packet.command[1] + single_measurement_packet.command[2]) + 1;
        // packet for continuous_measurement
        continuous_measurement_packet.command[0] = address;
        continuous_measurement_packet.command[1] = 0x06;
        continuous_measurement_packet.command[2] = 0x03;
        continuous_measurement_packet.command[3] = ~(address +
                continuous_measurement_packet.command[1] + continuous_measurement_packet.command[2]) + 1;
        // shutdown packet
        shutdown_packet.command[0] = address;
        shutdown_packet.command[1] = 0x04;
        shutdown_packet.command[2] = 0x02;
        shutdown_packet.command[3] = ~(address +
                shutdown_packet.command[1] + shutdown_packet.command[2]) + 1;
        // laser on packet
        laser_on_packet.command[0] = address;
        laser_on_packet.command[1] = 0x06;
        laser_on_packet.command[2] = 0x05;
        laser_on_packet.command[3] = 0x01;
        laser_on_packet.command[4] = ~(address +
                laser_on_packet.command[1] + laser_on_packet.command[2] + laser_on_packet.command[3]) + 1;
        // read parameter packet
        parameter_packet.command[0] = 0xFA;
        parameter_packet.command[1] = 0x06;
        parameter_packet.command[2] = 0x01;
        parameter_packet.command[3] = 0xFF;
        // set measure distance
        set_measure_distance_packet.command[0] = 0xFA;
        set_measure_distance_packet.command[1] = 0x04;
        set_measure_distance_packet.command[2] = 0x09;
        // set frequency packet
        set_frequency_packet.command[0] = 0xFA;
        set_frequency_packet.command[1] = 0x04;
        set_frequency_packet.command[2] = 0x0A;
        // set resolution packet
        set_resolution_packet.command[0] = 0xFA;
        set_resolution_packet.command[1] = 0x04;
        set_resolution_packet.command[2] = 0x0C;

        //FA 06 01 FF
        read_version_packet.command[0] = 0xFA;
        read_version_packet.command[1] = 0x06;
        read_version_packet.command[2] = 0x01;
        read_version_packet.command[3] = 0xFF;

        connection_flag_ = laserOn();

        return connection_flag_;
    }

    bool SEN_0366_DIST::singleMeasure() {
        delay_func(50);
        uint8_t *buffer;
        Write((uint8_t*)&single_measurement_packet, sizeof(SendPacket_4_t));
        Read(&buffer);
        distance_ = (buffer[3] - 0x30) * 100 + (buffer[4] - 0x30) * 10
                  + (buffer[5] - 0x30) + (buffer[7] - 0x30) * 0.1 + (buffer[8] - 0x30) * 0.01;

        return checkReturn(buffer,MEASURE_RET);
    }

    bool SEN_0366_DIST::continuousMeasure() {
      delay_func(100);
      uint8_t *buffer;
      Write((uint8_t*)&continuous_measurement_packet, sizeof(SendPacket_4_t ));
      Read(&buffer);
      return checkReturn(buffer,MEASURE_RET);
    }

    bool SEN_0366_DIST::laserOn() {
        delay_func(50);
        uint8_t *buffer;
        Write((uint8_t*)&laser_on_packet, sizeof(SendPacket_5_t));
        Read(&buffer);
        buff_ = buffer;
        return checkReturn(buffer,LASER_RET);
    }

    void SEN_0366_DIST::fetchParameter() {
        delay_func(50);
        uint8_t *buffer;
        Write((uint8_t*)&parameter_packet,sizeof(SendPacket_4_t ));
        Read(&buffer);
        buff_ = buffer;

    }

    bool SEN_0366_DIST::setMeasureRange(range_t range) {
        switch (range) {
            case RANGE_5_M:
              set_measure_distance_packet.command[3] = 0x05;
              set_measure_distance_packet.command[4] = 0xF4;
              break;
            case RANGE_10_M:
              set_measure_distance_packet.command[3] = 0x0A;
              set_measure_distance_packet.command[4] = 0xEF;
              break;
            case RANGE_30_M:
              set_measure_distance_packet.command[3] = 0x1E;
              set_measure_distance_packet.command[4] = 0xDB;
              break;
            case RANGE_50_M:
              set_measure_distance_packet.command[3] = 0x32;
              set_measure_distance_packet.command[4] = 0xC7;
              break;
            case RANGE_80_M:
              set_measure_distance_packet.command[3] = 0x50;
              set_measure_distance_packet.command[4] = 0xA9;
              break;
            default:
              return false;
          }
        delay_func(50);
        uint8_t *buffer;
        Write((uint8_t*)&set_measure_distance_packet, sizeof(SendPacket_5_t));
        Read(&buffer);
        return checkReturn(buffer,SET_MEASURE_RANGE_RET);
    }

    bool SEN_0366_DIST::setResolution(resolution_t resolution) {
      if (resolution == RESOLUTION_1_MM){
        set_resolution_packet.command[3] = 0x01;
        set_resolution_packet.command[4] = 0xF5;
      } else if (resolution == RESOLUTION_0_1_MM){
        set_resolution_packet.command[3] = 0x02;
        set_resolution_packet.command[4] = 0xF4;
      } else {
        return false;
      }
      delay_func(50);
      uint8_t *buffer;
      Write((uint8_t*)&set_resolution_packet, sizeof(SendPacket_5_t));
      Read(&buffer);
      return checkReturn(buffer,SET_RESOLUTION_RET);
    }

    bool SEN_0366_DIST::shutdown(){
        delay_func(50);
        uint8_t *buffer;
        Write((uint8_t*)&shutdown_packet, sizeof(SendPacket_4_t));
        Read(&buffer);
        return checkReturn(buffer,SHUTDOWN_RET);
    }

    void SEN_0366_DIST::readValue() {
      delay_func(50);
      uint8_t *buffer;
      Read(&buffer);
      buff_=buffer;
      double prev_distance = distance_;
      distance_ = (buffer[3] - 0x30) * 100 + (buffer[4] - 0x30) * 10
              + (buffer[5] - 0x30) + (buffer[7] - 0x30) * 0.1 + (buffer[8] - 0x30) * 0.01;
//      distance_ = filter_.update(distance_);
      if ((distance_ < 0 || distance_ > 70)||(!(checkReturn(buffer,MEASURE_RET)))){
          distance_ = prev_distance;
        }
    }

    bool SEN_0366_DIST::checkReturn(const uint8_t *buffer, command_id_t command_id) {
      switch(command_id){
        case LASER_RET:
          if (buffer[0] == address && buffer[1] == 0x06 && buffer[2] == 0x85 && buffer[3] == 0x01){
            uint16_t cs = ~(address + buffer[1] + buffer[2] + buffer[3]) + 1;
            cs = cs << 8;
            cs = cs >> 8;
            if (buffer[4] == cs){
              return true;
            }
          }
          connection_flag_ = false;
          return false;
        case MEASURE_RET:
          if (buffer[3]=='E' && buffer[4] == 'R' && buffer[5] == 'R'){
            return false;
          }
          connection_flag_ = false;
          return true;
        case SHUTDOWN_RET:
          if (buffer[0] == address && buffer[1] == 0x04 && buffer[2] == 0x82){
              uint8_t cs = ~(address + buffer[1] + buffer[2]) + 1;
              if (buffer[3] == cs){
                return true;
              }
          }
          connection_flag_ = false;
          return false;
        case SET_MEASURE_RANGE_RET:
          if (buffer[0]==0xFA && buffer[1] == 0x04 && buffer[2] == 0x89 && buffer[3] == 0x79){
            return true;
          }
          connection_flag_ = false;
          return false;
        case SET_FREQUENCY_RET:
          if (buffer[0]==0xFA && buffer[1] == 0x04 && buffer[2] == 0x8A && buffer[3] == 0x78){
            return true;
          }
          connection_flag_ = false;
          return false;
        case SET_RESOLUTION_RET:
          if (buffer[0]==0xFA && buffer[1] == 0x04 && buffer[2] == 0x8C && buffer[3] == 0x76){
              return true;
          }
          connection_flag_ = false;
          return false;
        case VERSION_RET:
          if (buffer[0]==0xFA && buffer[1] == 0x06 && buffer[2] == 0x81 && buffer[3] == 0x80){
            return true;
          }
          connection_flag_ = false;
          return false;
        default:
          break;
      }
      return false;
    }
}


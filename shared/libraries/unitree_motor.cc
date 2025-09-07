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

#include "unitree_motor.h"
#include <cstring>

namespace control {

UnitreeMotor::UnitreeMotor() {
  memset(send, 0, sizeof(send[0]) * 3);
  memset(recv, 0, sizeof(recv[0]) * 3);
}

void UnitreeMotor::ModifyData(const unsigned short motor_id) {
    send[motor_id].data.head.start[0] = 0xFE;
    send[motor_id].data.head.start[1] = 0xEE;
    send[motor_id].data.head.motorID = send[motor_id].id;
    send[motor_id].data.head.reserved = 0x0;
    send[motor_id].data.Mdata.mode = send[motor_id].mode;
    send[motor_id].data.Mdata.ModifyBit = 0xFF;
    send[motor_id].data.Mdata.ReadBit = 0x0;
    send[motor_id].data.Mdata.reserved = 0x0;
    send[motor_id].data.Mdata.Modify.L = 0;
    send[motor_id].data.Mdata.T = send[motor_id].T * 256;
    send[motor_id].data.Mdata.W = send[motor_id].W * 128;
    send[motor_id].data.Mdata.Pos = (int)((send[motor_id].Pos / 6.2832) * 16384.0);
    send[motor_id].data.Mdata.K_P = send[motor_id].K_P * 2048;
    send[motor_id].data.Mdata.K_W = send[motor_id].K_W * 1024;
    send[motor_id].data.Mdata.LowHzMotorCmdIndex = 0;
    send[motor_id].data.Mdata.LowHzMotorCmdByte = 0;
    send[motor_id].data.Mdata.Res[0] = send[motor_id].Res;
    send[motor_id].data.CRCdata.u32 = crc32_core((uint32_t*)(&(send[motor_id].data)), 7);
}

bool UnitreeMotor::ExtractData(const communication::package_t package) {
    motor_recv_t tmp;
    memcpy(&(tmp.data), package.data, package.length);
    if (tmp.data.CRCdata.u32 != crc32_core((uint32_t*)(&(tmp.data)), 18)) {
        tmp.correct = false;
        return tmp.correct;
    } else {
        tmp.id = tmp.data.head.motorID;
        tmp.mode = tmp.data.Mdata.mode;
        tmp.Temp = tmp.data.Mdata.Temp;
        tmp.MError = tmp.data.Mdata.MError;
        tmp.T = ((float)tmp.data.Mdata.T) / 256;
        tmp.W = ((float)tmp.data.Mdata.W) / 128 / gear_ratio;
        tmp.LW = tmp.data.Mdata.LW;
        tmp.Acc = (int)tmp.data.Mdata.Acc;
        tmp.Pos = 6.2832 * ((float)tmp.data.Mdata.Pos) / 16384 / gear_ratio;
        tmp.gyro[0] = ((float)tmp.data.Mdata.gyro[0]) * 0.00107993176;
        tmp.gyro[1] = ((float)tmp.data.Mdata.gyro[1]) * 0.00107993176;
        tmp.gyro[2] = ((float)tmp.data.Mdata.gyro[2]) * 0.00107993176;
        tmp.acc[0] = ((float)tmp.data.Mdata.acc[0]) * 0.0023911132;
        tmp.acc[1] = ((float)tmp.data.Mdata.acc[1]) * 0.0023911132;
        tmp.acc[2] = ((float)tmp.data.Mdata.acc[2]) * 0.0023911132;
        tmp.correct = true;
        memcpy(&recv[tmp.id], &tmp, sizeof(motor_recv_t));
        connection_flag_[tmp.id] = true;
        return tmp.correct;
    }
}

void UnitreeMotor::Stop(const unsigned short motor_id) {
    send[motor_id].id = motor_id;
    send[motor_id].mode = 0;
    send[motor_id].T = 0.0;
    send[motor_id].W = 0.0;
    send[motor_id].Pos = 0.0;
    send[motor_id].K_P = 0.0;
    send[motor_id].K_W = 0.0;
    ModifyData(motor_id);
}

void UnitreeMotor::Test(const unsigned short motor_id) {
    send[motor_id].id = motor_id;
    send[motor_id].mode = 5;
    send[motor_id].T = 0.0;
    send[motor_id].W = 0.0;
    send[motor_id].Pos = 0.0;
    send[motor_id].K_P = 0.0;
    send[motor_id].K_W = 0.0;
    ModifyData(motor_id);
}

void UnitreeMotor::Control(const unsigned short motor_id, const float torque, const float speed, const float position, const float Kp, const float Kd) {
    send[motor_id].id = motor_id;
    send[motor_id].mode = 10;
    send[motor_id].T = torque;
    send[motor_id].W = speed * gear_ratio;
    send[motor_id].Pos = position * gear_ratio;
    send[motor_id].K_P = Kp;
    send[motor_id].K_W = Kd;
    ModifyData(motor_id);
}

uint32_t UnitreeMotor::crc32_core(uint32_t* ptr, uint32_t len) {
    uint32_t xbit;
    uint32_t data;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; ++i) {
        xbit = 1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; ++bits) {
            if (CRC32 & 0x80000000) {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            } else
                CRC32 <<= 1;
            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }
    return CRC32;
}

}

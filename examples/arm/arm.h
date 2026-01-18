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

typedef struct {
  float base_translate_0;     /* translate 3508 motor                           */
  float base_yaw_rotate_1;   /* rotate the entire arm around an vertical axis  */
  float base_pitch_rotate_2;    /* rotate the entire arm around a horizontal axis */
  float forearm_pitch_3;       /* rotate the forearm on a horizontal axis        */
  float forearm_roll_4;     /* rotate the forearm on its axis                 */
  float wrist_5;       /* rotate the hand around an vertical axis        */
  float end_6;        /* rotate the hand on its axis                    */
} joint_state_t;

/**
 * @brief turn to a relative position
 * @note software range limitation defined in arm_config.h
 * @return 0 when the command is accepted, 1 otherwise
 */
int ArmTurnRelative(joint_state_t* target_joint_state);

/**
 * @brief turn to an absolute position
 * @note software range limitation defined in arm_config.h
 * @return 0 when the command is accepted, 1 otherwise
 */
int ArmTurnAbsolute(joint_state_t* target_joint_state);

/**
 * @brief Call all TransmitOutput() or equivalent function for each motor
 */
void ArmTransmitOutput();

/**
 * @brief print arm data
 */
void ArmPrintData();

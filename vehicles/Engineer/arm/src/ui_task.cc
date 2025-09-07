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
#include "ui_task.h"



communication::UserInterface* UI = nullptr;

uint32_t ui_val_graph_size = 10;
uint32_t ui_val_width = 2;
uint32_t ui_val_char_length = 15;

float joint0 = 0.0;
float joint1 = 0.0;
float joint2 = 0.0;
float joint3 = 0.0;
float joint4 = 0.0;
float joint5 = 0.0;
float joint6 = 0.0;

communication::graphic_data_t joint0_char_graph;
communication::graphic_data_t joint1_char_graph;
communication::graphic_data_t joint2_char_graph;
communication::graphic_data_t joint3_char_graph;
communication::graphic_data_t joint4_char_graph;
communication::graphic_data_t joint5_char_graph;
communication::graphic_data_t joint6_char_graph;

communication::package_t frame;

void UITask(void* arg){
  UNUSED(arg);

  UI->SetID(referee->game_robot_status.robot_id);

  while (!(referee->connection_flag_)) {
    print("Waiting for referee system to connect\r\n");
    osDelay(1000);
  }

  UI->SetID(referee->game_robot_status.robot_id);
  while(sbus->ch[6] < 90.0){
    if (!key->Read())break;
    refresh();
    print("refreshing\r\n");
    osDelay(1000);
  }

  print("UI task started, ID: %d\r\n",referee->game_robot_status.robot_id);
  osDelay(UI_TASK_DELAY);

  while (true) {
    joint0 = current_motor_angles.base_translate_0;
    joint1 = current_motor_angles.base_yaw_rotate_1;
    joint2 = current_motor_angles.base_pitch_rotate_2;
    joint3 = current_motor_angles.forearm_pitch_3;
    joint4 = current_motor_angles.forearm_roll_4;
    joint5 = current_motor_angles.wrist_5;
    joint6 = current_motor_angles.end_6;

    char joint0_char[40];
    char joint2_char[20];
    char joint3_char[20];
    char joint4_char[20];
    char joint5_char[20];
    char joint6_char[20];

    snprintf(joint0_char, 40, "Base: %.3f\r\nJoint1: %.3f", joint0,joint1);
    snprintf(joint2_char, 20, "Joint 2: %.3f", joint2);
    snprintf(joint3_char, 20, "Joint 3: %.3f", joint3);
    snprintf(joint4_char, 20, "Joint 4: %.3f", joint4);
    snprintf(joint5_char, 20, "Joint 5: %.3f", joint5);
    snprintf(joint6_char, 20, "Joint 6: %.3f", joint6);

    // Base
    UI->CharDraw(&joint0_char_graph,"J0", UI_Graph_Change, 0,
                 UI_Color_Orange,ui_val_graph_size, ui_val_char_length,
                 ui_val_width,1535, 640);

    UI->CharRefresh((uint8_t *) (&referee->graphic_character),
                    joint0_char_graph, joint0_char, strlen(joint0_char));
    referee->PrepareUIContent(communication::CHAR_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    referee_uart->Write(frame.data, frame.length);
    osDelay(UI_TASK_DELAY);

    // Joint 2
    UI->CharDraw(&joint2_char_graph,"J2", UI_Graph_Change, 0,
                 UI_Color_Orange,ui_val_graph_size, ui_val_char_length,
                 ui_val_width,1535, 600);
    UI->CharRefresh((uint8_t *) (&referee->graphic_character),
                    joint2_char_graph, joint2_char, strlen(joint2_char));
    referee->PrepareUIContent(communication::CHAR_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    referee_uart->Write(frame.data, frame.length);
    osDelay(UI_TASK_DELAY);

    // Joint 3
    UI->CharDraw(&joint3_char_graph,"J3", UI_Graph_Change, 0,
                 UI_Color_Orange,ui_val_graph_size, ui_val_char_length,
                 ui_val_width,1535, 580);
    UI->CharRefresh((uint8_t *) (&referee->graphic_character),
                    joint3_char_graph, joint3_char, strlen(joint3_char));
    referee->PrepareUIContent(communication::CHAR_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    referee_uart->Write(frame.data, frame.length);
    osDelay(UI_TASK_DELAY);

    // Joint 4
    UI->CharDraw(&joint4_char_graph,"J4", UI_Graph_Change, 1,
                 UI_Color_Orange,ui_val_graph_size, ui_val_char_length,
                 ui_val_width,1535, 560);
    UI->CharRefresh((uint8_t *) (&referee->graphic_character),
                    joint4_char_graph, joint4_char, strlen(joint4_char));
    referee->PrepareUIContent(communication::CHAR_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    referee_uart->Write(frame.data, frame.length);
    osDelay(UI_TASK_DELAY);

    // Joint 5
    UI->CharDraw(&joint5_char_graph,"J5", UI_Graph_Change, 1,
                 UI_Color_Orange,ui_val_graph_size, ui_val_char_length,
                 ui_val_width,1535, 540);
    UI->CharRefresh((uint8_t *) (&referee->graphic_character),
                    joint5_char_graph, joint5_char, strlen(joint5_char));
    referee->PrepareUIContent(communication::CHAR_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    referee_uart->Write(frame.data, frame.length);
    osDelay(UI_TASK_DELAY);

    // Joint 6
    UI->CharDraw(&joint6_char_graph,"J6", UI_Graph_Change, 1,
                 UI_Color_Orange,ui_val_graph_size, ui_val_char_length,
                 ui_val_width,1535, 520);
    UI->CharRefresh((uint8_t *) (&referee->graphic_character),
                    joint6_char_graph, joint6_char, strlen(joint6_char));
    referee->PrepareUIContent(communication::CHAR_GRAPH);
    frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
    referee_uart->Write(frame.data, frame.length);
    osDelay(UI_TASK_DELAY);
  }
}

/*
 * void refresh() will initialize all UI contents, however, if the UI
 * is not there, you can press Left Ctrl to refresh it within the main routine of UI task
 * */

void refresh(){
  char joint0_char[20]="base_init & J1 init";
  char joint1_char[20];
  char joint2_char[20]="J2 init";
  char joint3_char[20]="J3 init";
  char joint4_char[20]="J4 init";
  char joint5_char[20]="J5 init";
  char joint6_char[20]="J6 init";



  // Base
  UI->CharDraw(&joint0_char_graph,"J0", UI_Graph_Add, 0,
               UI_Color_Orange,ui_val_graph_size, ui_val_char_length,
               ui_val_width,1535, 640);

  UI->CharRefresh((uint8_t *) (&referee->graphic_character),
                  joint0_char_graph, joint0_char, strlen(joint0_char));
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  osDelay(UI_TASK_DELAY);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);


  // Joint 3
  UI->CharDraw(&joint3_char_graph,"J3", UI_Graph_Add, 0,
               UI_Color_Orange,ui_val_graph_size, ui_val_char_length,
               ui_val_width,1535, 580);
  UI->CharRefresh((uint8_t *) (&referee->graphic_character),
                  joint3_char_graph, joint3_char, strlen(joint3_char));
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  osDelay(UI_TASK_DELAY);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);

  // Joint 4
  UI->CharDraw(&joint4_char_graph,"J4", UI_Graph_Add, 1,
               UI_Color_Orange,ui_val_graph_size, ui_val_char_length,
               ui_val_width,1535, 560);
  UI->CharRefresh((uint8_t *) (&referee->graphic_character),
                  joint4_char_graph, joint4_char, strlen(joint4_char));
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  osDelay(UI_TASK_DELAY);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);

  // Joint 5
  UI->CharDraw(&joint5_char_graph,"J5", UI_Graph_Add, 1,
               UI_Color_Orange,ui_val_graph_size, ui_val_char_length,
               ui_val_width,1535, 540);
  UI->CharRefresh((uint8_t *) (&referee->graphic_character),
                  joint5_char_graph, joint5_char, strlen(joint5_char));
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  osDelay(UI_TASK_DELAY);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);

  // Joint 6
  UI->CharDraw(&joint6_char_graph,"J6", UI_Graph_Add, 1,
               UI_Color_Orange,ui_val_graph_size, ui_val_char_length,
               ui_val_width,1535, 520);
  UI->CharRefresh((uint8_t *) (&referee->graphic_character),
                  joint6_char_graph, joint6_char, strlen(joint6_char));
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  osDelay(UI_TASK_DELAY);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);

  // Joint 1
  UI->CharDraw(&joint1_char_graph,"J1", UI_Graph_Add, 2,
               UI_Color_Orange,ui_val_graph_size, ui_val_char_length,
               ui_val_width,1535, 620);
  UI->CharRefresh((uint8_t *) (&referee->graphic_character),
                  joint1_char_graph, joint1_char, strlen(joint1_char));
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);
  referee_uart->Write(frame.data, frame.length);


  // Joint 2
  UI->CharDraw(&joint2_char_graph,"J2", UI_Graph_Add, 0,
               UI_Color_Orange,ui_val_graph_size, ui_val_char_length,
               ui_val_width,1535, 600);
  UI->CharRefresh((uint8_t *) (&referee->graphic_character),
                  joint2_char_graph, joint2_char, strlen(joint2_char));
  referee->PrepareUIContent(communication::CHAR_GRAPH);
  frame = referee->Transmit(communication::STUDENT_INTERACTIVE);
  osDelay(UI_TASK_DELAY);
  referee_uart->Write(frame.data, frame.length);
  osDelay(UI_TASK_DELAY);
  referee_uart->Write(frame.data, frame.length);


}

void init_ui(){
  UI = new communication::UserInterface();
}

void kill_ui(){

}
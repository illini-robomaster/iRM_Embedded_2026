
#ifndef __UNITREEA1_CMD__
#define __UNITREEA1_CMD__
#include "usart.h"
#include "motor_msg.h"

extern motor_send_t MotorA1_send;  // 左腿一号电机数据体
extern motor_send_t MotorA1_send_right; // 右腿一号电机数据体

extern motor_recv_t Data;        // 左腿电机接收数据体
extern motor_recv_t MotorA1_recv_id00;   // 左腿00号电机接收数据体
extern motor_recv_t MotorA1_recv_id01;   // 左腿01号电机接收数据体
extern motor_recv_t MotorA1_recv_id02;   // 左腿02号电机接收数据体

/**
 @brief 对应电机参数修改
 @param send 为MotorA1_send_left或MotorA1_send_right，分别控制左右侧腿部
 @param id   发送接收目标电机的id
 @param pos  为电机旋转圈数，1为一圈
 @param KP   电机刚度系数
 @param KW   电机速度系数
*/
void modify_pos_cmd(motor_send_t *send,uint8_t id, float Pos, float KP, float KW);

// 速度模式
void modify_speed_cmd(motor_send_t *send,uint8_t id, float Omega);

// 力矩模式
void modify_torque_cmd(motor_send_t *send,uint8_t id, float torque);

// 电机停止
void modify_stop_cmd(motor_send_t *send, uint8_t id);

/// @brief 用来和电机通讯的代码，将获取的数据存入对应结构体中
/// @param huart 需要使用的串口，huart1为左侧，6为右侧

void unitreeA1_rxtx(UART_HandleTypeDef uart);

uint32_t crc32_core(uint32_t *ptr, uint32_t len);

#endif
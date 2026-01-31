#include "main.h"
#include "motor_msg.h"
#include <string.h>
#include <stdio.h>
#include "usart.h"
#include "A1/A1_motor_drive.h"
#include <cmsis_os.h>
#include "../../boards/DM_MC_01/Core/Inc/usart.h"
#include "bsp_print.h"

#define PI 3.14159

motor_send_t MotorA1_send;       // 左腿一号电机数据体

motor_recv_t Data;               // 左腿电机接收数据体
motor_recv_t MotorA1_recv_id00;  // 左腿00号电机接收数据体
motor_recv_t MotorA1_recv_id01;  // 左腿01号电机接收数据体
motor_recv_t MotorA1_recv_id02;  // 左腿02号电机接收数据体



// CRC校验位的代码
uint32_t crc32_core_Ver3(uint32_t *ptr, uint32_t len)
{
    uint32_t bits;
    uint32_t i;
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
                CRC32 <<= 1;
            if (data & xbit)
                CRC32 ^= dwPolynomial;

            xbit >>= 1;
        }
    }
    return CRC32;
}

// 电机位置修改
void modify_pos_cmd(motor_send_t *send,uint8_t id, float Pos, float KP, float KW)
{

    send->hex_len = 34;

    send->mode = 10;
	send->id   = id;

    send->Pos  = 9.1*Pos;  // 6.2832 = 2 PI // 原先为 6.2832*9.1*2*Pos
    send->W    = 0;
    send->T    = 0.0;
    send->K_P  = KP;
    send->K_W  = KW;
}

// 电机速度修改
void modify_speed_cmd(motor_send_t *send,uint8_t id, float Omega)
{

    send->hex_len = 34;

    send->mode = 10;
	send->id   = id;

    send->Pos  = 0;
    send->W    = Omega * 9.1f;
    send->T    = 0.0;
    send->K_P  = 0.0;
    send->K_W  = 3.0;
}

// 电机力矩修改
void modify_torque_cmd(motor_send_t *send,uint8_t id, float torque)
{

    send->hex_len = 34;

    send->mode = 10;
	send->id   = id;

    send->Pos  = 0.0;
    send->W    = 0.0;
    if (torque > 10.0f){torque = 0.0f;} // 限幅
    send->T    = torque / 9.1f;
    send->K_P  = 0.0;
    send->K_W  = 0.0;
}

// 电机停止
void modify_stop_cmd(motor_send_t *send, uint8_t id)
{
    send->hex_len = 34;

    send->mode = 0;
    send->id   = id;

    send->Pos  = 0.0;
    send->W    = 0.0;
    send->T    = 0.0;
    send->K_P  = 0.0;
    send->K_W  = 0.0;
}

/** 
 * @brief Interrupt function when uart transmit completes, use to change rs485_1 mode from send to receive. 
 * Because A1 only gives feedback (encoder position etc.) after a control command is sent to it.
 * So we need to quickly change the mode of rs485_1 to receive after the control command is sent. 
 * We cannot do it directly by setting gpio pin to 0 after the line HAL_UART_Transmit_DMA(), because at that time
 * the data is only in DMA but not sent.
 * */ 
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
  if(huart->Instance == USART1){
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET); // set 485_1 to receive mode
  }
}

// 电机发送接收函数
void unitreeA1_rxtx(UART_HandleTypeDef uart)
{
    
        uint8_t A1MotorA1_send[34]; // 发送数据
        uint8_t Date[78];       // 接收数据

        MotorA1_send.motor_send_data.head.start[0] = 0xFE;
        MotorA1_send.motor_send_data.head.start[1] = 0xEE;
        MotorA1_send.motor_send_data.head.motorID  = MotorA1_send.id;
        MotorA1_send.motor_send_data.head.reserved = 0x00;

        MotorA1_send.motor_send_data.Mdata.mode      = MotorA1_send.mode;  // mode = 10
        MotorA1_send.motor_send_data.Mdata.ModifyBit = 0xFF;
        MotorA1_send.motor_send_data.Mdata.ReadBit   = 0x00;
        MotorA1_send.motor_send_data.Mdata.reserved  = 0x00;
        MotorA1_send.motor_send_data.Mdata.Modify.F  = 0;
        MotorA1_send.motor_send_data.Mdata.T         = MotorA1_send.T * 256;
        MotorA1_send.motor_send_data.Mdata.W         = MotorA1_send.W * 128;
        MotorA1_send.motor_send_data.Mdata.Pos       = (int)((MotorA1_send.Pos / 6.2832f) * 16384.0f); // 单位 rad
        MotorA1_send.motor_send_data.Mdata.K_P       = MotorA1_send.K_P * 2048;
        MotorA1_send.motor_send_data.Mdata.K_W       = MotorA1_send.K_W * 1024;

        MotorA1_send.motor_send_data.Mdata.LowHzMotorCmdIndex = 0;
        MotorA1_send.motor_send_data.Mdata.LowHzMotorCmdByte  = 0;
        MotorA1_send.motor_send_data.Mdata.Res[0] = MotorA1_send.Res;

        MotorA1_send.motor_send_data.CRCdata.u32 = crc32_core_Ver3((uint32_t *)(&MotorA1_send.motor_send_data), 7); // CRC校验

        memcpy(A1MotorA1_send, &MotorA1_send.motor_send_data, 34);
        
        // HAL库 DMA 发送数据 + 接收数据
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET); // 485_1 in write mode
        HAL_UART_Transmit_DMA(&uart, A1MotorA1_send, 34);
        // use tx complete interrupt to set 485 to received mode
        osDelay(1); // wait for send data to go from memory to uart and receive data in uart then to memory, might be able to be shorter
        HAL_UART_Receive_DMA(&uart, Date, 78);

        // 接受数据处理
        // 1.没有处理温度数据 (可能正确，因为是整数?)
        // 2.检查数据类型是否都正确
        Data.motor_recv_data.head.motorID = Date[2];  
        Data.motor_recv_data.Mdata.mode   = Date[4];  
        Data.motor_recv_data.Mdata.Temp   = Date[6];
        Data.motor_recv_data.Mdata.MError = Date[7]; 
        Data.motor_recv_data.Mdata.T      = Date[13] << 8  | Date[12]; // 反拼
        Data.motor_recv_data.Mdata.W      = Date[15] << 8  | Date[14]; // 反拼
        Data.motor_recv_data.Mdata.Acc    = Date[27] << 8  | Date[26]; // 反拼
        Data.motor_recv_data.Mdata.Pos    = Date[33] << 24 | Date[32] << 16 | Date[31] << 8 | Date[30];  // 反拼

        Data.motor_id = Data.motor_recv_data.head.motorID;                               // ID     正确
        Data.mode     = Data.motor_recv_data.Mdata.mode;                                 // mode   正确
        Data.Temp     = Data.motor_recv_data.Mdata.Temp;                                 // Temp   正确 (整数)
        Data.MError   = Data.motor_recv_data.Mdata.MError;                               // MError 正确
        Data.T        = (float) Data.motor_recv_data.Mdata.T / 256;                      // T      正确
        Data.Pos      = (float) (Data.motor_recv_data.Mdata.Pos / (16384.0f/2/PI));      // Pos    正确
        Data.W        = (float) Data.motor_recv_data.Mdata.W / 128;                      // W      正确 (小数)
        Data.Acc      = (float) Data.motor_recv_data.Mdata.Acc;                          // Acc    貌似正确 (需要VOFA打印测试看是否连续)

        if (Data.motor_id == 0x00)
        {
            MotorA1_recv_id00.motor_id = Data.motor_id;
            MotorA1_recv_id00.mode     = Data.mode; 
            MotorA1_recv_id00.Temp     = Data.Temp;
            MotorA1_recv_id00.MError   = Data.MError;
            MotorA1_recv_id00.T        = Data.T * 9.1f;            // 减速后的扭矩
            MotorA1_recv_id00.W        = Data.W / 9.1f;            // 减速后的角速度
            MotorA1_recv_id00.Pos      = Data.Pos / 9.1f;          // 减速后的弧度     Data.Pos = 减速前的弧度
            MotorA1_recv_id00.Acc      = Data.Acc; 
        }

        if (Data.motor_id == 0x01)
        {
            MotorA1_recv_id01.motor_id = Data.motor_id;
            MotorA1_recv_id01.mode     = Data.mode; 
            MotorA1_recv_id01.Temp     = Data.Temp;
            MotorA1_recv_id01.MError   = Data.MError;
            MotorA1_recv_id01.T        = Data.T * 9.1f;
            MotorA1_recv_id01.W        = Data.W / 9.1f;   
            MotorA1_recv_id01.Pos      = Data.Pos / 9.1f;
            MotorA1_recv_id01.Acc      = Data.Acc; 

        }

        if (Data.motor_id == 0x02)
        {
            MotorA1_recv_id02.motor_id = Data.motor_id;
            MotorA1_recv_id02.mode     = Data.mode; 
            MotorA1_recv_id02.Temp     = Data.Temp;
            MotorA1_recv_id02.MError   = Data.MError;
            MotorA1_recv_id02.T        = Data.T * 9.1f;
            MotorA1_recv_id02.W        = Data.W / 9.1f;  
            MotorA1_recv_id02.Pos      = Data.Pos / 9.1f;
            MotorA1_recv_id02.Acc      = Data.Acc; 
        }

}

// 电机0位函数

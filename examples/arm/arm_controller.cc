#include "bsp_gpio.h"
#include "bsp_os.h"
#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "sbus.h"
#include "main.h"
#include "motor.h"
#include "unitree_motor.h"
#include "utils.h"
#include "dbus.h"

#include "arm.h"
#include "arm_config.h"

bsp::CAN* can = nullptr;

static remote::SBUS* sbus = nullptr;

#define RX_SIGNAL (1 << 0)
extern osThreadId_t defaultTaskHandle;

class CustomUART : public bsp::UART {
 public:
  using bsp::UART::UART;
 protected:
  /* notify application when rx data is pending read */
  void RxCompleteCallback() override final { osThreadFlagsSet(defaultTaskHandle, RX_SIGNAL); }
};

/* A1 Motors */
static control::UnitreeMotor* base_vert_rotate_motor = nullptr;
static control::UnitreeMotor* base_hor_rotate_motor = nullptr;
static control::UnitreeMotor* elbow_rotate_motor = nullptr;
static CustomUART* A1 = nullptr;

void RM_RTOS_Init(void) {
    print_use_uart(&huart6);
    bsp::SetHighresClockTimer(&htim5);    

    sbus = new remote::SBUS(&huart3);

    can = new bsp::CAN(&hcan1);

    A1 = new CustomUART(A1_UART);
    A1->SetupRx(300);
    A1->SetupTx(300);
    base_vert_rotate_motor = new control::UnitreeMotor();
    base_hor_rotate_motor = new control::UnitreeMotor();
    elbow_rotate_motor = new control::UnitreeMotor();
}

static joint_state_t last_targets = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//static joint_state_t target_state = {0.0,0.0,0.0,0.0,0.0,0.0,0.0};

#define DEBUG_MODE

void RM_RTOS_Default_Task(const void* args) {
    UNUSED(args);
    // A1 init state
    base_hor_rotate_motor->Stop( BASE_YAW_ID);
    base_vert_rotate_motor->Stop(BASE_PITCH_ID);
    elbow_rotate_motor->Stop(ELBOW_PITCH_ID);
    ArmTransmitOutput();
    
  // turn each motor for PI/16 degree every 2 seconds.
  // Should stop at defined limitation, see arm_config.h
    while (true) {
#ifdef DEBUG_MODE
        print("CH0: %-4d CH1: %-4d CH2: %-4d CH3: %-4d ", sbus->ch[0], sbus->ch[1], sbus->ch[2], sbus->ch[3]);
        print("CH4: %-4d CH5: %-4d CH6: %-4d CH7: %-4d ", sbus->ch[4], sbus->ch[5], sbus->ch[6], sbus->ch[7]);
        print("CH8: %-4d CH9: %-4d CH10: %-4d CH11: %-4d ", sbus->ch[8], sbus->ch[9], sbus->ch[10], sbus->ch[11]);
        print("CH12: %-4d CH13: %-4d CH14: %-4d CH15: %-4d ", sbus->ch[12], sbus->ch[13], sbus->ch[14], sbus->ch[15]);
        osDelay(100);
#else
        if (sbus->ch[6] > 0) {
            target_state = {0.0, sbus->ch[0] / sbus->CHANNEL_MAX, sbus->ch[1] / sbus->CHANNEL_MAX, sbus->ch[2] / sbus->CHANNEL_MAX, 0.0, 0.0, 0.0};
            ArmTurnRelative(&target_state);
            ArmPrintData();
            ArmTransmitOutput();
        } else
            osDelay(100);
#endif
    }
}

int ArmTurnRelative(joint_state_t* target) {
  joint_state_t* abs_target = new joint_state_t();
  abs_target->base_translate_0 = last_targets.base_translate_0 + target->base_translate_0;
  abs_target->base_yaw_rotate_1 = last_targets.base_yaw_rotate_1 + target->base_yaw_rotate_1;
  abs_target->base_pitch_rotate_2 = last_targets.base_pitch_rotate_2 + target->base_pitch_rotate_2;
  abs_target->forearm_pitch_3 = last_targets.forearm_pitch_3 + target->forearm_pitch_3;
  abs_target->forearm_roll_4 = last_targets.forearm_roll_4 + target->forearm_roll_4;
  abs_target->wrist_5 = last_targets.wrist_5 + target->wrist_5;
  abs_target->end_6 = last_targets.end_6 + target->end_6;

  return ArmTurnAbsolute(abs_target);
}

const float A1_Kp = 0.0025, A1_Kw = 0.004;

int ArmTurnAbsolute(joint_state_t* target) {
  if (target->base_yaw_rotate_1 >= BASE_VERT_ROTATE_MAX) {
    base_vert_rotate_motor->Control(BASE_PITCH_ID, 0.0, 0.0, BASE_VERT_ROTATE_MAX, A1_Kp, A1_Kw);
    target->base_yaw_rotate_1 = BASE_VERT_ROTATE_MAX;
  } else if (target->base_yaw_rotate_1 <= BASE_VERT_ROTATE_MIN) {
    base_vert_rotate_motor->Control(BASE_PITCH_ID, 0.0, 0.0, BASE_VERT_ROTATE_MIN,  A1_Kp, A1_Kw);
    target->base_yaw_rotate_1 = BASE_VERT_ROTATE_MIN;
  } else
    base_vert_rotate_motor->Control(BASE_PITCH_ID, 0.0, 0.0, target->base_yaw_rotate_1,  A1_Kp, A1_Kw);
  last_targets.base_yaw_rotate_1 = target->base_yaw_rotate_1;

  if (target->base_pitch_rotate_2 >= BASE_HOR_ROTATE_MAX) {
    base_hor_rotate_motor->Control(BASE_YAW_ID, 0.0, 0.0, BASE_HOR_ROTATE_MAX,  A1_Kp, A1_Kw);
    target->base_pitch_rotate_2 = BASE_HOR_ROTATE_MAX;
  } else if (target->base_pitch_rotate_2 <= BASE_HOR_ROTATE_MIN) {
    base_hor_rotate_motor->Control(BASE_YAW_ID, 0.0, 0.0, BASE_HOR_ROTATE_MIN,  A1_Kp, A1_Kw);
    target->base_pitch_rotate_2 = BASE_HOR_ROTATE_MIN;
  } else
    base_hor_rotate_motor->Control(BASE_YAW_ID, 0.0, 0.0, target->base_pitch_rotate_2,  A1_Kp, A1_Kw);
  last_targets.base_pitch_rotate_2 = target->base_pitch_rotate_2;

  if (target->forearm_pitch_3 >= ELBOW_ROTATE_MAX) {
    elbow_rotate_motor->Control(ELBOW_PITCH_ID, 0.0, 0.0, ELBOW_ROTATE_MAX,  A1_Kp, A1_Kw);
    target->forearm_pitch_3 = ELBOW_ROTATE_MAX;
  } else if (target->forearm_pitch_3 <= ELBOW_ROTATE_MIN) {
    elbow_rotate_motor->Control(ELBOW_PITCH_ID, 0.0, 0.0, ELBOW_ROTATE_MIN,  A1_Kp, A1_Kw);
    target->forearm_pitch_3 = ELBOW_ROTATE_MIN;
  } else
    elbow_rotate_motor->Control(ELBOW_PITCH_ID, 0.0, 0.0, target->forearm_pitch_3,  A1_Kp, A1_Kw);
  last_targets.forearm_pitch_3 = target->forearm_pitch_3;

  return 0;
}

void ArmTransmitOutput() {
  A1->Write((uint8_t*)(&(base_hor_rotate_motor->send[0].data)), base_hor_rotate_motor->send_length);
  osDelay(A1_CONTROL_DELAY);
  A1->Write((uint8_t*)(&(base_vert_rotate_motor->send[0].data)), base_vert_rotate_motor->send_length);
  osDelay(A1_CONTROL_DELAY);
  A1->Write((uint8_t*)(&(elbow_rotate_motor->send[0].data)), elbow_rotate_motor->send_length);
  osDelay(A1_CONTROL_DELAY);                          
}

void ArmPrintData() {
  print("Base Translate   : %10.4f\r\n", last_targets.base_translate_0);
  print("Base Vert Rotate : %10.4f\r\n", last_targets.base_yaw_rotate_1);
  print("Base Hor  Rotate : %10.4f\r\n", last_targets.base_pitch_rotate_2);
  print("Elbow Rotate     : %10.4f\r\n", last_targets.forearm_pitch_3);
  print("Forearm Rotate   : %10.4f\r\n", last_targets.forearm_roll_4);
  print("Wrist Rotate     : %10.4f\r\n", last_targets.wrist_5);
  print("Hand Rotate      : %10.4f\r\n", last_targets.end_6);
}

// Gear-ratio and ID-mapping check for the eoh_infant chassis, robot on a stand.
// Each wheel in CAN ID order 1..4 turns its ROTOR by an exact number of turns, measured
// by unwrapping the 13-bit rotor encoder (no gear-ratio assumption), then stops
// and holds for HOLD_MS. With a mark on each wheel, the operator checks where the
// mark ends up:
//   wheel turns = rotor turns / true ratio
// Verification pass: rotor turns chosen so each wheel should make a whole number of
// turns under the measured ratios (268:17 for IDs 1/3/4, ~7.80:1 for ID2), so every
// mark should return exactly to its start.
#include <cmath>

#include "bsp_print.h"
#include "cmsis_os.h"
#include "main.h"
#include "motor.h"
#include "usart.h"

namespace {

bsp::CAN* can = nullptr;
control::Motor3508* motor[4];
constexpr uint16_t ID[4] = {0x201, 0x202, 0x203, 0x204};

constexpr float RATIO[4] = {268.0f / 17.0f, 7.80f, 268.0f / 17.0f, 268.0f / 17.0f};
constexpr float WHEEL_TURNS[4] = {6.0f, 12.0f, 6.0f, 6.0f};
constexpr float MAX_ROTOR_SPEED = 50.0f;  // rad/s
constexpr uint32_t HOLD_MS = 12000;
constexpr int ONLY_INDEX = -1;  // -1 runs all wheels; 0..3 reruns one wheel
constexpr int COUNTDOWN_S = 20;

void Transmit(const int16_t out[4]) {
  for (int i = 0; i < 4; ++i) motor[i]->SetOutput(out[i]);
  control::MotorCANBase* container[] = {motor[0], motor[1], motor[2], motor[3]};
  control::MotorCANBase::TransmitOutput(container, 4);
  const uint8_t zero[8] = {};
  can->Transmit(0x1ff, zero, 8);
}

void Idle(uint32_t ms) {
  const int16_t zero[4] = {};
  for (uint32_t t = 0; t < ms; t += 10) {
    Transmit(zero);
    osDelay(10);
  }
}

// Unwrapped rotor angle from the [0, 2pi) encoder reading.
struct Unwrap {
  float last = 0.0f;
  float total = 0.0f;
  void Start(float theta) { last = theta; total = 0.0f; }
  void Update(float theta) {
    float d = theta - last;
    if (d > PI) d -= 2.0f * PI;
    if (d < -PI) d += 2.0f * PI;
    total += d;
    last = theta;
  }
};

}  // namespace

void RM_RTOS_Init() {
  print_use_uart(&huart7);
  can = new bsp::CAN(&hfdcan1, 0);
  for (int i = 0; i < 4; ++i) motor[i] = new control::Motor3508(can, ID[i]);
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  const uint8_t disable[8] = {255, 255, 255, 255, 255, 255, 255, 253};
  for (int i = 0; i < 3; ++i) {
    can->Transmit(0x1d, disable, 8);
    can->Transmit(0x11d, disable, 8);
    osDelay(10);
  }
  for (int s = COUNTDOWN_S; s > 0; --s) {
    print("[POS] put each wheel's mark at the top; starting in %d s\r\n", s);
    Idle(1000);
  }

  for (int w = 0; w < 4; ++w) {
    if (ONLY_INDEX >= 0 && w != ONLY_INDEX) continue;
    const float rotor_turns = WHEEL_TURNS[w] * RATIO[w];
    const float goal = rotor_turns * 2.0f * PI;
    print("[POS] >>> ID%d (0x%03X) MOVING: %.3f rotor turns (= %.0f wheel turns at %.3f:1) <<<\r\n",
          w + 1, ID[w], rotor_turns, WHEEL_TURNS[w], RATIO[w]);
    Unwrap pos;
    pos.Start(motor[w]->GetTheta());
    float integral = 0.0f;
    uint32_t last = HAL_GetTick();
    const uint32_t start = last;
    uint32_t settled_since = 0;
    uint32_t reported_ms = 0;
    bool stale = false;
    // Position stays actively held for HOLD_MS after arrival so the wheel cannot coast.
    while (HAL_GetTick() - start < 80000 && !(reported_ms && HAL_GetTick() - reported_ms > HOLD_MS)) {
      const uint32_t now = HAL_GetTick();
      const float dt = clip<float>((now - last) * 0.001f, 0.001f, 0.02f);
      last = now;
      pos.Update(motor[w]->GetTheta());
      stale |= !motor[w]->connection_flag_ || now - motor[w]->GetLastFeedbackMs() > 50;
      const float remaining = goal - pos.total;
      const float ramp_in = fminf(1.0f, (now - start) / 1000.0f);
      const float speed_cmd =
          clip<float>(4.0f * remaining, -MAX_ROTOR_SPEED * ramp_in, MAX_ROTOR_SPEED * ramp_in);
      const float error = speed_cmd - motor[w]->GetOmega();
      integral = clip<float>(integral + 20.0f * error * dt, -3000.0f, 3000.0f);
      int16_t out[4] = {};
      out[w] = static_cast<int16_t>(clip<float>(10.0f * error + integral, -4000.0f, 4000.0f));
      Transmit(out);
      if (fabsf(remaining) < 0.5f && fabsf(motor[w]->GetOmega()) < 3.0f) {
        if (!settled_since) settled_since = now;
        if (!reported_ms && now - settled_since > 300) {
          reported_ms = now;
          print("[POS] === ID%d STOPPED: rotor_turns=%.3f (target %.3f)%s - check the mark now (%lu s, holding) ===\r\n",
                w + 1, pos.total / (2.0f * PI), rotor_turns, stale ? " STALE" : "",
                (unsigned long)(HOLD_MS / 1000));
        }
      } else {
        settled_since = 0;
      }
      osDelay(2);
    }
    const int16_t zero[4] = {};
    Transmit(zero);
    print("[POS] ID%d released: final rotor_turns=%.3f\r\n", w + 1, pos.total / (2.0f * PI));
    Idle(1000);
  }
  print("[POS] ALL DONE\r\n");
  while (true) Idle(1000);
}

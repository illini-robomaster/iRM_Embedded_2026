// Bench test for the four eoh_infant chassis wheels, robot lifted off the ground.
// Runs automatically after boot, logs on UART7 (115200):
//   1. Open-loop current step per wheel (same current on every wheel).
//   2. Closed-loop speed steps per wheel through the eoh_mc02 ramp/filter/PI chain.
//   3. All four wheels together: spin pattern, then straight-drive pattern.
// Yaw, shooter and pitch are held at zero / disabled for the whole test.
#include <cmath>

#include "bsp_print.h"
#include "cmsis_os.h"
#include "controller.h"
#include "main.h"
#include "motor.h"
#include "usart.h"

namespace {

bsp::CAN* can = nullptr;
control::Motor3508* wheel[4];
// Same order, ratios and directions as eoh_mc02.cc.
constexpr uint16_t WHEEL_ID[4] = {0x203, 0x202, 0x204, 0x201};
constexpr const char* WHEEL_NAME[4] = {"FR", "BL", "BR", "FL"};
constexpr float RATIO[4] = {268.0f / 17.0f, 7.80f, 268.0f / 17.0f, 268.0f / 17.0f};
constexpr float DIRECTION[4] = {1.0f, 1.0f, -1.0f, 1.0f};
// Keep in sync with eoh_mc02.cc ChassisFeedforward() (set to zero to re-measure load).
constexpr float FF_STATIC[4] = {282.0f, 616.0f, 553.0f, 242.0f};
constexpr float FF_VISCOUS[4] = {12.6f, 13.5f, 11.6f, 10.7f};

constexpr float OPEN_LOOP_CURRENT = 1500.0f;  // counts, wheel frame
constexpr uint32_t OPEN_LOOP_MS = 1500;
constexpr float SPEED_STEPS[] = {10.0f, 20.0f, 40.0f, -20.0f};  // wheel rad/s
constexpr uint32_t STEP_MS = 1500;
constexpr uint32_t REST_MS = 1000;

enum class Mode { kIdle, kCurrent, kSpeed };

// Keep in sync with eoh_mc02.cc ChassisPI.
struct ChassisPI {
  float kp = 200.0f;
  float ki = 600.0f;
  float integral = 0.0f;
  void Reset() { integral = 0.0f; }
  float Compute(float error, bool integrate, float dt) {
    if (integrate) integral = clip<float>(integral + ki * error * dt, -4000.0f, 4000.0f);
    return clip<float>(kp * error + integral, -6000.0f, 6000.0f);
  }
};
// Per-wheel gains under test. Softer BR (M3508) gains (120 / 360) cut its current
// noise but did not reduce its overshoot and slowed reversal, so all match eoh.
constexpr float TEST_KP[4] = {200.0f, 200.0f, 200.0f, 200.0f};
constexpr float TEST_KI[4] = {600.0f, 600.0f, 600.0f, 600.0f};

struct WheelState {
  ChassisPI pid;
  float ramped = 0.0f;
  float filtered = 0.0f;
  int16_t command = 0;  // wheel-frame counts before direction flip
};
WheelState state[4];

float WheelOmega(int i) { return DIRECTION[i] * wheel[i]->GetOmega() / RATIO[i]; }

bool FeedbackFresh(int i) {
  return wheel[i]->connection_flag_ && HAL_GetTick() - wheel[i]->GetLastFeedbackMs() <= 100;
}

void ResetWheel(int i) {
  state[i].pid.Reset();
  state[i].ramped = 0.0f;
  state[i].filtered = 0.0f;
  state[i].command = 0;
}

// Identical to the eoh_mc02.cc chassis chain: ramp -> 10 ms filter (DM3519 only) -> PI.
int16_t SpeedControl(int i, float target_in, float dt) {
  WheelState& s = state[i];
  const float previous = s.ramped;
  const bool reversing = previous * target_in < 0.0f;
  const float target = reversing ? 0.0f : target_in;
  const bool slowing = reversing || fabsf(target) < fabsf(previous);
  const float step = (slowing ? 300.0f : 200.0f) * dt;
  s.ramped += clip<float>(target - previous, -step, step);
  if (i != 2) {
    s.filtered += (1.0f - expf(-dt / 0.01f)) * (s.ramped - s.filtered);
    if (fabsf(target) < 0.1f && fabsf(s.filtered) < 0.1f) s.filtered = 0.0f;
  } else {
    s.filtered = s.ramped;
  }
  if (fabsf(s.filtered) < 0.1f && fabsf(target) < 0.1f) s.pid.Reset();
  const float ff = FF_STATIC[i] * clip<float>(s.filtered, -1.0f, 1.0f) + FF_VISCOUS[i] * s.filtered;
  return static_cast<int16_t>(
      clip<float>(ff + s.pid.Compute(s.filtered - WheelOmega(i), fabsf(target - s.filtered) < 0.5f, dt),
                  -8000.0f, 8000.0f));
}

void TransmitWheels() {
  for (int i = 0; i < 4; ++i)
    wheel[i]->SetOutput(static_cast<int16_t>(DIRECTION[i] * state[i].command));
  control::MotorCANBase* container[] = {wheel[0], wheel[1], wheel[2], wheel[3]};
  control::MotorCANBase::TransmitOutput(container, 4);
  const uint8_t zero[8] = {};
  can->Transmit(0x1ff, zero, 8);  // yaw, feeder, flywheels off
}

void DisablePitch() {
  const uint8_t disable[8] = {255, 255, 255, 255, 255, 255, 255, 253};
  can->Transmit(0x1d, disable, 8);
  can->Transmit(0x11d, disable, 8);
}

// Runs one segment. targets[] are per wheel (rad/s or counts), active[] selects wheels.
void RunSegment(const char* tag, Mode mode, const float targets[4], const bool active[4],
                uint32_t duration_ms, bool reset = true) {
  if (reset)
    for (int i = 0; i < 4; ++i) ResetWheel(i);
  const uint32_t start = HAL_GetTick();
  uint32_t last = start;
  uint32_t last_print = 0;
  uint32_t next_tick = osKernelGetTickCount();
  while (HAL_GetTick() - start < duration_ms) {
    const uint32_t now = HAL_GetTick();
    const float dt = clip<float>((now - last) * 0.001f, 0.001f, 0.05f);
    last = now;
    bool fresh = true;
    for (int i = 0; i < 4; ++i) fresh &= FeedbackFresh(i);
    for (int i = 0; i < 4; ++i) {
      if (!fresh || !active[i] || mode == Mode::kIdle) {
        ResetWheel(i);
      } else if (mode == Mode::kCurrent) {
        state[i].command = static_cast<int16_t>(targets[i]);
        state[i].filtered = targets[i];
      } else {
        state[i].command = SpeedControl(i, targets[i], dt);
      }
    }
    TransmitWheels();
    if (now - last_print >= 20) {
      last_print = now;
      // t, then per wheel: target(filtered setpoint or counts) measured(rad/s) cmd fbI
      print("D,%s,%lu", tag, (unsigned long)(now - start));
      for (int i = 0; i < 4; ++i) {
        if (!active[i]) continue;
        print(",%s,%.2f,%.2f,%d,%d", WHEEL_NAME[i], targets[i], WheelOmega(i),
              state[i].command, static_cast<int>(DIRECTION[i] * wheel[i]->GetCurr()));
      }
      print(fresh ? "\r\n" : ",STALE\r\n");
    }
    next_tick += 5;
    if (static_cast<int32_t>(next_tick - osKernelGetTickCount()) <= 0)
      next_tick = osKernelGetTickCount() + 5;
    osDelayUntil(next_tick);
  }
}

void Rest(uint32_t ms) {
  const float zero[4] = {};
  const bool none[4] = {};
  RunSegment("REST", Mode::kIdle, zero, none, ms);
}

}  // namespace

void RM_RTOS_Init() {
  print_use_uart(&huart7);
  can = new bsp::CAN(&hfdcan1, 0);
  for (int i = 0; i < 4; ++i) {
    wheel[i] = new control::Motor3508(can, WHEEL_ID[i]);
    state[i].pid.kp = TEST_KP[i];
    state[i].pid.ki = TEST_KI[i];
  }
}

void RM_RTOS_Default_Task(const void* args) {
  UNUSED(args);
  for (int i = 0; i < 3; ++i) {
    DisablePitch();
    TransmitWheels();
    osDelay(10);
  }
  for (int s = 5; s > 0; --s) {
    TransmitWheels();
    print("[WHEEL TEST] starting in %d s | conn FR=%d BL=%d BR=%d FL=%d\r\n", s,
          FeedbackFresh(0), FeedbackFresh(1), FeedbackFresh(2), FeedbackFresh(3));
    for (int k = 0; k < 100; ++k) {
      TransmitWheels();
      osDelay(10);
    }
  }

  // Test order by CAN ID 1..4 (array indices 3, 1, 0, 2).
  constexpr int ID_ORDER[4] = {3, 1, 0, 2};

  // 1. Open-loop current step, each wheel alone.
  for (int w : ID_ORDER) {
    float targets[4] = {};
    bool active[4] = {};
    targets[w] = OPEN_LOOP_CURRENT;
    active[w] = true;
    print("[SEG] CUR wheel=%s id=0x%03X counts=%d\r\n", WHEEL_NAME[w], WHEEL_ID[w],
          (int)OPEN_LOOP_CURRENT);
    RunSegment("CUR", Mode::kCurrent, targets, active, OPEN_LOOP_MS);
    Rest(REST_MS);
  }

  // 2. Closed-loop speed steps, each wheel alone.
  for (int w : ID_ORDER) {
    bool first = true;
    for (float speed : SPEED_STEPS) {
      float targets[4] = {};
      bool active[4] = {};
      targets[w] = speed;
      active[w] = true;
      print("[SEG] SPD wheel=%s id=0x%03X target=%.1f\r\n", WHEEL_NAME[w], WHEEL_ID[w], speed);
      // Keep ramp/PI state between steps so each step starts from the running speed.
      RunSegment("SPD", Mode::kSpeed, targets, active, STEP_MS, first);
      first = false;
    }
    Rest(REST_MS);
  }

  // 3. All wheels: spin (same sign) and straight drive (eoh pattern for y=+20).
  const bool all[4] = {true, true, true, true};
  const float spin[4] = {15.0f, 15.0f, 15.0f, 15.0f};
  print("[SEG] SPIN all=15.0\r\n");
  RunSegment("SPIN", Mode::kSpeed, spin, all, 3000);
  Rest(REST_MS);
  const float b = 20.0f / 1.4142f;
  const float straight[4] = {-b, b, b, -b};  // vel = -beta, beta, -alpha, alpha with x=0
  print("[SEG] STRAIGHT y=20\r\n");
  RunSegment("STR", Mode::kSpeed, straight, all, 3000);
  Rest(REST_MS);

  print("[WHEEL TEST] DONE\r\n");
  while (true) {
    TransmitWheels();
    osDelay(10);
  }
}

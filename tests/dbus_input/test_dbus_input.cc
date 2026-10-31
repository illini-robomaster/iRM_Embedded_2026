/*
 * Host regression test for the DBUS input path of the Omni MC02 target.
 *
 * It compiles the production logic that decides what a burst contains
 * (shared/libraries/dbus_frame.h), how mouse displacement is consumed
 * (shared/libraries/dbus_mouse.h), how the BSP repairs the receive chain after a
 * HAL UART error (shared/bsp/bsp_uart_rx.h) and how the Omni gimbal reacts to the
 * consumed displacement (vehicles/Omni/mouse_gimbal_step.h). No HAL, no hardware.
 *
 * Build and run (host compiler, no cross toolchain involved):
 *
 *   cmake -S tests/dbus_input -B /tmp/irm-dbus-input-tests
 *   cmake --build /tmp/irm-dbus-input-tests
 *   (cd /tmp/irm-dbus-input-tests && ctest --output-on-failure)
 *
 * or directly
 *
 *   g++ -std=c++17 -Wall -Wextra -Werror -pthread \
 *       -Ishared/libraries -Ishared/bsp -Ivehicles/Omni \
 *       tests/dbus_input/test_dbus_input.cc -o /tmp/test_dbus_input && /tmp/test_dbus_input
 *
 * The DMA handle state machine is covered by a small model of the HAL rules the
 * restart depends on (see TestRxRestart*), not by the HAL itself. What is left to the
 * target: the register writes of the repair, the critical section of
 * remote::DBUS::TakeMouseDelta, and the HAL behaviour the model is transcribed from.
 */

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <thread>

#include "bsp_uart_rx.h"
#include "dbus_frame.h"
#include "dbus_mouse.h"
#include "mouse_gimbal_step.h"

namespace {

int g_checks = 0;
int g_failures = 0;

void Report(bool ok, const char* what, int line) {
  ++g_checks;
  if (ok) return;
  ++g_failures;
  std::printf("FAIL line %d: %s\n", line, what);
}

void CheckTrue(bool value, const char* what, int line) { Report(value, what, line); }

void CheckEqual(long expected, long actual, const char* what, int line) {
  ++g_checks;
  if (expected == actual) return;
  ++g_failures;
  std::printf("FAIL line %d: %s (expected %ld, got %ld)\n", line, what, expected, actual);
}

void CheckNear(float expected, float actual, float tolerance, const char* what, int line) {
  ++g_checks;
  const float diff = expected > actual ? expected - actual : actual - expected;
  if (diff <= tolerance) return;
  ++g_failures;
  std::printf("FAIL line %d: %s (expected %f, got %f)\n", line, what,
              static_cast<double>(expected), static_cast<double>(actual));
}

#define CHECK_TRUE(cond) CheckTrue((cond), #cond, __LINE__)
#define CHECK_EQ(expected, actual) CheckEqual((expected), (actual), #actual, __LINE__)
#define CHECK_NEAR(expected, actual, tol) CheckNear((expected), (actual), (tol), #actual, __LINE__)

constexpr uint32_t kCapacity = remote::kDbusBurstBufferBytes; /* 2 frames + 1 byte */

/* ------------------------------------------------------------------ framing */

void TestBurstFraming() {
  uint8_t buffer[kCapacity + 4] = {};

  remote::DbusBurst empty = remote::ParseDbusBurst(buffer, 0, kCapacity);
  CHECK_TRUE(empty.status == remote::DbusBurstStatus::kEmpty);
  CHECK_EQ(0, empty.frame_count);

  /* bsp::UART::Read reports -1 for a discarded (error-polluted) burst */
  remote::DbusBurst failed = remote::ParseDbusBurst(buffer, -1, kCapacity);
  CHECK_TRUE(failed.status == remote::DbusBurstStatus::kEmpty);
  CHECK_EQ(0, failed.frame_count);

  remote::DbusBurst one = remote::ParseDbusBurst(buffer, 18, kCapacity);
  CHECK_TRUE(one.status == remote::DbusBurstStatus::kAccepted);
  CHECK_EQ(1, one.frame_count);

  /* two frames in one burst (idle gap too short for a second interrupt) */
  remote::DbusBurst two = remote::ParseDbusBurst(buffer, 36, kCapacity);
  CHECK_TRUE(two.status == remote::DbusBurstStatus::kAccepted);
  CHECK_EQ(2, two.frame_count);

  remote::DbusBurst short_burst = remote::ParseDbusBurst(buffer, 17, kCapacity);
  CHECK_TRUE(short_burst.status == remote::DbusBurstStatus::kTooShort);
  CHECK_EQ(0, short_burst.frame_count);

  /* one stray byte: the prefix looks like a frame but the boundary is polluted */
  remote::DbusBurst ragged = remote::ParseDbusBurst(buffer, 19, kCapacity);
  CHECK_TRUE(ragged.status == remote::DbusBurstStatus::kRagged);
  CHECK_EQ(0, ragged.frame_count);

  remote::DbusBurst truncated = remote::ParseDbusBurst(buffer, kCapacity, kCapacity);
  CHECK_TRUE(truncated.status == remote::DbusBurstStatus::kTruncated);
  CHECK_EQ(0, truncated.frame_count);

  remote::DbusBurst overflow = remote::ParseDbusBurst(buffer, 54, kCapacity);
  CHECK_TRUE(overflow.status == remote::DbusBurstStatus::kTruncated);

  /* the old 19-byte buffer of the diagnostic cannot hold a 36-byte burst: that
   * configuration must report truncation instead of decoding a partial frame */
  remote::DbusBurst old_config = remote::ParseDbusBurst(buffer, 36, 19);
  CHECK_TRUE(old_config.status == remote::DbusBurstStatus::kTruncated);

  remote::DbusBurst null_data = remote::ParseDbusBurst(nullptr, 18, kCapacity);
  CHECK_TRUE(null_data.status == remote::DbusBurstStatus::kEmpty);
}

/* ------------------------------------------------------- mouse consumption */

void TestMouseDeltaConsumedOncePerFrame() {
  remote::MouseDeltaAccumulator accumulator;
  int16_t dx = 0;
  int16_t dy = 0;

  /* nothing received yet */
  CHECK_TRUE(!accumulator.Take(&dx, &dy, 1000));
  CHECK_EQ(0, dx);

  accumulator.Accumulate(7, -3, 1001);
  CHECK_TRUE(accumulator.Take(&dx, &dy, 1002));
  CHECK_EQ(7, dx);
  CHECK_EQ(-3, dy);

  /* the same frame must not be integrated by the next control iteration */
  CHECK_TRUE(!accumulator.Take(&dx, &dy, 1003));
  CHECK_EQ(0, dx);
  CHECK_EQ(0, dy);
}

void TestMouseDeltaAccumulatesFramesBetweenPolls() {
  remote::MouseDeltaAccumulator accumulator;

  /* three frames (6 ms of input at ~500 frames/s) between two 5 ms polls */
  accumulator.Accumulate(4, 1, 2000);
  accumulator.Accumulate(-2, 5, 2002);
  accumulator.Accumulate(10, -1, 2004);

  int16_t dx = 0;
  int16_t dy = 0;
  CHECK_TRUE(accumulator.Take(&dx, &dy, 2005));
  CHECK_EQ(12, dx);
  CHECK_EQ(5, dy);

  CHECK_TRUE(!accumulator.Take(&dx, &dy, 2006));
}

void TestMouseDeltaSaturates() {
  remote::MouseDeltaAccumulator accumulator;
  const uint32_t before = accumulator.overflow_count();

  accumulator.Accumulate(30000, -30000, 3000);
  accumulator.Accumulate(30000, -30000, 3002);

  int16_t dx = 0;
  int16_t dy = 0;
  CHECK_TRUE(accumulator.Take(&dx, &dy, 3003));
  CHECK_EQ(remote::kMouseDeltaAxisLimit, dx);
  CHECK_EQ(-remote::kMouseDeltaAxisLimit, dy);
  CHECK_EQ(before + 2u, accumulator.overflow_count());
}

void TestMouseDeltaStaleIsDropped() {
  remote::MouseDeltaAccumulator accumulator;
  int16_t dx = 0;
  int16_t dy = 0;

  /* link lost: the last frame is older than the freshness window */
  accumulator.Accumulate(50, 50, 4000);
  CHECK_TRUE(!accumulator.Take(&dx, &dy, 4000 + remote::kMouseDeltaStaleMs + 1u));
  CHECK_EQ(0, dx);
  CHECK_EQ(0, dy);

  /* dropped, not deferred: a later poll must not see it either */
  CHECK_TRUE(!accumulator.Take(&dx, &dy, 4000 + remote::kMouseDeltaStaleMs + 2u));

  /* exactly at the window boundary it is still used */
  accumulator.Accumulate(9, 9, 5000);
  CHECK_TRUE(accumulator.Take(&dx, &dy, 5000 + remote::kMouseDeltaStaleMs));
  CHECK_EQ(9, dx);
}

void TestMouseDeltaDiscard() {
  remote::MouseDeltaAccumulator accumulator;
  int16_t dx = 0;
  int16_t dy = 0;

  accumulator.Accumulate(20, 20, 6000);
  accumulator.Discard();
  CHECK_TRUE(!accumulator.HasPending());
  CHECK_TRUE(!accumulator.Take(&dx, &dy, 6001));
  CHECK_EQ(0, dx);
  CHECK_EQ(0, dy);
}

/* The target serializes the interrupt-side accumulation against the task-side
 * take with taskENTER_CRITICAL (remote::DBUS::TakeMouseDelta). A mutex stands in
 * for that critical section here: the invariant is the same, every accumulated
 * count must be taken exactly once. */
void TestMouseDeltaConcurrentWriterReader() {
  remote::MouseDeltaAccumulator accumulator;
  std::mutex guard;
  std::atomic<bool> writer_done(false);
  constexpr int kFrames = 20000;
  long consumed = 0;
  uint32_t now_ms = 7000;

  std::thread writer([&]() {
    for (int i = 0; i < kFrames; ++i) {
      std::lock_guard<std::mutex> lock(guard);
      accumulator.Accumulate(1, -1, now_ms);
      /* keep the accumulation fresh: the reader below runs "at 500 Hz" */
      if ((i % 8) == 0) ++now_ms;
    }
    writer_done = true;
  });

  std::thread reader([&]() {
    while (!writer_done) {
      int16_t dx = 0;
      int16_t dy = 0;
      std::lock_guard<std::mutex> lock(guard);
      if (accumulator.Take(&dx, &dy, now_ms)) consumed += dx;
    }
  });

  writer.join();
  reader.join();

  int16_t dx = 0;
  int16_t dy = 0;
  std::lock_guard<std::mutex> lock(guard);
  if (accumulator.Take(&dx, &dy, now_ms)) consumed += dx;

  CHECK_EQ(kFrames, consumed);
}

/* ------------------------------------------------------- gimbal reaction */

void TestGimbalStepScaling() {
  omni::MouseGimbalStep step = omni::ComputeMouseGimbalStep(10, -10, true, true);
  CHECK_NEAR(-10 * omni::kMouseYawGain, step.yaw, 1e-6f);
  CHECK_NEAR(10 * omni::kMousePitchGain, step.pitch, 1e-6f);

  /* per-iteration anti-jump clamp */
  step = omni::ComputeMouseGimbalStep(30000, -30000, true, true);
  CHECK_NEAR(-omni::kMouseStepLimit, step.yaw, 1e-6f);
  CHECK_NEAR(omni::kMouseStepLimit, step.pitch, 1e-6f);
}

void TestGimbalStepDisabledDropsMovement() {
  /* enabled -> disabled: the displacement was consumed and must not be applied */
  omni::MouseGimbalStep step = omni::ComputeMouseGimbalStep(100, 100, false, true);
  CHECK_NEAR(0.0f, step.yaw, 1e-6f);
  CHECK_NEAR(0.0f, step.pitch, 1e-6f);

  /* autoaim owns a locked gimbal: same, the movement is dropped, not deferred */
  step = omni::ComputeMouseGimbalStep(100, 100, true, false);
  CHECK_NEAR(0.0f, step.yaw, 1e-6f);
  CHECK_NEAR(0.0f, step.pitch, 1e-6f);

  /* after re-enabling, only newly received displacement is applied */
  step = omni::ComputeMouseGimbalStep(0, 0, true, true);
  CHECK_NEAR(0.0f, step.yaw, 1e-6f);
  CHECK_NEAR(0.0f, step.pitch, 1e-6f);
}

void TestDisabledLoopConsumesAndDrops() {
  /* full input path of one control iteration while the outputs are disabled,
   * followed by the first enabled iteration */
  remote::MouseDeltaAccumulator accumulator;
  int16_t dx = 0;
  int16_t dy = 0;

  accumulator.Accumulate(1000, 1000, 8000);
  CHECK_TRUE(accumulator.Take(&dx, &dy, 8001));
  omni::MouseGimbalStep disabled_step = omni::ComputeMouseGimbalStep(dx, dy, false, true);
  CHECK_NEAR(0.0f, disabled_step.yaw, 1e-6f);

  /* next iteration with a single fresh frame */
  accumulator.Accumulate(2, 2, 8010);
  CHECK_TRUE(accumulator.Take(&dx, &dy, 8011));
  CHECK_EQ(2, dx);
  omni::MouseGimbalStep enabled_step = omni::ComputeMouseGimbalStep(dx, dy, true, true);
  CHECK_NEAR(-2 * omni::kMouseYawGain, enabled_step.yaw, 1e-6f);
}

void TestEnabledWindowDropsPreEnableMovement() {
  /* The enabling iteration takes the displacement that arrived while the outputs were
   * still disabled at the top of the loop, so eoh_mc02 has to drop that value as well
   * as the accumulator when it closes the arm-up window. */
  remote::MouseDeltaAccumulator accumulator;
  int16_t dx = 0;
  int16_t dy = 0;

  accumulator.Accumulate(500, -500, 9000); /* arrived while disabled */
  CHECK_TRUE(accumulator.Take(&dx, &dy, 9001));
  CHECK_EQ(500, dx);

  /* arm-up window: both the accumulator and the already taken value are dropped */
  accumulator.Accumulate(120, 120, 9050);
  accumulator.Discard();
  dx = 0;
  dy = 0;

  const omni::MouseGimbalStep first_enabled = omni::ComputeMouseGimbalStep(dx, dy, true, true);
  CHECK_NEAR(0.0f, first_enabled.yaw, 1e-6f);
  CHECK_NEAR(0.0f, first_enabled.pitch, 1e-6f);

  /* only what arrives after the window is applied */
  accumulator.Accumulate(2, 0, 9100);
  CHECK_TRUE(accumulator.Take(&dx, &dy, 9101));
  CHECK_NEAR(-2 * omni::kMouseYawGain, omni::ComputeMouseGimbalStep(dx, dy, true, true).yaw,
             1e-6f);
}

/* -------------------------------------------------- UART error recovery */

bsp::RxChainSnapshot HealthyChain() {
  bsp::RxChainSnapshot chain = {};
  chain.uart_dma_request = true;
  chain.dma_stream_enabled = true;
  chain.dma_stream_full = false;
  chain.error_irq_enabled = true;
  chain.parity_irq_enabled = true;
  chain.idle_irq_enabled = true;
  return chain;
}

void TestRxRepairPlanning() {
  CHECK_TRUE(bsp::PlanRxRepair(HealthyChain(), false) == bsp::RxRepair::kNone);

  /* HAL_UART_IRQHandler error path: CR3.DMAR cleared but the stream still runs */
  bsp::RxChainSnapshot chain = HealthyChain();
  chain.uart_dma_request = false;
  CHECK_TRUE(bsp::PlanRxRepair(chain, false) == bsp::RxRepair::kRearmRequest);

  /* HAL_UART_EndRxTransfer disabled the error interrupts */
  chain = HealthyChain();
  chain.error_irq_enabled = false;
  CHECK_TRUE(bsp::PlanRxRepair(chain, false) == bsp::RxRepair::kRearmRequest);

  chain = HealthyChain();
  chain.parity_irq_enabled = false;
  CHECK_TRUE(bsp::PlanRxRepair(chain, false) == bsp::RxRepair::kRearmRequest);

  chain = HealthyChain();
  chain.idle_irq_enabled = false;
  CHECK_TRUE(bsp::PlanRxRepair(chain, false) == bsp::RxRepair::kRearmRequest);

  /* the transfer stopped: the stream has to be restarted, not only the request bits */
  chain = HealthyChain();
  chain.dma_stream_enabled = false;
  CHECK_TRUE(bsp::PlanRxRepair(chain, false) == bsp::RxRepair::kRearmStreamAndRequest);

  /* a pending error is always a full repair, even if the bits look healthy */
  CHECK_TRUE(bsp::PlanRxRepair(HealthyChain(), true) == bsp::RxRepair::kRearmStreamAndRequest);

  /* error evidence: the masks must stay disjoint so a code can be decoded */
  CHECK_EQ(0, bsp::kRxErrorParity & bsp::kRxErrorNoise);
  CHECK_EQ(0, bsp::kRxErrorParity & bsp::kRxErrorOverrun);
  CHECK_EQ(0, bsp::kRxErrorFrame & bsp::kRxErrorDma);
  const uint32_t pe_and_fe = bsp::kRxErrorParity | bsp::kRxErrorFrame;
  CHECK_EQ(bsp::kRxErrorFrame, pe_and_fe & bsp::kRxErrorFrame);
}

void TestRxBurstClassification() {
  /* armed chain: the burst in the buffer is the one to decode */
  CHECK_TRUE(bsp::ClassifyRxBurst(HealthyChain(), false) == bsp::RxBurstVerdict::kUsable);

  /* only the request/IRQ bits are missing: the stream kept filling the buffer, so
   * the burst is still whole */
  bsp::RxChainSnapshot chain = HealthyChain();
  chain.uart_dma_request = false;
  CHECK_TRUE(bsp::ClassifyRxBurst(chain, false) == bsp::RxBurstVerdict::kUsable);

  /* an error always wins over a healthy looking chain */
  CHECK_TRUE(bsp::ClassifyRxBurst(HealthyChain(), true) == bsp::RxBurstVerdict::kDiscarded);

  /* stopped stream without a full buffer: an aborted burst, never decoded */
  chain = HealthyChain();
  chain.dma_stream_enabled = false;
  CHECK_TRUE(bsp::ClassifyRxBurst(chain, false) == bsp::RxBurstVerdict::kDiscarded);

  /* stopped because the normal-mode transfer completed: the burst filled the buffer,
   * which is truncation and not a reception error */
  chain = HealthyChain();
  chain.dma_stream_enabled = false;
  chain.dma_stream_full = true;
  CHECK_TRUE(bsp::ClassifyRxBurst(chain, false) == bsp::RxBurstVerdict::kTruncated);

  /* ... unless an error is what stopped it */
  CHECK_TRUE(bsp::ClassifyRxBurst(chain, true) == bsp::RxBurstVerdict::kDiscarded);

  /* the truncation verdict is what makes the framing rule see the full length */
  uint8_t buffer[kCapacity] = {};
  const uint32_t full = kCapacity;
  CHECK_TRUE(remote::ParseDbusBurst(buffer, static_cast<int32_t>(full), kCapacity).status ==
             remote::DbusBurstStatus::kTruncated);
}

void TestRxErrorEvidenceCountsOnce() {
  /* the HAL error callback delivered the code: the polled path in Read() must not
   * count the same error a second time */
  bsp::RxErrorEvidence from_callback;
  from_callback.Record(bsp::kRxErrorOverrun);
  CHECK_EQ(1, from_callback.count);
  CHECK_EQ(bsp::kRxErrorOverrun, from_callback.last_code);
  CHECK_TRUE(from_callback.ConsumePending(bsp::kRxErrorOverrun));
  CHECK_EQ(1, from_callback.count);
  /* Read() clears huart->ErrorCode after consuming it, so the next burst passes 0 */
  CHECK_TRUE(!from_callback.ConsumePending(bsp::kRxErrorNone));
  CHECK_EQ(1, from_callback.count);

  /* the deferred abort callback never ran: ErrorCode is the only evidence, so the
   * polled path has to count it */
  bsp::RxErrorEvidence polled;
  CHECK_TRUE(polled.ConsumePending(bsp::kRxErrorFrame));
  CHECK_EQ(1, polled.count);
  CHECK_EQ(bsp::kRxErrorFrame, polled.last_code);
  CHECK_TRUE(!polled.ConsumePending(bsp::kRxErrorNone));
  CHECK_EQ(1, polled.count);

  /* a new error is counted again */
  CHECK_TRUE(polled.ConsumePending(bsp::kRxErrorNoise));
  CHECK_EQ(2, polled.count);
  CHECK_EQ(bsp::kRxErrorNoise, polled.last_code);

  /* a read without any error keeps the burst and counts nothing */
  bsp::RxErrorEvidence quiet;
  CHECK_TRUE(!quiet.ConsumePending(bsp::kRxErrorNone));
  CHECK_EQ(0, quiet.count);
  CHECK_EQ(bsp::kRxErrorNone, quiet.last_code);
}

/* ------------------------------------------------- DMA restart (HAL model) */

/* Minimal model of the HAL rules the restart depends on, transcribed from
 * stm32h7xx_hal_dma.c / stm32f4xx_hal_dma.c of this repository:
 * - HAL_DMA_Start()/HAL_DMAEx_MultiBufferStart() accept HAL_DMA_STATE_READY only;
 * - HAL_DMA_Abort() accepts HAL_DMA_STATE_BUSY only and leaves READY behind;
 * - HAL_DMA_Abort_IT() turns BUSY into ABORT and disables the stream, and its
 *   XferAbortCallback is only run from the stream's transfer-complete interrupt;
 * - the UART blocking error path arms UART_DMAAbortOnError and clears CR3.DMAR. */
enum class FakeDmaState { kReady,
                          kBusy,
                          kAbort,
                          kError };

struct FakeDma {
  FakeDmaState state = FakeDmaState::kReady;
  bool stream_enabled = false;
  bool stream_interrupt_enabled = false; /* polling-mode transfers never set this */
  bool stream_refuses_to_stop = false;   /* hardware that never clears DMA_SxCR_EN */
  bool xfer_abort_callback_armed = false;
  bool stale_abort_callback_ran = false;
  int start_attempts = 0;
  int start_accepted = 0;
};

bsp::RxDmaState ToRxDmaState(FakeDmaState state) {
  switch (state) {
    case FakeDmaState::kReady:
      return bsp::RxDmaState::kReady;
    case FakeDmaState::kBusy:
      return bsp::RxDmaState::kBusy;
    case FakeDmaState::kAbort:
      return bsp::RxDmaState::kAbort;
    default:
      return bsp::RxDmaState::kUnusable;
  }
}

bool FakeHalDmaStart(FakeDma& dma) {
  ++dma.start_attempts;
  if (dma.state != FakeDmaState::kReady) return false;
  dma.state = FakeDmaState::kBusy;
  dma.stream_enabled = true;
  ++dma.start_accepted;
  return true;
}

bool FakeHalDmaAbort(FakeDma& dma) {
  if (dma.state != FakeDmaState::kBusy) return false;
  dma.state = FakeDmaState::kReady;
  dma.stream_enabled = false;
  return true;
}

/* __HAL_DMA_DISABLE() followed by the bounded enable-wait: false means the stream is
 * still running and must not be reconfigured */
bool FakeHalDmaStop(FakeDma& dma) {
  if (dma.stream_refuses_to_stop) return false;
  dma.stream_enabled = false;
  return true;
}

/* HAL_UART_IRQHandler() blocking error path with CR3.DMAR set */
void FakeHalUartErrorPath(FakeDma& dma) {
  dma.xfer_abort_callback_armed = true;
  if (dma.state == FakeDmaState::kBusy) {
    dma.state = FakeDmaState::kAbort;
    dma.stream_enabled = false;
    /* the deferred callback needs the stream interrupt, which polling mode never
     * enables, so nothing reports the error here */
  }
}

/* What bsp::UART::PrepareRxDmaRestart() + StartRxDmaInto() do to the handle, driven
 * by the HAL-free plan; *ct_if_double_buffer receives the CT the restart selects. */
bool RestartRxDma(FakeDma& dma, uint8_t rx_index, bool double_buffer, bool* ct) {
  if (double_buffer) *ct = bsp::RxDoubleBufferCtForIndex(rx_index);

  dma.xfer_abort_callback_armed = false;
  dma.stream_interrupt_enabled = false;
  if (!FakeHalDmaStop(dma)) return false;

  switch (bsp::PlanRxStreamRestart(ToRxDmaState(dma.state))) {
    case bsp::RxStreamRestart::kReadyNow:
      break;
    case bsp::RxStreamRestart::kAbortThenStart:
      FakeHalDmaAbort(dma);
      break;
    case bsp::RxStreamRestart::kForceReadyThenStart:
      dma.state = FakeDmaState::kReady;
      break;
  }
  return FakeHalDmaStart(dma);
}

/* The restart the reviewed code attempted: HAL_DMA_Abort() then HAL_DMA_Start(). */
bool RestartRxDmaWithoutNormalization(FakeDma& dma) {
  FakeHalDmaAbort(dma);
  return FakeHalDmaStart(dma);
}

void TestRxRestartPlanning() {
  CHECK_TRUE(bsp::PlanRxStreamRestart(bsp::RxDmaState::kReady) ==
             bsp::RxStreamRestart::kReadyNow);
  CHECK_TRUE(bsp::PlanRxStreamRestart(bsp::RxDmaState::kBusy) ==
             bsp::RxStreamRestart::kAbortThenStart);
  /* the state HAL_DMA_Abort_IT() leaves behind refuses both HAL calls */
  CHECK_TRUE(bsp::PlanRxStreamRestart(bsp::RxDmaState::kAbort) ==
             bsp::RxStreamRestart::kForceReadyThenStart);
  CHECK_TRUE(bsp::PlanRxStreamRestart(bsp::RxDmaState::kUnusable) ==
             bsp::RxStreamRestart::kForceReadyThenStart);
}

void TestRxRestartAfterHalError() {
  /* regression: a reception error leaves HAL_DMA_STATE_ABORT behind, and the
   * abort-then-start sequence alone can never recover from it */
  FakeDma dma;
  CHECK_TRUE(RestartRxDmaWithoutNormalization(dma)); /* a running transfer aborts fine */
  CHECK_TRUE(dma.state == FakeDmaState::kBusy);

  FakeHalUartErrorPath(dma);
  CHECK_TRUE(dma.state == FakeDmaState::kAbort);
  CHECK_TRUE(!dma.stream_enabled);

  FakeDma stuck = dma;
  CHECK_TRUE(!RestartRxDmaWithoutNormalization(stuck));
  CHECK_EQ(1, stuck.start_accepted); /* still only the first, successful start */

  bool ct = false;
  CHECK_TRUE(RestartRxDma(dma, 1, false, &ct));
  CHECK_TRUE(dma.state == FakeDmaState::kBusy);
  CHECK_TRUE(dma.stream_enabled);

  /* a failed abort leaves HAL_DMA_STATE_ERROR, which must not block the retry either */
  FakeDma timed_out;
  timed_out.state = FakeDmaState::kError;
  CHECK_TRUE(RestartRxDma(timed_out, 0, false, &ct));
  CHECK_TRUE(timed_out.state == FakeDmaState::kBusy);
}

void TestRxRestartDropsStaleAbortCallback() {
  FakeDma dma;
  CHECK_TRUE(RestartRxDma(dma, 0, false, nullptr));
  FakeHalUartErrorPath(dma);
  CHECK_TRUE(dma.xfer_abort_callback_armed);

  /* the error left the abort callback armed on the stream; if it survived the
   * restart, any later transfer complete would report the old error again */
  CHECK_TRUE(RestartRxDma(dma, 0, false, nullptr));
  CHECK_TRUE(!dma.xfer_abort_callback_armed);
  CHECK_TRUE(!dma.stream_interrupt_enabled);
  CHECK_TRUE(!dma.stale_abort_callback_ran);
}

void TestRxRestartBoundedStop() {
  /* A stream that never acknowledges the disable must not be reconfigured: the
   * restart has to give up and let the next burst retry instead of hanging the
   * interrupt in the HAL's own enable-wait (which cannot time out there). */
  FakeDma dma;
  CHECK_TRUE(RestartRxDma(dma, 0, false, nullptr));
  dma.stream_refuses_to_stop = true;
  CHECK_TRUE(!RestartRxDma(dma, 0, false, nullptr));
  CHECK_EQ(1, dma.start_attempts);              /* only the first start happened */
  CHECK_TRUE(dma.state == FakeDmaState::kBusy); /* handle untouched by the failed try */
}

void TestRxDoubleBufferCtSelection() {
  /* a restart has to leave the stream writing rx_data_[rx_index_]: M0AR is
   * rx_data_[0], M1AR is rx_data_[1] and CT selects between them */
  CHECK_TRUE(!bsp::RxDoubleBufferCtForIndex(0));
  CHECK_TRUE(bsp::RxDoubleBufferCtForIndex(1));

  for (uint8_t index = 0; index < 2; ++index) {
    FakeDma dma;
    bool ct = false;
    CHECK_TRUE(RestartRxDma(dma, index, true, &ct));
    CHECK_EQ(index, ct ? 1 : 0);
  }

  /* ... which is the same relationship Read() keeps while it flips both on every
   * burst: the buffer the DMA writes and the one handed to the caller never match */
  bool ct = false;
  uint8_t rx_index = 0;
  for (int burst = 0; burst < 4; ++burst) {
    FakeDma dma;
    CHECK_TRUE(RestartRxDma(dma, rx_index, true, &ct));
    CHECK_EQ(rx_index, ct ? 1 : 0);
    /* healthy switch of Read(): flip the index, then the CT bit */
    rx_index = 1 - rx_index;
    ct = !ct;
    CHECK_EQ(rx_index, ct ? 1 : 0);
  }
}

}  // namespace

int main() {
  TestBurstFraming();
  TestMouseDeltaConsumedOncePerFrame();
  TestMouseDeltaAccumulatesFramesBetweenPolls();
  TestMouseDeltaSaturates();
  TestMouseDeltaStaleIsDropped();
  TestMouseDeltaDiscard();
  TestMouseDeltaConcurrentWriterReader();
  TestGimbalStepScaling();
  TestGimbalStepDisabledDropsMovement();
  TestDisabledLoopConsumesAndDrops();
  TestEnabledWindowDropsPreEnableMovement();
  TestRxRepairPlanning();
  TestRxBurstClassification();
  TestRxErrorEvidenceCountsOnce();
  TestRxRestartPlanning();
  TestRxRestartAfterHalError();
  TestRxRestartDropsStaleAbortCallback();
  TestRxRestartBoundedStop();
  TestRxDoubleBufferCtSelection();

  std::printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}

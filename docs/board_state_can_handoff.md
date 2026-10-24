# Board-state CAN handoff — STM32 / Embedded 2023

Date: October 24, 2026
Status: implemented, built, flashed, and verified against Jetson Vision 2026 on
the physical Type C **CAN1** pathway.

The authoritative wire specification is
`iRM_Vision_2026/documentation/board_state_can_protocol.md`. The Jetson
integration guide is
`iRM_Vision_2026/documentation/board_state_can_handoff.md`.

## 1. Current baseline

| Repository | Branch | Implementation commit |
|---|---|---|
| `iRM_Embedded_2023` | `ywh114/hw_vision2026` | `12825bddfd5e8672fd1de6d0c5cf4714697d703d` |
| `iRM_Vision_2026` | `ywh114/hw_vision2026` | `f26090dfdecd68b9fbb9ea3045b6c7273761382a` |

Current verified hardware setup:

- DJI Type C board, BMI088/IST8310 IMU and heater.
- `hcan1` for the Jetson link and the two example Motor6020s (`0x205`, `0x206`).
- Jetson `can0`, 1 Mbps, Classical CAN, standard 11-bit IDs, DLC=8.
- `GIMBAL_SENTRY` model; measured motor angles use `GetTheta()` minus the
  firmware-local gimbal offsets.
- Current flashed target: `example_minipc_cboardcan.elf` (CAN1).
- Alternative target for different wiring: `example_minipc_cboardcan_can2.elf`.
  CAN2 was not the active port on the verified setup.

This example streams telemetry only. It does not implement motor enable,
firing, or mode semantics. Its 1 kHz gimbal hold is not gated by the Jetson
handshake.

## 2. Files and responsibilities

| Path | Responsibility |
|---|---|
| `shared/libraries/board_state_can_protocol.h/.cc` | Pure wire codec and CBoard session FSM; no HAL/RTOS dependency |
| `examples/minipc/CBoardCAN.cc` | Type C example: IMU, gimbal, ISR→task RX queue, handshake, snapshot capture, 100 Hz streaming |
| `shared/bsp/bsp_can.h/.cc` | Nonblocking `TryTransmit`, bounded legacy `Transmit`, DLC-aware and strict registration, ISR-safe registration/dispatch |
| `examples/minipc/CMakeLists.txt` | CAN1 and CAN2 example targets |
| `shared/libraries/minipc_protocol.h` | Existing packed 29-byte `board_state_t`; unchanged wire layout |
| `examples/minipc/BoardState.cc` | Existing UART example; retained as a separate reference/regression target |

The new CAN example is Type C/bxCAN only. DM_MC_02/FDCAN support was not added;
the DM_CAN regression target still builds unchanged.

## 3. Build and flash

From the Embedded repository root:

```bash
cmake -S . -B build                 # only needed to register new targets
make -C build example_minipc_cboardcan.elf -j$(nproc)
make -C build example_minipc_cboardcan_can2.elf -j$(nproc)
```

Flash the port that is physically wired:

```bash
# Verified current setup: Jetson link on Type C CAN1.
make -C build flash-example_minipc_cboardcan

# Alternative only when the Jetson link is physically wired to CAN2.
make -C build flash-example_minipc_cboardcan_can2
```

Both targets compile the same source. CAN2 is selected by
`BOARD_STATE_CAN_USE_CAN2=1`; CAN1 is initialized first because it is the CAN2
master/clock dependency on STM32F4. Motors remain on `hcan1` in both variants.

Useful regression builds:

```bash
make -C build \
  example_minipc_cboardcan.elf \
  example_minipc_cboardcan_can2.elf \
  example_minipc_boardstate.elf \
  example_minipc.elf \
  example_can_mc02.elf
```

All five targets built successfully at handoff. `st-flash` reported the CAN1
image written and verified before the physical acceptance runs.

Before powering motors, use the robot's normal mechanical safety procedure.
Flashing alone is not a motion test, but this example enters the same kind of
gimbal hold after IMU calibration.

## 4. Runtime architecture

### 4.1 Tasks and queues

- CAN RX ISR validates standard data-frame dispatch, preserves DLC, and copies
  handshake frames into a bounded SPSC ring. It performs no session parsing,
  allocation, blocking, or CAN transmission.
- `commTask` starts immediately, including during IMU calibration. It answers
  START with NOT_READY until `imu->DataReady() && imu->CaliDone()`.
- After a matching final ACK and READY, `commTask` captures one 29-byte snapshot
  and queues its five fragments one at a time. Handshake responses have priority.
- The gimbal task runs independently at 1 kHz after IMU calibration. Telemetry
  never substitutes target angles for measured motor angles.
- `FillBoardState()` uses a short critical section while copying IMU and motor
  scalars so CAN feedback or the IMU task cannot tear one snapshot.

One CAN attempt is made per 1 ms communication-loop pass. A sample starts at a
10 ms cadence, and the next sample cannot begin until all five current fragments
have been accepted into hardware mailboxes. Bus congestion may reduce the rate;
it must not overlap sample sequences.

### 4.2 Session behavior

- START/final-ACK use big-endian session integers and zero reserved bytes.
- CBoard responds immediately and retries ACK+status+color every 100 ms until a
  matching final ACK arrives.
- A repeated START for the same session requests another response without
  resetting an active stream.
- A new session drops unsent fragments, resets sample sequencing, and requires
  a new final ACK before streaming.
- VERSION_MISMATCH is reported without streaming. A later version-1 START can
  establish a valid session.
- Readiness loss suspends streaming and invalidates a partial sample. Recovery
  cannot resume a torn sequence.
- There is no continuing Jetson heartbeat in v1. CBoard continues streaming
  until reset, readiness loss, or a new-session START. Jetson's 500 ms timeout
  drives reconnection.
- `QUEUED` means the hardware mailbox accepted a frame, not that the Jetson
  application received it. The repeated-READY/final-ACK mechanism covers final
  ACK loss.

## 5. Data sources and placeholders

The transmitted image is exactly `communication::board_state_t`, 29 bytes:

| Field | Source in `CBoardCAN.cc` | Deployment note |
|---|---|---|
| `q[0..3]` | `imu->INS_quat`, wxyz | Sent only after IMU readiness/calibration |
| `yaw` | `wrap(yaw_motor->GetTheta() - yaw_offset, -PI, PI)` | Measured motor angle, not IMU attitude or target |
| `pitch` | `wrap(pitch_motor->GetTheta() - pitch_offset, -PI, PI)` | Measured motor angle |
| `bullet_speed` | Fixed `15.0f` | **Example placeholder**; connect referee `shoot_data.bullet_speed` |
| `mode[0]` | Fixed `1` | **Example placeholder**; opaque to the protocol |
| ACK color | Fixed `Color::BLUE` | **Example placeholder**; use actual robot color, fixed per session |

Do not move offset correction to the Jetson. Do not assign protocol-owned
meanings to the mode byte. Protocol color values are red=1/blue=2 and are not
`auto_aim::Color` values.

## 6. CAN driver changes that must be preserved

### `TryTransmit()`

- Never busy-waits. Returns `QUEUED`, `BUSY`, or `ERROR`.
- `BUSY` means all TX mailboxes are occupied; retry the same frame later.
- `ERROR` includes invalid arguments, a stopped/errored controller, or HAL
  rejection.
- Copies short payloads into an eight-byte local buffer because the STM32 HAL
  reads all eight data bytes even when DLC is shorter.
- Uses the same short mailbox critical section as legacy `Transmit`; call it
  from task context, not the RX ISR.

### Legacy `Transmit()`

- Retains blocking completion behavior but now has a **2 ms deadline**.
- On timeout it aborts that mailbox and returns `-1` instead of spinning forever
  when auto-retransmission has no peer ACK.
- Do not remove the timeout to hide a physical CAN fault. Check the bus and the
  caller's error handling instead.

### RX callback registration

- Legacy `RegisterRxCallback()` and `RegisterDlcRxCallback()` replace an
  existing ID's callback **in the same slot**. `ServoMotor` and `Steering6020`
  rely on this override behavior; duplicate-ID rejection here would silently
  break them.
- The new link uses `RegisterUniqueDlcRxCallback()` so accidental ID collisions
  fail during initialization without changing legacy motor behavior.
- Registration and mailbox setup are protected against CAN RX IRQ concurrency.
- Non-standard and RTR frames are discarded; the board-state protocol requires
  standard data frames with DLC=8.

Debug-visible example health counters:

- `board_state_can_rx_dropped`: RX ring overflow.
- `board_state_can_tx_busy`: mailbox backpressure attempts.
- `board_state_can_tx_error`: controller/HAL/argument errors.

Also inspect `ip -details -statistics link show can0` on the Jetson. Userspace
counters alone do not replace controller error counters.

## 7. Wire quick reference

| ID | Direction | Contents |
|---|---|---|
| `0x320` | Jetson→CBoard | START `01 01 session_BE 00 00`; final ACK `03 01 session_BE 00 00` |
| `0x321` | CBoard→Jetson | ACK+color `02 01 session_BE status color` |
| `0x322` | CBoard→Jetson | Five board-state fragments per sample |

Fragment format:

```text
byte0 = (fragment_index << 5) | (sample_seq & 0x1F)
byte1 = session_id & 0xFF
bytes2..7 = six bytes of the little-endian 29-byte payload
```

Fragment 4 carries payload bytes 24..28 and zero in byte7. Sample sequence wraps
modulo 32. Handshake integers are big-endian; payload floats remain the same
little-endian image as the UART `board_state_t`.

There is no UART framing or application CRC on CAN. CAN provides per-frame
integrity; the session tag, sequence, fragment mask, and timeouts handle loss,
reordering, and stale samples.

## 8. Verification and bring-up

### Recorded results

- Pure codec/session checks, native Jetson tests, and vcan delayed-ready and
  stall/reconnect tests pass.
- Physical CAN1: three independent runs each produced **500 complete samples in
  5 seconds = 100 Hz**.
- Zero Jetson userspace drops, zero rejected samples, and zero CAN error
  counters in the passing runs.
- Live values included changing quaternion, measured yaw `-0.2`, pitch `-1.5`,
  bullet speed 15, and mode 1 from this example.
- Existing motor/vision-control regressions still build; the Jetson
  `relative_servo_test` baseline is 128/128.

### Jetson-side acceptance command

After flashing CAN1 and connecting the powered splitter:

```bash
cd /home/irm/iRM-Vision-2026
make -C build board_state_can_test -j$(nproc)
./build/board_state_can_test -c=configs/board_state_can.yaml
```

Expect a fresh session, `streaming=1 valid=1 color=blue`, approximately 50 new
packets every 0.5 seconds, and `live verification passed`.

## 9. Troubleshooting

### No ACK / Jetson BUS-OFF

Check the physical layer first: cable, CANH/CANL, common ground, splitter and
CBoard power, termination, and actual CAN port. A bad cable caused exactly this
failure during bring-up; both CAN1 and CAN2 firmware variants failed before the
cable was replaced. After replacement, CAN1 worked immediately at 100 Hz.

Reset Jetson `can0` after correcting the physical fault:

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

Do not debug a no-ACK bus by changing fragmentation or increasing timeouts.

### Traffic exists but no `0x321/0x322`

Confirm the firmware target matches the wired CBoard port. On the verified
setup, CAN2 firmware saw a healthy bus but no board-state response; CAN1 was the
correct target. Also confirm protocol version and ID allocation.

### Persistent NOT_READY

Allow normal IMU initialization/calibration and keep the board still. The
communication task is already running, so this is not a task-start ordering
problem. Check `DataReady()` and `CaliDone()` rather than bypassing readiness.

### Telemetry affects control timing

Confirm the link uses `TryTransmit()`, one attempt per comm-loop pass, and no
`Transmit()` busy-wait in the communication task. Inspect TX health counters and
bus load. Do not raise telemetry priority above the control task or remove the
mailbox critical sections.

## 10. Porting into a real vehicle

- Reuse `BoardStateCanServer` and the pure codec instead of duplicating the wire
  layout in a vehicle task.
- Register `0x320` with `RegisterUniqueDlcRxCallback()` on the physically correct
  CAN instance; leave existing motor callback replacement semantics intact.
- Keep ISR work bounded and parse frames in task context.
- Feed the server one coherent `board_state_t` snapshot and advance a fragment
  only after `QUEUED`.
- Replace all three example placeholders (color, bullet speed, mode) with real
  sources before deployment.
- Keep telemetry independent of motor-control enable and the existing
  angle-control protocol. Do not use this stream as authorization to move or
  fire.
- Recheck IDs, port, termination, and physical acceptance on each robot. If a
  future protocol adds source timestamps, heartbeat/stop semantics, or new
  fields, bump the protocol version on both sides together.

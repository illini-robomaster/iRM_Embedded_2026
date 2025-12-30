# DM-MC01 Board - PWM Configuration Guide

## MCU Information
- **MCU**: STM32F446RCT6
- **APB1 Timer Clock**: 84 MHz (TIM2, TIM3, TIM4, TIM5)
- **APB2 Timer Clock**: 168 MHz (TIM1, TIM8)

---

## Available PWM Channels

### TIM1 (APB2 - 168 MHz)
| Channel | GPIO Pin | Alternate Function | Description |
|---------|----------|-------------------|-------------|
| CH1 | PA8 | AF1 | General PWM |

### TIM2 (APB1 - 84 MHz)
| Channel | GPIO Pin | Alternate Function | Description |
|---------|----------|-------------------|-------------|
| CH4 | PB2 | AF1 | General PWM |

**Note**: TIM2 is configured with Prescaler=83, making effective clock = 1 MHz

### TIM3 (APB1 - 84 MHz)
| Channel | GPIO Pin | Alternate Function | Description |
|---------|----------|-------------------|-------------|
| CH2 | PB5 | AF2 | General PWM |

**Note**: TIM3 Period=4999 (5000 counts)

### TIM8 (APB2 - 168 MHz)
| Channel | GPIO Pin | Alternate Function | Description |
|---------|----------|-------------------|-------------|
| CH1 | PC6 | AF3 | Servo PWM |
| CH2 | PC7 | AF3 | Servo PWM |
| CH3 | PC8 | AF3 | Servo PWM |
| CH4 | PC9 | AF3 | Servo PWM |

**Default Config**: Prescaler=167, Period=19999 → **50Hz PWM** (for servos)

---

## Usage Examples

### Servo Control (TIM8 - Pre-configured for 50Hz)
```cpp
#include "bsp_pwm.h"

// TIM8 is pre-configured for servo: 168MHz / 168 / 20000 = 50Hz
// Clock after prescaler = 168MHz / 168 = 1 MHz
// Using 1MHz as clock_freq since prescaler is already set
bsp::PWM servo1(&htim8, 1, 1000000, 50, 1500);  // 1.5ms center
bsp::PWM servo2(&htim8, 2, 1000000, 50, 1500);
bsp::PWM servo3(&htim8, 3, 1000000, 50, 1500);
bsp::PWM servo4(&htim8, 4, 1000000, 50, 1500);

servo1.Start();
servo2.Start();
servo3.Start();
servo4.Start();

// Move servos
servo1.SetPulseWidth(1000);  // Min position
servo1.SetPulseWidth(2000);  // Max position
```

### Alternative: Use Full Clock Frequency
```cpp
// If you want full control over prescaler
// TIM8 APB2 = 168 MHz
bsp::PWM pwm(&htim8, 1, 168000000, 50, 1500);
pwm.Start();
// Note: This will reconfigure the timer period
```

### TIM1 High-Speed PWM
```cpp
// TIM1 at 168 MHz - for high-frequency applications
bsp::PWM led(&htim1, 1, 168000000, 10000, 50);  // 10kHz, 50us pulse
led.Start();
```

---

## GPIO Configuration (CubeMX Settings)

For each PWM pin:
1. **Mode**: `TIMx_CHy` (Alternate Function)
2. **GPIO Pull-up/Pull-down**: No pull-up/pull-down
3. **GPIO Speed**: High

---

## Pin Mapping Summary

```
PWM Outputs on DM-MC01 Board:
═══════════════════════════════════════════════════════════════
TIM1_CH1  → PA8   (168 MHz timer)
─────────────────────────────────────────────────────────────
TIM2_CH4  → PB2   (84 MHz timer, prescaled to 1 MHz)
─────────────────────────────────────────────────────────────
TIM3_CH2  → PB5   (84 MHz timer, period=4999)
─────────────────────────────────────────────────────────────
TIM8_CH1  → PC6   (168 MHz timer, configured for 50Hz servo)
TIM8_CH2  → PC7   (168 MHz timer, configured for 50Hz servo)
TIM8_CH3  → PC8   (168 MHz timer, configured for 50Hz servo)
TIM8_CH4  → PC9   (168 MHz timer, configured for 50Hz servo)
═══════════════════════════════════════════════════════════════
Total: 7 PWM channels available
```

---

## Default Timer Configurations (from CubeMX)

| Timer | Prescaler | Period | Clock After Prescaler | PWM Frequency |
|-------|-----------|--------|----------------------|---------------|
| TIM1 | 0 | Auto | 168 MHz | User defined |
| TIM2 | 83 | 0 | 1 MHz | User defined |
| TIM3 | 0 | 4999 | 84 MHz | 16.8 kHz |
| TIM8 | 167 | 19999 | 1 MHz | **50 Hz** |

---

## Common Applications for MC01

| Application | Recommended Timer | Configuration |
|------------|------------------|---------------|
| RC Servo | TIM8 CH1-4 | 50Hz, 1000-2000µs |
| LED PWM | TIM3 CH2 | Pre-configured 16.8kHz |
| Motor PWM | TIM1 CH1 | High-speed capable |
| Encoder | TIM2 | Pre-configured for encoder |

---

## Hardware Notes

### Onboard Servo Headers
The MC01 board has **4 servo connectors** directly connected to TIM8:
- **SERVO1**: PC6 (TIM8_CH1)
- **SERVO2**: PC7 (TIM8_CH2)
- **SERVO3**: PC8 (TIM8_CH3)
- **SERVO4**: PC9 (TIM8_CH4)

### Important Considerations
1. TIM8 is pre-configured for 50Hz servo operation
2. TIM2 CH4 (PB2) may conflict with BOOT1 pin on some boards
3. TIM3 is also used for FreeRTOS tick on some configurations
4. Check your specific board pinout before connecting

---

## Notes

1. **TIM8** is optimized for servo control (50Hz output)
2. **TIM1** is available for high-frequency applications
3. **TIM2** is a 32-bit timer useful for precise timing
4. The MC01 is a compact board - check physical pin availability
5. Some PWM pins may be shared with other functions (SPI, I2C)

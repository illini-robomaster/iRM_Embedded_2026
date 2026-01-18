# DJI Board Type A - PWM Configuration Guide

## MCU Information
- **MCU**: STM32F427IIH6
- **APB1 Timer Clock**: 84 MHz (TIM2, TIM3, TIM4, TIM5, TIM12)
- **APB2 Timer Clock**: 168 MHz (TIM1, TIM8)

---

## Available PWM Channels

### TIM1 (APB2 - 168 MHz)
| Channel | GPIO Pin | Alternate Function | Description |
|---------|----------|-------------------|-------------|
| CH1 | PA8 | AF1 | General PWM |
| CH4 | PE14 | AF1 | General PWM |

### TIM4 (APB1 - 84 MHz)
| Channel | GPIO Pin | Alternate Function | Description |
|---------|----------|-------------------|-------------|
| CH1 | PD12 | AF2 | General PWM |
| CH2 | PD13 | AF2 | General PWM |
| CH3 | PD14 | AF2 | General PWM |
| CH4 | PD15 | AF2 | General PWM |

### TIM5 (APB1 - 84 MHz)
| Channel | GPIO Pin | Alternate Function | Description |
|---------|----------|-------------------|-------------|
| CH1 | PH10 | AF2 | General PWM |
| CH2 | PH11 | AF2 | General PWM |
| CH3 | PH12 | AF2 | General PWM |
| CH4 | PI0 | AF2 | General PWM |

### TIM12 (APB1 - 84 MHz)
| Channel | GPIO Pin | Alternate Function | Description |
|---------|----------|-------------------|-------------|
| CH1 | PH6 | AF9 | General PWM |
| CH2 | PH9 | AF9 | General PWM |

---

## Usage Examples

### Basic PWM Initialization
```cpp
#include "bsp_pwm.h"

// Example: Servo on TIM4_CH1 (PD12)
// 50Hz output frequency, 1500us pulse width (center position)
bsp::PWM servo(&htim4, 1, 84000000, 50, 1500);
servo.Start();

// Change pulse width to move servo
servo.SetPulseWidth(1000);  // 1000us = full left
servo.SetPulseWidth(2000);  // 2000us = full right
```

### PWM for Motor Control (ESC)
```cpp
// ESC typically uses 50Hz with 1000-2000us pulse
bsp::PWM esc(&htim5, 1, 84000000, 50, 1000);
esc.Start();

// Arm ESC
esc.SetPulseWidth(1000);
osDelay(2000);

// Set throttle (1000=min, 2000=max)
esc.SetPulseWidth(1200);  // 20% throttle
```

### LED Brightness Control
```cpp
// High frequency PWM for LED (no flicker)
// 1kHz, 50% duty cycle
bsp::PWM led(&htim4, 2, 84000000, 1000, 500);
led.Start();

// Set brightness (0-1000us for 0-100% at 1kHz)
led.SetPulseWidth(250);  // 25% brightness
```

---

## GPIO Configuration (CubeMX Settings)

For each PWM pin, configure in CubeMX:
1. **Mode**: `TIMx_CHy` (Alternate Function)
2. **GPIO Pull-up/Pull-down**: No pull-up/pull-down
3. **GPIO Speed**: High
4. **Alternate Function**: See table above

---

## Pin Mapping Summary

```
PWM Outputs on Type A Board:
═══════════════════════════════════════════════════════════════
TIM1_CH1  → PA8   (168 MHz timer)
TIM1_CH4  → PE14  (168 MHz timer)
─────────────────────────────────────────────────────────────
TIM4_CH1  → PD12  (84 MHz timer)
TIM4_CH2  → PD13  (84 MHz timer)
TIM4_CH3  → PD14  (84 MHz timer)
TIM4_CH4  → PD15  (84 MHz timer)
─────────────────────────────────────────────────────────────
TIM5_CH1  → PH10  (84 MHz timer)
TIM5_CH2  → PH11  (84 MHz timer)
TIM5_CH3  → PH12  (84 MHz timer)
TIM5_CH4  → PI0   (84 MHz timer)
─────────────────────────────────────────────────────────────
TIM12_CH1 → PH6   (84 MHz timer)
TIM12_CH2 → PH9   (84 MHz timer)
═══════════════════════════════════════════════════════════════
Total: 12 PWM channels available
```

---

## Common Applications

| Application | Recommended Timer | Frequency | Pulse Width |
|------------|------------------|-----------|-------------|
| RC Servo | TIM4/TIM5 | 50 Hz | 1000-2000 µs |
| ESC (Brushless Motor) | TIM4/TIM5 | 50-400 Hz | 1000-2000 µs |
| LED Dimming | TIM4/TIM5 | 1-10 kHz | 0-period µs |
| Buzzer | TIM1 | 100-5000 Hz | 50% duty |
| DShot ESC | TIM1 | 300-1200 kHz | Special |

---

## Notes

1. **TIM1** is an advanced timer with complementary outputs and break functionality
2. **TIM4/TIM5** are general-purpose 32-bit timers (TIM5) and 16-bit timers (TIM4)
3. **TIM12** is a basic 16-bit timer with 2 channels
4. All timers on APB1 run at 84 MHz after prescaler
5. All timers on APB2 run at 168 MHz after prescaler

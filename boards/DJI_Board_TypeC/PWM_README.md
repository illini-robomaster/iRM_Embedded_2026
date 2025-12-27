# DJI Board Type C - PWM Configuration Guide

## MCU Information
- **MCU**: STM32F407IGH6
- **APB1 Timer Clock**: 84 MHz (TIM2, TIM3, TIM4, TIM5)
- **APB2 Timer Clock**: 168 MHz (TIM1, TIM8, TIM10)

---

## Available PWM Channels

### TIM1 (APB2 - 168 MHz)
| Channel | GPIO Pin | Alternate Function | Description |
|---------|----------|-------------------|-------------|
| CH1 | PE9 | AF1 | General PWM |
| CH2 | PE11 | AF1 | General PWM |
| CH3 | PE13 | AF1 | General PWM |

### TIM4 (APB1 - 84 MHz)
| Channel | GPIO Pin | Alternate Function | Description |
|---------|----------|-------------------|-------------|
| CH3 | PD14 | AF2 | General PWM |

### TIM5 (APB1 - 84 MHz)
| Channel | GPIO Pin | Alternate Function | Description |
|---------|----------|-------------------|-------------|
| CH1 | PH10 | AF2 | Servo PWM |
| CH2 | PH11 | AF2 | Servo PWM |
| CH3 | PH12 | AF2 | Servo PWM |

### TIM10 (APB2 - 168 MHz)
| Channel | GPIO Pin | Alternate Function | Description |
|---------|----------|-------------------|-------------|
| CH1 | PF6 | AF3 | General PWM |

---

## Usage Examples

### Basic PWM Initialization
```cpp
#include "bsp_pwm.h"

// Example: Servo on TIM5_CH1 (PH10)
// APB1 timer clock = 84 MHz
// 50Hz output frequency, 1500us pulse width (center position)
bsp::PWM servo(&htim5, 1, 84000000, 50, 1500);
servo.Start();

// Move servo
servo.SetPulseWidth(1000);  // 1000us = one extreme
servo.SetPulseWidth(2000);  // 2000us = other extreme
```

### High-Speed PWM on TIM1
```cpp
// TIM1 runs at 168 MHz - good for high-frequency applications
// Example: 10kHz PWM for motor driver
bsp::PWM motor(&htim1, 1, 168000000, 10000, 50);  // 50us = 50% at 10kHz
motor.Start();
```

### Buzzer Control
```cpp
// Buzzer on TIM1_CH3 (PE13)
// Play a 440Hz tone (A4 note)
bsp::PWM buzzer(&htim1, 3, 168000000, 440, 1136);  // ~50% duty
buzzer.Start();

// Change frequency for different notes
buzzer.SetFrequency(523);  // C5
buzzer.SetFrequency(659);  // E5
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
PWM Outputs on Type C Board:
═══════════════════════════════════════════════════════════════
TIM1_CH1  → PE9   (168 MHz timer)
TIM1_CH2  → PE11  (168 MHz timer)
TIM1_CH3  → PE13  (168 MHz timer)
─────────────────────────────────────────────────────────────
TIM4_CH3  → PD14  (84 MHz timer)
─────────────────────────────────────────────────────────────
TIM5_CH1  → PH10  (84 MHz timer)
TIM5_CH2  → PH11  (84 MHz timer)
TIM5_CH3  → PH12  (84 MHz timer)
─────────────────────────────────────────────────────────────
TIM10_CH1 → PF6   (168 MHz timer)
═══════════════════════════════════════════════════════════════
Total: 8 PWM channels available
```

---

## Hardware Notes

### Onboard Connections
- **TIM5 (PH10, PH11, PH12)**: Connected to servo connectors on the board
- **TIM1**: Available on expansion headers

### Default CubeMX Configuration
| Timer | Prescaler | Period | Initial Pulse |
|-------|-----------|--------|---------------|
| TIM1 | 0 | Auto | 1000 |
| TIM5 | 0 | Auto | 0 |

---

## Common Applications

| Application | Recommended Timer | Frequency | Pulse Width |
|------------|------------------|-----------|-------------|
| RC Servo | TIM5 | 50 Hz | 1000-2000 µs |
| ESC | TIM5 | 50-400 Hz | 1000-2000 µs |
| LED Dimming | TIM4 | 1-10 kHz | Variable |
| Buzzer | TIM1 | 100-5000 Hz | 50% duty |
| High-speed PWM | TIM1/TIM10 | 10-100 kHz | Variable |

---

## Notes

1. **TIM1** is an advanced timer - best for high-frequency applications
2. **TIM5** is a 32-bit general-purpose timer - best for precise timing
3. **TIM10** is a single-channel timer derived from TIM1 clock
4. Type C board has fewer PWM pins exposed compared to Type A
5. TIM5 channels are typically connected to onboard servo headers

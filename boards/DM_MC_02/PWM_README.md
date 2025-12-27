# DM-MC02 Board - PWM Configuration Guide

## MCU Information
- **MCU**: STM32H723VGT6
- **Timer Clock**: 240 MHz (All timers after internal multiplier)
- **APB1/APB2**: 120 MHz (timers get 2x = 240 MHz)

---

## Available PWM Channels

### TIM1 (240 MHz)
| Channel | GPIO Pin | Alternate Function | Description |
|---------|----------|-------------------|-------------|
| CH1 | PE9 | AF1 | General PWM |
| CH3 | PE13 | AF1 | General PWM |

**Default Config**: Prescaler=24, Period=10000 → **~960 Hz PWM**

### TIM2 (240 MHz)
| Channel | GPIO Pin | Alternate Function | Description |
|---------|----------|-------------------|-------------|
| CH1 | PA0 | AF1 | General PWM |
| CH3 | PA2 | AF1 | General PWM |

**Default Config**: Prescaler=24, Period=10000 → **~960 Hz PWM**

### TIM3 (240 MHz)
| Channel | GPIO Pin | Alternate Function | Description |
|---------|----------|-------------------|-------------|
| CH4 | PB1 | AF2 | General PWM |

**Default Config**: Prescaler=23 (24-1), Period=9999 (10000-1) → **~1 kHz PWM**

### TIM12 (240 MHz)
| Channel | GPIO Pin | Alternate Function | Description |
|---------|----------|-------------------|-------------|
| CH2 | PB15 | AF2 | General PWM |

**Default Config**: Prescaler=23 (24-1), Period=65535 → Low frequency capable

---

## Usage Examples

### Basic PWM Initialization
```cpp
#include "bsp_pwm.h"

// TIM1/TIM2 with default prescaler (24+1 = 25)
// Effective clock = 240 MHz / 25 = 9.6 MHz
// For precise control, account for prescaler or use raw 240MHz

// Method 1: Use full timer clock (240 MHz)
bsp::PWM pwm1(&htim1, 1, 240000000, 50, 1500);  // 50Hz servo
pwm1.Start();

// Method 2: Account for prescaler (if using CubeMX default)
// Clock after prescaler = 240MHz / 25 = 9.6 MHz
bsp::PWM pwm2(&htim2, 1, 9600000, 50, 1500);
pwm2.Start();
```

### Servo Control
```cpp
// 50Hz PWM for standard RC servo
bsp::PWM servo(&htim1, 1, 240000000, 50, 1500);
servo.Start();

// Servo positions
servo.SetPulseWidth(500);   // Minimum (some servos)
servo.SetPulseWidth(1000);  // -90 degrees
servo.SetPulseWidth(1500);  // Center (0 degrees)
servo.SetPulseWidth(2000);  // +90 degrees
servo.SetPulseWidth(2500);  // Maximum (some servos)
```

### High-Speed LED PWM
```cpp
// 10kHz PWM for flicker-free LED dimming
bsp::PWM led(&htim3, 4, 240000000, 10000, 50);  // 50us = 50% at 10kHz
led.Start();

// Brightness control (at 10kHz, period = 100us)
led.SetPulseWidth(10);   // 10% brightness
led.SetPulseWidth(50);   // 50% brightness
led.SetPulseWidth(90);   // 90% brightness
```

### Motor ESC Control
```cpp
// ESC initialization (1000-2000us protocol)
bsp::PWM esc(&htim2, 3, 240000000, 50, 1000);  // Start at minimum
esc.Start();

// ESC arming sequence
esc.SetPulseWidth(1000);
osDelay(2000);  // Wait for ESC to arm

// Throttle control
esc.SetPulseWidth(1100);  // 10% throttle
esc.SetPulseWidth(1500);  // 50% throttle
esc.SetPulseWidth(2000);  // 100% throttle
```

---

## GPIO Configuration (CubeMX Settings)

For each PWM pin:
1. **Mode**: `TIMx_CHy` (Alternate Function)
2. **GPIO Pull-up/Pull-down**: No pull-up/pull-down
3. **GPIO Speed**: Very High (for 240MHz operation)
4. **Alternate Function**: See table above

---

## Pin Mapping Summary

```
PWM Outputs on DM-MC02 Board:
═══════════════════════════════════════════════════════════════
TIM1_CH1  → PE9   (240 MHz, Prescaler=24, Period=10000)
TIM1_CH3  → PE13  (240 MHz, Prescaler=24, Period=10000)
─────────────────────────────────────────────────────────────
TIM2_CH1  → PA0   (240 MHz, Prescaler=24, Period=10000)
TIM2_CH3  → PA2   (240 MHz, Prescaler=24, Period=10000)
─────────────────────────────────────────────────────────────
TIM3_CH4  → PB1   (240 MHz, Prescaler=23, Period=9999)
─────────────────────────────────────────────────────────────
TIM12_CH2 → PB15  (240 MHz, Prescaler=23, Period=65535)
═══════════════════════════════════════════════════════════════
Total: 6 PWM channels available
```

---

## Default Timer Configurations (from CubeMX)

| Timer | Prescaler | Period | Effective Clock | Default PWM Freq |
|-------|-----------|--------|-----------------|------------------|
| TIM1 | 24 | 10000 | 9.6 MHz | ~960 Hz |
| TIM2 | 24 | 10000 | 9.6 MHz | ~960 Hz |
| TIM3 | 23 | 9999 | 10 MHz | ~1 kHz |
| TIM12 | 23 | 65535 | 10 MHz | ~153 Hz |

---

## STM32H7 Specific Notes

### Timer Clock Architecture
```
SYSCLK (480 MHz)
    │
    ├─→ D2PPRE1 (/2) → APB1 (120 MHz) → Timer x2 → TIM2,3,4,5,6,7,12,13,14 = 240 MHz
    │
    └─→ D2PPRE2 (/2) → APB2 (120 MHz) → Timer x2 → TIM1,8,15,16,17 = 240 MHz
```

### High-Resolution PWM
With 240 MHz timer clock, you can achieve very fine PWM resolution:
- At 50 Hz: 240MHz / 50 = 4,800,000 counts resolution
- At 1 kHz: 240MHz / 1000 = 240,000 counts resolution
- At 100 kHz: 240MHz / 100000 = 2,400 counts resolution

### DMA Considerations
- PWM with DMA can be used for protocols like WS2812 LED strips
- Ensure DMA buffers are in AXI SRAM (0x24000000) or D2 SRAM (0x30000000)
- DTCM RAM (0x20000000) is NOT accessible by DMA1/DMA2

---

## Common Applications for MC02

| Application | Recommended Timer | Configuration |
|------------|------------------|---------------|
| RC Servo | TIM1/TIM2 | 50Hz, 1000-2000µs |
| ESC (Brushless) | TIM1/TIM2 | 50-400Hz, 1000-2000µs |
| LED Dimming | TIM3 | 1-10 kHz |
| Motor H-Bridge | TIM1 | 10-50 kHz |
| Buzzer | TIM3/TIM12 | 100-5000 Hz |
| WS2812 LED | TIM + DMA | 800 kHz |

---

## Hardware Pin Reference

### Connector Pinout (Check board schematic)
| Function | Pin | Timer |
|----------|-----|-------|
| PWM1 | PE9 | TIM1_CH1 |
| PWM2 | PE13 | TIM1_CH3 |
| PWM3 | PA0 | TIM2_CH1 |
| PWM4 | PA2 | TIM2_CH3 |
| PWM5 | PB1 | TIM3_CH4 |
| PWM6 | PB15 | TIM12_CH2 |

---

## Important Notes

1. **STM32H7 runs at 240 MHz timer clock** - much faster than F4 series
2. **TIM2** is a 32-bit timer - useful for very long periods or high resolution
3. **TIM1** is an advanced timer with break functionality for motor control
4. **TIM12** has limited features (16-bit, 2 channels only)
5. **Check CubeMX configuration** - prescalers may already be set
6. **For servo applications**, the bsp::PWM class will reconfigure the timer
7. Some pins may have multiple functions - verify no conflicts

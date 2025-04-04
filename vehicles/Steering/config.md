# Steering Configurations
## ID Definition
### Gimbal Type C Board
|               | CAN | TX ID  | RX ID  |
| ------------- | --- | ------ | ------ |
| pitch_motor   | 1   | 0x01   | 0x02   |
| yaw_motor     | 1   | 0x1ff  | 0x206  |
| sl_motor      | 1   | 0x200  | 0x201  |
| sr_motor      | 1   | 0x200  | 0x202  |
| ld_motor      | 1   | 0x200  | 0x203  |
| can_bridge    | 2   | 0x20A  | 0x20B  |

### Chassis Type C Board
|               | CAN | TX ID  | RX ID  |
| ------------- | --- | ------ | ------ |
| trigger_motor        | 1   | 0x200  | 0x201  |
| motor2        | 1   | 0x200  | 0x202  |
| motor3        | 1   | 0x200  | 0x203  |
| motor4        | 1   | 0x200  | 0x204  |
| motor5        | 2   | 0x1ff  | 0x205  |
| motor6        | 2   | 0x1ff  | 0x206  |
| motor7        | 2   | 0x1ff  | 0x207  |
| motor8        | 2   | 0x1ff  | 0x208  |
| can_bridge    | 2   | 0x20B  | 0x20A  |

## RC Switch Definition
|                             | Function               |
| ----------------------------|------------------------|
| swl-down                    | Toggle Kill            |
| swl-up                      | Toggle spin mode       |
| swr-down (pre-calibration)  | Calibrate              |
| swr-down (post-calibration) | Do nothing             |
| swr-mid                     | Turn on flywheels      |
| swr-up                      | Slow Continue Shooting |

## Key Definition
|          | Function         |
| -------- | ---------------- |
| WASD     | Directions       |
| B        | Initialization   |
| SHIFT    | Toggle Spin      |
| R        | Recalibrate steer motors  |
| G        | Manual anti-jam  |

## Wheel Definition
|      | Function               |
|------|------------------------|
| Up   | Triple Shooting        |
| Down | Fast Continue Shooting |

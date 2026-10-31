# RoboMaster 2027 University Series — Referee System Communication Protocol (V1.0.0) — Handoff Summary

> Source: *RoboMaster 2027 机甲大师高校系列赛通信协议 V1.0.0（20261029）*, released 2026-10-29 (first release), DJI, 84 pages.
> Applies to **RMUC** (University Championship) and **RMUL** (University League). Items shared by both apply to both; items marked for one competition do **not** apply to the other.

---

## 0. TL;DR for the next engineer

- **Three serial data links** + **one network (MQTT/Protobuf) interface**:
  1. **Regular link** – Power Management Module `User` UART, **115200 baud**. Server/main-controller ↔ robot. Most game data (`0x00xx`, `0x01xx`, `0x02xx`, `0x03xx`).
  2. **Video-transmission (VTM) link** – VTM transmitter UART, **921600 baud**. Robot ↔ *custom terminal* via player client (`0x0310`, `0x0311`).
  3. **Radar wireless link** – radar receives RF from the referee "signal source" (`0x0A01`–`0x0A06`, enemy intel).
  4. **Custom terminal protocol** – Protobuf v3 over **MQTT**, server `192.168.12.1:3333`, terminal `192.168.12.2`; video stream on **UDP 3334** (HEVC 1080p60).
- UART settings: **8N1, no flow control, no parity**.
- Frame: `SOF(0xA5) | data_length(u16) | seq(u8) | CRC8 | cmd_id(u16) | data[n] | CRC16`. Header = 5 bytes; overhead = 9 bytes. Little-endian, packed structs.
- Typical latency **~130 ms, <1 % loss**; poor network **~200 ms, ~3 % loss**. Data is "for reference only".
- **2027 roster change:** robot slot **1 is now the "Heavy" robot (重装机器人**, struct names use `heavyengineer`) — it assembles (tech core) and can install a **modular 42 mm launcher**. **ID 2 is reserved** (former Engineer). Infantry are **3/4** in most tables (ID list still mentions 3/4/5). Sentry = 7.

---

## 1. Serial Protocol

### 1.1 Frame format

| Field | Offset | Size | Notes |
|---|---|---|---|
| SOF | 0 | 1 | Fixed `0xA5` |
| data_length | 1 | 2 | Length of `data` only |
| seq | 3 | 1 | Packet sequence number |
| CRC8 | 4 | 1 | CRC8 of header (bytes 0–3) |
| cmd_id | 5 | 2 | Command ID |
| data | 7 | n | Payload |
| frame_tail | 7+n | 2 | CRC16 over whole frame |

CRC details (Appendix 1): CRC8 poly `x^8+x^5+x^4+1`, init `0xFF`, table-driven; CRC16 init `0xFFFF`, table-driven (CRC-16/MCRF4XX-style reflected table starting `0x0000, 0x1189…`), appended **low byte first**. Reference C code (`Get/Verify/Append_CRC8_Check_Sum`, `Get/Verify/Append_CRC16_Check_Sum`) is in the PDF — copy the tables from there verbatim.

### 1.2 Command ID overview

| cmd_id | Len | Content | Direction | Rate | Link |
|---|---|---|---|---|---|
| 0x0001 | 11 | Game status | Server → all robots | 1 Hz | Regular |
| 0x0002 | 1 | Game result | Server → all robots | On game end | Regular |
| 0x0003 | 24 | Robot/structure HP | Server → all robots | 3 Hz | Regular |
| 0x0101 | 8 | Field events | Server → own team | 1 Hz | Regular |
| 0x0104 | 3 | Referee warning | Server → penalized team | On penalty, else 1 Hz | Regular |
| 0x0105 | 3 | Dart launch info | Server → own team | 1 Hz | Regular |
| 0x0201 | 17 | Robot performance status | Main ctrl → robot | 10 Hz | Regular |
| 0x0202 | 14 | Buffer energy & barrel heat | Main ctrl → robot | 10 Hz | Regular |
| 0x0203 | 12 | Robot position | Main ctrl → robot | 1 Hz | Regular |
| 0x0204 | 7 | Robot buffs | Server → robot | 3 Hz | Regular |
| 0x0206 | 7 | Damage event | Main ctrl → robot | On damage | Regular |
| 0x0207 | 7 | Shooting data | Main ctrl → robot | On each shot | Regular |
| 0x0208 | 8 | Projectile allowance & gold | Server → Heavy/Infantry/Sentry | 10 Hz | Regular |
| 0x0209 | 6 | RFID status | Server → robots with RFID | 3 Hz | Regular |
| 0x020A | 6 | Dart client command | Server → own dart | 3 Hz | Regular |
| 0x020B | 40 | Ground robot positions | Server → own sentry | 1 Hz | Regular |
| 0x020C | 2 | Radar mark progress | Server → own radar | 1 Hz | Regular |
| 0x020D | 14 | Sentry decision-sync info | Server → own sentry | 1 Hz | Regular |
| 0x020E | 8 | Radar decision-sync info | Server → own radar | 1 Hz | Regular |
| 0x0301 | 118 | Robot interaction data (sub-IDs) | Sender-defined | ≤30 Hz | Regular |
| 0x0303 | 12 | Minimap click command (client → robot) | Client → server → robot(s) | On trigger | Regular |
| 0x0305 | 48 | Radar positions to minimap | Radar → server → all own clients | ≤5 Hz | Regular |
| 0x0307 | 105 | Path data to minimap | Sentry/semi-auto robot → its operator client | ≤1 Hz | Regular |
| 0x0308 | 34 | Custom text message to client | Own robot → own client | ≤3 Hz | Regular |
| 0x0310 | 300 | Robot → custom terminal data | Robot → VTM → client's custom terminal | ≤50 Hz | VTM |
| 0x0311 | 60 | Custom terminal → robot command | Custom terminal → VTM → robot | ≤75 Hz | VTM |
| 0x0A01 | 24 | Enemy positions | Signal source → radar | 10 Hz | Radar wireless |
| 0x0A02 | 12 | Enemy HP | Signal source → radar | 10 Hz | Radar wireless |
| 0x0A03 | 10 | Enemy projectile allowance | Signal source → radar | 10 Hz | Radar wireless |
| 0x0A04 | 12 | Enemy macro status | Signal source → radar | 10 Hz | Radar wireless |
| 0x0A05 | 47 | Enemy buffs/states | Signal source → radar | 10 Hz | Radar wireless |
| 0x0A06 | 6 | Enemy jamming-wave key | Signal source → radar | 10 Hz | Radar wireless |

### 1.3 Regular-link payloads

#### 0x0001 `game_status_t` (11 B)
```c
typedef __packed struct {
  uint8_t  game_type     : 4; // 1 RMUC, 2 Univ. single-item, 3 ICRA RMUA, 4 RMUL 3v3, 5 RMUL infantry
  uint8_t  game_progress : 4; // 0 not started, 1 prep, 2 15s self-check, 3 5s countdown, 4 in game, 5 settling
  uint16_t stage_remain_time; // s
  uint64_t SyncTimeStamp;     // UNIX time, valid after robot syncs with referee NTP server
} game_status_t;
```

#### 0x0002 `game_result_t` (1 B) — `winner`: 0 draw, 1 red, 2 blue.

#### 0x0003 `game_robot_HP_t` (24 B) — HP is 0 if not on field / ejected
```c
uint16_t ally_1_robot_HP; uint16_t reserved; uint16_t ally_3_robot_HP; uint16_t ally_4_robot_HP;
int16_t  damage_difference;   // own total damage minus enemy total damage
uint16_t ally_7_robot_HP; uint16_t outpost_R_HP; uint16_t ally_base_HP;
uint16_t outpost_B_HP; uint16_t enemy_base_HP;
uint16_t ally_control_degree; uint16_t enemy_control_degree; // cumulative "control degree" (控制度)
```
Note: outposts are labeled **R/B (by starting side)**, not ally/enemy — outposts/positions can change control in 2027.

#### 0x0101 `event_data_t` (8 B: `uint32 event_data; uint32 event_data_2`)
`event_data`:
| Bits | Meaning |
|---|---|
| 0 | Own supply zone active (RMUC & RMUL) |
| 1–2 | Own small energy mechanism (rune): 0 inactive, 1 activated, 2 activating |
| 3–4 | Own large energy mechanism: same encoding |
| 5–7 | R fortress buff point: 0 none, 1 own, 2 enemy, 3 both; **4 = own team doesn't control that position** |
| 8–10 | B fortress buff point (same) |
| 11–13 | R base buff point (same) |
| 14–16 | B base buff point (same) |
| 17 | Own high-ground (制高点) buff point: 0/1 |
| 18–19 | R central highland buff point: 0/1 own/2 enemy; 3 = not controlled |
| 20–21 | B central highland buff point (same) |
| 22–24 | R outpost buff point (0–3; 4 = not controlled) |
| 25–27 | B outpost buff point (same) |
| 28–29 | R outpost position control: 0 full, 1 temporary, 2 not controlled |
| 30–31 | B outpost position control (same) |

`event_data_2`:
| Bits | Meaning |
|---|---|
| 0–1 | R base position control (0 full, 1 temp, 2 none) |
| 2–3 | B base position control |
| 4–12 | Time (s, 0–420) of enemy dart's last hit on own outpost/base |
| 13–15 | That hit's target: 0 default, 1 R outpost, 2 base fixed, 3 base random-fixed, 4 base random-moving, 5 base end-moving, 6 B outpost |
| 16–17 / 18–19 | R highland control zone activation (0–3) / control (0 neutral, 1 own, 2 enemy) — **RMUL only** |
| 20–21 / 22–23 | B highland control zone (same) — RMUL only |
| 24–25 / 26–27 | Central control zone (same) — RMUL only |

#### 0x0104 `referee_warning_t` (3 B)
`level` (1 both-yellow, 2 yellow, 3 red, 4 forfeit), `offending_robot_id` (0 for forfeit/both-yellow), `count` (violations at that level for that robot).

#### 0x0105 `dart_info_t` (3 B)
`uint8 dart_remaining_time` (s); `uint16 dart_info`: bits 0–2 last own-dart hit target (codes as above, 1 R outpost … 6 B outpost); bits 3–5 cumulative hits on that target (≤4); bits 6–8 currently selected target (0 default/outpost, 1 base fixed, 2 random-fixed, 3 random-moving, 4 end-moving); 9–15 reserved.

#### 0x0201 `robot_status_t` (17 B)
```c
uint8_t robot_id; uint8_t robot_level; uint16_t current_HP; uint16_t maximum_HP;
uint16_t shooter_barrel_cooling_value;  // heat cooling per second
uint16_t shooter_barrel_heat_limit; uint16_t chassis_power_limit;
float    bullet_speed_limit;            // NEW-ish: launch speed limit as float
uint8_t  power_management_gimbal_output:1, power_management_chassis_output:1, power_management_shooter_output:1; // 1 = 24V on
```

#### 0x0202 `power_heat_data_t` (14 B)
Bytes 0–7 **reserved** (former chassis voltage/current/power). `uint16 buffer_energy` (J), `uint16 shooter_17mm_barrel_heat`, `uint16 shooter_42mm_barrel_heat`.
⚠ The PDF struct reuses the name `reserved` 3× — rename locally (`reserved0/1/2`).

#### 0x0203 `robot_pos_t` (12 B) — `float x, y` (m), `float angle` (deg, speed-module heading, north = 0).

#### 0x0204 `buff_t` (7 B)
`uint8 recovery_buff` (% max HP/s), `uint16 cooling_buff` (absolute heat/s), `uint8 defence_buff` (%), `uint8 vulnerability_buff` (% negative defence), `uint16 attack_buff` (%).

#### 0x0206 `hurt_data_t` (7 B) — **new hit position fields**
```c
uint8_t armor_id:4;            // armor/speed-module ID for hit/collision/offline, else 0
uint8_t HP_deduction_reason:4; // 0 projectile hit, 1 armor offline, 2 super-cap module offline, 5 armor collision
uint16_t hit_sequence;         // increments per hit
int16_t hit_position_x, hit_position_y; // mm, origin = armor detection face bottom-left, x right, y up
```
Locally judged & sent immediately; the server's final ruling prevails.

#### 0x0207 `shoot_data_t` (7 B) — `bullet_type` (1 = 17 mm, 2 = 42 mm), `shooter_number` (1 = 17 mm, 2 reserved, 3 = 42 mm), `launching_frequency` (Hz), `float initial_speed` (m/s).

#### 0x0208 `projectile_allowance_t` (8 B) — `allowance_17mm`, `allowance_42mm`, `remaining_gold_coin`, `allowance_fortress` (reserve allowance provided by the fortress buff point, independent of actual activation).

#### 0x0209 `rfid_status_t` (6 B: `uint32 rfid_status; uint16 rfid_status_2`) — bit = card detected; **only valid in-match**.
- `rfid_status`: 0 own supply zone, 1 own assembly zone, 2 R base, 3 B base, 4 R fortress, 5 B fortress, 6 R central highland, 7 B central highland, 8 R outpost, 9 B outpost, 10 own high-ground, 11–13 R/B highland & central control zones (RMUL only), 14–21 terrain-crossing points (own/enemy, central-highland lower/upper, road lower/upper), 22–30 tunnel terrain-crossing points on own side (various positions).
- `rfid_status_2` bits 0–8: enemy-side tunnel terrain-crossing points.
- (The doc lists it as a 4-byte + 2-byte field; the 2nd field bits 0–8 are used.)

#### 0x020A `dart_client_cmd_t` (6 B) — `dart_launch_opening_status` (1 closed, 2 opening/closing, **0 open**), reserved, `target_change_time` (remaining match time at last target switch), `latest_launch_cmd_time` (remaining time at last launch confirm).

#### 0x020B `ground_robot_position_t` (40 B, to sentry) — floats (m): heavy x/y, reserved×2, infantry 3 x/y, infantry 4 x/y, reserved×2.
Field coordinate frame: **origin at field-boundary corner near red supply station; +X along long side toward blue; +Y along short side toward red's landing pad (停机坪)**.

#### 0x020C `radar_mark_data_t` (2 B, to radar) — `uint16 mark_progress`:
- Enemy bits (1 = marked progress ≥ 100 ⇒ vulnerable): 0 heavy, 2 inf3, 3 inf4, 5 sentry.
- Ally bits (1 = being marked ≥ 50, "special identification"): 6 heavy, 8 inf3, 9 inf4, 11 sentry. Others reserved.

#### 0x020D `sentry_info_t` (14 B: `uint32 sentry_info; uint16 sentry_info_2; uint64 sentry_info_3`)
- `sentry_info`: 0–10 allowance redeemed (excl. remote); 11–14 remote ammo redemptions count; 15–18 remote HP redemptions count; 19 can confirm free respawn; 20 can buy instant respawn; 21–30 gold cost for instant respawn; 31 reserved.
- `sentry_info_2`: 0 out-of-combat; 1–11 team 17 mm allowance still redeemable; 12–13 posture (1 attack, 2 defense, 3 move); 14 energy mechanism can enter activating state; 15 currently in **enhanced posture**.
- `sentry_info_3` (seconds, 8 bits each): 0–7 attack posture time before weakening; 8–15 defense; 16–23 move; 32–39 enhanced attack remaining; 40–47 enhanced defense; 48–55 enhanced move; rest reserved.

#### 0x020E `radar_info_t` (8 B)
- `radar_info`: 0–1 double-vulnerability chances (≤2); 2 enemy currently double-vulnerable; 3–4 own encryption level (= enemy jamming difficulty, starts 1, max 3); 5 key can currently be modified.
- `radar_info_2`: 0 armor-fortify zone active; 1–3 remaining fortify chances.
- `uint16 fortify_center_x, fortify_center_y, fortify_radius` (mm) — circular **armor fortification zone** (new 2027 mechanic).

### 1.4 0x0301 Robot interaction data

- Header: `uint16 data_cmd_id; uint16 sender_id; uint16 receiver_id; uint8 user_data[x]` — **x ≤ 112** (127 max packet − 9 frame − 6 header).
- Sender ID must match own ID; **own-team only**; receiver must be rule-permitted. To a player client: robots may only send to **their own** client; **radar may send to any own client**.
- Whole `0x0301` uplink ≤ **30 Hz** across all sub-IDs.
- **Receive bandwidth budget per 1000 ms:** base `x` = **3720 B** (Heavy, Infantry, Aerial, Dart) / **5120 B** (Radar, Sentry). Actual = `round(x · λ)`, **λ = 1 − Δ/20000**, Δ = own base HP lost.

| Sub-ID | Len | Function |
|---|---|---|
| 0x0200–0x02FF | ≤112 | Robot-to-robot |
| 0x0100 | 2 | Client: delete layer (`delete_type` 0 nop/1 layer/2 all, `layer` 0–9) |
| 0x0101 | 15 | Draw 1 figure |
| 0x0102 | 30 | Draw 2 figures |
| 0x0103 | 75 | Draw 5 figures |
| 0x0104 | 105 | Draw 7 figures |
| 0x0110 | 45 | Draw text (15 B figure config + 30 B chars) |
| 0x0120 | 4 | Sentry autonomous decision cmd (receiver = server `0x8080`) |
| 0x0121 | 13 | Radar autonomous decision cmd (receiver = server `0x8080`) |

#### UI figure `interaction_figure_t` (15 B)
```c
uint8_t  figure_name[3];                  // index for modify/delete
uint32_t operate_type:3;  // 0 nop, 1 add, 2 modify, 3 delete
uint32_t figure_type:3;   // 0 line, 1 rect, 2 circle, 3 ellipse, 4 arc, 5 float, 6 int, 7 char
uint32_t layer:4;         // 0-9
uint32_t color:4;         // 0 team red/blue, 1 yellow, 2 green, 3 orange, 4 magenta, 5 pink, 6 cyan, 7 black, 8 white
uint32_t details_a:9, details_b:9;
uint32_t width:10, start_x:11, start_y:11;
uint32_t details_c:10, details_d:11, details_e:11;
```
| Type | a | b | c | d | e |
|---|---|---|---|---|---|
| Line | – | – | – | end x | end y |
| Rect | – | – | – | opposite x | opposite y |
| Circle | – | – | radius | – | – |
| Ellipse | – | – | – | x semi-axis | y semi-axis |
| Arc | start angle | end angle | – | x semi-axis | y semi-axis |
| Float | font size | – | value ×1000 across c/d/e (32-bit) |
| Int | font size | – | int32 across c/d/e |
| Char | font size | length | – | – | – |

Screen: (0,0) bottom-left, (1920,1080) top-right. Angles: 0° = 12 o'clock, clockwise. Recommended font size : line width = 10 : 1.
(PDF typo: `operate_tpye`/`figure_tpye`; 0x0110 struct refers to `graphic_data_struct_t` = same as `interaction_figure_t`.)

#### 0x0120 `sentry_cmd_t` (uint32)
| Bits | Meaning |
|---|---|
| 0 | Confirm respawn (0 = don't respawn even if bar done) |
| 1 | Confirm buying instant respawn with gold |
| 2–12 | Target cumulative allowance to redeem at heal point — **must monotonically increase** (0→X costs X gold for X rounds) |
| 13–16 | Remote ammo redemption request count — monotonic, **+1 per step** |
| 17–20 | Remote HP redemption request count — monotonic, +1 per step |
| 21–23 | Posture: 1 attack, 2 defense, 3 move (default), **4/5/6 enhanced attack/defense/move** |
| 24 | Confirm putting energy mechanism into activating state |
| 25–31 | Reserved |

Server processes bits **low → high**, stopping at the first that can't be executed (e.g., not enough gold for instant respawn ⇒ later fields ignored until next packet).

#### 0x0121 `radar_cmd_t` (13 B)
| Offset | Size | Field |
|---|---|---|
| 0 | 1 | `radar_cmd` — double-vulnerability trigger counter (monotonic +1). If one is active, the next queues after it ends. |
| 1 | 1 | `password_cmd` — 1 = update own encryption key; 2 = submit cracked enemy key for verification |
| 2–7 | 6 | `password_1..6` — ASCII letters/digits |
| 8 | 1 | `armor_fortify_cmd` — fortify chance usage counter (monotonic +1) |
| 9 | 2 | `fortify_center_x` (mm) |
| 11 | 2 | `fortify_center_y` (mm) |

Key rules: key can only be changed **at match start and each time enemy cracks it (encryption level rises)**; otherwise ignored (key auto-resets to random anyway). For verification (cmd 2), re-submits within **10 s** are ignored.

### 1.5 Minimap interaction

#### 0x0303 `map_command_t` (12 B) — client → robot
`float target_position_x, y` (m; 0 if a robot ID is sent), `uint8 cmd_keyboard` (key pressed, 0 none), `uint8 target_robot_id` (0 if coords), `uint16 cmd_source` (sender ID).
- Aerial operator (云台手): ≥ **0.5 s** between sends. Mode 1: click own robot avatar → (optional key / enemy avatar) → click map ⇒ sends to that robot. Mode 2: skip avatar ⇒ sends to **all** own robots.
- **Semi-auto robot's operator**: ≥ **3 s** between sends; goes to that operator's robot. A semi-auto robot accepts both sources (distinguish via `cmd_source`).
- **Delivery quirk:** each trigger ⇒ 1 + 4 repeats at 100 ms (5 total), then the latest packet is resent at **1 Hz** until the next trigger. **Handle duplicates.**

#### 0x0305 `map_robot_data_t` (48 B) — radar → all own clients' minimap, ≤5 Hz
uint16 cm coordinates: enemy heavy x/y, reserved×2, enemy inf3, inf4, reserved×2, enemy sentry; then the same order for **ally** robots. Out-of-bounds → drawn at edge; x = y = 0 ⇒ not shown.

#### 0x0307 `map_data_t` (105 B) — sentry/semi-auto robot → its operator client, ≤1 Hz
`uint8 intention` (1 attack at target, 2 defend at target, 3 move to), `uint16 start_position_x/y` (**dm**), `int8 delta_x[49]`, `int8 delta_y[49]` (dm, each relative to previous point), `uint16 sender_id`. Minimap frame: origin bottom-left, +X right, +Y up, scaled to field.

#### 0x0308 `custom_info_t` (34 B) — robot → any own client, ≤3 Hz
`sender_id`, `receiver_id`, `uint8 user_data[30]` — **UTF-16** text (Chinese supported; mind endianness).

### 1.6 VTM link (custom terminal ↔ robot)
- **0x0310** (300 B): robot → custom terminal, ≤50 Hz.
- **0x0311** (60 B): custom terminal → robot, ≤75 Hz.
- **No retransmission.**

### 1.7 Radar wireless link (enemy intel)
- **0x0A01** (24 B): enemy positions, uint16 cm — heavy x/y, res×2, inf3 x/y, inf4 x/y, res×2, sentry x/y.
- **0x0A02** (12 B): enemy HP — heavy, res, inf3, inf4, res, sentry.
- **0x0A03** (10 B): enemy allowance — heavy, inf3 (incl. fortress reserve), inf4, res, sentry.
- **0x0A04** (12 B): `uint16 enemy remaining gold`, `uint16 enemy total gold`, `uint32` enemy buff-point states (supply, R/B fortress, R/B base, high-ground, R/B central highland, R/B outpost; encoding mirrors 0x0101 but from enemy perspective — 1 = enemy, 2 = own), `uint32` terrain-crossing/tunnel detections of enemy robots (bits 0–21) + R/B outpost & base position control from enemy view (bits 22–29).
- **0x0A05** (47 B): per enemy robot (heavy @0, inf3 @14, inf4 @21, sentry @28; 7 B blocks, slot @7 reserved): recovery %, cooling (abs), defence %, vulnerability %, attack %. Then @35 enemy sentry posture (1–6 incl. enhanced), @36–40 main state of heavy/res/inf3/inf4/sentry (0 alive, 1 dead, 2 invincible not weak, 3 invincible & weak), @41/43/45 enemy custom-armor fortify circle center x/y and radius (mm; radius 0 = no intel).
- **0x0A06** (6 B): enemy jamming-wave key — 6 ASCII letters/digits.

---

## 2. Custom Terminal Protocol (MQTT + Protobuf)

- **Protobuf v3 (≥3.15)**, raw binary payload over **MQTT**; **topic = message name**.
- Server **192.168.12.1:3333**, custom terminal **192.168.12.2**. MQTT **clientID = connected robot's ID**.
- Max QoS listed as **1** for all topics (note: sample C# code uses `ExactlyOnce` = QoS 2 and default IP `127.0.0.1` — adjust).
- **Video stream:** UDP **port 3334**, **1920×1080 @ 60 fps, HEVC/H.265**, no retransmission. Each UDP packet ≤ 1400 B: 8-byte **big-endian** prefix = `frame_id(u16, incrementing) | slice_idx(u16) | frame_total_bytes(u32)` + ≤1392 B payload.
- Keyboard/mouse control source (official client **or** custom terminal) must be chosen before match; only one at a time.

### 2.1 Topic overview

| Topic | Dir | Rate | Purpose |
|---|---|---|---|
| KeyboardMouseControl | Terminal → robot (VTM) | 75 Hz | KB/mouse input |
| CustomControl | Terminal → robot (VTM, as 0x0311) | 75 Hz | ≤60 B custom data |
| GameStatus | Server → terminal | 5 Hz | Match state |
| GlobalUnitStatus | Server → terminal | 1 Hz | Win-condition state |
| GlobalLogisticsStatus | Server → terminal | 1 Hz | Economy/tech |
| GlobalSpecialMechanism | Server → terminal | 1 Hz | Active timed mechanisms |
| Event | Server → terminal | Triggered | Global events |
| RobotInjuryStat | Server → terminal | 1 Hz | Damage stats per life |
| RobotRespawnStatus | Server → terminal | 1 Hz | Respawn |
| RobotStaticStatus | Server → terminal | 1 Hz | Config/attributes |
| RobotDynamicStatus | Server → terminal | 10 Hz | Real-time data |
| RobotModuleStatus | Server → terminal | 1 Hz | Module online states |
| RobotPosition | Server → terminal | 1 Hz | Pose |
| Buff | Server → terminal | On gain, then 1 Hz until lost | Buffs |
| PenaltyInfo | Server → terminal | Triggered | Penalties |
| RobotPathPlanInfo | Server → terminal | Triggered | Sentry path |
| MapClickInfo | Server → terminal | Triggered (limits as 0x0303) | Map marks |
| MapClickCmd | Terminal → server | Triggered (limits as 0x0303) | Send map mark |
| RadarInfoToClient | Server → terminal | Triggered | Radar positions |
| CustomByteBlock | Robot → terminal (VTM, = 0x0310) | 50 Hz | ≤2.4 kbit custom stream |
| AssemblyCommand | Terminal → server | ≤10 Hz | Heavy assembly |
| TechCoreMotionStateSync | Server → terminal | 1 Hz | Tech core state |
| RobotPerformanceSelectionCommand | Terminal → server | ≤10 Hz | Choose performance system / sentry control mode |
| RobotPerformanceSelectionSync | Server → terminal | 1 Hz | Current selection |
| CommonCommand | Terminal → server | ≤10 Hz | Buy ammo / respawn / remote heal |
| RuneActivateCommand | Terminal → server | ≤10 Hz | Start energy mechanism |
| RuneStatusSync | Server → terminal | 1 Hz | Energy mechanism state |
| SentryStatusSync | Server → terminal | 1 Hz | Sentry posture |
| DartCommand | Terminal → server | ≤10 Hz | Dart control |
| DartSelectTargetStatusSync | Server → terminal | 1 Hz | Dart target/gate |
| SentryCtrlCommand | Terminal → server | ≤10 Hz | Sentry command |
| SentryCtrlResult | Server → terminal | Triggered | Sentry command result |

### 2.2 Message definitions (condensed)

```proto
syntax = "proto3";

message KeyboardMouseControl {
  optional int32 mouse_x = 1;            // <0 left
  optional int32 mouse_y = 2;            // <0 down
  optional int32 mouse_z = 3;            // wheel, <0 back
  optional bool  left_button_down = 4;
  optional bool  right_button_down = 5;
  optional uint32 keyboard_value = 6;    // bits 0-15: W S A D Shift Ctrl Q E R F G Z X C V B
  optional bool  mid_button_down = 7;
}
message CustomControl { optional bytes data = 1; }           // ≤60 B → 0x0311

message GameStatus {
  optional uint32 current_round = 1; optional uint32 total_rounds = 2;
  optional uint32 red_score = 3; optional uint32 blue_score = 4;
  optional uint32 current_stage = 5;     // 0..5 same as 0x0001 game_progress
  optional int32 stage_countdown_sec = 6; optional int32 stage_elapsed_sec = 7;
  optional bool is_paused = 8;
  optional uint32 game_result = 9;       // 0 draw 1 red 2 blue; 255 unless stage==5
  optional uint32 end_reason = 10;       // 1 base destroyed, 2 control degree, 3 total damage,
                                         // 4 total remaining HP, 5 draw, 6 abnormal, 7 head referee; 255 unless stage==5
}

message GlobalUnitStatus {
  optional uint32 base_health = 1;
  optional uint32 base_status = 2;       // 0 has 100% def buff, 1 no buff armor closed, 2 no buff armor open
  optional uint32 base_shield = 3;       // own base position control: 1 own full, 2 enemy temporary
  optional uint32 outpost_R_health = 4;
  optional uint32 outpost_R_status = 5;  // 1 own full, 2 own temp, 3 enemy temp, 4 enemy full
  optional uint32 enemy_base_health = 6; optional uint32 enemy_base_status = 7;
  optional uint32 enemy_base_shield = 8; // 1 enemy full, 2 own temporary
  optional uint32 outpost_B_health = 9; optional uint32 outpost_B_status = 10;
  repeated uint32 robot_health = 11;     // own 1,3,4,7 then enemy 1,3,4,7
  repeated int32  robot_bullets = 12;    // own remaining allowance
  optional uint32 total_damage_ally = 13; optional uint32 total_damage_enemy = 14;
  optional uint32 total_CP_ally = 15; optional uint32 total_CP_enemy = 16;  // control degree
}

message GlobalLogisticsStatus {
  optional uint32 remaining_economy = 1; optional uint64 total_economy_obtained = 2;
  optional uint32 tech_level = 3;        // highest assembly difficulty completed by Heavy
  optional uint32 encryption_level = 4;  // radar info-wave mechanic
}

message GlobalSpecialMechanism {
  repeated uint32 mechanism_id = 1;      // 1 own fortress activated by enemy, 2 enemy fortress by us,
                                         // 3 R outpost pos., 4 B outpost pos., 5 R base pos., 6 B base pos.
  repeated int32 mechanism_time_sec = 2; // sign: <0 enemy progressing, >0 own progressing, 0 no change
}

message Event { optional int32 event_id = 1; optional string param = 2; }
```
`Event.event_id`: 1 kill (`"victim,killer"`), 2 outpost destroyed (11 / 111), 3 large rune result (`"arms,avg_rings"`), 4 rune activated (1 small/2 large), 5/6 R/B outpost position control change (1 red temp, 2 red full, 3 blue temp, 4 blue full), 7 R base position (1 red full, 2 blue temp), 8 B base position (1 blue full, 2 red temp), 9 dart hit (`"side,target"`), 10 enemy dart gate open, 11 base under attack (5 s cooldown), 12 enemy outpost stopped rotating, 13 enemy base armor opened, 14 Heavy died during temporary activation → forced-exit buffer, 15 assembly result (0 success, 1 pulled out, 2 timeout, 3 left zone too long, 4 Heavy died, 5 level-4 collaboration deadline missed, 6 quit, 7 done but no energy unit at settlement, 8 buffer expired), 16 Heavy installed modular 42 mm launcher.

```proto
message RobotInjuryStat {               // per life
  optional uint32 total_damage = 1; optional uint32 collision_damage = 2;
  optional uint32 small_projectile_damage = 3; optional uint32 large_projectile_damage = 4;
  optional uint32 dart_splash_damage = 5; optional uint32 module_offline_damage = 6;
  optional uint32 offline_damage = 7; optional uint32 penalty_damage = 8;
  optional uint32 server_kill_damage = 9; optional uint32 killer_id = 10; // 0 = unknown
}
message RobotRespawnStatus {
  optional bool is_pending_respawn = 1; optional uint32 total_respawn_progress = 2;
  optional uint32 current_respawn_progress = 3; optional bool can_free_respawn = 4;
  optional uint32 gold_cost_for_respawn = 5; optional bool can_pay_for_respawn = 6;
}
message RobotStaticStatus {
  optional uint32 connection_state = 1;  // 0 no, 1 yes
  optional uint32 field_state = 2;       // 0 on field, 1 not
  optional uint32 alive_state = 3;       // 0 unknown, 1 alive, 2 dead
  optional uint32 robot_id = 4; optional uint32 robot_type = 5;
  optional uint32 performance_system_shooter = 6; // 1 cooling-first, 2 burst-first
  optional uint32 performance_system_chassis = 7; // 1 HP-first, 2 power-first
  optional uint32 level = 8; optional uint32 max_health = 9; optional uint32 max_heat = 10;
  optional float heat_cooldown_rate = 11; optional uint32 max_power = 12;
  optional uint32 max_buffer_energy = 13; optional uint32 max_chassis_energy = 14;
}
message RobotDynamicStatus {
  optional uint32 current_health = 1; optional float current_heat = 2;
  optional float last_projectile_fire_rate = 3; optional uint32 reserved = 4;
  optional uint32 current_buffer_energy = 5; optional uint32 current_experience = 6;
  optional uint32 experience_for_upgrade = 7; optional uint32 total_projectiles_fired = 8;
  optional uint32 remaining_ammo = 9; optional bool is_out_of_combat = 10;
  optional uint32 out_of_combat_countdown = 11;
  optional bool can_remote_heal = 12; optional bool can_remote_ammo = 13;
}
message RobotModuleStatus {             // each: 0 offline, 1 online, 2 treated offline (bad install)
  optional uint32 power_manager = 1; optional uint32 rfid = 2; optional uint32 light_strip = 3;
  optional uint32 small_shooter = 4; optional uint32 big_shooter = 5; optional uint32 uwb = 6;
  optional uint32 armor = 7; optional uint32 video_transmission = 8; optional uint32 capacitor = 9;
  optional uint32 main_controller = 10;
  optional uint32 modular_42mm = 11;     // Heavy only: has permission to install modular 42mm launcher
}
message RobotPosition {
  optional float x = 1; optional float y = 2; optional float z = 3;
  optional float yaw = 4;                // deg, north = 0
  optional uint32 robot_id = 5;          // owner of coords; 7/107 = sentry (commander client only)
}
message Buff {
  optional uint32 robot_id = 1;
  optional uint32 buff_type = 2;         // 1 attack, 2 defence/vulnerability, 3 cooling, 4 chassis power,
                                         // 5 recovery, 6 redeemable allowance, 7 terrain-crossing (pre-stage)
  optional int32 buff_level = 3;         // % for 1,2,4,5; absolute heat/s for 3
  optional uint32 buff_max_time = 4; optional uint32 buff_left_time = 5;
}
message PenaltyInfo {
  optional uint32 penalty_type = 1;      // 1 yellow, 2 both yellow, 3 red, 4 over-power, 5 over-heat, 6 over-speed
  optional uint32 penalty_effect_sec = 2; optional uint32 total_penalty_num = 3;
}
message RobotPathPlanInfo {             // mirrors 0x0307
  optional uint32 intention = 1;         // 1 attack, 2 defend, 3 move
  optional uint32 start_pos_x = 2; optional uint32 start_pos_y = 3; // dm
  repeated int32 offset_x = 4 [packed = true]; // 49 × (-128..127)
  repeated int32 offset_y = 5 [packed = true];
  optional uint32 sender_id = 6;
}
message MapClickInfo /* and identical MapClickCmd */ {
  optional uint32 is_send_all = 1;       // 0 specified clients, 1 all except sentry, 2 incl. sentry
  optional bytes  robot_id = 2;          // fixed 7 bytes, e.g. red {1,7,0,...}, blue {101,107,0,...}
  optional uint32 mode = 3;              // 1 attack, 2 defend, 3 alert, 4 custom
  optional uint32 enemy_id = 4;
  optional uint32 ascii = 5;             // custom icon (mode 4): 'C'-'L','N','O','Q'-'Z'
  optional uint32 type = 6;              // 1 map point, 2 enemy robot
  optional float map_x = 7; optional float map_y = 8;
}
message RadarInfoToClient { repeated RadarSingleRobotInfo info = 1; } // order: enemy 1,3,4,7; own 1,3,4,7
message RadarSingleRobotInfo {
  optional uint32 target_pos_x = 1; optional uint32 target_pos_y = 2; // cm
  optional uint32 is_high_light = 3;     // 0 no, 1 special-ID, 2 special-ID but target's positioning module offline
}
message CustomByteBlock { optional bytes data = 1; }          // ≤2.4 kbit (=300 B, 0x0310)
message AssemblyCommand {
  optional uint32 operation = 1;         // 0 start exchange, 1 confirm assembly, 2 cancel
  optional uint32 difficulty = 2;
}
message TechCoreMotionStateSync {
  optional uint32 maximum_difficulty_level = 1;
  optional uint32 basic_state = 2;       // 1 initial, 2 moving, 3 reached pose
  optional uint32 putin_state = 3; optional uint32 move_state = 4; optional uint32 rotate_state = 5; // energy unit: 0/1
  optional uint32 enemy_core_status = 6; // (only when own lvl-4 available) 0 none, 1 enemy assembling non-lvl4, 2 lvl4
  optional uint32 remain_time_all = 7;   // own lvl-4 total remaining
  optional uint32 remain_time_step = 8;  // per-step interval remaining (lvl-4 or death buffer)
}
message RobotPerformanceSelectionCommand /* and ...Sync */ {
  optional uint32 shooter = 1; optional uint32 chassis = 2;
  optional uint32 sentry_control = 3;    // 0 auto, 1 semi-auto
}
message CommonCommand {
  optional uint32 cmd_type = 1;          // 1 buy 17mm (param multiple of 10), 2 buy 42mm, 3 confirm respawn,
                                         // 4 buy instant respawn, 5 remote ammo, 6 remote HP
  optional uint32 param = 2;
}
message RuneActivateCommand { optional uint32 activate = 1; } // 1 = start
message RuneStatusSync {
  optional uint32 rune_status = 1;       // 1 inactive, 2 activating, 3 activated
  optional uint32 activated_arms = 2;
  optional float average_rings = 3;      // only large rune while activating
}
message SentryStatusSync {
  optional uint32 posture_id = 1;        // 1 attack, 2 defense, 3 move
  optional bool is_weakened = 2; optional bool is_powered = 3;
}
message DartCommand {
  optional uint32 target_id = 1;         // 1 outpost, 2 base fixed, 3 random-fixed, 4 random-moving, 5 end-moving
  optional bool open = 2;                // gate
  optional bool launch_confirm = 3;
}
message DartSelectTargetStatusSync {
  optional uint32 target_id = 1;
  optional uint32 open = 2;              // 0 closed, 1 opening, 2 open  (NB: differs from 0x020A encoding)
}
message SentryCtrlCommand {
  optional uint32 command_id = 1;        // 0 invalid, 1 refill at heal point, 3 remote ammo, 4 remote heal,
                                         // 5 confirm respawn, 6 pay-respawn, 7/8/9 attack/defense/move posture,
                                         // 10/11/12 enhanced attack/defense/move
}
message SentryCtrlResult { optional uint32 command_id = 1; optional uint32 result_code = 2; } // 0 = success
```

---

## 3. ID Reference (Appendix 2)

| Entity | Red | Blue |
|---|---|---|
| Heavy (重装) | 1 | 101 |
| Reserved (old Engineer) | 2 | 102 |
| Infantry | 3/4/5 | 103/104/105 |
| Aerial | 6 | 106 |
| Sentry | 7 | 107 |
| Dart | 8 | 108 |
| Radar | 9 | 109 |
| Base | 10 | 110 |
| Outpost | 11 (R outpost) | 111 (B outpost) |

Player client IDs: red Heavy `0x0101`, reserved `0x0102`, red Infantry `0x0103–0x0105`, red Aerial `0x0106`; blue Heavy `0x0165`, reserved `0x0166`, blue Infantry `0x0167–0x0169`, blue Aerial `0x016A`. **Referee server `0x8080`** (target for sentry/radar decision commands).
"R/B" outposts/positions = the one on red/blue's side **at match start** (control can change hands).

---

## 4. New / notable 2027 mechanics reflected in the protocol

- **Heavy robot** (slot 1) replaces separate Hero/Engineer slots: assembly/tech-core (`AssemblyCommand`, `TechCoreMotionStateSync`, tech_level, difficulty up to level 4 with collaboration deadline), **modular 42 mm launcher** install permission/event.
- **Position control & control degree (控制度)**: outpost/base *positions* (阵地) can be temporarily/fully controlled by either side; cumulative control degree is a win criterion (end_reason 2).
- **Damage difference** (`damage_difference`, total damage end_reason 3).
- **Fortress reserve allowance**, team-level 17 mm redeemable pool.
- **Sentry**: 3 postures + **enhanced postures**, weakening timers, remote ammo/HP redemption, instant respawn, can trigger energy mechanism activation; auto vs semi-auto control mode.
- **Radar**: double vulnerability (≤2 chances), **encryption key / jamming-wave crack** game (levels 1–3), **armor fortification circular zone** on custom armor.
- **Hit position** reporting on armor (0x0206) and enemy fortify intel (0x0A05).
- Terrain-crossing RFID points greatly expanded (tunnels).
- RMUL-only: R/B highland control zones and central control zone.

---

## 5. Gotchas & errata spotted in V1.0.0

1. `power_heat_data_t` declares `reserved` three times — won't compile; rename.
2. Bit-field typos `operate_tpye`, `figure_tpye`; 0x0110 references undefined `graphic_data_struct_t` (use `interaction_figure_t`). The 0x0110 table also re-lists the 0x0301 header (offsets 0–5) — actual sub-payload is 45 B (15 + 30).
3. `RadarInfoToClient` proto in the PDF is malformed (`repeated uint32 RadarSingleRobotInfo;` with no tag) — intended as `repeated RadarSingleRobotInfo`; confirm with organizers.
4. MQTT sample uses QoS 2 (`ExactlyOnce`) and IP `127.0.0.1`; spec says max QoS 1 and server `192.168.12.1:3333`.
5. Dart gate status encodings differ: `0x020A` (1 closed, 2 transitioning, 0 open) vs `DartSelectTargetStatusSync` (0 closed, 1 opening, 2 open).
6. ID appendix still lists infantry 5/105 and client 0x0105/0x0169, while all data tables only carry infantry 3 & 4.
7. Units vary: m (0x0203/0x020B/0x0303), cm (0x0305/0x0A01/radar), dm (0x0307), mm (fortify zones, hit position).
8. 0x0303 is repeated (5× burst + 1 Hz) — de-duplicate.
9. Monotonic counters (sentry 0x0120, radar 0x0121) must increase by exactly the allowed step or the command is rejected; keep state across packets.
10. RFID bits are 0 outside matches even if the card is present.

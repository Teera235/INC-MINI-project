# Hardware Test Sketches

Each sketch isolates one subsystem so you can validate wiring and tuning before flashing the integrated firmware in `firmware/main/main.ino`.

All sketches use the same pin assignments as the production firmware. Open Serial Monitor at **115200 baud**.

## Index

| Sketch | What it tests | Hardware required |
|---|---|---|
| `test_displays/` | All three TM1637 7-segment displays | Displays on D22..D32 |
| `test_buzzer_leds/` | Buzzer + 4 status LEDs (Card, Scan, Weigh, Kettle) | LEDs D8/D7/D6/A3, buzzer D9 |
| `test_card_reader/` | LDR card reader pattern matching and color | LDR clock A8, data A9, color A10, LED A3 |
| `test_servos/` | X scan servo + Y rotation servo | Servos D44/D45 |
| `test_distance_scan/` | VL53L0X distance + 3D dimension scan flow | VL53L0X (I2C) + servos |
| `test_temp_relay/` | DS18B20 + relay closed-loop heating | DS18B20 D10, relay D12, LED A3 |
| `test_motor/` | DC conveyor motor open loop | Motor on D5/D4/D3 (legacy pinout) |
| `test_conveyor/` | Conveyor with transfer and dispatch patterns | Motor on D5/D4/D3 |
| `test_fan_rps/` | Fan tachometer + PID + feed-forward PWM | Fan PWM D11, tach A2 |
| `test_weight_fft/` | LC oscillator + FFT frequency measurement | LC tank into A0 |
| `test_weight_display/` | Quadratic weight model + TM1637 with drift comp | LC + displays |
| `test_weight_counter/` | Peak-to-peak amplitude counter (legacy) | LC into A0 |

## Recommended verification order

1. `test_displays` - confirm displays light up
2. `test_buzzer_leds` - confirm LEDs and buzzer
3. `test_card_reader` - calibrate THRESHOLD_HOLE / THRESHOLD_DATA if needed
4. `test_servos` - confirm both servos respond
5. `test_distance_scan` - calibrate VL53L0X position
6. `test_fan_rps` - tune tachometer threshold and feed-forward map
7. `test_temp_relay` - confirm relay polarity (active LOW) and DS18B20
8. `test_conveyor` - confirm motor direction
9. `test_weight_fft` - optional, for LC oscillator validation only

After each subsystem passes, flash `firmware/main/main.ino` for the integrated cell.

## Common interactive commands

Most sketches accept single-character commands followed by a number:

| Command | Sketch | Effect |
|---|---|---|
| `s10` | test_fan_rps | set target 10 RPS |
| `w500` | test_fan_rps | set target by weight 500 g |
| `p400` | test_fan_rps | raw PWM open loop |
| `S` | test_fan_rps, test_servos | run sweep |
| `t45` | test_temp_relay | set target 45 C |
| `f120` | test_conveyor | motor forward PWM 120 |
| `b100` | test_conveyor | motor backward PWM 100 |
| `D` | test_conveyor | run dispatch step pattern |
| `T` | test_conveyor | run 4 s transfer |
| `12` | test_buzzer_leds | card LED at 2 Hz |
| `b` | test_buzzer_leds | single beep |
| `X` | most sketches | stop everything |

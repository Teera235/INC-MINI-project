# INC-MINI-Project

An integrated Arduino-based automation cell that authenticates a swipe card, scans a box for three orthogonal dimensions using a single distance sensor, derives weight from the card programme, drives a fan proportional to weight, heats water to a target temperature, and dispatches the box on a conveyor.

The system runs as a single deterministic state machine on an Arduino Mega 2560 with no startup calibration.

---

## Table of Contents

- [Overview](#overview)
- [Hardware](#hardware)
- [Pin Map](#pin-map)
- [Card Types](#card-types)
- [Process Flow](#process-flow)
- [State Machine](#state-machine)
- [Subsystems](#subsystems)
- [Display Behaviour](#display-behaviour)
- [Repository Layout](#repository-layout)
- [Build and Upload](#build-and-upload)
- [Tools](#tools)
- [Test Sketches](#test-sketches)

---

## Overview

```mermaid
%%{init: {'theme':'base','themeVariables':{'primaryColor':'#ffffff','primaryBorderColor':'#000000','primaryTextColor':'#000000','lineColor':'#000000','secondaryColor':'#f5f5f5','clusterBkg':'#fafafa','clusterBorder':'#000000'}}}%%
flowchart LR
    A[Card Reader] --> B[3D Dimension Scan]
    B --> C[Conveyor Transfer]
    C --> D[Weight from Card]
    D --> E[Fan Speed Control]
    D --> F[Water Heating]
    E --> G[Done]
    F --> G

    classDef node fill:#ffffff,stroke:#000000,stroke-width:1.5px,color:#000000;
    class A,B,C,D,E,F,G node;
```

| Stage | Spec requirement | Implementation |
|---|---|---|
| Card | Swipe card A/B/C, display 1/2/3, beep on success, display E on reverse | LDR clock/data/color, pattern match, live bit display |
| Scan | Measure W, L, H with one sensor, display each in mm resolution | VL53L0X + X servo sweep, Y servo rotates box 0/90/180 deg |
| Transfer | Move box from scan platform to scale automatically | DC motor conveyor 4 s |
| Weight | Resolve weight from card programme (mid of range) | 300 g / 500 g / 700 g for cards A / B / C |
| Fan | Speed proportional to weight: 200 g = 1 RPS, 800 g = 20 RPS | Timer1 10-bit PWM, feed-forward + PID, display RPM |
| Heating | Target temp proportional to weight: 200 g = 35 C, 800 g = 50 C | DS18B20, relay, display target and current temp |
| Dispatch | Conveyor step-pattern after heating complete | DC motor 15-cycle step pattern |

---

## Hardware

- Arduino Mega 2560
- Adafruit VL53L0X time-of-flight distance sensor (I2C)
- DS18B20 temperature sensor (1-wire)
- LC oscillator front-end into ADC pin A0 (kept for future weight measurement)
- Hall or optical fan tachometer into A2
- 4-wire PC fan driven from Timer1 10-bit fast PWM at 31.25 kHz
- SSR or mechanical relay driving a 220 V 150 W kettle
- DC gear motor on H-bridge (ENA, IN1, IN2)
- Servo X for distance scan sweep (60-120 degrees)
- Servo Y for box rotation (0, 90, 180 degrees)
- LDR-based card reader: clock A8, data A9, color A10
- Three TM1637 4-digit 7-segment displays
- Buzzer
- LED card system (D8), LED scan system (D7), LED weigh system (D6)
- LED kettle status / card indicator (A3)

---

## Pin Map

```mermaid
%%{init: {'theme':'base','themeVariables':{'primaryColor':'#ffffff','primaryBorderColor':'#000000','primaryTextColor':'#000000','lineColor':'#000000','secondaryColor':'#f5f5f5','clusterBkg':'#fafafa','clusterBorder':'#000000'}}}%%
flowchart LR
    subgraph MCU[Arduino Mega 2560]
      M1[A0 FFT ADC]
      M2[A2 Fan Tacho]
      M3[A8 LDR Clock]
      M4[A9 LDR Data]
      M5[A10 LDR Color]
      M6[A3 Kettle and Card LED]
      M7[D11 PWM Fan]
      M8[D10 DS18B20]
      M9[D12 Relay]
      M10[D9 Buzzer]
      M11[D8 LED Card]
      M12[D7 LED Scan]
      M13[D6 LED Weigh]
      M14[D5..D3 Motor]
      M15[D44..D45 Servos]
      M16[D22..D32 TM1637]
    end

    classDef mcu fill:#ffffff,stroke:#000000,stroke-width:1.2px,color:#000000;
    class M1,M2,M3,M4,M5,M6,M7,M8,M9,M10,M11,M12,M13,M14,M15,M16 mcu;
```

| Function | Pin | Notes |
|---|---|---|
| LC oscillator ADC | A0 | reserved for optional LC weighing |
| Fan tachometer | A2 | analog input, adaptive thresholding |
| Card LDR clock | A8 | hole pattern timing |
| Card LDR data | A9 | hole pattern bits |
| Card LDR color | A10 | color identification and removal detection |
| Kettle / card LED | A3 | HIGH while heating or card insertion |
| PWM fan | D11 | Timer1 OC1A, 10-bit fast PWM 31.25 kHz |
| DS18B20 | D10 | OneWire bus |
| Relay kettle | D12 | active LOW |
| Buzzer | D9 | status tones |
| LED card system | D8 | blink 2 Hz idle, solid during read, blink 2 Hz on success |
| LED scan system | D7 | blink 0.25 Hz waiting, blink 0.5 Hz scanning, solid done |
| LED weigh system | D6 | off until box on scale, blink 1 Hz weighing, blink 0.25 Hz done |
| Motor ENA | D5 | PWM speed |
| Motor IN1 | D4 | direction |
| Motor IN2 | D3 | direction |
| Servo X | D44 | scan sweep axis |
| Servo Y | D45 | box rotation axis |
| Display 1 CLK / DIO | D22 / D24 | TM1637 (programme number) |
| Display 2 CLK / DIO | D26 / D28 | TM1637 (live bits / dim L / temp) |
| Display 3 CLK / DIO | D30 / D32 | TM1637 (bit count / dim H / RPM) |

---

## Card Types

```mermaid
%%{init: {'theme':'base','themeVariables':{'primaryColor':'#ffffff','primaryBorderColor':'#000000','primaryTextColor':'#000000','lineColor':'#000000','secondaryColor':'#f5f5f5','clusterBkg':'#fafafa','clusterBorder':'#000000'}}}%%
flowchart LR
    CA[Card A - White - 200 to 400 g] --> P1[Programme 1 - 300 g]
    CB[Card B - Yellow - 400 to 600 g] --> P2[Programme 2 - 500 g]
    CC[Card C - Green - 600 to 800 g] --> P3[Programme 3 - 700 g]

    classDef node fill:#ffffff,stroke:#000000,stroke-width:1.5px,color:#000000;
    class CA,CB,CC,P1,P2,P3 node;
```

| Card | Color | Weight range | Resolved | Display | Pattern |
|---|---|---|---|---|---|
| A | White | 200-400 g | 300 g | 1 | 0101010111 |
| B | Yellow | 400-600 g | 500 g | 2 | 0101100111 |
| C | Green | 600-800 g | 700 g | 3 | 0100110111 |
| Reversed / unknown | any | - | - | E | unrecognised pattern |

The resolved weight is the centre of each card's range, used to compute target temperature and fan RPS.

---

## Process Flow

```mermaid
%%{init: {'theme':'base','themeVariables':{'primaryColor':'#ffffff','primaryBorderColor':'#000000','primaryTextColor':'#000000','lineColor':'#000000','secondaryColor':'#f5f5f5','clusterBkg':'#fafafa','clusterBorder':'#000000'}}}%%
sequenceDiagram
    autonumber
    participant U as Operator
    participant CR as Card Reader
    participant DS as Dimension Scanner
    participant CV as Conveyor
    participant TC as Thermal Controller

    U->>CR: Swipe card
    CR-->>U: Live bit count and pattern on displays 2 and 3
    CR-->>U: Display 1 shows 1/2/3 with beep, or E on error
    CR->>DS: Trigger scan sequence
    DS->>DS: Reference distance, scan dim1 at 0 deg
    DS->>DS: Rotate Y to 90 deg, scan dim2
    DS->>DS: Rotate Y to 180 deg, scan dim3
    DS-->>U: Display W L H on three 7-seg, double beep
    DS->>CV: Trigger transfer
    CV->>TC: Box arrives, weight resolved from card
    TC-->>U: Display weight, beep 3 times
    TC->>TC: Heat with relay, regulate fan via feed-forward + PID
    TC-->>U: Continuous beep while heating, display temp and RPM
    TC-->>U: Long beep 3 s at target, relay off
    TC-->>U: Final triple beep, cycle complete, return to card wait
```

---

## State Machine

```mermaid
%%{init: {'theme':'base','themeVariables':{'primaryColor':'#ffffff','primaryBorderColor':'#000000','primaryTextColor':'#000000','lineColor':'#000000','secondaryColor':'#f5f5f5','clusterBkg':'#fafafa','clusterBorder':'#000000'}}}%%
stateDiagram-v2
    direction LR
    [*] --> WAIT_CARD: boot, no calibration
    WAIT_CARD --> CARD_VALID_DISPLAY: pattern matched
    WAIT_CARD --> CARD_INVALID_DISPLAY: reversed or unknown
    CARD_INVALID_DISPLAY --> WAIT_CARD: 2 s timeout
    CARD_VALID_DISPLAY --> WAIT_OBJECT: 2 s display
    WAIT_OBJECT --> REF_DIST: object detected
    REF_DIST --> SCAN_DIM1
    SCAN_DIM1 --> ROTATE_TO_DIM2
    ROTATE_TO_DIM2 --> SCAN_DIM2
    SCAN_DIM2 --> ROTATE_TO_DIM3
    ROTATE_TO_DIM3 --> SCAN_DIM3
    SCAN_DIM3 --> SCAN_DONE
    SCAN_DONE --> TRANSFER_TO_SCALE
    TRANSFER_TO_SCALE --> WEIGH_DONE: weight from card, FF PWM applied
    WEIGH_DONE --> HEATING: after beep sequence
    HEATING --> BOIL_COMPLETE: temperature reached
    BOIL_COMPLETE --> DONE: final beep
    DONE --> WAIT_CARD: cycle reset
```

The legacy states `STATE_FAN_CALIB` and `STATE_WEIGHT_ZERO_CALIB` collapse into a one-loop pass-through to `STATE_WAIT_CARD`. The legacy `STATE_WAIT_ON_SCALE` and `STATE_WEIGHING` immediately forward to `STATE_WEIGH_DONE` since weight comes from the card.

---

## Subsystems

### Card Reader

Three LDRs read clock, data, and color tracks from a swipe card. The clock channel triggers on a falling edge below ADC 500. Each rising clock edge samples the data LDR; ten bits are compared against patterns for cards A, B, and C. A reversed card or any unknown pattern shows E.

A removal protection timer resets the reader if the color channel rises above ADC 55 for more than 500 ms during a read. After a successful match the firmware waits until the clock LDR exceeds ADC 900 (card fully removed) before resetting.

Live indication during the swipe:
- Display 1 stays as a dash
- Display 2 shows the most recent four bits as `0101`-style digits
- Display 3 shows the running bit count from 0 to 10
- Card LED on A3 lights solid while the card is in the slot
- Buzzer beeps 150 ms on success, 500 ms on error

After ten bits the resolved digit (1/2/3 or E) replaces the dash on display 1 for two seconds, then the system advances to dimension scanning automatically.

### Dimension Scanner

The VL53L0X returns distance in millimetres. A reference background is averaged over ten stable reads. An object is confirmed when two consecutive deltas exceed 1 cm. The X servo sweeps from 60 to 120 degrees and back, recording the maximum protrusion above the reference plane. The Y servo rotates the box platform to 0, 90, and 180 degrees so the same sensor sees three orthogonal faces.

Results are displayed on the three TM1637 modules in centimetres with 1 mm resolution (format XX.XX).

LED scan behaviour:
- Waiting for box: blink 0.25 Hz
- Scanning: blink 0.5 Hz with continuous buzzer
- Done: solid, double beep

### Weight from Card

Weight is resolved from the card programme without LC measurement. Each card maps to the centre of its specified range:

```
Card A (200-400 g) -> 300 g
Card B (400-600 g) -> 500 g
Card C (600-800 g) -> 700 g
```

The legacy LC pipeline (FFT, median filter, parabolic peak interpolation, baseline drift compensation) remains in the source for reference but is not consulted in the active flow. To re-enable real LC weighing, restore the `STATE_WAIT_ON_SCALE` and `STATE_WEIGHING` bodies and remove the `weightFromCard()` call in `STATE_TRANSFER_TO_SCALE`.

LED weigh behaviour:
- Box not yet on scale: off
- Box transferring: blink 1 Hz
- Weight resolved: blink 0.25 Hz with three beeps

### Fan Speed Control

Fan target RPS is mapped linearly from weight:

```
200 g -> 1 RPS
800 g -> 20 RPS
targetRPS = 1 + (weight - 200) * 19 / 600
```

Open-loop feed-forward sets the PWM directly from the resolved RPS the moment weight is known:

```
1 RPS  -> PWM 150
20 RPS -> PWM 900
```

A discrete PID loop running every 20 ms then trims any residual error. Tachometer pulses on A2 are detected with adaptive thresholding (no startup calibration) and a 5-sample median filter rejects pulse noise. RPM (= RPS x 60) is shown on display 3 during heating.

### Thermal Control

Target temperature is mapped linearly from weight:

```
200 g -> 35 C
800 g -> 50 C
targetTemp = 35 + (weight - 200) * 15 / 600
```

DS18B20 is read once per second. The relay drives the kettle with a 1.0 / 0.3 degree hysteresis band. The kettle status LED on A3 mirrors the relay state. A continuous buzzer tone sounds while heating. When the target is reached the relay opens, the LED turns off, and a 3 s long beep fires. The current temperature continues to be displayed after heating ends.

### Conveyor

Transfer from scan platform to scale: forward at PWM 120 for 4 seconds.

Dispatch after heating: 15 step-cycles of backward 250 ms then forward 250 ms at PWM 100, followed by a final backward run at PWM 70 for 2 seconds. The fan PID keeps running during all conveyor operations.

---

## Display Behaviour

| Phase | Display 1 | Display 2 | Display 3 |
|---|---|---|---|
| Boot | dash | blank | blank |
| Waiting for card | dash | blank | blank |
| Card inserted, reading bits | dash | last 4 bits e.g. `0101` | bit count 1..10 |
| Card matched | 1, 2, or 3 | (blank) | (blank) |
| Card error | E | (blank) | (blank) |
| Waiting for box | dash | blank | blank |
| Scan complete | width XX.XX cm | length XX.XX cm | height XX.XX cm |
| Weight resolved | weight in 10 g (e.g. 300) | (blank) | (blank) |
| Heating | weight | target temp C | RPM |
| Boil complete | weight | current temp C | RPM |
| Done | done text on all three | done text | done text |

---

## Repository Layout

```
inc-mini-project/
├── README.md
├── .gitignore
├── docs/
│   └── images/
│       ├── analysis_result.png
│       ├── pattern_model.png
│       └── plot_output.png
├── firmware/
│   ├── main/
│   │   └── main.ino
│   ├── main_cheat/
│   │   └── main_cheat.ino
│   └── tests/
│       ├── test_weight_fft/
│       │   └── test_weight_fft.ino
│       ├── test_weight_display/
│       │   └── test_weight_display.ino
│       ├── test_weight_counter/
│       │   └── test_weight_counter.ino
│       └── test_motor/
│           └── test_motor.ino
└── tools/
    ├── collect_training_data.py
    ├── plot_graph.py
    └── realtime_plot.py
```

`firmware/main/` is the active production firmware. `firmware/main_cheat/` is an alternate sketch retained for bench testing with hardcoded constants and shorter calibration paths.

---

## Build and Upload

Open `firmware/main/main.ino` in the Arduino IDE and target Arduino Mega 2560.

Required libraries:

- `arduinoFFT` by Enrique Condes
- `OneWire` by Paul Stoffregen
- `Adafruit_VL53L0X` by Adafruit
- `Servo` (bundled with IDE)

With `arduino-cli`:

```bash
arduino-cli core install arduino:avr
arduino-cli lib install "arduinoFFT" "OneWire" "Adafruit VL53L0X"
arduino-cli compile --fqbn arduino:avr:mega firmware/main
arduino-cli upload --fqbn arduino:avr:mega -p COM5 firmware/main
```

Serial monitor at 115200 baud reports state transitions, bit reads, color values, weight resolution, target setpoints, and runtime telemetry.

---

## Tools

| Script | Purpose |
|---|---|
| `collect_training_data.py` | Records peak frequency and timing features against known weights into CSV (for future LC calibration) |
| `plot_graph.py` | Fits exponential decay and prints constants for firmware |
| `realtime_plot.py` | Live frequency and weight plot using cubic calibration model |

```bash
pip install pyserial numpy pandas matplotlib scipy
```

Edit `PORT` in each script before running.

---

## Test Sketches

| Sketch | Subsystem | Notes |
|---|---|---|
| `test_weight_fft` | LC oscillator and FFT | Streams filtered frequency, baseline, and delta |
| `test_weight_display` | Quadratic weight model and TM1637 | Includes drift compensation |
| `test_weight_counter` | Peak-to-peak amplitude counter | Cross-check for FFT path |
| `test_motor` | Fan PID and 10-bit PWM | Reports measured and target RPS |

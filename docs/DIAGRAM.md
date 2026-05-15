# System Diagram - main.ino

## Full State Machine (Detailed)

```mermaid
%%{init: {'theme':'base','themeVariables':{'primaryColor':'#ffffff','primaryBorderColor':'#000000','primaryTextColor':'#000000','lineColor':'#000000','secondaryColor':'#f5f5f5','clusterBkg':'#fafafa','clusterBorder':'#000000'}}}%%
stateDiagram-v2
    direction TB

    [*] --> WAIT_CARD: Power ON (no calibration)

    state "CARD READING" as card_group {
        WAIT_CARD --> WAIT_CARD: no card (LED blink 2Hz, display dash)
        WAIT_CARD --> CARD_VALID: pattern A/B/C matched
        WAIT_CARD --> CARD_INVALID: unknown or reversed pattern
        CARD_INVALID --> WAIT_CARD: display E for 2s then reset
        CARD_VALID --> WAIT_OBJECT: display 1/2/3 for 2s then advance
    }

    state "DIMENSION SCAN" as scan_group {
        WAIT_OBJECT --> WAIT_OBJECT: no box (LED blink 0.25Hz)
        WAIT_OBJECT --> REF_DIST: VL53L0X delta > 2cm
        REF_DIST --> SCAN_DIM1: reference captured
        SCAN_DIM1 --> ROTATE_90: servo X sweep done, dim1 stored
        ROTATE_90 --> SCAN_DIM2: Y at 90 deg, new ref captured
        SCAN_DIM2 --> ROTATE_180: dim2 stored
        ROTATE_180 --> SCAN_DIM3: Y at 180 deg, new ref captured
        SCAN_DIM3 --> SCAN_DONE: dim3 stored
        SCAN_DONE --> TRANSFER: display W/L/H, double beep, wait 2s
    }

    state "WEIGHT + HEATING" as heat_group {
        TRANSFER --> WEIGH_DONE: conveyor 4s, weight from card
        WEIGH_DONE --> HEATING: beep x3, display weight + target temp
        HEATING --> BOIL_COMPLETE: temp >= target (relay off, beep 3s)
        BOIL_COMPLETE --> DONE: final triple beep
    }

    DONE --> WAIT_CARD: reset all, cycle again
```

---

## Detailed Flow with Actions

```mermaid
%%{init: {'theme':'base','themeVariables':{'primaryColor':'#ffffff','primaryBorderColor':'#000000','primaryTextColor':'#000000','lineColor':'#000000','secondaryColor':'#f5f5f5','clusterBkg':'#fafafa','clusterBorder':'#000000'}}}%%
flowchart TD
    START([Power ON]) --> WC

    subgraph CARD[Card Reader Phase]
        WC[WAIT_CARD<br/>LED Card blink 2Hz<br/>Display 1: dash<br/>Display 2/3: blank]
        WC -->|LDR clock LOW| READING[Card Inserted<br/>LED Card solid<br/>Read 10 bits<br/>Display 2: last 4 bits<br/>Display 3: bit count]
        READING -->|pattern = A| VALID[CARD_VALID<br/>Display 1: 1<br/>Beep 150ms]
        READING -->|pattern = B| VALID2[CARD_VALID<br/>Display 1: 2<br/>Beep 150ms]
        READING -->|pattern = C| VALID3[CARD_VALID<br/>Display 1: 3<br/>Beep 150ms]
        READING -->|unknown/reversed| INVALID[CARD_INVALID<br/>Display 1: E<br/>Beep 500ms]
        INVALID -->|2s timeout| WC
        VALID --> WAIT2s[Wait 2 seconds]
        VALID2 --> WAIT2s
        VALID3 --> WAIT2s
    end

    WAIT2s --> WO

    subgraph SCAN[Dimension Scan Phase]
        WO[WAIT_OBJECT<br/>LED Scan blink 0.25Hz<br/>VL53L0X polling every 200ms]
        WO -->|delta > 2cm| REF[REF_DIST<br/>Average 10 readings x 50ms]
        REF --> D1[SCAN_DIM1<br/>LED Scan blink 0.5Hz<br/>Buzzer ON<br/>Servo X: 60 to 120 to 60<br/>Record max height]
        D1 --> R90[ROTATE_TO_DIM2<br/>Servo Y: 0 to 90 deg<br/>New reference]
        R90 --> D2[SCAN_DIM2<br/>Servo X sweep again]
        D2 --> R180[ROTATE_TO_DIM3<br/>Servo Y: 90 to 180 deg<br/>New reference]
        R180 --> D3[SCAN_DIM3<br/>Servo X sweep again]
        D3 --> DONE_SCAN[SCAN_DONE<br/>LED Scan solid<br/>Buzzer OFF<br/>Double beep<br/>Display 1: W cm<br/>Display 2: L cm<br/>Display 3: H cm<br/>Wait 2s]
    end

    DONE_SCAN --> XFER

    subgraph WEIGHT[Weight + Transfer Phase]
        XFER[TRANSFER_TO_SCALE<br/>Conveyor forward 4s @ PWM 120<br/>Weight = center of card range<br/>Card A: 300g<br/>Card B: 500g<br/>Card C: 700g<br/>Set fan PWM feed-forward<br/>Reset PID]
        XFER --> WD[WEIGH_DONE<br/>LED Weigh blink 0.25Hz<br/>Display 1: weight in 10g<br/>Beep x3]
    end

    WD --> HEAT

    subgraph THERMAL[Heating Phase]
        HEAT[HEATING<br/>Relay ON<br/>LED Kettle ON<br/>Buzzer blink 300ms<br/>Fan PID running<br/>Display 1: weight<br/>Display 2: target temp<br/>Display 3: RPM]
        HEAT -->|temp >= target| BC[BOIL_COMPLETE<br/>Relay OFF<br/>LED Kettle OFF<br/>Buzzer 3s long<br/>Display 2: current temp]
        BC -->|2s| FINAL[DONE<br/>Triple beep<br/>All displays: DONE<br/>Wait 5s]
    end

    FINAL -->|reset| WC

    classDef phase fill:#ffffff,stroke:#000000,stroke-width:1.5px,color:#000000;
    classDef action fill:#f9f9f9,stroke:#333333,stroke-width:1px,color:#000000;
    class WC,READING,VALID,VALID2,VALID3,INVALID,WAIT2s action;
    class WO,REF,D1,R90,D2,R180,D3,DONE_SCAN action;
    class XFER,WD action;
    class HEAT,BC,FINAL action;
```

---

## Pin Wiring Diagram

```mermaid
%%{init: {'theme':'base','themeVariables':{'primaryColor':'#ffffff','primaryBorderColor':'#000000','primaryTextColor':'#000000','lineColor':'#000000','secondaryColor':'#f5f5f5','clusterBkg':'#fafafa','clusterBorder':'#000000'}}}%%
flowchart LR
    subgraph ANALOG[Analog Inputs]
        A0[A0: LC Oscillator]
        A2[A2: Fan Tachometer]
        A8[A8: LDR Clock]
        A9[A9: LDR Data]
        A10[A10: LDR Color]
    end

    subgraph DIGITAL_OUT[Digital Outputs]
        D3[D3: Motor IN2]
        D4[D4: Motor IN1]
        D5[D5: Motor ENA]
        D6[D6: LED Weigh]
        D7[D7: LED Scan]
        D8[D8: LED Card]
        D9[D9: Buzzer]
        D10[D10: DS18B20]
        D11[D11: Fan PWM]
        D12[D12: Relay]
        DA3[A3: LED Kettle]
    end

    subgraph SERVO[Servos]
        D44[D44: Servo X scan]
        D45[D45: Servo Y rotate]
    end

    subgraph DISPLAY[TM1637 Displays]
        TM1[D22/D24: Display 1]
        TM2[D26/D28: Display 2]
        TM3[D30/D32: Display 3]
    end

    subgraph I2C[I2C Bus]
        VL[SDA/SCL: VL53L0X]
    end

    classDef io fill:#ffffff,stroke:#000000,stroke-width:1.2px,color:#000000;
    class A0,A2,A8,A9,A10,D3,D4,D5,D6,D7,D8,D9,D10,D11,D12,DA3,D44,D45,TM1,TM2,TM3,VL io;
```

---

## Display Content by State

```mermaid
%%{init: {'theme':'base','themeVariables':{'primaryColor':'#ffffff','primaryBorderColor':'#000000','primaryTextColor':'#000000','lineColor':'#000000','secondaryColor':'#f5f5f5','clusterBkg':'#fafafa','clusterBorder':'#000000'}}}%%
flowchart TD
    subgraph DISP1[Display 1 - D22/D24]
        D1A[Idle: dash]
        D1B[Card reading: dash]
        D1C[Card matched: 1 or 2 or 3]
        D1D[Card error: E]
        D1E[Scan done: W cm]
        D1F[Weight: 300/500/700]
        D1G[Heating: weight]
    end

    subgraph DISP2[Display 2 - D26/D28]
        D2A[Idle: blank]
        D2B[Card reading: last 4 bits]
        D2C[Scan done: L cm]
        D2D[Heating: target temp]
        D2E[Boil done: current temp]
    end

    subgraph DISP3[Display 3 - D30/D32]
        D3A[Idle: blank]
        D3B[Card reading: bit count 1-10]
        D3C[Scan done: H cm]
        D3D[Heating: RPM]
    end

    classDef disp fill:#ffffff,stroke:#000000,stroke-width:1px,color:#000000;
    class D1A,D1B,D1C,D1D,D1E,D1F,D1G,D2A,D2B,D2C,D2D,D2E,D3A,D3B,D3C,D3D disp;
```

---

## Weight Resolution Logic

```mermaid
%%{init: {'theme':'base','themeVariables':{'primaryColor':'#ffffff','primaryBorderColor':'#000000','primaryTextColor':'#000000','lineColor':'#000000','secondaryColor':'#f5f5f5','clusterBkg':'#fafafa','clusterBorder':'#000000'}}}%%
flowchart LR
    CARD_A[Card A<br/>Pattern: 0101010111<br/>Color: White] -->|weightFromCard| W300[300g]
    CARD_B[Card B<br/>Pattern: 0101100111<br/>Color: Yellow] -->|weightFromCard| W500[500g]
    CARD_C[Card C<br/>Pattern: 0100110111<br/>Color: Green] -->|weightFromCard| W700[700g]

    W300 --> TEMP35[Target: 32.5 C<br/>RPS: 4.17]
    W500 --> TEMP38[Target: 37.5 C<br/>RPS: 10.5]
    W700 --> TEMP43[Target: 42.5 C<br/>RPS: 16.8]

    classDef card fill:#ffffff,stroke:#000000,stroke-width:1.5px,color:#000000;
    classDef result fill:#f5f5f5,stroke:#000000,stroke-width:1px,color:#000000;
    class CARD_A,CARD_B,CARD_C card;
    class W300,W500,W700,TEMP35,TEMP38,TEMP43 result;
```

---

## Fan Control Pipeline

```mermaid
%%{init: {'theme':'base','themeVariables':{'primaryColor':'#ffffff','primaryBorderColor':'#000000','primaryTextColor':'#000000','lineColor':'#000000','secondaryColor':'#f5f5f5','clusterBkg':'#fafafa','clusterBorder':'#000000'}}}%%
flowchart LR
    WT[Weight] -->|weightToTargetRPS| TGT[Target RPS<br/>1 to 20]
    TGT -->|feedForwardPWM| FF[Initial PWM<br/>150 to 900]
    FF --> FAN[Fan Motor<br/>Timer1 31.25kHz]
    FAN --> TACH[Tachometer A2]
    TACH -->|pulse interval| RAW[Raw RPS]
    RAW -->|median of 5| SMOOTH[Smoothed RPS]
    SMOOTH --> PID[PID Controller<br/>Kp=0.5 Ki=1.0 Kd=0.05<br/>every 20ms]
    PID -->|trim PWM| FAN
    SMOOTH -->|x 60| RPM[Display RPM]

    classDef block fill:#ffffff,stroke:#000000,stroke-width:1.2px,color:#000000;
    class WT,TGT,FF,FAN,TACH,RAW,SMOOTH,PID,RPM block;
```

---

## Thermal Control Pipeline

```mermaid
%%{init: {'theme':'base','themeVariables':{'primaryColor':'#ffffff','primaryBorderColor':'#000000','primaryTextColor':'#000000','lineColor':'#000000','secondaryColor':'#f5f5f5','clusterBkg':'#fafafa','clusterBorder':'#000000'}}}%%
flowchart LR
    WT2[Weight] -->|weightToTargetTemp| TT[Target Temp<br/>35 to 50 C]
    TT --> CMP{Current >= Target?}
    DS[DS18B20<br/>Read every 1s] --> CMP
    CMP -->|No and below margin| RON[Relay ON<br/>LED Kettle ON]
    CMP -->|Yes or above margin| ROFF[Relay OFF<br/>LED Kettle OFF]
    CMP -->|Temp reached| BEEP[Buzzer 3s<br/>State: BOIL_COMPLETE]

    classDef block fill:#ffffff,stroke:#000000,stroke-width:1.2px,color:#000000;
    classDef decision fill:#f9f9f9,stroke:#000000,stroke-width:1.5px,color:#000000;
    class WT2,TT,DS,RON,ROFF,BEEP block;
    class CMP decision;
```

---

## LED Behaviour Summary

```mermaid
%%{init: {'theme':'base','themeVariables':{'primaryColor':'#ffffff','primaryBorderColor':'#000000','primaryTextColor':'#000000','lineColor':'#000000','secondaryColor':'#f5f5f5','clusterBkg':'#fafafa','clusterBorder':'#000000'}}}%%
gantt
    title LED States Through Process
    dateFormat X
    axisFormat %s

    section LED Card D8
    Blink 2Hz (idle)       :0, 10
    Solid (card in)        :10, 15
    Blink 2Hz (success)    :15, 20
    Off                    :20, 100

    section LED Scan D7
    Off                    :0, 20
    Blink 0.25Hz (wait)    :20, 30
    Blink 0.5Hz (scanning) :30, 60
    Solid (done)           :60, 65
    Blink 0.25Hz           :65, 70
    Off                    :70, 100

    section LED Weigh D6
    Off                    :0, 65
    Blink 1Hz (transfer)   :65, 70
    Blink 0.25Hz (done)    :70, 75
    Off                    :75, 100

    section LED Kettle A3
    Off                    :0, 75
    Solid (heating)        :75, 95
    Off (done)             :95, 100
```

---

## Timing Estimate (Typical Cycle)

| Phase | Duration | Notes |
|---|---|---|
| Boot to ready | instant | no calibration |
| Card swipe | 1-3 s | depends on swipe speed |
| Card display | 2 s | fixed |
| Wait for box | user dependent | until box placed |
| Reference capture | 2.5 s | 10 x 50ms + readStable |
| Scan dim1 | ~8 s | 120 steps x (40ms + 50ms readStable) |
| Rotate Y 90 | ~2 s | 90 steps x 15ms + 500ms settle |
| Scan dim2 | ~8 s | same |
| Rotate Y 180 | ~2 s | 90 steps |
| Scan dim3 | ~8 s | same |
| Display result | 2 s | fixed |
| Conveyor transfer | 4 s | fixed |
| Weight beep | 2.5 s | 3 beeps + wait |
| Heating | variable | depends on water temp and target |
| Boil complete | 5 s | 3s beep + 2s wait |
| Done display | 5 s | fixed |
| **Total (excl. heating)** | **~50 s** | |

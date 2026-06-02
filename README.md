# 🤖 VURP — Valentin's Ultimate Rover Project

A WiFi-controlled tracked rover, built together by Valentin and Papa.

**Goal:** Learn electronics & coding while building something genuinely cool — a
tank-tread rover you can drive from a phone or laptop browser, with sensors and a
glowing LED ring.

> 🚀 **Start here:** [SETUP.md](SETUP.md) — set up VS Code + PlatformIO and upload
> our first program. Code lives in [src/main.cpp](src/main.cpp).

---

## How it works (the big picture)

```text
   Phone / Laptop  ── WiFi ──►  XIAO ESP32-C3 (the brain)
                                     │
              ┌──── I²C bus (2 wires: SDA + SCL) ────┬──────────┐
              │                  │                   │          │
        Motor FeatherWing   OLED screen        ToF sensor      │
           (0x60)            (0x3C)             (0x29)          │
            │   │           "status/IP"      "wall ahead!"      │
            ▼   ▼                                               │
        Left    Right                              LED ring ◄───┘
        tread   tread   ◄── 2 DC motors           (NeoPixel, on its own pin)
                            (own battery!)

                        GPS module ──► separate UART (serial) pins
```

The ESP32 runs its **own WiFi network** ("VURP") and serves a **web page with an
on-screen joystick**. No app to install — just connect and drive. Steering is
**differential**: run the treads at different speeds to turn, opposite directions
to spin in place.

**Why this layout:** the XIAO ESP32-C3 has very few pins, so we put the motor
driver + OLED + ToF sensor all on the shared **I²C bus** (just 2 pins for all three).
That keeps pins free and is a great lesson in how I²C addresses work.

---

## Parts list (BOM)

| Part | What it does | Status |
| --- | --- | --- |
| **XIAO ESP32-C3** | The brain + WiFi | ✅ Chosen |
| **Adafruit Motor FeatherWing** (DC + Stepper, I²C/TB6612) | Drives the 2 tread motors over I²C (only 2 pins!) | ✅ Chosen |
| **Tracked chassis** (tank treads + 2 DC motors) | Locomotion | ❓ Confirm we have one |
| **LED ring** (NeoPixel / WS2812) | Lights & status display | ✅ Have it |
| **ToF distance sensor** (VL53L0X-type, I²C) | Obstacle detection — phase 1 sensor | ✅ Have it |
| **Tiny OLED screen** (I²C) | Shows WiFi IP, speed, distance | ✅ Have it |
| **Ultrasonic** (HC-SR04) | Backup distance sensor — needs 5V + divider | 🟡 Later |
| **u-blox GPS** (UART) | Outdoor position logging | 🟡 Stretch |
| **Camera** (B0068 etc.) | FPV video — needs the full Raspberry Pi | 🔴 Season 2 |
| **Battery — logic** (for the XIAO, via USB power bank or regulator) | Powers the brain | ❓ Confirm |
| **Battery — motors** (4×AA or LiPo, into FeatherWing terminals) | Powers the treads, **separate from logic** | ❓ Confirm |
| **Header wires + soldering** | To connect XIAO ↔ FeatherWing (it's not a stack) | ❓ Need to solder |
| **Breadboard + jumper wires** | Wiring it all up | ❓ Confirm |
| **On/off switch** | So we're not pulling batteries constantly | nice-to-have |

> ⚠️ **Key rule:** a microcontroller pin can't drive a motor directly — it needs a
> **motor driver** in between. And motors get their **own** battery power, never
> through the microcontroller's pins.

---

## The plan (milestones)

We'll build in small wins so Valentin sees progress every session:

- [x] **1. Blink** — ✅ board boots & uploads, "Hello from VURP!" on Serial. (LED wiring next session.)
- [ ] **2. Light up** — control the NeoPixel LED ring (colors, animations).
- [ ] **3. WiFi page** — ESP32 serves a web page; a button on the phone turns the ring on/off.
- [ ] **4. One motor** — wire up the motor driver, spin one tread forward/back.
- [ ] **5. Two motors** — both treads; drive forward, back, and turn.
- [ ] **6. Joystick** — web page joystick → smooth driving from the phone.
- [ ] **7. Screen** — wire the OLED onto the I²C bus; show the rover's WiFi IP + status.
- [ ] **8. Sensors** — read the ToF distance; flash the ring red and stop when close to an obstacle.
- [ ] **9. Polish** — mount everything on the chassis, tidy wiring, name it, show it off.
- [ ] **Stretch:** GPS logging outdoors; then the full Raspberry Pi + camera for FPV driving.

---

## Decisions made

- ✅ **Brain:** XIAO ESP32-C3 (WiFi, hosts its own web page).
- ✅ **Motor driver:** Adafruit Motor FeatherWing over I²C — beats the L298N here
  because it saves precious GPIO pins and is far more efficient.
- ✅ **Phase-1 sensors:** ToF distance + OLED screen (both I²C). Ultrasonic, GPS,
  and cameras come later.
- ✅ **Language & tools:** Arduino C++, written in **VS Code + PlatformIO**.
- ℹ️ **Note:** the XIAO ESP32-C3 has **no user-programmable onboard LED** (unlike
  most XIAO boards) — so "Blink" uses an **external LED** on a breadboard.

## Decisions still open

- [ ] **Chassis:** confirm we have a tracked chassis with 2 DC motors.
- [ ] **Power:** how we power logic vs. motors (two separate supplies).

---

## Project log

- **2026-06-02** — Project kicked off. Chose wireless web control + ESP32 brain.
  Locked in: XIAO ESP32-C3 + Adafruit Motor FeatherWing (I²C), with ToF + OLED as
  phase-1 sensors. Set up this repo (VS Code + PlatformIO).
- **2026-06-02** — ✅ **Milestone 1 (software half):** first upload working,
  "Hello from VURP!" on the Serial Monitor. The C3 wasn't detected at first; fixed
  by forcing USB download mode (**hold B, tap R, release B**) — good trick to
  remember if the port ever vanishes. LED wiring still to do.

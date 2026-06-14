# 🤖 VURP — Valentin's Ultimate Rover Project

A WiFi-controlled tracked rover, built together by Valentin and Papa.

**Goal:** Learn electronics & coding while building something genuinely cool — a
tank-tread rover you can drive from a phone or laptop browser, with sensors and a
glowing LED ring.

> 🚀 **Start here:** [SETUP.md](SETUP.md) — set up VS Code + PlatformIO and upload
> our first program. Code lives in [src/main.cpp](src/main.cpp).
> The rover web page lives in [data/index.html](data/index.html); upload the
> filesystem image when that file changes.

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
- [x] **2. Light up** — ✅ NeoPixel ring runs a rotating rainbow (powered from 3V3, data on D2).
- [x] **3. WiFi page** — ✅ ESP32 serves a web page; a button on the phone turns the ring on/off.
- [x] **4. Motor test** — ✅ spun the M3/M4 treads forward/back from the web page.
- [x] **5. Two motors** — ✅ both treads: drive forward, back, and turn (spin in place).
- [x] **6. Joystick** — ✅ on-screen touch joystick → smooth proportional driving from the phone.
- [x] **7. Screen** — ✅ OLED on the shared I²C bus shows the WiFi name, IP, and live status.
- [ ] **8. Sensors** — read the ToF distance; flash the ring red and stop when close to an obstacle. 👉 **next**
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

- ✅ **Chassis:** confirmed — tracked chassis with 2 DC motors on hand.
- ✅ **FeatherWing wired:** the ~4 connection wires to the XIAO are soldered.
- ✅ **I²C pins:** on the XIAO ESP32-C3, **SDA = D4 / GPIO6** and
  **SCL = D5 / GPIO7**. The Motor FeatherWing uses address **0x60**.

## Decisions still open

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
- **2026-06-03** — ✅ **Milestone 1 done** (LED blinks) and ✅ **Milestone 2 done**
  (NeoPixel rainbow, powered from 3V3, data on D2). Confirmed we have the tracked
  chassis and the Motor FeatherWing wires are soldered. **Next session: Friday
  Jun 5 — Milestone 3 (WiFi control page).**
- **2026-06-05** — ✅ **Milestone 3 done**: the XIAO ESP32-C3 creates the **VURP**
  WiFi network and serves a phone-friendly page at **http://192.168.4.1/**. The
  button turns the NeoPixel ring rainbow on/off.
- **2026-06-05** — **Milestone 4 test firmware prepared**: added the Adafruit
  Motor FeatherWing library and web buttons for **M3/M4 forward / stop / reverse**.
  First tests use low speed and auto-stop after 1 second.
- **2026-06-05** — Refactored the rover web controls: HTML moved to
  **data/index.html**, and buttons now call tiny JSON API endpoints instead of
  refreshing the whole page.
- **2026-06-05** — Improved the motor test controls: added low/medium/max speed
  buttons and a short full-power start boost so the tank treads can overcome
  static friction more reliably.
- **2026-06-05** — Replaced the main motor test buttons with a 4-way drive pad:
  forward/back drive both treads, left/right spin in place by running M3 and M4
  in opposite directions.
- **2026-06-14** — ✅ **Milestones 4, 5 & 6 done**: motor test, two-tread driving,
  and now a real **on-screen joystick** (`/api/drive`). The page sends a stick
  vector and the firmware does the differential-drive mixing (`left = y + x`,
  `right = y - x`) into proportional tread speeds. Added a **deadman watchdog**:
  the rover stops itself if the phone stops sending updates (lock / out of range).
  Built, uploaded (firmware + LittleFS), and tested on the rover — drives great.
  **Next: Milestone 7 — wire the OLED onto the I²C bus and show IP + status.**
- **2026-06-14** — ✅ **Milestone 7 done**: wired the **OLED** onto the shared I²C
  bus (no soldering — piggybacks on SDA/SCL alongside the FeatherWing). Boot now
  runs a full **I²C scan** that names each device; it found `0x3C` (OLED), `0x60`
  + `0x70` (the FeatherWing's PCA9685, which also answers the "All Call" address).
  The screen shows the WiFi name, IP, ring state, speed, and an `idle`/`moving`
  state — and only redraws on change so it never stutters the joystick. Contrast
  set to max for brightness. **Next: Milestone 8 — ToF obstacle sensor.**

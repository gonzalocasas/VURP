# 🛠️ Setting up the workshop (one-time)

We program the rover from **VS Code** using the **PlatformIO** extension. Do these
steps together with Valentin — he can do the clicking. You only do this **once**.

## 1. Install the PlatformIO extension

1. Open **VS Code**.
2. Click the **Extensions** icon in the left sidebar (the four little squares).
3. Search for **"PlatformIO IDE"** and click **Install**.
4. Wait for it to finish setting up (it downloads some tools — grab a snack ☕).
   When it's ready you'll see a little **alien/ant head 👽 icon** in the left sidebar.

> PlatformIO will download the ESP32 toolchain the first time we build — that's
> normal and only happens once.

## 2. Open the firmware project

1. In VS Code: **File → Open Folder…** and choose the **`VURP`** project folder
   (the one with `platformio.ini` and `README.md` inside).
2. PlatformIO reads `platformio.ini` and already knows our board is the
   **Seeed XIAO ESP32-C3**. Nothing to configure. 🎉

## 3. Plug in the board

1. Connect the **XIAO ESP32-C3** to the Mac with a **USB-C data cable**.
   ⚠️ Some cheap cables are *charge-only* and won't work — if the board is never
   found, **try a different cable first**.

## 4. Upload our first program

At the **bottom blue bar** of VS Code, PlatformIO adds little buttons:

- **✓ (checkmark)** = Build (compile the code)
- **→ (arrow)** = Upload (send it to the board)
- **🔌 (plug)** = Serial Monitor (see messages from the board)

Steps:

1. Click **→ (Upload)**. First time it compiles a lot — be patient.
2. Click the **🔌 (Serial Monitor)**.
3. 🎉 You should see **"Hello from VURP! :)"** and then `LED on` / `LED off`
   scrolling. The toolchain works!

> 💡 If upload fails: unplug/replug, try a different USB-C cable. The port is
> auto-detected, but you can force it in `platformio.ini` with `upload_port`.
> If it's really stubborn: hold **BOOT**, tap **RESET**, release **BOOT**, then upload.

## 5. (Same session) Wire the LED

Now make the blink real. On a breadboard:

```text
  XIAO D10 --> [220 ohm resistor] --> LED long leg (+)
                                      LED short leg (-) --> XIAO GND
```

- The **resistor** protects the LED — never skip it.
- LED **long leg = +**, **short leg = -**.
- The LED should blink once per second, matching `LED on` / `LED off`. ✅

That's Milestone 1 done — the board is alive and Valentin built his first circuit.
Next up: **Milestone 2 — light up the NeoPixel ring.**

---

### How the project is organized

```text
VURP/                <- open THIS folder in VS Code
  platformio.ini     <- project settings (board, libraries)
  src/
    main.cpp         <- the program that runs on the rover
  README.md          <- the master plan
  SETUP.md           <- this guide
```

As we add milestones, the code in `src/main.cpp` grows, and any libraries we need
get listed in `platformio.ini` (PlatformIO downloads them automatically).

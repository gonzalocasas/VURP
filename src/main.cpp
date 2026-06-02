/*
 * VURP — Milestone 2: "Light up the NeoPixel ring"
 * -------------------------------------------------
 * A ring of color LEDs (NeoPixels / WS2812) doing a smooth, rotating rainbow.
 * Each LED can be ANY color, all from a single data wire. Magic. ✨
 *
 * Board: Seeed XIAO ESP32-C3   |   Tools: VS Code + PlatformIO
 *
 * WIRING (3 wires from the ring to the XIAO):
 *
 *     Ring  VCC / "+"          -->  XIAO  3V3 pin   (see note below)
 *     Ring  GND / "-"          -->  XIAO  GND pin
 *     Ring  DIN (data INPUT)   -->  XIAO  D2   (through a ~330 ohm resistor if you have one)
 *
 *   - Use the ring's "DIN" / arrow-IN side, NOT "DOUT". Data flows in the arrow's direction.
 *   - Why 3V3 and not 5V? The XIAO sends data at 3.3V. Powering the ring at 3.3V too
 *     makes the data level match perfectly -> reliable, no flicker, no level shifter.
 *     (The XIAO DOES have a 5V pin for brighter setups, but then 3.3V data is marginal.)
 *   - We keep brightness LOW so the 3V3 pin can safely power it.
 *
 * >>> IMPORTANT: set NUM_PIXELS to match YOUR ring (count the LEDs on it). <<<
 */

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN     D2     // data wire goes here
#define NUM_PIXELS  16     // <-- CHANGE THIS to how many LEDs are on your ring (8/12/16/24...)
#define BRIGHTNESS  40     // 0 = off, 255 = blinding. 40 is gentle + USB-safe.

// Create our ring. NEO_GRB + NEO_KHZ800 is the setting for almost all WS2812 rings.
Adafruit_NeoPixel ring(NUM_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Hello from VURP! Lighting up the ring... 🌈");

  ring.begin();                 // start talking to the ring
  ring.setBrightness(BRIGHTNESS);
  ring.show();                  // push the (currently all-off) colors out
}

void loop() {
  // "hueOffset" slowly climbs, which rotates the rainbow around the ring.
  // 'static' means it keeps its value between loops (it remembers).
  static uint16_t hueOffset = 0;

  for (int i = 0; i < NUM_PIXELS; i++) {
    // Spread the full color wheel (0..65535) evenly across all the pixels,
    // then add the moving offset so the pattern spins.
    uint16_t hue = hueOffset + (i * 65536L / NUM_PIXELS);

    // ColorHSV picks a color from the rainbow; gamma32 makes it look natural to the eye.
    ring.setPixelColor(i, ring.gamma32(ring.ColorHSV(hue)));
  }

  ring.show();          // send all the colors to the ring at once
  hueOffset += 256;     // advance the rainbow a little for the next frame
  delay(20);            // ~50 frames per second = smooth motion
}

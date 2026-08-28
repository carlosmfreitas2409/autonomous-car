#include "strip.h"

#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

#include "config.h"

namespace strip {

namespace {

Adafruit_NeoPixel pixels(config::STRIP_LED_COUNT, config::PIN_STRIP_DATA,
                         NEO_BRG + NEO_KHZ800);

Effect effect = Effect::OFF;
Effect rendered_effect = Effect::OFF;

uint32_t last_frame_ms = 0;
uint32_t last_show_ms = 0;

bool strobe_on = false;
uint32_t strobe_phase_started_ms = 0;
uint16_t rainbow_hue = 0;
uint16_t pulse_phase = 0;

uint32_t solid_color(Effect e) {
  switch (e) {
    case Effect::RED:   return Adafruit_NeoPixel::Color(255, 0, 0);
    case Effect::GREEN: return Adafruit_NeoPixel::Color(0, 255, 0);
    case Effect::BLUE:  return Adafruit_NeoPixel::Color(0, 0, 255);
    case Effect::CYAN:  return Adafruit_NeoPixel::Color(0, 255, 255);
    default:            return 0;
  }
}

void render(uint32_t now_ms) {
  switch (effect) {
    case Effect::OFF:
      if (rendered_effect == effect) return;
      pixels.clear();
      break;

    case Effect::RED:
    case Effect::GREEN:
    case Effect::BLUE:
    case Effect::CYAN:
      if (rendered_effect == effect &&
          (now_ms - last_show_ms) < config::STRIP_REFRESH_MS) {
        return;
      }
      pixels.fill(solid_color(effect));
      break;

    case Effect::STROBE: {
      if (rendered_effect != effect) {
        strobe_on = true;
      } else {
        const uint16_t phase_dur =
            strobe_on ? config::STROBE_ON_MS : config::STROBE_OFF_MS;
        if ((now_ms - strobe_phase_started_ms) < phase_dur) return;
        strobe_on = !strobe_on;
      }
      strobe_phase_started_ms = now_ms;
      pixels.fill(strobe_on ? Adafruit_NeoPixel::Color(255, 255, 255) : 0);
      break;
    }

    case Effect::RED_PULSE: {
      pulse_phase += config::PULSE_STEP;
      const uint16_t tri =
          (pulse_phase < 32768u) ? pulse_phase : (uint16_t)(65535u - pulse_phase);
      const uint8_t value = (uint8_t)(25u + (((uint32_t)(tri >> 7) * 230u) / 255u));
      pixels.fill(
          Adafruit_NeoPixel::gamma32(Adafruit_NeoPixel::ColorHSV(0, 255, value)));
      break;
    }

    case Effect::RED_STROBE: {
      if (rendered_effect != effect) {
        strobe_on = true;
      } else {
        const uint16_t phase_dur =
            strobe_on ? config::STROBE_ON_MS : config::STROBE_OFF_MS;
        if ((now_ms - strobe_phase_started_ms) < phase_dur) return;
        strobe_on = !strobe_on;
      }
      strobe_phase_started_ms = now_ms;
      pixels.fill(strobe_on ? Adafruit_NeoPixel::Color(255, 0, 0) : 0);
      break;
    }

    case Effect::RAINBOW: {
      const uint16_t count = pixels.numPixels();
      for (uint16_t i = 0; i < count; ++i) {
        const uint16_t hue = rainbow_hue + (uint16_t)(((uint32_t)i * 65536ul) / count);
        pixels.setPixelColor(
            i, Adafruit_NeoPixel::gamma32(Adafruit_NeoPixel::ColorHSV(hue)));
      }
      rainbow_hue += config::RAINBOW_HUE_STEP;
      break;
    }
  }

  rendered_effect = effect;
  last_show_ms = now_ms;
  pixels.show();
}

}

void init() {
  pixels.begin();
  pixels.setBrightness(config::STRIP_BRIGHTNESS);
  pixels.clear();
  pixels.show();

  effect = Effect::OFF;
  rendered_effect = Effect::OFF;
}

void set_effect(uint8_t id) {
  effect = (id > (uint8_t)Effect::RED_STROBE) ? Effect::OFF
                                              : static_cast<Effect>(id);
}

void off() {
  effect = Effect::OFF;
}

void update(uint32_t now_ms) {
  if ((now_ms - last_frame_ms) < config::STRIP_FRAME_MS) return;
  last_frame_ms = now_ms;

  render(now_ms);
}

}

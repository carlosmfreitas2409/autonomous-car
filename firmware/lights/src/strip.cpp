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
uint16_t sweep_head = 0;
uint32_t sweep_step_started_ms = 0;
uint8_t police_step = 0;
uint32_t police_phase_started_ms = 0;

// SAE J3134's ADS-marker turquoise (CIE 1931 x 0.012-0.20, y 0.32-0.495), the
// colour of Mercedes' DRIVE PILOT marker lights. Full blue beside full green
// lands at y ~0.21 on a WS2812, a sky blue; blue at a third of green sits in
// the middle of the turquoise box (x 0.15, y 0.37) for datasheet primaries.
constexpr uint8_t TURQUOISE_BLUE = 85;

// Pure yellow is hue 10923; a WS2812's green outshines its red, so a little
// toward red keeps it from reading lime.
constexpr uint16_t YELLOW_HUE = 9000;

uint32_t solid_color(Effect e) {
  switch (e) {
    case Effect::RED:   return Adafruit_NeoPixel::Color(255, 0, 0);
    case Effect::GREEN: return Adafruit_NeoPixel::Color(0, 255, 0);
    case Effect::BLUE:  return Adafruit_NeoPixel::Color(0, 0, 255);
    case Effect::CYAN:  return Adafruit_NeoPixel::Color(0, 255, TURQUOISE_BLUE);
    default:            return 0;
  }
}

void fill_range(uint16_t start, uint16_t end, uint32_t color) {
  for (uint16_t i = start; i < end; ++i) {
    pixels.setPixelColor(i, color);
  }
}

void fill_left(uint32_t color) {
  const uint16_t count = pixels.numPixels();
  uint16_t front = config::STRIP_FRONT_COUNT;
  if (front > count) front = count;
  fill_range(0, front, color);
  // Last trio is the rear; only the final pixel sits on the left half.
  if (count >= config::STRIP_REAR_COUNT && (count - 1) >= front) {
    pixels.setPixelColor(count - 1, color);
  }
}

void fill_right(uint32_t color) {
  const uint16_t count = pixels.numPixels();
  if (count < 2) return;
  uint16_t start = config::STRIP_FRONT_COUNT;
  if (start >= (uint16_t)(count - 1)) return;
  fill_range(start, (uint16_t)(count - 1), color);
}

// How far LED `i` is from the rear centre, walking round the strip either
// way. The strip is a loop round the car - one side, across to the other,
// down it, and the rear trio last - so from the rear trio's middle LED both
// ways lead up the two sides at once, and they meet at the front.
uint16_t outline_distance(uint16_t i) {
  const uint16_t count = pixels.numPixels();
  const uint16_t rear_centre = count - config::STRIP_REAR_COUNT + config::STRIP_REAR_COUNT / 2;
  const uint16_t one_way = (i > rear_centre) ? i - rear_centre : rear_centre - i;
  const uint16_t other_way = count - one_way;
  return (one_way < other_way) ? one_way : other_way;
}

uint16_t police_step_ms(uint8_t step) {
  switch (step) {
    case 0:
    case 2:
    case 4:
    case 6:
      return config::POLICE_FLASH_MS;
    case 1:
    case 5:
      return config::POLICE_GAP_MS;
    default:
      return config::POLICE_PAUSE_MS;
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

    case Effect::BLUE_BREATHE: {
      // RED_PULSE's triangle, blue and slower: a dock waiting for someone.
      pulse_phase += config::BREATHE_STEP;
      const uint16_t tri =
          (pulse_phase < 32768u) ? pulse_phase : (uint16_t)(65535u - pulse_phase);
      const uint8_t value = (uint8_t)(25u + (((uint32_t)(tri >> 7) * 230u) / 255u));
      pixels.fill(Adafruit_NeoPixel::gamma32(
          Adafruit_NeoPixel::ColorHSV(43690u, 255, value)));
      break;
    }

    case Effect::YELLOW_PULSE: {
      // RED_PULSE's triangle in yellow: a person is driving.
      pulse_phase += config::PULSE_STEP;
      const uint16_t tri =
          (pulse_phase < 32768u) ? pulse_phase : (uint16_t)(65535u - pulse_phase);
      const uint8_t value = (uint8_t)(25u + (((uint32_t)(tri >> 7) * 230u) / 255u));
      pixels.fill(Adafruit_NeoPixel::gamma32(
          Adafruit_NeoPixel::ColorHSV(YELLOW_HUE, 255, value)));
      break;
    }

    case Effect::CYAN_SWEEP: {
      // A turquoise arrow: from the rear centre up both sides together,
      // meeting at the front, again and again - about to move forward.
      if (rendered_effect == effect &&
          (now_ms - sweep_step_started_ms) < config::SWEEP_STEP_MS) {
        return;
      }
      sweep_step_started_ms = now_ms;
      const uint16_t count = pixels.numPixels();
      if (count <= config::STRIP_REAR_COUNT) return;
      const uint16_t tip = count / 2;  // the farthest any LED is from the rear centre
      const uint16_t cycle = tip + config::SWEEP_TAIL + config::SWEEP_PAUSE_STEPS;
      sweep_head = (rendered_effect == effect) ? (uint16_t)((sweep_head + 1) % cycle) : 0;
      pixels.clear();
      for (uint16_t i = 0; i < count; ++i) {
        const uint16_t d = outline_distance(i);
        if (d > sweep_head || sweep_head - d >= config::SWEEP_TAIL) continue;
        // Gamma on the brightness only: gamma per channel turns blue 85 into
        // ~15 and the turquoise into green.
        const uint8_t level = Adafruit_NeoPixel::gamma8(
            (uint8_t)(255u - (sweep_head - d) * (255u / config::SWEEP_TAIL)));
        pixels.setPixelColor(i, Adafruit_NeoPixel::Color(
                                    0, level, (uint8_t)((uint16_t)level * TURQUOISE_BLUE / 255u)));
      }
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

    case Effect::POLICE: {
      if (rendered_effect != effect) {
        police_step = 0;
      } else if ((now_ms - police_phase_started_ms) < police_step_ms(police_step)) {
        return;
      } else {
        police_step = (uint8_t)((police_step + 1) % 8);
      }
      police_phase_started_ms = now_ms;

      pixels.clear();
      const bool left_on = (police_step == 0 || police_step == 2);
      const bool right_on = (police_step == 4 || police_step == 6);
      if (left_on) {
        fill_left(Adafruit_NeoPixel::Color(255, 0, 0));
      }
      if (right_on) {
        fill_right(Adafruit_NeoPixel::Color(0, 0, 255));
      }
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
  effect = (id > (uint8_t)Effect::YELLOW_PULSE) ? Effect::OFF
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

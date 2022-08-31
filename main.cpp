#include <math.h>
#include <string>

#include "hardware/pwm.h"
#include "hardware/pll.h"
#include "hardware/clocks.h"

#include "breakout_dotmatrix.hpp"
#include "time.h"

// This allows underclocking CLK_REF to allow a slower sys clk
#define TIME_MULT 4

using namespace pimoroni;

int main() {
  I2C i2c(BOARD::BREAKOUT_GARDEN);
  BreakoutDotMatrix display(&i2c);

  bool led_toggle = false;

  pwm_config cfg = pwm_get_default_config();
  pwm_init(pwm_gpio_to_slice_num(PICO_DEFAULT_LED_PIN), &cfg, true);
  pwm_set_wrap(pwm_gpio_to_slice_num(PICO_DEFAULT_LED_PIN), 255);
  pwm_set_gpio_level(PICO_DEFAULT_LED_PIN, 0);
  gpio_set_function(PICO_DEFAULT_LED_PIN, GPIO_FUNC_PWM);

  display.init();
  display.show();

  while(true) {
    absolute_time_t at = get_absolute_time();
    uint64_t t = to_us_since_boot(at) * TIME_MULT;

    std::string count;
    if (t < 100000000) {
        count = std::to_string((t / 1000000) % 100);
    }
    else if (t < 600000000) {
        display.set_decimal(false, true);
        count = std::to_string(t / 6000000);
    }
    else {
        display.set_decimal(false, false);
        count = std::to_string(t / 60000000);
    }

    uint16_t left, right;
    if(count.length() == 1) {
        left = '0';
        right = count[0];
    }
    else {
        left = count[0];
        right = count[1];
    }

    display.set_character(0, left);
    display.set_character(5, right);
    display.show();

    pwm_set_gpio_level(PICO_DEFAULT_LED_PIN, led_toggle ? 64 : 0);
    led_toggle = !led_toggle;

    at = get_absolute_time();
    t = to_us_since_boot(at) * TIME_MULT;
    uint64_t w = t / 500000;
    w = (w + 1) * 500000 - t;
    
    sleep_us(w / TIME_MULT);
  }
}
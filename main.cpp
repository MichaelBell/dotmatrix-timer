#include <math.h>
#include <string>

#include "time.h"

#include "hardware/pwm.h"
#include "hardware/adc.h"

#include "st7789.hpp"
#include "breakout_dotmatrix.hpp"

// This allows underclocking CLK_REF to allow a slower sys clk
#define TIME_MULT 1

using namespace pimoroni;

#define POWER_DOWN_BUTTON 18
#define POWER_UP_BUTTON 19

static uint8_t power = 10;

void gpio_callback(uint gpio, uint32_t events)
{
    if (gpio == POWER_DOWN_BUTTON && power > 5) power -= 1;
    else if (gpio == POWER_UP_BUTTON && power < 35) power += 1;
}

ST7789 st7789(240, 240, ROTATE_0, false, {
spi0,
1,
2,
3,
PIN_UNUSED,
6,
0
});
PicoGraphics_PenP4 graphics(st7789.width, st7789.height, nullptr);

int main() {
  I2C i2c(BOARD::BREAKOUT_GARDEN);
  BreakoutDotMatrix display(&i2c);
  Pen BG = graphics.create_pen(0, 0, 0);
  Pen WHITE = graphics.create_pen(255, 255, 255);  

  gpio_set_function(POWER_DOWN_BUTTON, GPIO_FUNC_SIO);
  gpio_set_function(POWER_UP_BUTTON, GPIO_FUNC_SIO);
  gpio_set_dir(POWER_DOWN_BUTTON, GPIO_IN);
  gpio_set_dir(POWER_UP_BUTTON, GPIO_IN);
  gpio_pull_up(POWER_DOWN_BUTTON);
  gpio_pull_up(POWER_UP_BUTTON);

  gpio_set_irq_enabled_with_callback(POWER_DOWN_BUTTON, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
  gpio_set_irq_enabled(POWER_UP_BUTTON, GPIO_IRQ_EDGE_RISE, true);

  const float adc_conversion_factor = 3.3f / (1 << 12);
  adc_init();
  adc_gpio_init(29);
  adc_select_input(3);

  pwm_config cfg = pwm_get_default_config();
  pwm_init(pwm_gpio_to_slice_num(PICO_DEFAULT_LED_PIN), &cfg, true);
  pwm_set_wrap(pwm_gpio_to_slice_num(PICO_DEFAULT_LED_PIN), 255);
  pwm_set_gpio_level(PICO_DEFAULT_LED_PIN, 255);
  gpio_set_function(PICO_DEFAULT_LED_PIN, GPIO_FUNC_PWM);

  st7789.set_backlight(50 + power * 5);

  display.init();
  display.set_power(power);
  display.show();

  while(true) {
    absolute_time_t at = get_absolute_time();
    uint64_t t = to_us_since_boot(at) * TIME_MULT;

    std::string count;
    uint16_t left, right;

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

    if(count.length() == 1) {
        left = '0';
        right = count[0];
    }
    else {
        left = count[0];
        right = count[1];
    }

    float voltage = adc_read() * adc_conversion_factor;
    float sys_voltage = voltage * 3;

#if 0    
    left = '0' + int(sys_voltage);
    right = '0' + (int(sys_voltage * 10) % 10);
    display.set_decimal(false, true);
#endif

    display.set_power(power);
    st7789.set_backlight(50 + power * 5);

    display.set_character(0, left);
    display.set_character(5, right);
    display.show();

    pwm_set_gpio_level(PICO_DEFAULT_LED_PIN, ((t / 500000) & 1) ? 64 : 0);

    graphics.set_pen(BG);
    graphics.clear();
    graphics.set_pen(WHITE);

    char buffer[32];
    graphics.text("Steam!", Point(0, 0), 240, 4);
    sprintf(buffer, "Power: %d", power);
    graphics.text(buffer, Point(0, 32), 240, 4);

    sprintf(buffer, "Volts: %.2f", sys_voltage);
    graphics.text(buffer, Point(0, 64), 240, 4);
    
    st7789.update(&graphics);

#if 0
    at = get_absolute_time();
    t = to_us_since_boot(at) * TIME_MULT;
    uint64_t w = t / 250000;
    w = (w + 1) * 250000 - t;
    
    sleep_us(w / TIME_MULT);
#endif
  }
}
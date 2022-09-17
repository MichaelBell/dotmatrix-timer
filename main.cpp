#include <math.h>
#include <string>

#include "time.h"

#include "hardware/pwm.h"
#include "hardware/adc.h"

#include "st7789.hpp"

// This allows underclocking CLK_REF to allow a slower sys clk
#define TIME_MULT 1

using namespace pimoroni;

#define POWER_DOWN_BUTTON 18
#define POWER_UP_BUTTON 19

static uint8_t power = 8;
static uint64_t last_edge_us = 0;

void gpio_callback(uint gpio, uint32_t events)
{
    absolute_time_t at = get_absolute_time();
    uint64_t t = to_us_since_boot(at) * TIME_MULT;
    if (t < last_edge_us + 10000) return;
    last_edge_us = t;

    if (gpio == POWER_DOWN_BUTTON && power > 1) power -= 1;
    else if (gpio == POWER_UP_BUTTON && power < 35) power += 1;
}

ST7789 st7789(240, 240, ROTATE_0, false, {
spi0,
5,
6,
7,
PIN_UNUSED,
8,
PIN_UNUSED
});
PicoGraphics_PenP4 graphics(st7789.width, st7789.height, nullptr);

#define BL_PIN 4

int main() {
  Pen BLACK = graphics.create_pen(0, 0, 0);
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
  pwm_set_gpio_level(PICO_DEFAULT_LED_PIN, 64);
  gpio_set_function(PICO_DEFAULT_LED_PIN, GPIO_FUNC_PWM);

  cfg = pwm_get_default_config();
  pwm_init(pwm_gpio_to_slice_num(BL_PIN), &cfg, true);
  pwm_set_wrap(pwm_gpio_to_slice_num(BL_PIN), 63);
  pwm_set_gpio_level(BL_PIN, power);
  gpio_set_function(BL_PIN, GPIO_FUNC_PWM);

  graphics.set_pen(WHITE);
  graphics.clear();
  graphics.set_pen(BLACK);
  graphics.text("Power:", Point(8, 0), 240, 4);
  graphics.text("Volts:", Point(8, 32), 240, 4);
  graphics.text("Time:", Point(8, 64), 240, 4);

  st7789.update(&graphics);
  
  st7789.set_bounds(Rect(0, 96, 240, 144));
  st7789.write_image_data((void*)0x13100000);

  st7789.set_bounds(Rect(135, 0, 110, 96));
  graphics.bounds.w = 105;
  graphics.bounds.h = 96;

  while(true) {
    absolute_time_t at = get_absolute_time();
    uint64_t t = to_us_since_boot(at) * TIME_MULT;

    std::string count;
    uint16_t left, right;

    float voltage = adc_read() * adc_conversion_factor;
    float sys_voltage = voltage * 3;

    pwm_set_gpio_level(PICO_DEFAULT_LED_PIN, ((t / 1000000) & 1) ? 64 : 0);
    pwm_set_gpio_level(BL_PIN, power);

    graphics.set_pen(WHITE);
    graphics.clear();
    graphics.set_pen(BLACK);

    char buffer[32];
    sprintf(buffer, "%d", power);
    graphics.text(buffer, Point(0, 0), 105, 4);

    sprintf(buffer, "%.2f", sys_voltage);
    graphics.text(buffer, Point(0, 32), 105, 4);

    uint32_t secs = t / 1000000;
    sprintf(buffer, "%d:%02d", secs / 60, secs % 60);
    graphics.text(buffer, Point(0, 64), 105, 4);
    
    st7789.update(&graphics);

#if 1
    at = get_absolute_time();
    t = to_us_since_boot(at) * TIME_MULT;
    uint64_t w = t / 1000000;
    w = (w + 1) * 1000000 - t;
    
    sleep_us(w / TIME_MULT);
#endif
  }
}
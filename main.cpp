#include <math.h>
#include <string>

#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/structs/syscfg.h"

#include "breakout_dotmatrix.hpp"
#include "time.h"

// This allows underclocking CLK_REF to allow a slower sys clk
#define TIME_MULT 4

using namespace pimoroni;

static uint16_t i2c_data_cmds[512];

#define I2C_CMD_TRANSFER_LEN 16
const uint16_t __in_flash("dma-data") dm_i2c_cmd_buf[] __attribute__((aligned(2048))) = {
  0x40e, 62, 81, 73, 69, 574, 0x401, 14, 17, 25, 21, 19, 17, 526, 0x40c, 0x201,
  0x40e, 62, 81, 73, 69, 574, 0x401, 4, 6, 4, 4, 4, 4, 526, 0x40c, 0x201,
  0x40e, 62, 81, 73, 69, 574, 0x401, 14, 17, 16, 8, 4, 2, 543, 0x40c, 0x201,
  0x40e, 62, 81, 73, 69, 574, 0x401, 31, 8, 4, 8, 16, 17, 526, 0x40c, 0x201,
  0x40e, 62, 81, 73, 69, 574, 0x401, 8, 12, 10, 9, 31, 8, 520, 0x40c, 0x201,
  0x40e, 62, 81, 73, 69, 574, 0x401, 31, 1, 15, 16, 16, 17, 526, 0x40c, 0x201,
  0x40e, 62, 81, 73, 69, 574, 0x401, 12, 2, 1, 15, 17, 17, 526, 0x40c, 0x201,
  0x40e, 62, 81, 73, 69, 574, 0x401, 31, 16, 8, 4, 2, 2, 514, 0x40c, 0x201,
  0x40e, 62, 81, 73, 69, 574, 0x401, 14, 17, 17, 14, 17, 17, 526, 0x40c, 0x201,
  0x40e, 62, 81, 73, 69, 574, 0x401, 14, 17, 17, 30, 16, 8, 518, 0x40c, 0x201,
  0x40e, 0, 66, 127, 64, 512, 0x401, 14, 17, 25, 21, 19, 17, 526, 0x40c, 0x201,
  0x40e, 0, 66, 127, 64, 512, 0x401, 4, 6, 4, 4, 4, 4, 526, 0x40c, 0x201,
  0x40e, 0, 66, 127, 64, 512, 0x401, 14, 17, 16, 8, 4, 2, 543, 0x40c, 0x201,
  0x40e, 0, 66, 127, 64, 512, 0x401, 31, 8, 4, 8, 16, 17, 526, 0x40c, 0x201,
  0x40e, 0, 66, 127, 64, 512, 0x401, 8, 12, 10, 9, 31, 8, 520, 0x40c, 0x201,
  0x40e, 0, 66, 127, 64, 512, 0x401, 31, 1, 15, 16, 16, 17, 526, 0x40c, 0x201,
  0x40e, 0, 66, 127, 64, 512, 0x401, 12, 2, 1, 15, 17, 17, 526, 0x40c, 0x201,
  0x40e, 0, 66, 127, 64, 512, 0x401, 31, 16, 8, 4, 2, 2, 514, 0x40c, 0x201,
  0x40e, 0, 66, 127, 64, 512, 0x401, 14, 17, 17, 14, 17, 17, 526, 0x40c, 0x201,
  0x40e, 0, 66, 127, 64, 512, 0x401, 14, 17, 17, 30, 16, 8, 518, 0x40c, 0x201,
  0x40e, 66, 97, 81, 73, 582, 0x401, 14, 17, 25, 21, 19, 17, 526, 0x40c, 0x201,
  0x40e, 66, 97, 81, 73, 582, 0x401, 4, 6, 4, 4, 4, 4, 526, 0x40c, 0x201,
  0x40e, 66, 97, 81, 73, 582, 0x401, 14, 17, 16, 8, 4, 2, 543, 0x40c, 0x201,
  0x40e, 66, 97, 81, 73, 582, 0x401, 31, 8, 4, 8, 16, 17, 526, 0x40c, 0x201,
  0x40e, 66, 97, 81, 73, 582, 0x401, 8, 12, 10, 9, 31, 8, 520, 0x40c, 0x201,
  0x40e, 66, 97, 81, 73, 582, 0x401, 31, 1, 15, 16, 16, 17, 526, 0x40c, 0x201,
  0x40e, 66, 97, 81, 73, 582, 0x401, 12, 2, 1, 15, 17, 17, 526, 0x40c, 0x201,
  0x40e, 66, 97, 81, 73, 582, 0x401, 31, 16, 8, 4, 2, 2, 514, 0x40c, 0x201,
  0x40e, 66, 97, 81, 73, 582, 0x401, 14, 17, 17, 14, 17, 17, 526, 0x40c, 0x201,
  0x40e, 66, 97, 81, 73, 582, 0x401, 14, 17, 17, 30, 16, 8, 518, 0x40c, 0x201,
  0x40e, 33, 65, 69, 75, 561, 0x401, 14, 17, 25, 21, 19, 17, 526, 0x40c, 0x201,
  0x40e, 33, 65, 69, 75, 561, 0x401, 4, 6, 4, 4, 4, 4, 526, 0x40c, 0x201,
  0x40e, 33, 65, 69, 75, 561, 0x401, 14, 17, 16, 8, 4, 2, 543, 0x40c, 0x201,
  0x40e, 33, 65, 69, 75, 561, 0x401, 31, 8, 4, 8, 16, 17, 526, 0x40c, 0x201,
  0x40e, 33, 65, 69, 75, 561, 0x401, 8, 12, 10, 9, 31, 8, 520, 0x40c, 0x201,
  0x40e, 33, 65, 69, 75, 561, 0x401, 31, 1, 15, 16, 16, 17, 526, 0x40c, 0x201,
  0x40e, 33, 65, 69, 75, 561, 0x401, 12, 2, 1, 15, 17, 17, 526, 0x40c, 0x201,
  0x40e, 33, 65, 69, 75, 561, 0x401, 31, 16, 8, 4, 2, 2, 514, 0x40c, 0x201,
  0x40e, 33, 65, 69, 75, 561, 0x401, 14, 17, 17, 14, 17, 17, 526, 0x40c, 0x201,
  0x40e, 33, 65, 69, 75, 561, 0x401, 14, 17, 17, 30, 16, 8, 518, 0x40c, 0x201,
  0x40e, 24, 20, 18, 127, 528, 0x401, 14, 17, 25, 21, 19, 17, 526, 0x40c, 0x201,
  0x40e, 24, 20, 18, 127, 528, 0x401, 4, 6, 4, 4, 4, 4, 526, 0x40c, 0x201,
  0x40e, 24, 20, 18, 127, 528, 0x401, 14, 17, 16, 8, 4, 2, 543, 0x40c, 0x201,
  0x40e, 24, 20, 18, 127, 528, 0x401, 31, 8, 4, 8, 16, 17, 526, 0x40c, 0x201,
  0x40e, 24, 20, 18, 127, 528, 0x401, 8, 12, 10, 9, 31, 8, 520, 0x40c, 0x201,
  0x40e, 24, 20, 18, 127, 528, 0x401, 31, 1, 15, 16, 16, 17, 526, 0x40c, 0x201,
  0x40e, 24, 20, 18, 127, 528, 0x401, 12, 2, 1, 15, 17, 17, 526, 0x40c, 0x201,
  0x40e, 24, 20, 18, 127, 528, 0x401, 31, 16, 8, 4, 2, 2, 514, 0x40c, 0x201,
  0x40e, 24, 20, 18, 127, 528, 0x401, 14, 17, 17, 14, 17, 17, 526, 0x40c, 0x201,
  0x40e, 24, 20, 18, 127, 528, 0x401, 14, 17, 17, 30, 16, 8, 518, 0x40c, 0x201,
  0x40e, 39, 69, 69, 69, 569, 0x401, 14, 17, 25, 21, 19, 17, 526, 0x40c, 0x201,
  0x40e, 39, 69, 69, 69, 569, 0x401, 4, 6, 4, 4, 4, 4, 526, 0x40c, 0x201,
  0x40e, 39, 69, 69, 69, 569, 0x401, 14, 17, 16, 8, 4, 2, 543, 0x40c, 0x201,
  0x40e, 39, 69, 69, 69, 569, 0x401, 31, 8, 4, 8, 16, 17, 526, 0x40c, 0x201,
  0x40e, 39, 69, 69, 69, 569, 0x401, 8, 12, 10, 9, 31, 8, 520, 0x40c, 0x201,
  0x40e, 39, 69, 69, 69, 569, 0x401, 31, 1, 15, 16, 16, 17, 526, 0x40c, 0x201,
  0x40e, 39, 69, 69, 69, 569, 0x401, 12, 2, 1, 15, 17, 17, 526, 0x40c, 0x201,
  0x40e, 39, 69, 69, 69, 569, 0x401, 31, 16, 8, 4, 2, 2, 514, 0x40c, 0x201,
  0x40e, 39, 69, 69, 69, 569, 0x401, 14, 17, 17, 14, 17, 17, 526, 0x40c, 0x201,
  0x40e, 39, 69, 69, 69, 569, 0x401, 14, 17, 17, 30, 16, 8, 518, 0x40c, 0x201,
  0x40e, 60, 74, 73, 73, 560, 0x401, 14, 17, 25, 21, 19, 17, 526, 0x40c, 0x201,
  0x40e, 60, 74, 73, 73, 560, 0x401, 4, 6, 4, 4, 4, 4, 526, 0x40c, 0x201,
  0x40e, 60, 74, 73, 73, 560, 0x401, 14, 17, 16, 8, 4, 2, 543, 0x40c, 0x201,
  0x40e, 60, 74, 73, 73, 560, 0x401, 31, 8, 4, 8, 16, 17, 526, 0x40c, 0x201,
  0x40e, 60, 74, 73, 73, 560, 0x401, 8, 12, 10, 9, 31, 8, 520, 0x40c, 0x201,
  0x40e, 60, 74, 73, 73, 560, 0x401, 31, 1, 15, 16, 16, 17, 526, 0x40c, 0x201,
  0x40e, 60, 74, 73, 73, 560, 0x401, 12, 2, 1, 15, 17, 17, 526, 0x40c, 0x201,
  0x40e, 60, 74, 73, 73, 560, 0x401, 31, 16, 8, 4, 2, 2, 514, 0x40c, 0x201,
  0x40e, 60, 74, 73, 73, 560, 0x401, 14, 17, 17, 14, 17, 17, 526, 0x40c, 0x201,
  0x40e, 60, 74, 73, 73, 560, 0x401, 14, 17, 17, 30, 16, 8, 518, 0x40c, 0x201,
  0x40e, 1, 113, 9, 5, 515, 0x401, 14, 17, 25, 21, 19, 17, 526, 0x40c, 0x201,
  0x40e, 1, 113, 9, 5, 515, 0x401, 4, 6, 4, 4, 4, 4, 526, 0x40c, 0x201,
  0x40e, 1, 113, 9, 5, 515, 0x401, 14, 17, 16, 8, 4, 2, 543, 0x40c, 0x201,
  0x40e, 1, 113, 9, 5, 515, 0x401, 31, 8, 4, 8, 16, 17, 526, 0x40c, 0x201,
  0x40e, 1, 113, 9, 5, 515, 0x401, 8, 12, 10, 9, 31, 8, 520, 0x40c, 0x201,
  0x40e, 1, 113, 9, 5, 515, 0x401, 31, 1, 15, 16, 16, 17, 526, 0x40c, 0x201,
  0x40e, 1, 113, 9, 5, 515, 0x401, 12, 2, 1, 15, 17, 17, 526, 0x40c, 0x201,
  0x40e, 1, 113, 9, 5, 515, 0x401, 31, 16, 8, 4, 2, 2, 514, 0x40c, 0x201,
  0x40e, 1, 113, 9, 5, 515, 0x401, 14, 17, 17, 14, 17, 17, 526, 0x40c, 0x201,
  0x40e, 1, 113, 9, 5, 515, 0x401, 14, 17, 17, 30, 16, 8, 518, 0x40c, 0x201,
  0x40e, 54, 73, 73, 73, 566, 0x401, 14, 17, 25, 21, 19, 17, 526, 0x40c, 0x201,
  0x40e, 54, 73, 73, 73, 566, 0x401, 4, 6, 4, 4, 4, 4, 526, 0x40c, 0x201,
  0x40e, 54, 73, 73, 73, 566, 0x401, 14, 17, 16, 8, 4, 2, 543, 0x40c, 0x201,
  0x40e, 54, 73, 73, 73, 566, 0x401, 31, 8, 4, 8, 16, 17, 526, 0x40c, 0x201,
  0x40e, 54, 73, 73, 73, 566, 0x401, 8, 12, 10, 9, 31, 8, 520, 0x40c, 0x201,
  0x40e, 54, 73, 73, 73, 566, 0x401, 31, 1, 15, 16, 16, 17, 526, 0x40c, 0x201,
  0x40e, 54, 73, 73, 73, 566, 0x401, 12, 2, 1, 15, 17, 17, 526, 0x40c, 0x201,
  0x40e, 54, 73, 73, 73, 566, 0x401, 31, 16, 8, 4, 2, 2, 514, 0x40c, 0x201,
  0x40e, 54, 73, 73, 73, 566, 0x401, 14, 17, 17, 14, 17, 17, 526, 0x40c, 0x201,
  0x40e, 54, 73, 73, 73, 566, 0x401, 14, 17, 17, 30, 16, 8, 518, 0x40c, 0x201,
  0x40e, 6, 73, 73, 41, 542, 0x401, 14, 17, 25, 21, 19, 17, 526, 0x40c, 0x201,
  0x40e, 6, 73, 73, 41, 542, 0x401, 4, 6, 4, 4, 4, 4, 526, 0x40c, 0x201,
  0x40e, 6, 73, 73, 41, 542, 0x401, 14, 17, 16, 8, 4, 2, 543, 0x40c, 0x201,
  0x40e, 6, 73, 73, 41, 542, 0x401, 31, 8, 4, 8, 16, 17, 526, 0x40c, 0x201,
  0x40e, 6, 73, 73, 41, 542, 0x401, 8, 12, 10, 9, 31, 8, 520, 0x40c, 0x201,
  0x40e, 6, 73, 73, 41, 542, 0x401, 31, 1, 15, 16, 16, 17, 526, 0x40c, 0x201,
  0x40e, 6, 73, 73, 41, 542, 0x401, 12, 2, 1, 15, 17, 17, 526, 0x40c, 0x201,
  0x40e, 6, 73, 73, 41, 542, 0x401, 31, 16, 8, 4, 2, 2, 514, 0x40c, 0x201,
  0x40e, 6, 73, 73, 41, 542, 0x401, 14, 17, 17, 14, 17, 17, 526, 0x40c, 0x201,
  0x40e, 6, 73, 73, 41, 542, 0x401, 14, 17, 17, 30, 16, 8, 518, 0x40c, 0x201,
};
const uint16_t __in_flash("dma-data2") pwm_level[32] = {
    0, 4, 8, 12, 16, 20, 24, 28,
    32, 36, 40, 44, 48, 52, 56, 60,
    64, 60, 56, 48, 44, 40, 36, 32,
    28, 24, 20, 16, 12, 8, 4, 0
};
const void* __in_flash("dma-data3") chain_cmd[1] = { &pwm_level[0] };

uint16_t* write_i2c_reg_value(uint16_t* data_cmd_ptr, uint8_t reg, uint8_t value) {
    data_cmd_ptr[0] = uint16_t(reg) | I2C_IC_DATA_CMD_RESTART_BITS;
    data_cmd_ptr[1] = uint16_t(value) | I2C_IC_DATA_CMD_STOP_BITS;
    return data_cmd_ptr + 2;
}

uint16_t* write_i2c_reg_buffer(uint16_t* data_cmd_ptr, uint8_t reg, uint16_t buf_len, uint8_t* buf) {
    *data_cmd_ptr++ = uint16_t(reg) | I2C_IC_DATA_CMD_RESTART_BITS;
    for (uint16_t i = 0; i < buf_len; ++i) {
        *data_cmd_ptr++ = buf[i];
    }
    data_cmd_ptr[-1] |= I2C_IC_DATA_CMD_STOP_BITS;
    return data_cmd_ptr;
}

int main() {
  I2C i2c(BOARD::BREAKOUT_GARDEN);
  BreakoutDotMatrix display(&i2c);

  bool led_toggle = false;

  pwm_config cfg = pwm_get_default_config();
  pwm_init(pwm_gpio_to_slice_num(PICO_DEFAULT_LED_PIN), &cfg, true);
  pwm_set_wrap(pwm_gpio_to_slice_num(PICO_DEFAULT_LED_PIN), 255);
  pwm_set_gpio_level(PICO_DEFAULT_LED_PIN, 0);
  gpio_set_function(PICO_DEFAULT_LED_PIN, GPIO_FUNC_PWM);

  int dma_chan = dma_claim_unused_channel(true);
  int pwm_dma_chan = dma_claim_unused_channel(true);
  int chain_dma_chan = dma_claim_unused_channel(true);

  display.init();
  display.set_power(10);
  display.show();

  {
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_ring(&c, false, 11);
    channel_config_set_dreq(&c, i2c_get_dreq(i2c0, true));
    dma_channel_configure(dma_chan, &c, &i2c_get_hw(i2c0)->data_cmd, dm_i2c_cmd_buf, I2C_CMD_TRANSFER_LEN, true);
  }
  
  {
    dma_timer_set_fraction(0, 1, 15625);

    dma_channel_config c = dma_channel_get_default_config(pwm_dma_chan);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_dreq(&c, dma_get_timer_dreq(0));
    channel_config_set_chain_to(&c, chain_dma_chan);
    dma_channel_configure(pwm_dma_chan, &c, &pwm_hw->slice[pwm_gpio_to_slice_num(PICO_DEFAULT_LED_PIN)].cc, pwm_level, 32, true);
  }

  {
    dma_channel_config c = dma_channel_get_default_config(chain_dma_chan);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, false);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_chain_to(&c, dma_chan);
    dma_channel_configure(chain_dma_chan, &c, &dma_hw->ch[pwm_dma_chan].al3_read_addr_trig, chain_cmd, 1, false);
  }
  
#if 1
  syscfg_hw->mempowerdown = 0xff;
  clocks_hw->wake_en1 = 0x703c;
  clocks_hw->wake_en0 = 0x00ef0f7f;
  clocks_hw->sleep_en1 = 0x703c;
  clocks_hw->sleep_en0 = 0x00ef0f7f;

  while (1) __asm volatile ("wfi");
#else
  while(true) {
    absolute_time_t at = get_absolute_time();
    uint64_t t = to_us_since_boot(at) * TIME_MULT;

#if 0
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
    uint16_t* i2c_cmd_ptr = write_i2c_reg_buffer(&i2c_data_cmds[0], LTP305::CMD_MATRIX_L, LTP305::BUFFER_LENGTH, &display.buf_matrix_left[1]);
    i2c_cmd_ptr = write_i2c_reg_buffer(i2c_cmd_ptr, LTP305::CMD_MATRIX_R, LTP305::BUFFER_LENGTH, &display.buf_matrix_right[1]);
    i2c_cmd_ptr = write_i2c_reg_value(i2c_cmd_ptr, LTP305::CMD_UPDATE, 1);

    dma_channel_wait_for_finish_blocking(dma_chan);
    dma_channel_transfer_from_buffer_now(dma_chan, &i2c_data_cmds[0], i2c_cmd_ptr - &i2c_data_cmds[0]);
#else
    int count = (t / 1000000) % 100;
    dma_channel_wait_for_finish_blocking(dma_chan);
    dma_channel_transfer_from_buffer_now(dma_chan, &dm_i2c_cmd_buf[I2C_CMD_TRANSFER_LEN * count], I2C_CMD_TRANSFER_LEN);
#endif

    pwm_set_gpio_level(PICO_DEFAULT_LED_PIN, led_toggle ? 64 : 0);
    led_toggle = !led_toggle;

    at = get_absolute_time();
    t = to_us_since_boot(at) * TIME_MULT;
    uint64_t w = t / 500000;
    w = (w + 1) * 500000 - t;
    
    sleep_us(w / TIME_MULT);
  }
#endif
}
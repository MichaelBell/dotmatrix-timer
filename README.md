# Dotmatrix timer

A timer displayed on a Pimoroni dot matrix breakout connected over I2C.

This uses a very slow clock and reduce voltage to minimise current drawn by the RP2040.
To acheive a low overall current draw you should also change OPTS in pimoroni-pico/drivers/ltp305/ltp305.hpp from 0b1110 to 0b1001, which reduces the max current for the dot matrix.

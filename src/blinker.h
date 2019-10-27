#pragma once

#ifndef BLINKER_H
#define BLINKER_H

#include "nrf_drv_clock.h"
#include "app_error.h"
#include "app_timer.h"

#define FREQUENCY_BLINKING 1000

void blinker_init(void);

ret_code_t start_blinking();

ret_code_t stop_blinking();

#endif // BLINKER_H
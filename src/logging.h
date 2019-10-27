#pragma once

#ifndef LOGGING_H
#define LOGGING_H

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_log_backend_flash.h"

void logging_init(void);

void logging_process(void);

#endif // LOGGING_H
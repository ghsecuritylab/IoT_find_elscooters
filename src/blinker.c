#include "boards.h"
#include "nrf_drv_gpiote.h"
#include "nrf_log.h"

#include "blinker.h"
#include "logging.h"
#include "interface.h"

APP_TIMER_DEF(m_blinker_timer_id);

uint32_t logging_m_counter = 0;

static void timer_for_led_blink_control_handler(void * p_context)
{
   if (get_state() != SCANNING)
   {
       nrf_gpio_pin_toggle(LED_2);
       if (logging_m_counter % 2 == 0)
       {
           nrf_gpio_pin_toggle(LED_3);
       }
   }
   logging_m_counter++;
   NRF_LOG_RAW_INFO("counter = %d\n", logging_m_counter);
}

void blinker_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_create(&m_blinker_timer_id,
        APP_TIMER_MODE_REPEATED,
        timer_for_led_blink_control_handler);
    APP_ERROR_CHECK(err_code);

    nrf_gpio_pin_clear(LED_2);
}


ret_code_t start_blinking()
{
    return app_timer_start(m_blinker_timer_id, APP_TIMER_TICKS(FREQUENCY_BLINKING), NULL);
}

ret_code_t stop_blinking()
{
    nrf_gpio_pin_set(LED_2);

    return app_timer_stop(m_blinker_timer_id);
}
#include "nrf_cli.h"
#include "nrf_log.h"
#include "sdk_common.h"
#include "interface.h"

extern uint32_t logging_m_counter;

static void cmd_counter(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    ASSERT(p_cli);
    ASSERT(p_cli->p_ctx && p_cli->p_iface && p_cli->p_name);


    if ((argc == 1) || nrf_cli_help_requested(p_cli))
    {
        nrf_cli_help_print(p_cli, NULL, 0);
        return;
    }

    if (argc != 2)
    {
        nrf_cli_error(p_cli, "%s: bad parameter count", argv[0]);
        return;
    }

    /* subcommands have their own handlers and they are not processed here */
    nrf_cli_error(p_cli, "%s: unknown parameter: %s", argv[0], argv[1]);
//    nrf_cli_print(p_cli, "counter: %d", logging_m_counter);
}


static void cmd_counter_start(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    if (argc != 1)
    {
        nrf_cli_error(p_cli, "%s: bad parameter count", argv[0]);
        return;
    }

    reset_count();
    set_state(START);
    nrf_cli_print(p_cli, "Start scanning VOI scooters");
}

static void cmd_counter_stop(nrf_cli_t const * p_cli, size_t argc, char **argv)
{
    if (argc != 1)
    {
        nrf_cli_error(p_cli, "%s: bad parameter count", argv[0]);
        return;
    }
  
    set_state(STOP);
    nrf_cli_print(p_cli, "Scanned VOI scooters: %d", get_count());
}

NRF_CLI_CREATE_STATIC_SUBCMD_SET(m_sub_counter)
{
    NRF_CLI_CMD(start,  NULL, "Start searching VOI scooters (max 50)",  cmd_counter_start),
    NRF_CLI_CMD(stop,   NULL, "Stop searching VOI scooters",   cmd_counter_stop),
    NRF_CLI_SUBCMD_SET_END
};

NRF_CLI_CMD_REGISTER(counter, &m_sub_counter, "Display timer counter on screen",
                     cmd_counter);



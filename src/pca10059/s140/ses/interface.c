#include "interface.h"
#include <string.h>



voi_scanner_t scanner;

uint16_t get_count(void)
{
    return scanner.count;
}

void increment_count(void)
{
    if (scanner.count < MAX_SUPPORT_UNITS_TO_SCAN)
    {
        scanner.count++;
    }
}

void reset_count(void)
{
    scanner.count = 0;

#if ADDRESS_BOOK_SUPPORTED == 1
    for (int i = 0; i < MAX_SUPPORT_UNITS_TO_SCAN; i++)
    {
        memset(scanner[i], 0, BLE_GAP_ADDR_LEN);
    }
#endif
}

voi_scanning_state_t get_state(void)
{
    return scanner.state;
}

void set_state(voi_scanning_state_t const new_state)
{
    scanner.state = new_state;
}

void insert_address_if_not_available(uint8_t * const new_address)
{ 
    bool found = false;
#if ADDRESS_BOOK_SUPPORTED == 1
    for (int i = 0; i < scanner.count; i++)
    {
        if (strcmp(scanner.addr[i], new_address))
        {
            found = true;
            break;
        }
    }

    if ((found) && (get_count() < MAX_SUPPORT_UNITS_TO_SCAN))
    {
        increment_count();
        memcpy(scanner.addr[get_count()], new_address, BLE_GAP_ADDR_LEN);
    }
#endif
}

void init_voi_scanner(void)
{
    reset_count();
    set_state(IDLE);
}
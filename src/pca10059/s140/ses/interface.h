#ifndef INTERFACE_SCANNER_H
#define INTERFACE_SCANNER_H

#include <stdint.h>
#include <stdio.h>
#include "ble_gap.h"

#define ADDRESS_BOOK_SUPPORTED 0
#define MAX_SUPPORT_UNITS_TO_SCAN 50

typedef enum
{
    IDLE = 0,
    START = 1,
    SCANNING = 2,
    STOP = 3
} voi_scanning_state_t;

typedef struct
{
    voi_scanning_state_t state;
    uint16_t count;
#if ADDRESS_BOOK_SUPPORTED == 1
    uint8_t addr[MAX_SUPPORT_UNITS_TO_SCAN][BLE_GAP_ADDR_LEN];
#endif
} voi_scanner_t;

uint16_t get_count(void);

void increment_count(void);

void reset_count(void);

voi_scanning_state_t get_state(void);

void set_state(voi_scanning_state_t const new_state);

void insert_address_if_not_available(uint8_t * const new_address);

void init_voi_scanner(void);

#endif // INTERFACE_SCANNER_H
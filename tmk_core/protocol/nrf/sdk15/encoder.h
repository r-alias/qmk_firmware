#pragma once

#include "quantum.h"

typedef uint32_t pin_t; //for nrf

void encoder_init(void);
void encoder_read(void);

void encoder_update_kb(int8_t index, bool clockwise);
void encoder_update_user(int8_t index, bool clockwise);

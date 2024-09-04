#ifndef BNO85_H
#define BNO85_H

#include <iostream>
#include <cstring>
extern "C"
{
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include <sh2_err.h>
#include <sh2_SensorValue.h>

}

void run_bno085(i2c_inst_t *i2c);

#endif
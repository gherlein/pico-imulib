#ifndef BNOO85_H
#define BNOO85_H

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include <sh2_err.h>
#include <sh2_SensorValue.h>

#ifdef __cplusplus
extern "C"
{
#endif

    void run_bno085(i2c_inst_t *i2c);
    bool init_i2c_hal(void);
    bool enableReports(void);
    bool enableCalibration(void);
    bool hasReset(void);

#ifdef __cplusplus
}
#endif

#endif // BNOO85_H

#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/types.h"

#include "bno_common.h"
#include "bno085_imu.h"

void run_bno085(i2c_inst_t *i2c)
{
    int rc = init_i2c_hal();
    if (!rc)
        return;
    rc = enableReports();
    if (!rc)
        return;
    rc = enableCalibration();
    if (!rc)
        return;
    while (true)
    {
        sleep_ms(100);
        sh2_service();
        if (hasReset())
        {
            enableCalibration();
            enableReports();
        }
    }
}
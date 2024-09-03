#include <stdio.h>
#include <iomanip>
#include <iostream>

#include <bno055_imu.h>
#include <bno085_imu.h>

void run_bno085(i2c_inst_t *i2c)
{
    // imu::bno85 imu(16, 17);
    imu::bno85 imu(i2c);
    auto rc = imu.init_i2c_hal();
    if (!rc)
        return;
    rc = imu.enableReports();
    if (!rc)
        return;
    rc = imu.enableCalibration();
    if (!rc)
        return;
    while (true)
    {
        sleep_ms(100);
        sh2_service();
        if (imu.hasReset())
        {
            imu.enableCalibration();
            imu.enableReports();
        }
    }
}

#ifdef OLD
void run_bno055()
{
    imu::bno55 imu(16, 17);
    while (true)
    {
        // calibration
        auto [gyro, accl, mag, sys] = imu.getCalibrationStatus();
        std::cout << "{\"cal_gyro\":" << unsigned(gyro) << ", \"cal_acc\":" << unsigned(accl)
                  << ", \"cal_mag\":" << unsigned(mag) << ", \"cal_sys\":" << unsigned(sys) << "}\n";
        // accl
        // bno055_accel_float_t accel;
        // bno055_convert_float_accel_xyz_msq(&accel);
        // std::cout << std::fixed << std::setprecision(2);
        // std::cout << "{\"acc_x\":" << accel.x << ", \"acc_y\":" << accel.y << ", \"acc_z\":" << accel.z << "}\n";
        // euler
        auto [h, p, r] = imu.getEulerAngles();
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "{\"y\":" << h << ", \"p\":" << p << ", \"r\":" << r << "}\n";
        sleep_ms(100);
    }
}
#endif

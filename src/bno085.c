#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "stdio.h"
#include "string.h"

#include "bno_common.h"
#include "sh2_hal.h"
#include "sh2_err.h"
#include "sh2_SensorValue.h"

bool reset_occurred = false;
const uint8_t BNO085_I2C_ADDR = 0x4A;
sh2_SensorValue_t sensor_value;
sh2_SensorEvent_t sensor_event;
u8 accStatus, gyroStatus, magStatus;

bool hal_reset(void);
void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent);
void sensorHandler(void *cookie, sh2_SensorEvent_t *event);
int i2c_open(sh2_Hal_t *self);
void i2c_close(sh2_Hal_t *self);
int i2c_write(sh2_Hal_t *self, uint8_t *buffer, unsigned len);
int i2c_read(sh2_Hal_t *self, uint8_t *buffer, unsigned len, uint32_t *t_us);
uint32_t getTimeUs(sh2_Hal_t *self);
bool enableReport(sh2_SensorId_t sensorId, float frequency);

i2c_inst_t *i2cX;
sh2_Hal_t sh2_hal;
sh2_ProductIds_t prodIds;

// bno85(int sda, int scl, uint freq = 400 * 1000)
void bno85(i2c_inst_t *i2c)
{
	i2cX = i2c;
}

bool init_i2c_hal(void)
{
	uint8_t dummy;
	int rc = i2c_read_blocking(i2cX, BNO085_I2C_ADDR, &dummy, 1, false);

	if (rc < 1)
	{
		printf("i2c_read_blocking dummy test failed\n");
		return false;
	}

	printf("i2c_read_blocking dummy test successful: \n");
	if (!hal_reset())
		return false;

	sh2_hal.open = i2c_open;
	sh2_hal.close = i2c_close;
	sh2_hal.read = i2c_read;
	sh2_hal.write = i2c_write;
	sh2_hal.getTimeUs = getTimeUs;

	int status = sh2_open(&sh2_hal, hal_callback, NULL);
	if (status != SH2_OK)
	{

		printf("sh2_open failed\n");
		return false;
	}
	memset(&prodIds, 0, sizeof(prodIds));
	status = sh2_getProdIds(&prodIds);
	if (status != SH2_OK)
	{
		printf("sh2_getProdIds failed\n");
		return false;
	}
	for (int n = 0; n < prodIds.numEntries; n++)
	{ // sh2.h line 60
		printf("Part: %d %d\n", prodIds.entry[n], prodIds.entry[n].swPartNumber);
		printf("Version: %d.%d.%d\n", prodIds.entry[n].swVersionMajor,
			   prodIds.entry[n].swVersionMinor, prodIds.entry[n].swVersionPatch);
		printf("Build: %n", prodIds.entry[n].swBuildNumber);
	}
	status = sh2_setSensorCallback(sensorHandler, NULL);
	if (status != SH2_OK)
	{
		printf("sh2_setSensorCallback failed\n");
		return false;
	}
	return true;
}

bool enableCalibration(void)
{
	int status = sh2_setCalConfig(SH2_CAL_ACCEL | SH2_CAL_GYRO | SH2_CAL_MAG);
	printf("sh2_setCalConfig %n", status);
	return (status == SH2_OK);
}

bool enableReports(void)
{
	if (!enableReport(SH2_ACCELEROMETER, 5))
	{
		return false;
	}
	if (!enableReport(SH2_ROTATION_VECTOR, 10))
	{
		return false;
	}
	if (!enableReport(SH2_GYROSCOPE_CALIBRATED, 5))
	{
		return false;
	}
	if (!enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 5))
	{
		return false;
	}
	return true;
}

bool enableReport(sh2_SensorId_t sensorId, float frequency)
{
	uint32_t interval_us = (uint32_t)(1000000 / frequency);
	static sh2_SensorConfig_t config;
	config.sensorSpecific = 0;
	config.batchInterval_us = 0;
	config.wakeupEnabled = false;
	config.changeSensitivity = 0;
	config.alwaysOnEnabled = false;
	config.reportInterval_us = interval_us;
	config.changeSensitivityEnabled = false;
	config.changeSensitivityRelative = false;
	bool status = sh2_setSensorConfig(sensorId, &config);
	if (status != SH2_OK)
	{
		printf("sh2_setSensorConfig failed %d", sensorId);
	}
	return (status == SH2_OK);
}

bool hasReset(void)
{
	bool x = reset_occurred;
	reset_occurred = false;
	if (x)
		printf("imu was reset\n");
}

bool hal_reset(void)
{
	// Figure 1-27: 1 – reset
	uint8_t soft_reset_pkt[5] = {5, 0, 1, 0, 1};
	for (int i = 0;
		 i < 5;
		 i++)
	{
		int rc = i2c_write_blocking(i2cX, BNO085_I2C_ADDR,
									soft_reset_pkt, 5, false);
		if (rc == 5)
		{
			return true;
		}
		else
		{
			printf("hal software reset failed\n");
			sleep_ms(50);
		}
	}
	sleep_ms(300);
	return false;
}

// callback for reset events and other non-sensor events received from SH-2 sensor hub
void hal_callback(void *cookie,
				  sh2_AsyncEvent_t *
					  pEvent)
{
	if (pEvent->eventId == SH2_RESET)
	{
		reset_occurred = true;
		printf("hal_callback SH2_RESET\n");
	}
}

void sensorHandler(void *cookie, sh2_SensorEvent_t *event)
{
	sensor_event = *event;
	int status = sh2_decodeSensorEvent(&sensor_value, &sensor_event);
	if (status != SH2_OK)
	{

		printf("sh2_decodeSensorEvent failed\n");
		return;
	}

	bool cal_event = false;
	switch (sensor_event.reportId)
	{
	// The absolute rotation vector provides an orientation output that is
	// expressed as a quaternion referenced to magnetic north and gravity
	// game rotation vector output aligns the quaternion output to an arbitrary orientation
	case SH2_ROTATION_VECTOR:
		printf("{%f %f %f %f}\n", sensor_value.un.rotationVector.i,
			   sensor_value.un.rotationVector.j,
			   sensor_value.un.rotationVector.k,
			   sensor_value.un.rotationVector.real);
		break;
	case SH2_ACCELEROMETER:
		cal_event = true;
		accStatus = (sensor_event.report[2] & 0x03);
		break;
	case SH2_GYROSCOPE_CALIBRATED:
		cal_event = true;
		gyroStatus = (sensor_event.report[2] & 0x03);
		break;
	case SH2_MAGNETIC_FIELD_CALIBRATED:
		cal_event = true;
		magStatus = (sensor_event.report[2] & 0x03);
		break;
	}

	if (cal_event)
	{
		// printf(" "{" <<
		//     "\"cal_gyro\":" << unsigned(gyroStatus) << ", " <<
		//"\"cal_acc\":" << unsigned(accStatus) << ", " <<
		//     "\"cal_mag\":" << unsigned(magStatus) << "}\n";
	}
}

int i2c_open(sh2_Hal_t *self)
{
	printf("i2c_open\n");
	if (!hal_reset())
		return 1;
	return 0;
}

void i2c_close(sh2_Hal_t *self)
{
	printf("i2c_close\n");
}

int i2c_read(sh2_Hal_t *self, uint8_t *buffer, unsigned len, uint32_t *t_us)
{
	*t_us = to_us_since_boot(get_absolute_time());
	uint8_t shtp_header[4]; // DS: 1.3.1 SHTP
	int rc = i2c_read_blocking(i2cX, BNO085_I2C_ADDR, shtp_header, 4, false);
	if (rc != 4)
	{
		printf("i2c_read shtp_header: %d\n", rc);
		return 0;
	}
	uint16_t length = (shtp_header[1] << 8) | shtp_header[0];
	length &= 0x7FFF;
	// DEBUG_PRINT("shtp_header 0 : 0x%02x, 0b%08b\r\n", shtp_header[0], shtp_header[0]);
	// DEBUG_PRINT("shtp_header 1 : 0x%02x, 0b%08b\r\n", shtp_header[1], shtp_header[1]);
	// DEBUG_PRINT("shtp_header 2 : 0x%02x, 0b%08b\r\n", shtp_header[2], shtp_header[2]);
	// DEBUG_PRINT("shtp_header 3 : 0x%02x, 0b%08b\r\n", shtp_header[3], shtp_header[3]);
	// DEBUG_PRINT("length %d\r\n", length);
	if (length > len)
	{
		printf("i2c_read shtp_header length %d\n", length);
		return 0;
	}
	rc = i2c_read_blocking(i2cX, BNO085_I2C_ADDR, buffer, length, false);
	if (rc != length)
	{
		printf("i2c_read_blocking buffer rc %d", length);
		return 0;
	}
	return length;
}

int i2c_write(sh2_Hal_t *self, uint8_t *buffer, unsigned len)
{
	uint16_t length = (len > SH2_HAL_MAX_TRANSFER_OUT) ? SH2_HAL_MAX_TRANSFER_OUT : len;
	int rc = i2c_write_blocking(i2cX, BNO085_I2C_ADDR, buffer, length, false);

	// printf(" "i2c_write " << rc << std::endl;
	if (rc != length)
		return 0;
	return rc;
}

uint32_t getTimeUs(sh2_Hal_t *self)
{
	return to_us_since_boot(get_absolute_time());
}

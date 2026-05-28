#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>

#define IMU_NODE DT_NODELABEL(lsm6dsl)

static const struct device *imu;

int main(void)
{
    imu = DEVICE_DT_GET(IMU_NODE);
// check that device is ready (dev academy taught me well)
    if (!device_is_ready(imu)) {
        LOG_ERR("LSM6DSL not ready!");
        return -ENODEV;
    }
    LOG_INF("LSM6DSL found: %s", imu->name);

    configure_imu(imu);

}

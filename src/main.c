#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define IMU_NODE DT_NODELABEL(mpu6050)

int main(void)
{
	const struct device *imu = DEVICE_DT_GET(IMU_NODE);

	if (!device_is_ready(imu)) {
		printk("MPU-6050 not ready -- check wiring, overlay, and prj.conf\n");
		return -ENODEV;
	}

	printk("MPU-6050 ready: %s\n", imu->name);

	while (1) {
		struct sensor_value ax, ay, az, gx, gy, gz;

		int rc = sensor_sample_fetch(imu);

		if (rc != 0) {
			printk("sensor_sample_fetch failed: %d\n", rc);
			k_msleep(500);
			continue;
		}

		sensor_channel_get(imu, SENSOR_CHAN_ACCEL_X, &ax);
		sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Y, &ay);
		sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Z, &az);
		sensor_channel_get(imu, SENSOR_CHAN_GYRO_X, &gx);
		sensor_channel_get(imu, SENSOR_CHAN_GYRO_Y, &gy);
		sensor_channel_get(imu, SENSOR_CHAN_GYRO_Z, &gz);

		printk("accel (m/s^2): x=%d.%06d y=%d.%06d z=%d.%06d   "
		       "gyro (rad/s): x=%d.%06d y=%d.%06d z=%d.%06d\n",
		       ax.val1, ax.val2, ay.val1, ay.val2, az.val1, az.val2,
		       gx.val1, gx.val2, gy.val1, gy.val2, gz.val1, gz.val2);

		k_msleep(500);
	}

	return 0;
}
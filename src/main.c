#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define IMU_NODE DT_NODELABEL(mpu6050)
#define CALIBRATION_SAMPLES 100

// "Zero" the IMU by averaging a number of samples to determine the bias for each axis.
static struct {
    double ax, ay, az;
    double gx, gy, gz;
} bias;

static void calibrate_imu(const struct device *imu) {
    struct sensor_value ax, ay, az, gx, gy, gz;
    bias.ax = bias.ay = bias.az = 0;
    bias.gx = bias.gy = bias.gz = 0;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        int rc = sensor_sample_fetch(imu);
        if (rc != 0) {
            printk("sensor_sample_fetch failed during calibration: %d\n", rc);
            k_msleep(100);
            continue;
        }

        sensor_channel_get(imu, SENSOR_CHAN_ACCEL_X, &ax);
        sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Y, &ay);
        sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Z, &az);
        sensor_channel_get(imu, SENSOR_CHAN_GYRO_X, &gx);
        sensor_channel_get(imu, SENSOR_CHAN_GYRO_Y, &gy);
        sensor_channel_get(imu, SENSOR_CHAN_GYRO_Z, &gz);

        bias.ax += ax.val1 + ax.val2 / 1e6;
        bias.ay += ay.val1 + ay.val2 / 1e6;
        bias.az += az.val1 + az.val2 / 1e6;
        bias.gx += gx.val1 + gx.val2 / 1e6;
        bias.gy += gy.val1 + gy.val2 / 1e6;
        bias.gz += gz.val1 + gz.val2 / 1e6;

        k_msleep(10); // Small delay between samples
    }

    // Average the biases
    bias.ax /= CALIBRATION_SAMPLES;
    bias.ay /= CALIBRATION_SAMPLES;
    bias.az = (bias.az / CALIBRATION_SAMPLES) - 9.80665; //subtract gravity from z axis
    bias.gx /= CALIBRATION_SAMPLES;
    bias.gy /= CALIBRATION_SAMPLES;
    bias.gz /= CALIBRATION_SAMPLES;

    printk("Calibration complete. Biases:\n");
    printk("Accel: x=%.6f y=%.6f z=%.6f\n", bias.ax, bias.ay, bias.az);
    printk("Gyro: x=%.6f y=%.6f z=%.6f\n", bias.gx, bias.gy, bias.gz);
}

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

        double ax_v = (ax.val1 + ax.val2 / 1e6) - bias.ax;
		double ay_v = (ay.val1 + ay.val2 / 1e6) - bias.ay;
		double az_v = (az.val1 + az.val2 / 1e6) - bias.az;
		double gx_v = (gx.val1 + gx.val2 / 1e6) - bias.gx;
		double gy_v = (gy.val1 + gy.val2 / 1e6) - bias.gy;
		double gz_v = (gz.val1 + gz.val2 / 1e6) - bias.gz;

		printk("accel (m/s^2): x=%.6f y=%.6f z=%.6f   "
		       "gyro (rad/s): x=%.6f y=%.6f z=%.6f\n",
		       ax_v, ay_v, az_v, gx_v, gy_v, gz_v);
		k_msleep(500);
	}

	return 0;
}

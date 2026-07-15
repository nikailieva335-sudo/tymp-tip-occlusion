#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/storage/flash_map.h>


LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define IMU_NODE DT_NODELABEL(mpu6050)
#define CALIBRATION_SAMPLES 100
#define SAMPLE_PERIOD_MS 20

// NVS stores the reference angle calibrated by a clinician. Once the device is booted, the notebook is read and the
// Accelerometer data is compared to the NVS data

#define NVS_PARTITION nvs_storage
#define NVS_PARTITION_DEVICE FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET FIXED_PARTITION_OFFSET(NVS_PARTITION)
#define REFERENCE_ANGLE_ID 1

// The tolerance of the angle
#define ANGLE_THRESHOLD_DEG 45.0
#define REFERENCE_SAMPLES 100


// Clinician calibration of "correct" angle
// Gyro and Accelerometer filter's trust ratio

#define FILTER_ALPHA 0.98
#define PI 3.14159265358979323846

// Reference angle
struct reference_angle {
    double pitch;
    double roll;
};

static struct nvs_fs nvs;

// The clinician calibrated pitch and roll
static double reference_pitch;
static double reference_roll;

// "Zero" the IMU by averaging a number of samples to determine the bias for each axis.

static struct {
    double ax, ay, az;
    double gx, gy, gz;
} bias;

static void calibrate_imu(const struct device *imu) {
    struct sensor_value ax, ay, az, gx, gy, gz;
    bias.ax = bias.ay = bias.az = 0;
    bias.gx = bias.gy = bias.gz = 0;
// Loops 100 tmes.
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

//Clinician calibration of device reference angle
static void reference_angle(const struct device *imu)
{
    double sum_pitch = 0.0;
    double sum_roll = 0.0;
    int captured = 0;

    for (int i = 0; i < REFERENCE_SAMPLES; i++) {
        struct sensor_value ax, ay, az;
        int rc = sensor_sample_fetch(imu);
        if (rc != 0) {
            printk("failed during reference angle capture: %d\n", rc);
            k_msleep(100);
            continue;
        }
        sensor_channel_get(imu, SENSOR_CHAN_ACCEL_X, &ax);
		sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Y, &ay);
		sensor_channel_get(imu, SENSOR_CHAN_ACCEL_Z, &az);

		double ax_v = (ax.val1 + ax.val2 / 1e6) - bias.ax;
		double ay_v = (ay.val1 + ay.val2 / 1e6) - bias.ay;
		double az_v = (az.val1 + az.val2 / 1e6) - bias.az;

		sum_pitch += atan2(-ax_v, sqrt(ay_v * ay_v + az_v * az_v)) * 180.0 / PI;
		sum_roll += atan2(ay_v, az_v) * 180.0 / PI;
		captured++;

		k_msleep(10);
	}

	reference_pitch = sum_pitch / captured;
	reference_roll = sum_roll / captured;

	printk("Reference angle captured: pitch=%.2f roll=%.2f\n",
	       reference_pitch, reference_roll);
}

// Checks the notebook for a saved reference angle. If none is found, it prompts the user to hold the device at the correct angle and saves that as the reference.
static int setup_reference_angle(const struct device *imu)
{
	nvs.flash_device = NVS_PARTITION_DEVICE;
	if (!device_is_ready(nvs.flash_device)) {
		printk("Flash device not ready -- can't persist reference angle\n");
		return -ENODEV;
	}
	nvs.offset = NVS_PARTITION_OFFSET;

	struct flash_pages_info page_info;
	int rc = flash_get_page_info_by_offs(nvs.flash_device, nvs.offset, &page_info);

	if (rc != 0) {
		printk("Unable to get flash page info: %d\n", rc);
		return rc;
	}

	nvs.sector_size = page_info.size;
	nvs.sector_count = 3U;

	rc = nvs_mount(&nvs);
	if (rc != 0) {
		printk("NVS mount failed: %d\n", rc);
		return rc;
	}
    struct reference_angle saved;

	rc = nvs_read(&nvs, REFERENCE_ANGLE_ID, &saved, sizeof(saved));
	if (rc == sizeof(saved)) {
		reference_pitch = saved.pitch;
		reference_roll = saved.roll;
		printk("Loaded saved reference angle: pitch=%.2f roll=%.2f\n",
		       reference_pitch, reference_roll);
		return 0;
	}

	printk("No saved reference angle found -- running first-time setup.\n");
	printk("Now hold the device at the CORRECT insertion angle...\n");
	k_msleep(2000); /* give the operator a moment to get into position */
	reference_angle(imu);

	saved.pitch = reference_pitch;
	saved.roll = reference_roll;
	rc = nvs_write(&nvs, REFERENCE_ANGLE_ID, &saved, sizeof(saved));
	if (rc < 0) {
		printk("Warning: failed to save reference angle: %d\n", rc);
		return rc;
	}

	printk("Reference angle saved -- this step won't repeat on future boots.\n");
	return 0;
}

//
int main(void)
{
	const struct device *imu = DEVICE_DT_GET(IMU_NODE);

	if (!device_is_ready(imu)) {
		printk("MPU-6050 not ready\n");
		return -ENODEV;
	}

	printk("MPU-6050 ready: %s\n", imu->name);
	printk("Calibrating -- keep device still and flat...\n");
	calibrate_imu(imu);

	int rc = setup_reference_angle(imu);

	if (rc != 0) {
		printk("Reference angle setup failed: %d\n", rc);
		return rc;
	}

	double pitch_deg = 0.0;
	double roll_deg = 0.0;
	bool initialized = false;
	int64_t last_time = k_uptime_get();

	while (1) {
		struct sensor_value ax, ay, az, gx, gy, gz;

		rc = sensor_sample_fetch(imu);

		if (rc != 0) {
			printk("sensor_sample_fetch failed: %d\n", rc);
			k_msleep(SAMPLE_PERIOD_MS);
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

		double accel_pitch = atan2(-ax_v, sqrt(ay_v * ay_v + az_v * az_v)) * 180.0 / PI;
		double accel_roll = atan2(ay_v, az_v) * 180.0 / PI;

        // Gyro rates, converted rad/s -> deg/s
		double gyro_pitch_rate = gy_v * 180.0 / PI;
		double gyro_roll_rate = gx_v * 180.0 / PI;

		int64_t now = k_uptime_get();
		double dt = (now - last_time) / 1000.0; // seconds

		last_time = now;

		if (!initialized) {
			// Initialize the filter with the accelerometer angles on the first iteration
			pitch_deg = accel_pitch;
			roll_deg = accel_roll;
			initialized = true;
		} else {
			pitch_deg = FILTER_ALPHA * (pitch_deg + gyro_pitch_rate * dt) +
				    (1.0 - FILTER_ALPHA) * accel_pitch;
			roll_deg = FILTER_ALPHA * (roll_deg + gyro_roll_rate * dt) +
				   (1.0 - FILTER_ALPHA) * accel_roll;
		}

		printk("pitch=%.2f roll=%.2f   (reference: pitch=%.2f roll=%.2f)\n",
		       pitch_deg, roll_deg, reference_pitch, reference_roll);

		k_msleep(SAMPLE_PERIOD_MS);
	}

	return 0;
}

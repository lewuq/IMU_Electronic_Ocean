/*
 * Copyright (c) 2026 Seeed Technology Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Electronic ocean for XIAO STM32C5 and two 6 x 10 RGB Matrices for XIAO.
 * The on-board LSM6DS3TR-C controls the water direction and wave energy.
 * First DIN connects to D0/PA0; its DOUT connects to the second DIN.
 * Two upright panels are stacked: first on top, second below, with matching
 * orientations, forming one 6-column x 20-row bottle.
 */

#include <math.h>
#include <stdint.h>
#include <string.h>
#include <soc.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#define IMU_NODE DT_ALIAS(imu0)
#define RGB_DATA_NODE DT_ALIAS(rgb_data)
#define USER_LED_NODE DT_ALIAS(led0)
#define IMU_WAKE_NODE DT_ALIAS(imu_wake)
#define MODE_BUTTON_NODE DT_ALIAS(mode_button)

#define PANEL_COLS 6
#define PANEL_ROWS 10
#define PANEL_COUNT 2
#define MATRIX_COLS PANEL_COLS
#define MATRIX_ROWS (PANEL_ROWS * PANEL_COUNT)
#define PIXEL_COUNT (MATRIX_COLS * MATRIX_ROWS)
#define FRAME_MS 20
#define GRAVITY 9.80665f

/* Limits both heat and supply current while retaining saturated colors. */
#define MASTER_BRIGHTNESS 3U

static const struct device *const imu = DEVICE_DT_GET(IMU_NODE);
static const struct gpio_dt_spec rgb_data =
	GPIO_DT_SPEC_GET(RGB_DATA_NODE, gpios);
static const struct gpio_dt_spec user_led =
	GPIO_DT_SPEC_GET(USER_LED_NODE, gpios);
static const struct gpio_dt_spec imu_wake =
	GPIO_DT_SPEC_GET(IMU_WAKE_NODE, gpios);
static const struct gpio_dt_spec mode_button =
	GPIO_DT_SPEC_GET(MODE_BUTTON_NODE, gpios);
static const struct i2c_dt_spec imu_i2c = I2C_DT_SPEC_GET(IMU_NODE);
static struct gpio_callback imu_wake_cb;
static struct gpio_callback mode_button_cb;
static volatile bool sleeping;
static volatile bool button_wake_requested;
K_SEM_DEFINE(wake_sem, 0, 1);

struct rgb
{
	uint8_t r;
	uint8_t g;
	uint8_t b;
};

static struct rgb pixels[PIXEL_COUNT];
/* Bottle mounting: screen right = IMU -X; screen up = IMU +Y.
 * Accelerometers measure specific force, so physical gravity is negated.
 * Change these signs only if the XIAO is mounted in the opposite orientation.
 */
#define SCREEN_X_SIGN (1.0f)
#define SCREEN_Y_SIGN (-1.0f)
#define WATER_FRACTION 0.5f
#define STILL_SLEEP_MS 90000
#define BUTTON_SLEEP_MS 2000
#define SHAKE_GUARD_MS 3000
#define SHAKE_SLEEP_DELTA (GRAVITY * 0.80f)
#define ANGLE_CHANGE_DOT 0.99863f /* cos(3 degrees) */
#define MODE_COUNT 5
#define TEXT_SCROLL_CYCLE_FRAMES 650U /* 650 x 20 ms = 13 seconds */

enum display_mode
{
	MODE_NEON,
	MODE_OCEAN_BLUE,
	MODE_CYAN,
	MODE_PURPLE,
	MODE_TEXT,
};

static enum display_mode display_mode;

#define LSM6DSL_CTRL1_XL 0x10
#define LSM6DSL_CTRL2_G 0x11
#define LSM6DSL_WAKE_UP_SRC 0x1B
#define LSM6DSL_TAP_CFG 0x58
#define LSM6DSL_WAKE_UP_THS 0x5B
#define LSM6DSL_WAKE_UP_DUR 0x5C
#define LSM6DSL_MD1_CFG 0x5E

static void imu_wake_handler(const struct device *port,
							 struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	k_sem_give(&wake_sem);
}

static void mode_button_handler(const struct device *port,
								struct gpio_callback *cb, uint32_t pins)
{
	ARG_UNUSED(port);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);
	if (sleeping)
	{
		button_wake_requested = true;
		k_sem_give(&wake_sem);
	}
}

static int configure_imu(void);

static inline float clampf(float value, float low, float high)
{
	return fminf(high, fmaxf(low, value));
}

/*
 * Each panel starts at its top-left and runs left-to-right in every row.
 * All 60 pixels of the top panel precede all 60 pixels of the bottom panel.
 * The renderer uses row 0 as the bottom, so the row coordinate is inverted.
 */
static inline int pixel_index(int column, int row)
{
	return (MATRIX_ROWS - 1 - row) * PANEL_COLS + column;
}

static struct rgb hsv_to_rgb(uint8_t hue, uint8_t saturation, uint8_t value)
{
	uint8_t region = hue / 43U;
	uint8_t remainder = (uint8_t)((hue - region * 43U) * 6U);
	uint8_t p = (uint8_t)(((uint16_t)value * (255U - saturation)) >> 8);
	uint8_t q = (uint8_t)(((uint16_t)value *
						   (255U - (((uint16_t)saturation * remainder) >> 8))) >>
						  8);
	uint8_t t = (uint8_t)(((uint16_t)value *
						   (255U - (((uint16_t)saturation *
									 (255U - remainder)) >>
									8))) >>
						  8);

	switch (region)
	{
	case 0:
		return (struct rgb){value, t, p};
	case 1:
		return (struct rgb){q, value, p};
	case 2:
		return (struct rgb){p, value, t};
	case 3:
		return (struct rgb){p, q, value};
	case 4:
		return (struct rgb){t, p, value};
	default:
		return (struct rgb){value, p, q};
	}
}

static inline uint8_t limit_brightness(uint8_t channel)
{
	return (uint8_t)(((uint16_t)channel * MASTER_BRIGHTNESS) / 100U);
}

static inline void cycle_wait_until(uint32_t deadline)
{
	while ((int32_t)(DWT->CYCCNT - deadline) < 0)
	{
		__NOP();
	}
}

static bool cycle_counter_start(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0U;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	__DSB();
	__ISB();

	uint32_t before = DWT->CYCCNT;

	for (volatile int i = 0; i < 32; ++i)
	{
		__NOP();
	}
	return DWT->CYCCNT != before;
}

/*
 * PA0 has no SPI-MOSI alternate function on STM32C5. Generate the WS2812
 * 800 kHz waveform directly with GPIOA BSRR and the Cortex-M cycle counter.
 * One 120-pixel frame takes at least 3.6 ms with interrupts locked.
 */
static void matrix_show(void)
{
	uint8_t grb[PIXEL_COUNT * 3];
	uint8_t *out = grb;

	for (int i = 0; i < PIXEL_COUNT; ++i)
	{
		*out++ = limit_brightness(pixels[i].g);
		*out++ = limit_brightness(pixels[i].r);
		*out++ = limit_brightness(pixels[i].b);
	}

	const uint32_t period = SystemCoreClock / 800000U;
	const uint32_t t0_high =
		(uint32_t)(((uint64_t)SystemCoreClock * 400U) / 1000000000ULL);
	const uint32_t t1_high =
		(uint32_t)(((uint64_t)SystemCoreClock * 850U) / 1000000000ULL);
	const uint32_t pin_set = BIT(0);
	const uint32_t pin_clear = BIT(16);
	unsigned int key = irq_lock();

	for (size_t byte = 0; byte < sizeof(grb); ++byte)
	{
		uint8_t value = grb[byte];

		for (uint8_t mask = 0x80U; mask != 0U; mask >>= 1)
		{
			GPIOA->BSRR = pin_set;
			uint32_t start = DWT->CYCCNT;

			cycle_wait_until(start + ((value & mask) ? t1_high : t0_high));
			GPIOA->BSRR = pin_clear;
			cycle_wait_until(start + period);
		}
	}

	GPIOA->BSRR = pin_clear;
	irq_unlock(key);
	k_busy_wait(80);
}

static int set_imu_motion_wakeup(bool enable)
{
	uint8_t source;
	int ret = 0;

	if (enable)
	{
		/* 26 Hz, +/-4 g; require about 1.25 g for two samples. */
		ret |= i2c_reg_write_byte_dt(&imu_i2c, LSM6DSL_CTRL2_G, 0x00);
		ret |= i2c_reg_write_byte_dt(&imu_i2c, LSM6DSL_CTRL1_XL, 0x28);
		ret |= i2c_reg_update_byte_dt(&imu_i2c, LSM6DSL_TAP_CFG, 0x80, 0x80);
		ret |= i2c_reg_update_byte_dt(&imu_i2c, LSM6DSL_WAKE_UP_THS, 0x3f, 0x14);
		ret |= i2c_reg_update_byte_dt(&imu_i2c, LSM6DSL_WAKE_UP_DUR, 0x60, 0x20);
		ret |= i2c_reg_update_byte_dt(&imu_i2c, LSM6DSL_MD1_CFG, 0x20, 0x20);
		ret |= i2c_reg_read_byte_dt(&imu_i2c, LSM6DSL_WAKE_UP_SRC, &source);
	}
	else
	{
		ret |= i2c_reg_update_byte_dt(&imu_i2c, LSM6DSL_MD1_CFG, 0x20, 0x00);
	}
	return ret;
}

static int enter_imu_sleep(void)
{
	memset(pixels, 0, sizeof(pixels));
	matrix_show();
	k_sem_reset(&wake_sem);
	if (set_imu_motion_wakeup(true) < 0)
	{
		return -EIO;
	}
	button_wake_requested = false;
	sleeping = true;
	printk("Sleep: move IMU or hold D1 for 2 seconds to wake\n");
	while (1)
	{
		k_sem_take(&wake_sem, K_FOREVER);
		if (!button_wake_requested)
		{
			break;
		}

		button_wake_requested = false;
		int64_t pressed_at = k_uptime_get();

		while (gpio_pin_get_dt(&mode_button) > 0 &&
			   k_uptime_get() - pressed_at < BUTTON_SLEEP_MS)
		{
			k_msleep(20);
		}
		if (gpio_pin_get_dt(&mode_button) > 0)
		{
			while (gpio_pin_get_dt(&mode_button) > 0)
			{
				k_msleep(20);
			}
			break;
		}
		printk("Wake cancelled: D1 was not held for 2 seconds\n");
	}
	sleeping = false;
	if (set_imu_motion_wakeup(false) < 0 || configure_imu() < 0)
	{
		return -EIO;
	}
	printk("Wake complete\n");
	return 0;
}

static int configure_imu(void)
{
	struct sensor_value odr = {.val1 = 104};
	struct sensor_value accel_range;
	struct sensor_value gyro_range;
	int ret;

	sensor_g_to_ms2(4, &accel_range);
	sensor_degrees_to_rad(500, &gyro_range);

	ret = sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ,
						  SENSOR_ATTR_FULL_SCALE, &accel_range);
	ret |= sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ,
						   SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
	ret |= sensor_attr_set(imu, SENSOR_CHAN_GYRO_XYZ,
						   SENSOR_ATTR_FULL_SCALE, &gyro_range);
	ret |= sensor_attr_set(imu, SENSOR_CHAN_GYRO_XYZ,
						   SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
	return ret;
}

/*
 * A single gravity-aligned liquid surface spans both panels. Its tangent
 * rotates continuously through 360 degrees: no upright/upside-down switch.
 * Solve the fill threshold each frame to preserve the covered pixel area,
 * including fractional edge coverage, even when waves reach a bottle wall.
 */
struct glyph
{
	char character;
	uint8_t rows[5];
};

static const struct glyph font[] = {
	{'H', {0x7, 0x5, 0x7, 0x5, 0x5}},
	{'E', {0x7, 0x4, 0x6, 0x4, 0x7}},
	{'L', {0x5, 0x5, 0x5, 0x5, 0x7}},
	{'O', {0x7, 0x5, 0x5, 0x5, 0x7}},
	{' ', {0x0, 0x0, 0x0, 0x0, 0x0}},
	{'C', {0x7, 0x4, 0x4, 0x4, 0x7}},
	{'A', {0x2, 0x5, 0x7, 0x5, 0x5}},
	{'N', {0x7, 0x5, 0x5, 0x5, 0x5}},
	{'E', {0x7, 0x4, 0x6, 0x4, 0x7}},
	{'D', {0x6, 0x5, 0x5, 0x5, 0x6}},
	{'T', {0x7, 0x2, 0x2, 0x2, 0x2}},
	{'U', {0x5, 0x5, 0x5, 0x5, 0x7}},
	{'I', {0x7, 0x2, 0x2, 0x2, 0x7}},
	{'O', {0x7, 0x5, 0x5, 0x5, 0x7}},
	{'M', {0x5, 0x7, 0x7, 0x5, 0x5}},
	{'A', {0x2, 0x5, 0x7, 0x5, 0x5}},
	{'K', {0x5, 0x5, 0x6, 0x5, 0x5}},
	{'R', {0x6, 0x5, 0x6, 0x5, 0x5}},
};

static void render_text(uint32_t frame)
{
	static const char message[] = "ELECTRONIC OCEAN";
	const int message_length = sizeof(message) - 1;
	const int message_width = message_length * 4;
	int scroll_range = message_width + MATRIX_ROWS;
	int scroll = (int)(((uint64_t)frame * scroll_range /
						TEXT_SCROLL_CYCLE_FRAMES) %
					   scroll_range);
	int origin = MATRIX_ROWS - scroll;
	uint8_t hue = (uint8_t)(frame / 2U);

	memset(pixels, 0, sizeof(pixels));
	for (int letter = 0; letter < message_length; ++letter)
	{
		const uint8_t *rows = NULL;

		for (size_t i = 0; i < ARRAY_SIZE(font); ++i)
		{
			if (font[i].character == message[letter])
			{
				rows = font[i].rows;
				break;
			}
		}
		if (rows == NULL)
		{
			continue;
		}
		for (int y = 0; y < 5; ++y)
		{
			for (int x = 0; x < 3; ++x)
			{
				int screen_x = origin + letter * 4 + x;

				if (screen_x >= 0 && screen_x < MATRIX_ROWS &&
					(rows[y] & BIT(2 - x)))
				{
					/* Marquee orientation rotated 180 degrees. */
					pixels[pixel_index(y, screen_x)] =
						hsv_to_rgb((uint8_t)(hue + screen_x * 5), 220, 210);
				}
			}
		}
	}
}

static void render_ocean(float down_x, float down_y, float motion,
						 uint32_t frame)
{
	float field[PIXEL_COUNT];
	float low = -32.0f;
	float high = 32.0f;
	float phase = frame * 0.08f;
	uint8_t moving_hue = (uint8_t)(frame / 2U);
	float breathe = 0.62f + 0.38f * (0.5f + 0.5f * sinf(frame * 0.045f));

	if (display_mode == MODE_TEXT)
	{
		render_text(frame);
		return;
	}

	for (int row = 0; row < MATRIX_ROWS; ++row)
	{
		for (int column = 0; column < MATRIX_COLS; ++column)
		{
			float x = column + 0.5f - MATRIX_COLS * 0.5f;
			float y = row + 0.5f - MATRIX_ROWS * 0.5f;
			float tangent = -down_y * x + down_x * y;
			float ripple = (0.18f + motion * 0.85f) *
							   sinf(tangent * 0.85f + phase) +
						   (0.08f + motion * 0.30f) *
							   sinf(tangent * 1.55f - phase * 1.7f);

			field[row * MATRIX_COLS + column] =
				down_x * x + down_y * y + ripple;
		}
	}

	for (int iteration = 0; iteration < 16; ++iteration)
	{
		float level = (low + high) * 0.5f;
		float area = 0.0f;

		for (int i = 0; i < PIXEL_COUNT; ++i)
		{
			area += clampf(field[i] - level + 0.5f, 0.0f, 1.0f);
		}
		if (area > PIXEL_COUNT * WATER_FRACTION)
		{
			low = level;
		}
		else
		{
			high = level;
		}
	}

	float level = (low + high) * 0.5f;

	for (int row = 0; row < MATRIX_ROWS; ++row)
	{
		for (int column = 0; column < MATRIX_COLS; ++column)
		{
			float depth = field[row * MATRIX_COLS + column] - level;
			float coverage = clampf(depth + 0.5f, 0.0f, 1.0f);
			float foam = clampf(1.0f - fabsf(depth) / 1.2f, 0.0f, 1.0f);
			float shimmer = 0.5f + 0.5f *
									   sinf(phase * 1.9f + column * 0.9f - row * 0.6f);
			uint8_t hue;

			switch (display_mode)
			{
			case MODE_OCEAN_BLUE:
				hue = 165U;
				break;
			case MODE_CYAN:
				hue = 125U;
				break;
			case MODE_PURPLE:
				hue = 205U;
				break;
			default:
				hue = (uint8_t)(moving_hue + column * 12 + row * 5);
				break;
			}
			uint8_t saturation = (uint8_t)(235.0f - foam * 120.0f);
			float mode_brightness = display_mode == MODE_NEON ? 1.0f : breathe;
			uint8_t value = (uint8_t)(coverage * mode_brightness *
									  (100.0f + shimmer * 30.0f + foam * 100.0f));

			pixels[pixel_index(column, row)] =
				hsv_to_rgb(hue, saturation, value);
		}
	}
}

static float sensor_value_float(const struct sensor_value *value)
{
	return (float)value->val1 + (float)value->val2 / 1000000.0f;
}

int main(void)
{
	float filtered_x = 0.0f;
	float filtered_y = -GRAVITY;
	float down_x = 0.0f;
	float down_y = -1.0f;
	float wave_motion = 0.0f;
	bool gravity_initialized = false;
	float previous_g = GRAVITY;
	uint32_t frame = 0;
	int64_t last_angle_change;
	float reference_x = 0.0f;
	float reference_y = 0.0f;
	float reference_z = 0.0f;
	bool have_reference = false;
	int64_t button_pressed_since = -1;
	int64_t shake_guard_until;

	if (!gpio_is_ready_dt(&rgb_data) ||
		gpio_pin_configure_dt(&rgb_data, GPIO_OUTPUT_INACTIVE) < 0)
	{
		printk("RGB matrix D0 is not ready\n");
		return 0;
	}
	if (!gpio_is_ready_dt(&imu_wake) || !i2c_is_ready_dt(&imu_i2c) ||
		gpio_pin_configure_dt(&imu_wake, GPIO_INPUT) < 0 ||
		gpio_pin_interrupt_configure_dt(&imu_wake, GPIO_INT_EDGE_TO_ACTIVE) < 0)
	{
		printk("IMU wake interrupt is not ready\n");
		return 0;
	}
	if (!gpio_is_ready_dt(&mode_button) ||
		gpio_pin_configure_dt(&mode_button, GPIO_INPUT) < 0 ||
		gpio_pin_interrupt_configure_dt(&mode_button, GPIO_INT_EDGE_TO_ACTIVE) < 0)
	{
		printk("Grove Button on D1 is not ready\n");
		return 0;
	}
	gpio_init_callback(&mode_button_cb, mode_button_handler, BIT(mode_button.pin));
	if (gpio_add_callback(mode_button.port, &mode_button_cb) < 0)
	{
		printk("Mode button callback failed\n");
		return 0;
	}
	gpio_init_callback(&imu_wake_cb, imu_wake_handler, BIT(imu_wake.pin));
	if (gpio_add_callback(imu_wake.port, &imu_wake_cb) < 0)
	{
		printk("IMU wake callback failed\n");
		return 0;
	}
	/* Keep the XIAO on-board green user LED off; it is not a matrix pixel. */
	if (gpio_is_ready_dt(&user_led))
	{
		(void)gpio_pin_configure_dt(&user_led, GPIO_OUTPUT_INACTIVE);
	}
	if (!cycle_counter_start())
	{
		printk("CPU cycle counter is not available\n");
		return 0;
	}
	if (!device_is_ready(imu))
	{
		printk("LSM6DS3TR-C is not ready\n");
		return 0;
	}
	if (configure_imu() < 0)
	{
		printk("IMU configuration failed\n");
		return 0;
	}

	memset(pixels, 0, sizeof(pixels));
	matrix_show();
	printk("Electronic ocean ready: %d columns x %d rows, %d pixels on D0\n",
		   MATRIX_COLS, MATRIX_ROWS, PIXEL_COUNT);
	if (enter_imu_sleep() < 0)
	{
		printk("IMU sleep setup failed\n");
		return 0;
	}
	last_angle_change = k_uptime_get();
	shake_guard_until = last_angle_change + SHAKE_GUARD_MS;

	while (1)
	{
		int64_t frame_start = k_uptime_get();
		struct sensor_value accel[3];
		struct sensor_value gyro[3];
		float ax, ay, az, gx, gy, gz, magnitude, motion;

		if (gpio_pin_get_dt(&mode_button) > 0)
		{
			if (button_pressed_since < 0)
			{
				button_pressed_since = frame_start;
			}
			else if (frame_start - button_pressed_since >= BUTTON_SLEEP_MS)
			{
				if (enter_imu_sleep() < 0)
				{
					printk("IMU sleep setup failed\n");
					return 0;
				}
				last_angle_change = k_uptime_get();
				shake_guard_until = last_angle_change + SHAKE_GUARD_MS;
				button_pressed_since = -1;
				have_reference = false;
				continue;
			}
		}
		else
		{
			if (button_pressed_since >= 0 &&
				frame_start - button_pressed_since >= 50 &&
				frame_start - button_pressed_since < BUTTON_SLEEP_MS)
			{
				display_mode =
					(enum display_mode)((display_mode + 1) % MODE_COUNT);
				printk("Display mode: %d\n", display_mode);
			}
			button_pressed_since = -1;
		}

		if (sensor_sample_fetch(imu) < 0 ||
			sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, accel) < 0 ||
			sensor_channel_get(imu, SENSOR_CHAN_GYRO_XYZ, gyro) < 0)
		{
			k_msleep(FRAME_MS);
			continue;
		}

		ax = sensor_value_float(&accel[0]);
		ay = sensor_value_float(&accel[1]);
		az = sensor_value_float(&accel[2]);
		gx = sensor_value_float(&gyro[0]);
		gy = sensor_value_float(&gyro[1]);
		gz = sensor_value_float(&gyro[2]);
		magnitude = sqrtf(ax * ax + ay * ay + az * az);
		/* A short fling can exist for only one 20 ms frame: latch one peak. */
		if (frame_start >= shake_guard_until &&
			fabsf(magnitude - GRAVITY) > SHAKE_SLEEP_DELTA)
		{
			printk("Sleep: strong shake detected\n");
			if (enter_imu_sleep() < 0)
			{
				printk("IMU sleep setup failed\n");
				return 0;
			}
			last_angle_change = k_uptime_get();
			shake_guard_until = last_angle_change + SHAKE_GUARD_MS;
			have_reference = false;
			continue;
		}
		if (magnitude > 0.1f)
		{
			float nx = ax / magnitude;
			float ny = ay / magnitude;
			float nz = az / magnitude;
			float dot = nx * reference_x + ny * reference_y + nz * reference_z;

			if (!have_reference || dot < ANGLE_CHANGE_DOT)
			{
				reference_x = nx;
				reference_y = ny;
				reference_z = nz;
				have_reference = true;
				last_angle_change = frame_start;
			}
		}
		if (frame_start - last_angle_change >= STILL_SLEEP_MS)
		{
			if (enter_imu_sleep() < 0)
			{
				printk("IMU sleep setup failed\n");
				return 0;
			}
			last_angle_change = k_uptime_get();
			shake_guard_until = last_angle_change + SHAKE_GUARD_MS;
			have_reference = false;
			continue;
		}

		/* Project gravity into the display plane, retaining its sign. */
		float screen_x = SCREEN_X_SIGN * ax;
		float screen_y = SCREEN_Y_SIGN * ay;

		if (!gravity_initialized)
		{
			filtered_x = screen_x;
			filtered_y = screen_y;
			gravity_initialized = true;
		}
		filtered_x += 0.22f * (screen_x - filtered_x);
		filtered_y += 0.22f * (screen_y - filtered_y);
		float in_plane = sqrtf(filtered_x * filtered_x +
							   filtered_y * filtered_y);

		/* Face-up/face-down has no unique in-plane downhill direction. */
		if (in_plane > GRAVITY * 0.12f)
		{
			down_x = filtered_x / in_plane;
			down_y = filtered_y / in_plane;
		}

		motion = fabsf(magnitude - GRAVITY) * 0.16f +
				 fabsf(magnitude - previous_g) * 0.38f +
				 (fabsf(gx) + fabsf(gy) + fabsf(gz)) * 0.025f;
		motion = clampf(motion, 0.0f, 1.0f);
		previous_g = magnitude;

		wave_motion += (motion > wave_motion ? 0.30f : 0.025f) *
					   (motion - wave_motion);
		render_ocean(down_x, down_y, wave_motion, frame);
		matrix_show();

		if ((frame % 50U) == 0U)
		{
			printk("Ocean: gravity_x=%d%% gravity_y=%d%% motion=%d%%\n",
				   (int)(down_x * 100.0f),
				   (int)(down_y * 100.0f),
				   (int)(wave_motion * 100.0f));
		}

		++frame;
		int32_t remaining = FRAME_MS -
							(int32_t)(k_uptime_get() - frame_start);

		if (remaining > 0)
		{
			k_msleep(remaining);
		}
	}

	return 0;
}

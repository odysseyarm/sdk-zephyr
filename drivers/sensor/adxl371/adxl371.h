/*
 * Copyright (c) 2018 Analog Devices Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_ADXL371_ADXL371_H_
#define ZEPHYR_DRIVERS_SENSOR_ADXL371_ADXL371_H_

#include <zephyr/drivers/sensor.h>
#include <zephyr/types.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
#include <zephyr/drivers/spi.h>
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
#include <zephyr/drivers/i2c.h>
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c) */

/*
 * ADXL371 registers definition
 */
#define ADXL371_DEVID		0x00u  /* Analog Devices accelerometer ID */
#define ADXL371_DEVID_MST	0x01u  /* Analog Devices MEMS device ID */
#define ADXL371_PARTID		0x02u  /* Device ID */
#define ADXL371_REVID		0x03u  /* product revision ID*/
#define ADXL371_STATUS_1	0x04u  /* Status register 1 */
#define ADXL371_STATUS_2	0x05u  /* Status register 2 */
#define ADXL371_FIFO_ENTRIES_2	0x06u  /* Valid data samples in the FIFO */
#define ADXL371_FIFO_ENTRIES_1	0x07u  /* Valid data samples in the FIFO */
#define ADXL371_X_DATA_H	0x08u  /* X-axis acceleration data [11:4] */
#define ADXL371_X_DATA_L	0x09u  /* X-axis acceleration data [3:0] */
#define ADXL371_Y_DATA_H	0x0Au  /* Y-axis acceleration data [11:4] */
#define ADXL371_Y_DATA_L	0x0Bu  /* Y-axis acceleration data [3:0] */
#define ADXL371_Z_DATA_H	0x0Cu  /* Z-axis acceleration data [11:4] */
#define ADXL371_Z_DATA_L	0x0Du  /* Z-axis acceleration data [3:0] */
#define ADXL371_X_MAXPEAK_H	0x15u  /* X-axis MaxPeak acceleration data */
#define ADXL371_X_MAXPEAK_L	0x16u  /* X-axis MaxPeak acceleration data */
#define ADXL371_Y_MAXPEAK_H	0x17u  /* Y-axis MaxPeak acceleration data */
#define ADXL371_Y_MAXPEAK_L	0x18u  /* Y-axis MaxPeak acceleration data */
#define ADXL371_Z_MAXPEAK_H	0x19u  /* Z-axis MaxPeak acceleration data */
#define ADXL371_Z_MAXPEAK_L	0x1Au  /* Z-axis MaxPeak acceleration data */
#define ADXL371_OFFSET_X	0x20u  /* X axis offset */
#define ADXL371_OFFSET_Y	0x21u  /* Y axis offset */
#define ADXL371_OFFSET_Z	0x22u  /* Z axis offset */
#define ADXL371_X_THRESH_ACT_H	0x23u  /* X axis Activity Threshold [15:8] */
#define ADXL371_X_THRESH_ACT_L	0x24u  /* X axis Activity Threshold [7:0] */
#define ADXL371_Y_THRESH_ACT_H	0x25u  /* Y axis Activity Threshold [15:8] */
#define ADXL371_Y_THRESH_ACT_L	0x26u  /* Y axis Activity Threshold [7:0] */
#define ADXL371_Z_THRESH_ACT_H	0x27u  /* Z axis Activity Threshold [15:8] */
#define ADXL371_Z_THRESH_ACT_L	0x28u  /* Z axis Activity Threshold [7:0] */
#define ADXL371_TIME_ACT	0x29u  /* Activity Time */
#define ADXL371_X_THRESH_INACT_H	0x2Au  /* X axis Inactivity Threshold */
#define ADXL371_X_THRESH_INACT_L	0x2Bu  /* X axis Inactivity Threshold */
#define ADXL371_Y_THRESH_INACT_H	0x2Cu  /* Y axis Inactivity Threshold */
#define ADXL371_Y_THRESH_INACT_L	0x2Du  /* Y axis Inactivity Threshold */
#define ADXL371_Z_THRESH_INACT_H	0x2Eu  /* Z axis Inactivity Threshold */
#define ADXL371_Z_THRESH_INACT_L	0x2Fu  /* Z axis Inactivity Threshold */
#define ADXL371_TIME_INACT_H	0x30u  /* Inactivity Time [15:8] */
#define ADXL371_TIME_INACT_L	0x31u  /* Inactivity Time [7:0] */
#define ADXL371_X_THRESH_ACT2_H	0x32u  /* X axis Activity2 Threshold [15:8] */
#define ADXL371_X_THRESH_ACT2_L	0x33u  /* X axis Activity2 Threshold [7:0] */
#define ADXL371_Y_THRESH_ACT2_H	0x34u  /* Y axis Activity2 Threshold [15:8] */
#define ADXL371_Y_THRESH_ACT2_L	0x35u  /* Y axis Activity2 Threshold [7:0] */
#define ADXL371_Z_THRESH_ACT2_H	0x36u  /* Z axis Activity2 Threshold [15:8] */
#define ADXL371_Z_THRESH_ACT2_L	0x37u  /* Z axis Activity2 Threshold [7:0] */
#define ADXL371_HPF		0x38u  /* High Pass Filter */
#define ADXL371_FIFO_SAMPLES	0x39u  /* FIFO Samples */
#define ADXL371_FIFO_CTL	0x3Au  /* FIFO Control */
#define ADXL371_INT1_MAP	0x3Bu  /* Interrupt 1 mapping control */
#define ADXL371_INT2_MAP        0x3Cu  /* Interrupt 2 mapping control */
#define ADXL371_TIMING		0x3Du  /* Timing */
#define ADXL371_MEASURE		0x3Eu  /* Measure */
#define ADXL371_POWER_CTL	0x3Fu  /* Power control */
#define ADXL371_SELF_TEST	0x40u  /* Self Test */
#define ADXL371_RESET		0x41u  /* Reset */
#define ADXL371_FIFO_DATA	0x42u  /* FIFO Data */

#define ADXL371_DEVID_VAL	0xADu  /* Analog Devices accelerometer ID */
#define ADXL371_MST_DEVID_VAL	0x1Du  /* Analog Devices MEMS device ID */
#define ADXL371_PARTID_VAL	0xFAu  /* Device ID */
#define ADXL371_REVID_VAL	0x02u  /* product revision ID*/
#define ADXL371_RESET_CODE	0x52u  /* Writing code 0x52 resets the device */

#define ADXL371_READ		0x01u
#define ADXL371_REG_READ(x)	(((x & 0xFF) << 1) | ADXL371_READ)
#define ADXL371_REG_WRITE(x)	((x & 0xFF) << 1)
#define ADXL371_TO_I2C_REG(x)	((x) >> 1)

/* ADXL371_POWER_CTL */
#define ADXL371_POWER_CTL_INSTANT_ON_TH_MSK	BIT(5)
#define ADXL371_POWER_CTL_INSTANT_ON_TH_MODE(x)	(((x) & 0x1) << 5)
#define ADXL371_POWER_CTL_FIL_SETTLE_MSK	BIT(4)
#define ADXL371_POWER_CTL_FIL_SETTLE_MODE(x)	(((x) & 0x1) << 4)
#define ADXL371_POWER_CTL_LPF_DIS_MSK		BIT(3)
#define ADXL371_POWER_CTL_LPF_DIS_MODE(x)	(((x) & 0x1) << 3)
#define ADXL371_POWER_CTL_HPF_DIS_MSK		BIT(2)
#define ADXL371_POWER_CTL_HPF_DIS_MODE(x)	(((x) & 0x1) << 2)
#define ADXL371_POWER_CTL_MODE_MSK		GENMASK(1, 0)
#define ADXL371_POWER_CTL_MODE(x)		(((x) & 0x3) << 0)

/* ADXL371_MEASURE */
#define ADXL371_MEASURE_AUTOSLEEP_MSK		BIT(6)
#define ADXL371_MEASURE_AUTOSLEEP_MODE(x)	(((x) & 0x1) << 6)
#define ADXL371_MEASURE_LINKLOOP_MSK		GENMASK(5, 4)
#define ADXL371_MEASURE_LINKLOOP_MODE(x)	(((x) & 0x3) << 4)
#define ADXL371_MEASURE_LOW_NOISE_MSK		BIT(3)
#define ADXL371_MEASURE_LOW_NOISE_MODE(x)	(((x) & 0x1) << 3)
#define ADXL371_MEASURE_BANDWIDTH_MSK		GENMASK(2, 0)
#define ADXL371_MEASURE_BANDWIDTH_MODE(x)	(((x) & 0x7) << 0)

/* ADXL371_TIMING */
#define ADXL371_TIMING_ODR_MSK			GENMASK(7, 5)
#define ADXL371_TIMING_ODR_MODE(x)		(((x) & 0x7) << 5)
#define ADXL371_TIMING_WAKE_UP_RATE_MSK		GENMASK(4, 2)
#define ADXL371_TIMING_WAKE_UP_RATE_MODE(x)	(((x) & 0x7) << 2)
#define ADXL371_TIMING_EXT_CLK_MSK		BIT(1)
#define ADXL371_TIMING_EXT_CLK_MODE(x)		(((x) & 0x1) << 1)
#define ADXL371_TIMING_EXT_SYNC_MSK		BIT(0)
#define ADXL371_TIMING_EXT_SYNC_MODE(x)		(((x) & 0x1) << 0)

/* ADXL371_FIFO_CTL */
#define ADXL371_FIFO_CTL_FORMAT_MSK		GENMASK(5, 3)
#define ADXL371_FIFO_CTL_FORMAT_MODE(x)		(((x) & 0x7) << 3)
#define ADXL371_FIFO_CTL_MODE_MSK		GENMASK(2, 1)
#define ADXL371_FIFO_CTL_MODE_MODE(x)		(((x) & 0x3) << 1)
#define ADXL371_FIFO_CTL_SAMPLES_MSK		BIT(0)
#define ADXL371_FIFO_CTL_SAMPLES_MODE(x)	(((x) > 0xFF) ? 1 : 0)

/* ADXL371_STATUS_1 */
#define ADXL371_STATUS_1_DATA_RDY(x)		(((x) >> 0) & 0x1)
#define ADXL371_STATUS_1_FIFO_RDY(x)		(((x) >> 1) & 0x1)
#define ADXL371_STATUS_1_FIFO_FULL(x)		(((x) >> 2) & 0x1)
#define ADXL371_STATUS_1_FIFO_OVR(x)		(((x) >> 3) & 0x1)
#define ADXL371_STATUS_1_USR_NVM_BUSY(x)	(((x) >> 5) & 0x1)
#define ADXL371_STATUS_1_AWAKE(x)		(((x) >> 6) & 0x1)
#define ADXL371_STATUS_1_ERR_USR_REGS(x)	(((x) >> 7) & 0x1)

/* ADXL371_STATUS_2 */
#define ADXL371_STATUS_2_INACT(x)		(((x) >> 4) & 0x1)
#define ADXL371_STATUS_2_ACTIVITY(x)		(((x) >> 5) & 0x1)
#define ADXL371_STATUS_2_ACTIVITY2(x)		(((x) >> 6) & 0x1)

/* ADXL371_INT1_MAP */
#define ADXL371_INT1_MAP_DATA_RDY_MSK		BIT(0)
#define ADXL371_INT1_MAP_DATA_RDY_MODE(x)	(((x) & 0x1) << 0)
#define ADXL371_INT1_MAP_FIFO_RDY_MSK		BIT(1)
#define ADXL371_INT1_MAP_FIFO_RDY_MODE(x)	(((x) & 0x1) << 1)
#define ADXL371_INT1_MAP_FIFO_FULL_MSK		BIT(2)
#define ADXL371_INT1_MAP_FIFO_FULL_MODE(x)	(((x) & 0x1) << 2)
#define ADXL371_INT1_MAP_FIFO_OVR_MSK		BIT(3)
#define ADXL371_INT1_MAP_FIFO_OVR_MODE(x)	(((x) & 0x1) << 3)
#define ADXL371_INT1_MAP_INACT_MSK		BIT(4)
#define ADXL371_INT1_MAP_INACT_MODE(x)		(((x) & 0x1) << 4)
#define ADXL371_INT1_MAP_ACT_MSK		BIT(5)
#define ADXL371_INT1_MAP_ACT_MODE(x)		(((x) & 0x1) << 5)
#define ADXL371_INT1_MAP_AWAKE_MSK		BIT(6)
#define ADXL371_INT1_MAP_AWAKE_MODE(x)		(((x) & 0x1) << 6)
#define ADXL371_INT1_MAP_LOW_MSK		BIT(7)
#define ADXL371_INT1_MAP_LOW_MODE(x)		(((x) & 0x1) << 7)

/* ADXL371_INT2_MAP */
#define ADXL371_INT2_MAP_DATA_RDY_MSK		BIT(0)
#define ADXL371_INT2_MAP_DATA_RDY_MODE(x)	(((x) & 0x1) << 0)
#define ADXL371_INT2_MAP_FIFO_RDY_MSK		BIT(1)
#define ADXL371_INT2_MAP_FIFO_RDY_MODE(x)	(((x) & 0x1) << 1)
#define ADXL371_INT2_MAP_FIFO_FULL_MSK		BIT(2)
#define ADXL371_INT2_MAP_FIFO_FULL_MODE(x)	(((x) & 0x1) << 2)
#define ADXL371_INT2_MAP_FIFO_OVR_MSK		BIT(3)
#define ADXL371_INT2_MAP_FIFO_OVR_MODE(x)	(((x) & 0x1) << 3)
#define ADXL371_INT2_MAP_INACT_MSK		BIT(4)
#define ADXL371_INT2_MAP_INACT_MODE(x)		(((x) & 0x1) << 4)
#define ADXL371_INT2_MAP_ACT_MSK		BIT(5)
#define ADXL371_INT2_MAP_ACT_MODE(x)		(((x) & 0x1) << 5)
#define ADXL371_INT2_MAP_AWAKE_MSK		BIT(6)
#define ADXL371_INT2_MAP_AWAKE_MODE(x)		(((x) & 0x1) << 6)
#define ADXL371_INT2_MAP_LOW_MSK		BIT(7)
#define ADXL371_INT2_MAP_LOW_MODE(x)		(((x) & 0x1) << 7)

/* ADXL371_HPF */
#define ADXL371_HPF_CORNER(x)			(((x) & 0x3) << 0)

enum adxl371_axis {
	ADXL371_X_AXIS,
	ADXL371_Y_AXIS,
	ADXL371_Z_AXIS
};

enum adxl371_op_mode {
	ADXL371_STANDBY,
	ADXL371_WAKE_UP,
	ADXL371_INSTANT_ON,
	ADXL371_FULL_BW_MEASUREMENT
};

enum adxl371_bandwidth {
	ADXL371_BW_200HZ,
	ADXL371_BW_400HZ,
	ADXL371_BW_800HZ,
	ADXL371_BW_1600HZ,
	ADXL371_BW_3200HZ,
	ADXL371_BW_LPF_DISABLED = 0xC,
};

enum adxl371_hpf_corner {
	ADXL371_HPF_CORNER_0,
	ADXL371_HPF_CORNER_1,
	ADXL371_HPF_CORNER_2,
	ADXL371_HPF_CORNER_3,
	ADXL371_HPF_DISABLED,
};

enum adxl371_act_proc_mode {
	ADXL371_DEFAULT,
	ADXL371_LINKED,
	ADXL371_LOOPED
};

enum adxl371_odr {
	ADXL371_ODR_400HZ,
	ADXL371_ODR_800HZ,
	ADXL371_ODR_1600HZ,
	ADXL371_ODR_3200HZ,
	ADXL371_ODR_6400HZ
};

enum adxl371_instant_on_th_mode {
	ADXL371_INSTANT_ON_LOW_TH,
	ADXL371_INSTANT_ON_HIGH_TH
};

enum adxl371_wakeup_rate {
	ADXL371_WUR_52ms,
	ADXL371_WUR_104ms,
	ADXL371_WUR_208ms,
	ADXL371_WUR_512ms,
	ADXL371_WUR_2048ms,
	ADXL371_WUR_4096ms,
	ADXL371_WUR_8192ms,
	ADXL371_WUR_24576ms
};

enum adxl371_filter_settle {
	ADXL371_FILTER_SETTLE_370,
	ADXL371_FILTER_SETTLE_16
};

enum adxl371_fifo_format {
	ADXL371_XYZ_FIFO,
	ADXL371_X_FIFO,
	ADXL371_Y_FIFO,
	ADXL371_XY_FIFO,
	ADXL371_Z_FIFO,
	ADXL371_XZ_FIFO,
	ADXL371_YZ_FIFO,
	ADXL371_XYZ_PEAK_FIFO,
};

enum adxl371_fifo_mode {
	ADXL371_FIFO_BYPASSED,
	ADXL371_FIFO_STREAMED,
	ADXL371_FIFO_TRIGGERED,
	ADXL371_FIFO_OLD_SAVED
};

struct adxl371_fifo_config {
	enum adxl371_fifo_mode fifo_mode;
	enum adxl371_fifo_format fifo_format;
	uint16_t fifo_samples;
};

struct adxl371_activity_threshold {
	uint16_t thresh;
	bool referenced;
	bool enable;
};

struct adxl371_xyz_accel_data {
	int16_t x;
	int16_t y;
	int16_t z;
};

struct adxl371_transfer_function {
	int (*read_reg_multiple)(const struct device *dev, uint8_t reg_addr,
				 uint8_t *value, uint16_t len);
	int (*write_reg)(const struct device *dev, uint8_t reg_addr,
			 uint8_t value);
	int (*read_reg)(const struct device *dev, uint8_t reg_addr,
			uint8_t *value);
	int (*write_reg_mask)(const struct device *dev, uint8_t reg_addr,
			      uint32_t mask, uint8_t value);
};

struct adxl371_data {
	struct adxl371_xyz_accel_data sample;
	const struct adxl371_transfer_function *hw_tf;
	struct adxl371_fifo_config fifo_config;
	enum adxl371_act_proc_mode act_proc_mode;
#ifdef CONFIG_ADXL371_TRIGGER
	struct gpio_callback gpio_cb;

	sensor_trigger_handler_t th_handler;
	const struct sensor_trigger *th_trigger;
	sensor_trigger_handler_t drdy_handler;
	const struct sensor_trigger *drdy_trigger;
	const struct device *dev;

#if defined(CONFIG_ADXL371_TRIGGER_OWN_THREAD)
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_ADXL371_THREAD_STACK_SIZE);
	struct k_sem gpio_sem;
	struct k_thread thread;
#elif defined(CONFIG_ADXL371_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif
#endif /* CONFIG_ADXL371_TRIGGER */
};

struct adxl371_dev_config {
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
	struct i2c_dt_spec i2c;
#endif
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
	struct spi_dt_spec spi;
#endif
	int (*bus_init)(const struct device *dev);
#ifdef CONFIG_ADXL371_TRIGGER
	struct gpio_dt_spec interrupt;
#endif

	enum adxl371_bandwidth bw;
	enum adxl371_hpf_corner hpf;
	enum adxl371_odr odr;

	bool max_peak_detect_mode;

	/* Device Settings */
	bool autosleep;

	struct adxl371_activity_threshold activity_th;
	struct adxl371_activity_threshold activity2_th;
	struct adxl371_activity_threshold inactivity_th;
	struct adxl371_fifo_config fifo_config;

	enum adxl371_wakeup_rate wur;
	enum adxl371_instant_on_th_mode	th_mode;
	enum adxl371_filter_settle filter_settle;
	enum adxl371_op_mode op_mode;

	uint16_t inactivity_time;
	uint8_t activity_time;
	uint8_t int1_config;
	uint8_t int2_config;
};

int adxl371_spi_init(const struct device *dev);
int adxl371_i2c_init(const struct device *dev);

#ifdef CONFIG_ADXL371_TRIGGER
int adxl371_get_status(const struct device *dev,
		       uint8_t *status1, uint8_t *status2, uint16_t *fifo_entries);

int adxl371_reg_write_mask(const struct device *dev,
			   uint8_t reg_addr, uint32_t mask, uint8_t data);

int adxl371_trigger_set(const struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler);

int adxl371_init_interrupt(const struct device *dev);
#endif /* CONFIG_ADT7420_TRIGGER */

#endif /* ZEPHYR_DRIVERS_SENSOR_ADXL371_ADXL371_H_ */

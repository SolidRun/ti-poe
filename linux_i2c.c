#include <stdint.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <stdio.h>

static int i2c_dev_fd;
static uint8_t cached_i2c_slave_addr;

int linux_i2c_open_dev(char *dev)
{
	i2c_dev_fd = open(dev, O_RDWR);
	if (i2c_dev_fd < 0) {
		printf("%s: open failed: %s\n", dev, strerror(errno));
		return -1;
	}

	return 0;
}

static int linux_i2c_set_slave_addr(uint8_t addr)
{
	int ret;

	if (addr == cached_i2c_slave_addr)
		return 0;

	ret = ioctl(i2c_dev_fd, I2C_SLAVE, addr);
	if (ret < 0)
		printf("Address set to 0x%02x failed: %s\n", addr,
				strerror(errno));
	else
		cached_i2c_slave_addr = addr;

	return ret;
}

static int linux_i2c_smbus_xfer(uint8_t read_write, uint8_t cmd, uint32_t size,
		union i2c_smbus_data *data)
{
	struct i2c_smbus_ioctl_data ioctl_arg;

	ioctl_arg.read_write = read_write;
	ioctl_arg.command = cmd;
	ioctl_arg.size = size;
	ioctl_arg.data = data;

	return ioctl(i2c_dev_fd, I2C_SMBUS, &ioctl_arg);
}

/*
 * Implementation of tps238x.c I2C access routines
 *
 * Note: Return value of these routines is not well defined in TI code. This
 * implementation assumes that return value of 0 is success, and >0 is
 * failure.
 */

uint8_t tps_WriteI2CReg(uint8_t i2cAddress, uint8_t registerAddress,
		uint8_t Value)
{
	union i2c_smbus_data data;
	int ret;

	ret = linux_i2c_set_slave_addr(i2cAddress);
	if (ret < 0)
		return 1;

	data.byte = Value;
	ret = linux_i2c_smbus_xfer(I2C_SMBUS_WRITE, registerAddress,
			I2C_SMBUS_BYTE_DATA, &data);
	if (ret < 0) {
		printf("%s: Write to 0x%02x failed: %s\n", __func__,
			registerAddress, strerror(errno));
		return 1;
	}

	return 0;
}

uint8_t tps_WriteI2CMultiple(uint8_t i2cAddress, uint8_t registerAddress,
		uint8_t *writeValues, uint8_t numWriteBytes)
{
	int ret;

	ret = linux_i2c_set_slave_addr(i2cAddress);
	if (ret < 0)
		return 1;

	ret = linux_i2c_smbus_xfer(I2C_SMBUS_WRITE, registerAddress,
			I2C_SMBUS_BYTE, NULL);
	if (ret < 0) {
		printf("%s: Register 0x%02x set failed: %s\n", __func__,
			registerAddress, strerror(errno));
		return 1;
	}

	for (uint8_t cnt = 0; cnt < numWriteBytes; cnt++) {
		ret = linux_i2c_smbus_xfer(I2C_SMBUS_WRITE, writeValues[cnt],
				I2C_SMBUS_BYTE, NULL);
		if (ret < 0) {
			printf("%s: Register 0x%02x write failed: %s\n",
					__func__, registerAddress,
					strerror(errno));
			return 1;
		}
	}

	return 0;
}

uint8_t tps_ReadI2CReg(uint8_t i2cAddress, uint8_t registerAddress,
		uint8_t *readValue)
{
	union i2c_smbus_data data;
	int ret;

	ret = linux_i2c_set_slave_addr(i2cAddress);
	if (ret < 0)
		return 1;

	ret = linux_i2c_smbus_xfer(I2C_SMBUS_READ, registerAddress,
		I2C_SMBUS_BYTE_DATA, &data);
	if (ret < 0) {
		printf("%s: Read from 0x%02x failed: %s\n", __func__,
			registerAddress, strerror(errno));
		return 1;
	}

	*readValue = data.byte;

	return 0;
}

uint8_t tps_ReadI2CMultiple(uint8_t i2cAddress, uint8_t registerAddress,
		uint8_t *readValue, uint8_t numReadBytes)
{
	union i2c_smbus_data data;
	int ret;

	ret = linux_i2c_set_slave_addr(i2cAddress);
	if (ret < 0)
		return 1;

	ret = linux_i2c_smbus_xfer(I2C_SMBUS_WRITE, registerAddress,
			I2C_SMBUS_BYTE, NULL);
	if (ret < 0) {
		printf("%s: Register 0x%02x set failed: %s\n", __func__,
			registerAddress, strerror(errno));
		return 1;
	}

	for (uint8_t cnt = 0; cnt < numReadBytes; cnt++) {
		ret = linux_i2c_smbus_xfer(I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE,
				&data);
		if (ret < 0) {
			printf("%s: Register 0x%02x read failed: %s\n",
					__func__, registerAddress,
					strerror(errno));
			return 1;
		}
		readValue[cnt] = data.byte;
	}

	return 0;
}

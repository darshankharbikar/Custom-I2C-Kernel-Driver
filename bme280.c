#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/mutex.h>

#define BME280_CHIP_ID_REG 0xD0
#define BME280_EXPECTED_CHIP_ID 0x60
#define BME280_TEMP_MSB_REG 0xFA

struct bme280_data {
	struct i2c_client *client;
	struct mutex lock;
	u8 chip_id;
};

static ssize_t temp_show(struct device *dev,
			 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct bme280_data *data = i2c_get_clientdata(client);
	int msb, lsb, xlsb;
	s32 raw_temp;
	int ret;

	mutex_lock(&data->lock);

	ret = i2c_smbus_read_byte_data(client, BME280_TEMP_MSB_REG);
	if (ret < 0)
		goto out;
	msb = ret;

	ret = i2c_smbus_read_byte_data(client, BME280_TEMP_MSB_REG + 1);
	if (ret < 0)
		goto out;
	lsb = ret;

	ret = i2c_smbus_read_byte_data(client, BME280_TEMP_MSB_REG + 2);
	if (ret < 0)
		goto out;
	xlsb = ret;

	raw_temp = ((msb << 12) | (lsb << 4) | (xlsb >> 4));

	mutex_unlock(&data->lock);

	/* raw ADC value (uncompensated). Real driver would apply calibration */
	return sprintf(buf, "%d\n", raw_temp);

out:
	mutex_unlock(&data->lock);
	return ret;
}

static DEVICE_ATTR_RO(temp);

static struct attribute *bme280_attrs[] = {
	&dev_attr_temp.attr,
	NULL,
};
ATTRIBUTE_GROUPS(bme280);

static int bme280_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct bme280_data *data;
	int ret;

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	i2c_set_clientdata(client, data);
	data->client = client;
	mutex_init(&data->lock);

	ret = i2c_smbus_read_byte_data(client, BME280_CHIP_ID_REG);
	if (ret < 0)
		return ret;

	data->chip_id = ret;
	if (data->chip_id != BME280_EXPECTED_CHIP_ID) {
		dev_err(&client->dev, "Invalid chip ID: 0x%x\n", data->chip_id);
		return -ENODEV;
	}

	dev_info(&client->dev, "BME280 detected, chip ID: 0x%x\n", data->chip_id);

	return sysfs_create_groups(&client->dev.kobj, bme280_groups);
}

static void bme280_remove(struct i2c_client *client)
{
	sysfs_remove_groups(&client->dev.kobj, bme280_groups);
}

static const struct i2c_device_id bme280_id[] = {
	{ "bme280", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bme280_id);

#ifdef CONFIG_OF
static const struct of_device_id bme280_of_match[] = {
	{ .compatible = "bosch,bme280" },
	{ }
};
MODULE_DEVICE_TABLE(of, bme280_of_match);
#endif

static struct i2c_driver bme280_driver = {
	.driver = {
		.name = "bme280_custom",
		.of_match_table = of_match_ptr(bme280_of_match),
	},
	.probe = bme280_probe,
	.remove = bme280_remove,
	.id_table = bme280_id,
};

module_i2c_driver(bme280_driver);

MODULE_AUTHOR("Darshan Kharbikar");
MODULE_DESCRIPTION("Custom BME280 I2C Driver for Raspberry Pi (Linux 6.x)");
MODULE_LICENSE("GPL");

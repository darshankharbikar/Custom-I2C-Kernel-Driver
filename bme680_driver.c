// bme680_driver.c      # Main kernel module source

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/sysfs.h>
#include <linux/slab.h>
#include <linux/mutex.h>

#define BME680_CHIP_ID_REG      0xD0
#define BME680_EXPECTED_CHIP_ID 0x61
#define BME680_TEMP_MSB_REG     0x22  // Temp data: 0x22(MSB),0x23(LSB),0x24(XLSB)

struct bme680_data {
    struct i2c_client *client;
    struct mutex lock;
    u8 chip_id;
};

static ssize_t temp_show(struct device *dev,
                struct device_attribute *attr, char *buf)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct bme680_data *data = i2c_get_clientdata(client);
    int msb, lsb, xlsb;
    s32 raw_temp;
    int ret;

    mutex_lock(&data->lock);

    // Trigger forced measurement
    ret = i2c_smbus_write_byte_data(client, 0x74, 0x24);  // osrs_t=x1, forced
    if (ret < 0) goto out;
    usleep_range(5000, 6000);  // 5ms wait per datasheet

    // Read temperature data registers
    ret = i2c_smbus_read_byte_data(client, BME680_TEMP_MSB_REG);
    if (ret < 0) goto out;
    msb = ret;

    ret = i2c_smbus_read_byte_data(client, BME680_TEMP_MSB_REG + 1);
    if (ret < 0) goto out;
    lsb = ret;

    ret = i2c_smbus_read_byte_data(client, BME680_TEMP_MSB_REG + 2);
    if (ret < 0) goto out;
    xlsb = ret;

    raw_temp = (msb << 12) | (lsb << 4) | (xlsb >> 4);
    mutex_unlock(&data->lock);
    return sprintf(buf, "%d\n", raw_temp);  // Raw 20-bit ADC

out:
    mutex_unlock(&data->lock);
    dev_err(&client->dev, "Temp read failed: %d\n", ret);
    return ret;
}

static DEVICE_ATTR_RO(temp);

static struct attribute *bme680_attrs[] = {
    &dev_attr_temp.attr,
    NULL,
};
ATTRIBUTE_GROUPS(bme680);

static int bme680_probe(struct i2c_client *client,
            const struct i2c_device_id *id)
{
    struct bme680_data *data;
    int ret;

    data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    i2c_set_clientdata(client, data);
    data->client = client;
    mutex_init(&data->lock);

    ret = i2c_smbus_read_byte_data(client, BME680_CHIP_ID_REG);
    if (ret < 0)
        return ret;

    data->chip_id = ret;
    if (data->chip_id != BME680_EXPECTED_CHIP_ID) {
        dev_err(&client->dev, "Invalid chip ID: 0x%x (expected 0x61)\n", 
                data->chip_id);
        return -ENODEV;
    }

    // BME680 config: temp x1 oversampling, forced mode, no gas heater
    ret = i2c_smbus_write_byte_data(client, 0x72, 0x01);  // osrs_h x1
    if (ret < 0) return ret;
    ret = i2c_smbus_write_byte_data(client, 0x74, 0x24);  // osrs_t x1, forced
    if (ret < 0) return ret;
    ret = i2c_smbus_write_byte_data(client, 0x75, 0x00);  // Disable gas heater

    dev_info(&client->dev, "BME680 detected, chip ID: 0x%x\n", data->chip_id);
    return sysfs_create_groups(&client->dev.kobj, bme680_groups);
}

static void bme680_remove(struct i2c_client *client)
{
    sysfs_remove_groups(&client->dev.kobj, bme680_groups);
}

static const struct i2c_device_id bme680_id[] = {
    { "bme680", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, bme680_id);

#ifdef CONFIG_OF
static const struct of_device_id bme680_of_match[] = {
    { .compatible = "bosch,bme680" },
    { }
};
MODULE_DEVICE_TABLE(of, bme680_of_match);
#endif

static struct i2c_driver bme680_driver = {
    .driver = {
        .name = "bme680_custom",
        .of_match_table = of_match_ptr(bme680_of_match),
    },
    .probe = bme680_probe,
    .remove = bme680_remove,
    .id_table = bme680_id,
};

module_i2c_driver(bme680_driver);

MODULE_AUTHOR("Darshan Kharbikar");
MODULE_DESCRIPTION("Custom BME680 I2C Driver for Raspberry Pi (Linux 6.x)");
MODULE_LICENSE("GPL");


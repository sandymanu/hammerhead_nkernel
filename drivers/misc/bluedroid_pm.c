/*
 * drivers/misc/bluedroid_pm.c
 *
 * Copyright (c) 2014, NVIDIA Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * ldisc support based on drivers/bluetooth/bcmbt_lpm_ldisc.c
 *	Copyright 2003 - 2011 Broadcom Corporation.
 *
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/rfkill.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/wakelock.h>
#include <linux/serial_core.h>
#include <linux/tty.h>

#define BCM_SHARED_UART_MAGIC	0x80
#define TIO_ASSERT_BT_WAKE	_IO(BCM_SHARED_UART_MAGIC, 3)
#define TIO_DEASSERT_BT_WAKE	_IO(BCM_SHARED_UART_MAGIC, 4)
#define TIO_GET_BT_WAKE_STATE	_IO(BCM_SHARED_UART_MAGIC, 5)

/* BT may not be ready to sleep, these parameters configure retries */
#define TX_RETRY_INTERVAL 5
#define MAX_SLEEP_RETRIES 5

struct bluedroid_pm_data {
	struct gpio_desc *gpio_shutdown;
	struct gpio_desc *host_wake;
	struct gpio_desc *ext_wake;
	bool is_blocked;
	unsigned host_wake_irq;
	bool wake_on_irq;
	int retries;
	struct regulator *vdd_3v3;
	struct regulator *vdd_1v8;
	struct rfkill *rfkill;
	struct wake_lock wake_lock;
	struct device *dev;
	struct delayed_work retry_work;
};

static struct tty_ldisc_ops bluedroid_pm_ldisc_ops;

static irqreturn_t bluedroid_pm_hostwake_isr(int irq, void *dev_id)
{
	/* This handler is empty, but allows us to register with pm_suspend */
	return IRQ_HANDLED;
}

/**
 * Handles bluedroid_pm retries
 * @param data: bluedroid_pm delayed work structure.
 */
static void bluedroid_pm_work(struct work_struct *work)
{
	struct bluedroid_pm_data *bluedroid_pm =
		container_of(work, struct bluedroid_pm_data, retry_work.work);

	bluedroid_pm->retries = bluedroid_pm->retries + 1;

	if (bluedroid_pm->retries >= MAX_SLEEP_RETRIES)
		dev_err(bluedroid_pm->dev, "Couldn't sleep after %d tries",
			bluedroid_pm->retries);

	if (!gpiod_get_value_cansleep(bluedroid_pm->host_wake)) {
		/* BT can sleep */
		dev_dbg(bluedroid_pm->dev, "Tx and Rx are idle, BT sleeping");
		gpiod_set_value_cansleep(bluedroid_pm->ext_wake, 1);
		wake_unlock(&bluedroid_pm->wake_lock);
	} else {
		/* Try again later */
		dev_dbg(bluedroid_pm->dev, "Rx is busy, try again later");
		schedule_delayed_work(&bluedroid_pm->retry_work,
			(TX_RETRY_INTERVAL * HZ));
	}
}

static int bluedroid_pm_tty_ioctl(struct tty_struct *tty, struct file *file,
				unsigned int cmd, unsigned long arg)
{
	struct bluedroid_pm_data *bluedroid_pm;

	if (!tty || tty->magic != TTY_MAGIC)
		return -ENODEV;

	bluedroid_pm = tty->driver_data;
	if (!bluedroid_pm)
		return -ENODEV;

	switch (cmd) {
	case TIO_ASSERT_BT_WAKE:
		dev_dbg(bluedroid_pm->dev, "TIO_ASSERT_BT_WAKE\n");
		gpiod_set_value_cansleep(bluedroid_pm->ext_wake, 0);
		wake_lock(&bluedroid_pm->wake_lock);
		cancel_delayed_work_sync(&bluedroid_pm->retry_work);
		return 0;

	case TIO_DEASSERT_BT_WAKE:
		dev_dbg(bluedroid_pm->dev, "TIO_DEASSERT_BT_WAKE\n");
		bluedroid_pm->retries = 0;
		schedule_delayed_work(&bluedroid_pm->retry_work, 0);
		return 0;

	case TIO_GET_BT_WAKE_STATE:
		dev_dbg(bluedroid_pm->dev, "TIO_GET_BT_WAKE_STATE\n");
		return gpiod_get_value_cansleep(bluedroid_pm->ext_wake);

	default:
		return n_tty_ioctl_helper(tty, file, cmd, arg);
	}
}

static int bluedroid_pm_tty_init(void)
{
	int err;

	/* Inherit the N_TTY's ops */
	n_tty_inherit_ops(&bluedroid_pm_ldisc_ops);

	bluedroid_pm_ldisc_ops.owner = THIS_MODULE;
	bluedroid_pm_ldisc_ops.name = "bluedroid_pm_tty";
	bluedroid_pm_ldisc_ops.ioctl = bluedroid_pm_tty_ioctl;

	err = tty_register_ldisc(N_BRCM_HCI, &bluedroid_pm_ldisc_ops);
	if (err)
		pr_err("can't register N_BRCM_HCI line discipline\n");
	else
		pr_info("N_BRCM_HCI line discipline registered\n");

	return err;
}

static void bluedroid_pm_tty_cleanup(void)
{
	int err;

	err = tty_unregister_ldisc(N_BRCM_HCI);
	if (err)
		pr_err("can't unregister N_BRCM_HCI line discipline\n");
	else
		pr_info("N_BRCM_HCI line discipline removed\n");
}

static int bluedroid_pm_rfkill_set_power(void *data, bool blocked)
{
	struct bluedroid_pm_data *bluedroid_pm = data;
	/*
	 * check if BT gpio_shutdown line status and current request are same.
	 * If same, then return, else perform requested operation.
	 */
	if (gpiod_get_value_cansleep(bluedroid_pm->gpio_shutdown) == blocked)
		return 0;

	if (blocked) {
		gpiod_set_value_cansleep(bluedroid_pm->gpio_shutdown, 1);
		regulator_disable(bluedroid_pm->vdd_3v3);
		regulator_disable(bluedroid_pm->vdd_1v8);
		wake_unlock(&bluedroid_pm->wake_lock);
	} else {
		int ret = 0;

		ret = regulator_enable(bluedroid_pm->vdd_3v3);
		if (ret) {
			dev_err(bluedroid_pm->dev, "vdd_3v3 not enabled\n");
			return ret;
		}

		ret = regulator_enable(bluedroid_pm->vdd_1v8);
		if (ret) {
			dev_err(bluedroid_pm->dev, "vdd_1v8 not enabled\n");
			regulator_disable(bluedroid_pm->vdd_3v3);
			return ret;
		}
		gpiod_set_value_cansleep(bluedroid_pm->gpio_shutdown, 0);
	}
	bluedroid_pm->is_blocked = blocked;

	return 0;
}

static const struct rfkill_ops bluedroid_pm_rfkill_ops = {
	.set_block = bluedroid_pm_rfkill_set_power,
};

static const struct of_device_id bluedroid_match[] = {
	{ .compatible = "bcm,bt-bcm4354" },
	{}
};
MODULE_DEVICE_TABLE(of, bluedroid_match);

static int bluedroid_pm_probe(struct platform_device *pdev)
{
	static struct bluedroid_pm_data *bluedroid_pm;
	int ret;

	bluedroid_pm = devm_kzalloc(&pdev->dev, sizeof(*bluedroid_pm),
				GFP_KERNEL);
	if (!bluedroid_pm)
		return -ENOMEM;

	bluedroid_pm->dev = &pdev->dev;

	bluedroid_pm->vdd_3v3 = devm_regulator_get(&pdev->dev, "avdd");
	if (IS_ERR(bluedroid_pm->vdd_3v3)) {
		dev_err(bluedroid_pm->dev, "regulator avdd not available\n");
		return PTR_ERR(bluedroid_pm->vdd_3v3);
	}

	bluedroid_pm->vdd_1v8 = devm_regulator_get(&pdev->dev, "dvdd");
	if (IS_ERR(bluedroid_pm->vdd_1v8)) {
		dev_err(bluedroid_pm->dev, "regulator dvdd not available\n");
		return PTR_ERR(bluedroid_pm->vdd_1v8);
	}

	bluedroid_pm->gpio_shutdown = devm_gpiod_get(bluedroid_pm->dev,
					"bt_reg_on", GPIOD_OUT_HIGH);
	if (IS_ERR(bluedroid_pm->gpio_shutdown)) {
		dev_err(bluedroid_pm->dev, "shutdown gpio not registered\n");
		return PTR_ERR(bluedroid_pm->gpio_shutdown);
	}

	bluedroid_pm->ext_wake = devm_gpiod_get(bluedroid_pm->dev,
					"bt_ext_wake", GPIOD_OUT_LOW);
	if (IS_ERR(bluedroid_pm->ext_wake)) {
		dev_err(bluedroid_pm->dev, "ext_wake gpio not registered\n");
		return PTR_ERR(bluedroid_pm->ext_wake);
	}

	bluedroid_pm->host_wake = devm_gpiod_get(bluedroid_pm->dev,
					"bt_host_wake", GPIOD_IN);
	if (IS_ERR(bluedroid_pm->host_wake)) {
		dev_err(bluedroid_pm->dev, "host_wake gpio not registered\n");
		return PTR_ERR(bluedroid_pm->host_wake);
	}

	bluedroid_pm->host_wake_irq = gpiod_to_irq(bluedroid_pm->host_wake);
	ret = devm_request_irq(bluedroid_pm->dev,
				bluedroid_pm->host_wake_irq,
				bluedroid_pm_hostwake_isr,
				IRQF_TRIGGER_RISING,
				"bluetooth hostwake", bluedroid_pm);
	if (ret) {
		dev_err(bluedroid_pm->dev, "Failed to get host_wake irq\n");
		return ret;
	}

	/*
	 * At this point the GPIOs and regulators avaiable to
	 * register with rfkill are defined
	 */
	bluedroid_pm->rfkill = rfkill_alloc("bluedroid_pm", &pdev->dev,
			RFKILL_TYPE_BLUETOOTH, &bluedroid_pm_rfkill_ops,
			bluedroid_pm);

	if (unlikely(!bluedroid_pm->rfkill))
		return -ENOMEM;

	bluedroid_pm->is_blocked = true;
	rfkill_set_states(bluedroid_pm->rfkill,
		bluedroid_pm->is_blocked, false);

	ret = rfkill_register(bluedroid_pm->rfkill);
	if (unlikely(ret)) {
		rfkill_destroy(bluedroid_pm->rfkill);
		dev_err(bluedroid_pm->dev, "Couldn't register rfkill");
		return ret;
	}

	wake_lock_init(&bluedroid_pm->wake_lock, WAKE_LOCK_SUSPEND,
			"bluedroid_pm");

	INIT_DELAYED_WORK(&bluedroid_pm->retry_work, bluedroid_pm_work);

	ret = bluedroid_pm_tty_init();
	if (ret) {
		dev_err(bluedroid_pm->dev, "tty_init failed");
		cancel_delayed_work_sync(&bluedroid_pm->retry_work);
		wake_lock_destroy(&bluedroid_pm->wake_lock);
		rfkill_unregister(bluedroid_pm->rfkill);
		rfkill_destroy(bluedroid_pm->rfkill);
		return ret;
	}

	platform_set_drvdata(pdev, bluedroid_pm);
	dev_dbg(bluedroid_pm->dev, "driver successfully registered");
	return 0;
}

static int bluedroid_pm_remove(struct platform_device *pdev)
{
	struct bluedroid_pm_data *bluedroid_pm = platform_get_drvdata(pdev);

	bluedroid_pm_tty_cleanup();
	cancel_delayed_work_sync(&bluedroid_pm->retry_work);
	wake_lock_destroy(&bluedroid_pm->wake_lock);
	rfkill_unregister(bluedroid_pm->rfkill);
	rfkill_destroy(bluedroid_pm->rfkill);
	return 0;
}

static int __maybe_unused bluedroid_pm_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bluedroid_pm_data *bluedroid_pm = platform_get_drvdata(pdev);

	if (!bluedroid_pm->is_blocked)
		bluedroid_pm->wake_on_irq =
			(enable_irq_wake(bluedroid_pm->host_wake_irq) == 0);

	return 0;
}

static int __maybe_unused bluedroid_pm_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct bluedroid_pm_data *bluedroid_pm = platform_get_drvdata(pdev);

	if (bluedroid_pm->wake_on_irq) {
		disable_irq_wake(bluedroid_pm->host_wake_irq);
		bluedroid_pm->wake_on_irq = false;
	}

	return 0;
}

static SIMPLE_DEV_PM_OPS(bluedroid_pm_ops,
	bluedroid_pm_suspend, bluedroid_pm_resume);

static struct platform_driver bluedroid_pm_driver = {
	.probe = bluedroid_pm_probe,
	.remove = bluedroid_pm_remove,
	.driver = {
		.name = "bluedroid_pm",
		.pm = &bluedroid_pm_ops,
		.of_match_table = bluedroid_match,
	},
};

module_platform_driver(bluedroid_pm_driver);

MODULE_ALIAS_LDISC(N_BRCM_HCI);
MODULE_DESCRIPTION("bluedroid PM");
MODULE_AUTHOR("NVIDIA");
MODULE_LICENSE("GPL");

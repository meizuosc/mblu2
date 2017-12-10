/*
 * meizu Class Core
 *
 * Copyright (C) 2005 John Lenz <lenz@cs.wisc.edu>
 * Copyright (C) 2005-2007 Richard Purdie <rpurdie@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
/*dixiaobing@wind-mobi.com 20150617 start*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/meizu.h>


static struct class *meizu_class;
DECLARE_RWSEM(meizu_list_lock);
EXPORT_SYMBOL_GPL(meizu_list_lock);

LIST_HEAD(meizu_list);
EXPORT_SYMBOL_GPL(meizu_list);

/**
 * meizu_classdev_register - register a new object of meizu_classdev class.
 * @parent: The device to register.
 * @meizu_cdev: the meizu_classdev structure for this device.
 */
int meizu_classdev_register(struct device *parent, struct meizu_classdev *meizu_cdev)
{
	meizu_cdev->dev = device_create(meizu_class, parent, 0, meizu_cdev,
				      "%s", meizu_cdev->name);
	if (IS_ERR(meizu_cdev->dev))
		return PTR_ERR(meizu_cdev->dev);
	/* add to the list of meizu */
	down_write(&meizu_list_lock);
	list_add_tail(&meizu_cdev->node, &meizu_list);
	up_write(&meizu_list_lock);

	dev_dbg(parent, "Registered meizu device: %s\n",
			meizu_cdev->name);

	return 0;
}
EXPORT_SYMBOL_GPL(meizu_classdev_register);

/**
 * meizu_classdev_unregister - unregisters a object of meizu_properties class.
 * @meizu_cdev: the meizu device to unregister
 *
 * Unregisters a previously registered via meizu_classdev_register object.
 */
void meizu_classdev_unregister(struct meizu_classdev *meizu_cdev)
{
	device_unregister(meizu_cdev->dev);

	down_write(&meizu_list_lock);
	list_del(&meizu_cdev->node);
	up_write(&meizu_list_lock);
}
EXPORT_SYMBOL_GPL(meizu_classdev_unregister);

static int __init meizu_init(void)
{
	meizu_class = class_create(THIS_MODULE, "meizu");
	if (IS_ERR(meizu_class))
		return PTR_ERR(meizu_class);
	return 0;
}

static void __exit meizu_exit(void)
{
	class_destroy(meizu_class);
}

subsys_initcall(meizu_init);
module_exit(meizu_exit);

MODULE_AUTHOR("dixiaobing");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("meizu Class Interface");
/*dixiaobing@wind-mobi.com 20150617 end*/
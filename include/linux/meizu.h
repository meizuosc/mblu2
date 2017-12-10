/*
 * Driver model for meizus and meizu triggers
 *
 * Copyright (C) 2005 John Lenz <lenz@cs.wisc.edu>
 * Copyright (C) 2005 Richard Purdie <rpurdie@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
 /*dixiaobing@wind-mobi.com 20150617 start*/
#ifndef __LINUX_meizuS_H_INCLUDED
#define __LINUX_meizuS_H_INCLUDED

#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/rwsem.h>
#include <linux/timer.h>
#include <linux/workqueue.h>


struct device;

struct meizu_classdev {
	const char		*name;
	struct device		*dev;
	struct list_head	 node;			/* meizu Device list */
};

extern int meizu_classdev_register(struct device *parent,
				 struct meizu_classdev *meizu_cdev);
extern void meizu_classdev_unregister(struct meizu_classdev *meizu_cdev);

#endif
/*dixiaobing@wind-mobi.com 20150617 end*/
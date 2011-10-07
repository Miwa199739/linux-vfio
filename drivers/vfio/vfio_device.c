/*
 * Copyright (C) 2011 Red Hat, Inc.  All rights reserved.
 *     Author: Alex Williamson <alex.williamson@redhat.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Derived from original vfio:
 * Copyright 2010 Cisco Systems, Inc.  All rights reserved.
 * Author: Tom Lyon, pugs@cisco.com
 */

/*
 * VFIO device module: Common device handling and callouts to other drivers
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/eventfd.h>
#include <linux/uaccess.h>
#include <linux/compat.h>
#include <linux/vfio.h>

#include "vfio_private.h"

extern int vfio_release_device(struct vfio_device *device);

static int vfio_device_release(struct inode *inode, struct file *filep)
{
	struct vfio_device *device = filep->private_data;

	vfio_release_device(device);

	if (device->ops->put)
		device->ops->put(device);

	return 0;
}

static long vfio_device_unl_ioctl(struct file *filep,
				  unsigned int cmd, unsigned long arg)
{
	struct vfio_device *device = filep->private_data;

	if (device->ops->ioctl)
		return device->ops->ioctl(device, cmd, arg);

	return -ENOSYS;
}

static ssize_t vfio_device_read(struct file *filep, char __user *buf,
				size_t count, loff_t *ppos)
{
	struct vfio_device *device = filep->private_data;

printk("%s count %ld, offset %lx\n", __func__, count, *ppos);
	if (device->ops->read)
		return device->ops->read(device, buf, count, ppos);

	return -EINVAL;
}

static ssize_t vfio_device_write(struct file *filep, const char __user *buf,
				 size_t count, loff_t *ppos)
{
	struct vfio_device *device = filep->private_data;

	if (device->ops->write)
		return device->ops->write(device, buf, count, ppos);

	return -EINVAL;
}

static int vfio_device_mmap(struct file *filep, struct vm_area_struct *vma)
{
	struct vfio_device *device = filep->private_data;

	if (device->ops->mmap)
		return device->ops->mmap(device, vma);

	return -EINVAL;
}
	
#ifdef CONFIG_COMPAT
static long vfio_device_compat_ioctl(struct file *filep,
				     unsigned int cmd, unsigned long arg)
{
	arg = (unsigned long)compat_ptr(arg);
	return vfio_device_unl_ioctl(filep, cmd, arg);
}
#endif	/* CONFIG_COMPAT */

const struct file_operations vfio_device_fops = {
	.owner		= THIS_MODULE,
	.release	= vfio_device_release,
	.read		= vfio_device_read,
	.write		= vfio_device_write,
	.unlocked_ioctl	= vfio_device_unl_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= vfio_device_compat_ioctl,
#endif
	.mmap		= vfio_device_mmap,
};

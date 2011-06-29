/*
 * Copyright 2010 Cisco Systems, Inc.  All rights reserved.
 * Author: Tom Lyon, pugs@cisco.com
 *
 * This program is free software; you may redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Portions derived from drivers/uio/uio.c:
 * Copyright(C) 2005, Benedikt Spranger <b.spranger@linutronix.de>
 * Copyright(C) 2005, Thomas Gleixner <tglx@linutronix.de>
 * Copyright(C) 2006, Hans J. Koch <hjk@linutronix.de>
 * Copyright(C) 2006, Greg Kroah-Hartman <greg@kroah.com>
 *
 * Portions derived from drivers/uio/uio_pci_generic.c:
 * Copyright (C) 2009 Red Hat, Inc.
 * Author: Michael S. Tsirkin <mst@redhat.com>
 */

/*
 * VFIO main module: driver to allow non-privileged user programs
 * to imlpement direct mapped device drivers for PCI* devices
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/mm.h>
#include <linux/idr.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/eventfd.h>
#include <linux/pci.h>
#include <linux/iommu.h>
#include <linux/uaccess.h>
#include <linux/suspend.h>
#include <linux/compat.h>
#include <linux/delay.h>

#include <linux/vfio.h>


#define DRIVER_VERSION	"0.1"
#define DRIVER_AUTHOR	"Tom Lyon <pugs@cisco.com>"
#define DRIVER_DESC	"VFIO - User Level PCI meta-driver"

/*
 * Only a very few platforms today (Intel X7500) fully support
 * both DMA remapping and interrupt remapping in the IOMMU.
 * Everyone has DMA remapping but interrupt remapping is missing
 * in some Intel hardware and software, and its missing in the AMD
 * IOMMU software. Interrupt remapping is needed to really protect the
 * system from user level driver mischief.  Until it is in more platforms
 * we allow the admin to load the module with allow_unsafe_intrs=1
 * which will make this driver useful (but not safe)
 * on those platforms.
 */
static int allow_unsafe_intrs;
module_param(allow_unsafe_intrs, int, 0);
MODULE_PARM_DESC(allow_unsafe_intrs,
	"Allow use of IOMMUs which do not support interrupt remapping");

static int vfio_major = -1;
static DEFINE_IDR(vfio_idr);
static int vfio_max_minor;
/* Protect idr accesses */
static DEFINE_MUTEX(vfio_minor_lock);

/*
 * Does [a1,b1) overlap [a2,b2) ?
 */
static inline int overlap(loff_t a1, loff_t b1, loff_t a2, loff_t b2)
{
	/*
	 * Ranges overlap if they're not disjoint; and they're
	 * disjoint if the end of one is before the start of
	 * the other one.
	 */
	return !(b2 <= a1 || b1 <= a2);
}

static int vfio_pci_reset_function(struct vfio_dev *vdev)
{
	if (unlikely(vdev->remove_pending))
		return -EBUSY;

	return pci_reset_function(vdev->pdev);
}

static int vfio_open(struct inode *inode, struct file *filep)
{
	struct vfio_dev *vdev;
	int ret = 0;

	mutex_lock(&vfio_minor_lock);
	vdev = idr_find(&vfio_idr, iminor(inode));
	mutex_unlock(&vfio_minor_lock);
	if (!vdev)
		return -ENODEV;

	mutex_lock(&vdev->vgate);
	if (!vdev->refcnt) {
		u16 cmd;
		vfio_pci_reset_function(vdev);
		pci_save_state(vdev->pdev);
		vdev->pci_saved_state = pci_store_saved_state(vdev->pdev);
		if (!vdev->pci_saved_state)
			printk(KERN_DEBUG "%s: Couldn't store %s saved state\n",
			       __func__, dev_name(&vdev->pdev->dev));
		pci_read_config_word(vdev->pdev, PCI_COMMAND, &cmd);
		if (vdev->pci_2_3 && (cmd & PCI_COMMAND_INTX_DISABLE)) {
			cmd &= ~PCI_COMMAND_INTX_DISABLE;
			pci_write_config_word(vdev->pdev, PCI_COMMAND, cmd);
		}
		ret = pci_enable_device(vdev->pdev);
	}
	if (!ret) {
		vdev->refcnt++;
		filep->private_data = vdev;
	}

	mutex_unlock(&vdev->vgate);

	return ret;
}

/*
 * Disable PCI device
 */
static void vfio_disable_pci(struct vfio_dev *vdev)
{
	int bar;
	struct pci_dev *pdev = vdev->pdev;

	if (vfio_pci_reset_function(vdev) == 0) {
		if (pci_load_and_free_saved_state(pdev, &vdev->pci_saved_state))
			printk(KERN_INFO"%s: Couldn't reload %s saved state\n",
			       __func__, dev_name(&pdev->dev));
		else
			pci_restore_state(pdev);
	}

	for (bar = PCI_STD_RESOURCES; bar <= PCI_STD_RESOURCE_END; bar++) {
		if (!vdev->barmap[bar])
			continue;
		pci_iounmap(pdev, vdev->barmap[bar]);
		pci_release_selected_regions(pdev, 1 << bar);
		vdev->barmap[bar] = NULL;
	}
	pci_disable_device(pdev);
}

static int vfio_release(struct inode *inode, struct file *filep)
{
	struct vfio_dev *vdev = filep->private_data;

	mutex_lock(&vdev->vgate);
	if (--vdev->refcnt == 0) {
		/* we don't need to hold igate here since there are
		 * no other users doing ioctls
		 */
		if (vdev->ev_msix)
			vfio_drop_msix(vdev);
		if (vdev->ev_msi)
			vfio_drop_msi(vdev);
		if (vdev->ev_irq) {
			free_irq(vdev->pdev->irq, vdev);
			eventfd_ctx_put(vdev->ev_irq);
			vdev->ev_irq = NULL;
			vdev->irq_disabled = false;
			vdev->virq_disabled = false;
		}
		vfio_nl_freeclients(vdev);
		kfree(vdev->vconfig);
		vdev->vconfig = NULL;
		kfree(vdev->pci_config_map);
		vdev->pci_config_map = NULL;
		vfio_disable_pci(vdev);
		vfio_domain_unset(vdev);
	}
	mutex_unlock(&vdev->vgate);
	wake_up(&vdev->dev_idle_q);
	return 0;
}

static ssize_t vfio_read(struct file *filep, char __user *buf,
			size_t count, loff_t *ppos)
{
	struct vfio_dev *vdev = filep->private_data;
	struct pci_dev *pdev = vdev->pdev;
	int pci_space;

	pci_space = vfio_offset_to_pci_space(*ppos);

	/* config reads are OK before iommu domain set */
	if (pci_space == VFIO_PCI_CONFIG_RESOURCE)
		return vfio_config_readwrite(0, vdev, buf, count, ppos);

	/* no other reads until IOMMU domain set */
	if (!vdev->uiommu)
		return -EINVAL;
	if (pci_space > PCI_ROM_RESOURCE)
		return -EINVAL;
	if (pci_resource_flags(pdev, pci_space) & IORESOURCE_IO)
		return vfio_io_readwrite(0, vdev, buf, count, ppos);
	else if (pci_resource_flags(pdev, pci_space) & IORESOURCE_MEM)
		return vfio_mem_readwrite(0, vdev, buf, count, ppos);
	else if (pci_space == PCI_ROM_RESOURCE)
		return vfio_mem_readwrite(0, vdev, buf, count, ppos);
	return -EINVAL;
}

static int vfio_msix_check(struct vfio_dev *vdev, loff_t start, loff_t len)
{
	struct pci_dev *pdev = vdev->pdev;
	u16 pos;
	u32 table_offset;
	u16 table_size;
	u8 bir;
	loff_t lo, hi, startp, endp;

	pos = pci_find_capability(pdev, PCI_CAP_ID_MSIX);
	if (!pos)
		return 0;

	pci_read_config_word(pdev, pos + PCI_MSIX_FLAGS, &table_size);
	table_size = (table_size & PCI_MSIX_FLAGS_QSIZE) + 1;
	pci_read_config_dword(pdev, pos + PCI_MSIX_TABLE, &table_offset);
	bir = table_offset & PCI_MSIX_FLAGS_BIRMASK;
	lo = table_offset >> PAGE_SHIFT;
	hi = (table_offset + PCI_MSIX_ENTRY_SIZE * table_size + PAGE_SIZE - 1)
		>> PAGE_SHIFT;
	startp = start >> PAGE_SHIFT;
	endp = (start + len + PAGE_SIZE - 1) >> PAGE_SHIFT;
	if (bir == vfio_offset_to_pci_space(start) &&
	    overlap(lo, hi, startp, endp)) {
		printk(KERN_WARNING "%s: cannot write msi-x vectors\n",
			__func__);
		return -EINVAL;
	}
	return 0;
}

static ssize_t vfio_write(struct file *filep, const char __user *buf,
			size_t count, loff_t *ppos)
{
	struct vfio_dev *vdev = filep->private_data;
	struct pci_dev *pdev = vdev->pdev;
	int pci_space, ret;

	/* no writes until IOMMU domain set */
	if (!vdev->uiommu)
		return -EINVAL;
	pci_space = vfio_offset_to_pci_space(*ppos);
	if (pci_space == VFIO_PCI_CONFIG_RESOURCE)
		return vfio_config_readwrite(1, vdev, (char __user *)buf,
					     count, ppos);
	if (pci_space > PCI_ROM_RESOURCE)
		return -EINVAL;
	if (pci_resource_flags(pdev, pci_space) & IORESOURCE_IO)
		return vfio_io_readwrite(1, vdev,
					(char __user *)buf, count, ppos);
	else if (pci_resource_flags(pdev, pci_space) & IORESOURCE_MEM) {
		if (allow_unsafe_intrs) {
			/* don't allow writes to msi-x vectors */
			ret = vfio_msix_check(vdev, *ppos, count);
			if (ret)
				return ret;
		}
		return vfio_mem_readwrite(1, vdev, (char __user *)buf,
					  count, ppos);
	}
	return -EINVAL;
}

static int vfio_mmap(struct file *filep, struct vm_area_struct *vma)
{
	struct vfio_dev *vdev = filep->private_data;
	struct pci_dev *pdev = vdev->pdev;
	unsigned long requested, actual, start, phys;
	int pci_space, ret;

	/* no reads or writes until IOMMU domain set */
	if (!vdev->uiommu)
		return -EINVAL;

	if (vma->vm_end < vma->vm_start)
		return -EINVAL;
	if ((vma->vm_flags & VM_SHARED) == 0)
		return -EINVAL;


	pci_space = vfio_offset_to_pci_space((u64)vma->vm_pgoff << PAGE_SHIFT);
	/*
	 * Can't mmap ROM - see vfio_mem_readwrite
	 */
	if (pci_space > PCI_STD_RESOURCE_END)
		return -EINVAL;
	if ((pci_resource_flags(pdev, pci_space) & IORESOURCE_MEM) == 0)
		return -EINVAL;
	actual = pci_resource_len(pdev, pci_space) >> PAGE_SHIFT;

	requested = (vma->vm_end - vma->vm_start) >> PAGE_SHIFT;
	if (requested > actual || actual == 0)
		return -EINVAL;

	/*
	 * Even though we don't make use of the barmap for the mmap,
	 * we need to request the region and the barmap tracks that.
	 */
	if (!vdev->barmap[pci_space]) {
		ret = pci_request_selected_regions(pdev, (1 << pci_space),
						   vdev->name);
		if (ret)
			return ret;
		vdev->barmap[pci_space] = pci_iomap(pdev, pci_space, 0);
	}

	start = vma->vm_pgoff << PAGE_SHIFT;
	if (allow_unsafe_intrs && (vma->vm_flags & VM_WRITE)) {
		/*
		 * Deter users from screwing up MSI-X intrs
		 */
		ret = vfio_msix_check(vdev, start, vma->vm_end - vma->vm_start);
		if (ret)
			return ret;
	}

	vma->vm_private_data = vdev;
	vma->vm_flags |= VM_IO | VM_RESERVED;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
	phys = pci_resource_start(pdev, pci_space) >> PAGE_SHIFT;

	return remap_pfn_range(vma, vma->vm_start, phys,
			       vma->vm_end - vma->vm_start,
			       vma->vm_page_prot);
}

static long vfio_unl_ioctl(struct file *filep,
			unsigned int cmd,
			unsigned long arg)
{
	struct vfio_dev *vdev = filep->private_data;
	void __user *uarg = (void __user *)arg;
	int __user *intargp = (void __user *)arg;
	size_t __user *sizeargp = (void __user *)arg;
	struct pci_dev *pdev = vdev->pdev;
	struct vfio_dma_map dm;
	int fd, nfd, ret = 0;
	size_t bar;

	if (!vdev)
		return -EINVAL;

	switch (cmd) {

	case VFIO_MAP_DMA:
		if (copy_from_user(&dm, uarg, sizeof dm))
			return -EFAULT;
		ret = vfio_dma_map_dm(vdev->uiommu, &dm);
		if (!ret && copy_to_user(uarg, &dm, sizeof dm))
			ret = -EFAULT;
		break;

	case VFIO_UNMAP_DMA:
		if (copy_from_user(&dm, uarg, sizeof dm))
			return -EFAULT;
		ret = vfio_dma_unmap_dm(vdev->uiommu, &dm);
		break;

	case VFIO_SET_IRQ_EVENTFD:
		if (get_user(fd, intargp))
			return -EFAULT;
		if (!pdev->irq)
			return -EINVAL;
		mutex_lock(&vdev->igate);
		if (vdev->ev_irq) {
			eventfd_ctx_put(vdev->ev_irq);
			free_irq(pdev->irq, vdev);
			vdev->irq_disabled = false;
			vdev->ev_irq = NULL;
		}
		if (fd < 0)
			goto igate_unlock;

		if (vdev->ev_msi) {	/* irq and msi both use pdev->irq */
			ret = -EINVAL;
			goto igate_unlock;
		}

		vdev->ev_irq = eventfd_ctx_fdget(fd);
		if (!vdev->ev_irq) {
			ret = -EINVAL;
			goto igate_unlock;
		}
		ret = request_irq(pdev->irq, vfio_interrupt, vdev->pci_2_3 ?
				  IRQF_SHARED : 0, vdev->name, vdev);
		if (vdev->virq_disabled)
			vfio_disable_intx(vdev);

igate_unlock:
		mutex_unlock(&vdev->igate);
		break;

	case VFIO_SET_MSI_EVENTFDS:
		if (get_user(nfd, intargp))
			return -EFAULT;
		intargp++;
		mutex_lock(&vdev->igate);
		if (vdev->ev_irq) {	/* irq and msi both use pdev->irq */
			ret = -EINVAL;
		} else {
			if (nfd > 0 && !vdev->ev_msi)
				ret = vfio_setup_msi(vdev, nfd, intargp);
			else if (nfd == 0 && vdev->ev_msi)
				vfio_drop_msi(vdev);
			else
				ret = -EINVAL;
		}
		mutex_unlock(&vdev->igate);
		break;

	case VFIO_SET_MSIX_EVENTFDS:
		if (get_user(nfd, intargp))
			return -EFAULT;
		intargp++;
		mutex_lock(&vdev->igate);
		if (nfd > 0 && !vdev->ev_msix)
			ret = vfio_setup_msix(vdev, nfd, intargp);
		else if (nfd == 0 && vdev->ev_msix)
			vfio_drop_msix(vdev);
		else
			ret = -EINVAL;
		mutex_unlock(&vdev->igate);
		break;

	case VFIO_GET_BAR_LEN:
		if (get_user(bar, sizeargp))
			return -EFAULT;
		if (bar > PCI_ROM_RESOURCE)
			return -EINVAL;
		if (pci_resource_start(pdev, bar))
			bar = pci_resource_len(pdev, bar);
		else
			bar = 0;
		if (put_user(bar, sizeargp))
			return -EFAULT;
		break;

	case VFIO_SET_DOMAIN:
		if (get_user(fd, intargp))
			return -EFAULT;
		if (fd >= 0)
			ret = vfio_domain_set(vdev, fd, allow_unsafe_intrs);
		else
			ret = vfio_domain_unset(vdev);
		break;

	case VFIO_UNMASK_IRQ:
		ret = vfio_irq_eoi(vdev);
		break;

	case VFIO_SET_UNMASK_IRQ_EVENTFD:
		if (copy_from_user(&fd, uarg, sizeof fd))
			return -EFAULT;
		ret = vfio_irq_eoi_eventfd(vdev, fd);
		break;

	case VFIO_RESET_FUNCTION:
		ret = vfio_pci_reset_function(vdev);
		break;

	default:
		return -EINVAL;
	}
	return ret;
}

#ifdef CONFIG_COMPAT
static long vfio_compat_ioctl(struct file *filep,
			unsigned int cmd,
			unsigned long arg)
{
	arg = (unsigned long)compat_ptr(arg);
	return vfio_unl_ioctl(filep, cmd, arg);
}
#endif	/* CONFIG_COMPAT */

static const struct file_operations vfio_fops = {
	.owner		= THIS_MODULE,
	.open		= vfio_open,
	.release	= vfio_release,
	.read		= vfio_read,
	.write		= vfio_write,
	.unlocked_ioctl	= vfio_unl_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= vfio_compat_ioctl,
#endif
	.mmap		= vfio_mmap,
};

static int vfio_get_devnum(struct vfio_dev *vdev)
{
	int id, ret;

retry:
	if (unlikely(idr_pre_get(&vfio_idr, GFP_KERNEL) == 0))
		return -ENOMEM;

	mutex_lock(&vfio_minor_lock);

	if (vfio_major < 0) {
		vfio_major = register_chrdev(0, "vfio", &vfio_fops);
		if (vfio_major < 0) {
			mutex_unlock(&vfio_minor_lock);
			return vfio_major;
		}
	}
	
	ret = idr_get_new(&vfio_idr, vdev, &id);
	if (ret == 0 && id > MINORMASK) {
		idr_remove(&vfio_idr, id);
		ret = -ENOSPC;
	} 

	mutex_unlock(&vfio_minor_lock);

	if (ret < 0) {
		if (ret == -EAGAIN)
			goto retry;
		return ret;
	}

	if (id > vfio_max_minor)
		vfio_max_minor = id;

	return MKDEV(vfio_major, id);
}

int vfio_validate(struct vfio_dev *vdev)
{
	int rc = 0;
	int id;

	mutex_lock(&vfio_minor_lock);
	for (id = 0; id <= vfio_max_minor; id++)
		if (vdev == idr_find(&vfio_idr, id))
			goto out;
	rc = 1;
out:
	mutex_unlock(&vfio_minor_lock);
	return rc;
}

static void vfio_free_minor(struct vfio_dev *vdev)
{
	mutex_lock(&vfio_minor_lock);
	idr_remove(&vfio_idr, MINOR(vdev->devnum));
	mutex_unlock(&vfio_minor_lock);
}

/*
 * Verify that the device supports Interrupt Disable bit in command register,
 * per PCI 2.3, by flipping this bit and reading it back: this bit was readonly
 * in PCI 2.2.  (from uio_pci_generic)
 */
static int verify_pci_2_3(struct pci_dev *pdev)
{
	u16 orig, new;
	u8 pin;

	pci_read_config_byte(pdev, PCI_INTERRUPT_PIN, &pin);
	if (pin == 0)		/* irqs not needed */
		return 0;

	pci_read_config_word(pdev, PCI_COMMAND, &orig);
	pci_write_config_word(pdev, PCI_COMMAND,
			      orig ^ PCI_COMMAND_INTX_DISABLE);
	pci_read_config_word(pdev, PCI_COMMAND, &new);
	/* There's no way to protect against
	 * hardware bugs or detect them reliably, but as long as we know
	 * what the value should be, let's go ahead and check it. */
	if ((new ^ orig) & ~PCI_COMMAND_INTX_DISABLE) {
		dev_err(&pdev->dev, "Command changed from 0x%x to 0x%x: "
			"driver or HW bug?\n", orig, new);
		return -EBUSY;
	}
	if (!((new ^ orig) & PCI_COMMAND_INTX_DISABLE)) {
		dev_warn(&pdev->dev, "Device does not support disabling "
			 "interrupts, exclusive interrupt required.\n");
		return -ENODEV;
	}
	/* Now restore the original value. */
	pci_write_config_word(pdev, PCI_COMMAND, orig);
	return 0;
}

static int vfio_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct vfio_dev *vdev;
	int err;
	u8 type;

	if (!iommu_found())
		return -EINVAL;

	pci_read_config_byte(pdev, PCI_HEADER_TYPE, &type);
	if ((type & 0x7F) != PCI_HEADER_TYPE_NORMAL)
		return -EINVAL;

	vdev = kzalloc(sizeof(struct vfio_dev), GFP_KERNEL);
	if (!vdev)
		return -ENOMEM;

	vdev->pdev = pdev;
	vdev->pci_2_3 = (verify_pci_2_3(pdev) == 0);

	mutex_init(&vdev->vgate);
	mutex_init(&vdev->igate);
	mutex_init(&vdev->ngate);
	INIT_LIST_HEAD(&vdev->nlc_list);
	init_waitqueue_head(&vdev->dev_idle_q);
	init_waitqueue_head(&vdev->nl_wait_q);

	err = vfio_get_devnum(vdev);
	if (err < 0)
		goto err_get_devnum;
	vdev->devnum = err;
	err = 0;

	sprintf(vdev->name, "vfio%d", MINOR(vdev->devnum));
	pci_set_drvdata(pdev, vdev);
	vdev->dev = device_create(vfio_class->class, &pdev->dev,
				  vdev->devnum, vdev, vdev->name);
	if (IS_ERR(vdev->dev)) {
		printk(KERN_ERR "VFIO: device register failed\n");
		err = PTR_ERR(vdev->dev);
		goto err_device_create;
	}

	err = vfio_dev_add_attributes(vdev);
	if (err)
		goto err_vfio_dev_add_attributes;

	return 0;

err_vfio_dev_add_attributes:
	device_destroy(vfio_class->class, vdev->devnum);
err_device_create:
	vfio_free_minor(vdev);
err_get_devnum:
	kfree(vdev);
	return err;
}

static void vfio_remove(struct pci_dev *pdev)
{
	struct vfio_dev *vdev = pci_get_drvdata(pdev);

	/* prevent further opens */
	vfio_free_minor(vdev);

	vdev->remove_pending = true;

	/* notify users */
	vfio_nl_remove(vdev);

	/* wait for all closed */
	wait_event(vdev->dev_idle_q, vdev->refcnt == 0);

	vfio_dev_del_attributes(vdev);
	device_destroy(vfio_class->class, vdev->devnum);
	pci_set_drvdata(pdev, NULL);
	kfree(vdev);
}

static struct pci_error_handlers vfio_error_handlers = {
	.error_detected	= vfio_error_detected,
	.mmio_enabled	= vfio_mmio_enabled,
	.link_reset	= vfio_link_reset,
	.slot_reset	= vfio_slot_reset,
	.resume		= vfio_error_resume,
};

static struct pci_driver driver = {
	.name		= "vfio",
	.id_table	= NULL, /* only dynamic id's */
	.probe		= vfio_probe,
	.remove		= vfio_remove,
	.err_handler	= &vfio_error_handlers,
};

static atomic_t vfio_pm_suspend_count;
static int vfio_pm_suspend_result;
static DECLARE_WAIT_QUEUE_HEAD(vfio_pm_wait_q);

/*
 * Notify user level drivers of hibernation/suspend request
 * Send all the notifies in parallel, collect all the replies
 * If one ULD can't suspend, none can
 */
static int vfio_pm_suspend(void)
{
	struct vfio_dev *vdev;
	int id, alive = 0;

	mutex_lock(&vfio_minor_lock);
	atomic_set(&vfio_pm_suspend_count, 0);
	vfio_pm_suspend_result = NOTIFY_DONE;
	for (id = 0; id <= vfio_max_minor; id++) {
		vdev = idr_find(&vfio_idr, id);
		if (!vdev)
			continue;
		if (vdev->refcnt == 0)
			continue;
		alive++;
		if (vfio_nl_upcall(vdev, VFIO_MSG_PM_SUSPEND, 0, 0) == 0)
			atomic_inc(&vfio_pm_suspend_count);
	}
	mutex_unlock(&vfio_minor_lock);
	if (alive > atomic_read(&vfio_pm_suspend_count))
		return NOTIFY_BAD;

	/* sleep for reply */
	if (wait_event_interruptible_timeout(vfio_pm_wait_q,
	    (atomic_read(&vfio_pm_suspend_count) == 0),
	    VFIO_SUSPEND_REPLY_TIMEOUT) <= 0) {
		printk(KERN_ERR "vfio upcall suspend reply timeout\n");
		return NOTIFY_BAD;
	}
	return vfio_pm_suspend_result;
}

static int vfio_pm_resume(void)
{
	struct vfio_dev *vdev;
	int id;

	mutex_lock(&vfio_minor_lock);
	for (id = 0; id <= vfio_max_minor; id++) {
		vdev = idr_find(&vfio_idr, id);
		if (!vdev)
			continue;
		if (vdev->refcnt == 0)
			continue;
		vfio_nl_upcall(vdev, VFIO_MSG_PM_RESUME, 0, 0);
	}
	mutex_unlock(&vfio_minor_lock);
	return NOTIFY_DONE;
}


void vfio_pm_process_reply(int reply)
{
	if (vfio_pm_suspend_result == NOTIFY_DONE) {
		if (reply != NOTIFY_DONE)
			vfio_pm_suspend_result = NOTIFY_BAD;
	}
	if (atomic_dec_and_test(&vfio_pm_suspend_count))
		wake_up(&vfio_pm_wait_q);
}

static int vfio_pm_notify(struct notifier_block *this,
			  unsigned long event, void *notused)
{
	switch (event) {
	case PM_HIBERNATION_PREPARE:
	case PM_SUSPEND_PREPARE:
		return vfio_pm_suspend();
		break;
	case PM_POST_HIBERNATION:
	case PM_POST_SUSPEND:
		return vfio_pm_resume();
		break;
	default:
		return NOTIFY_DONE;
	}
}

static struct notifier_block vfio_pm_nb = {
	.notifier_call = vfio_pm_notify,
};


static void __exit cleanup(void)
{
	if (vfio_major >= 0)
		unregister_chrdev(vfio_major, "vfio");

	pci_unregister_driver(&driver);
	vfio_eoi_eventfd_exit();
	unregister_pm_notifier(&vfio_pm_nb);
	vfio_nl_exit();
	vfio_class_destroy();
	vfio_uninit_pci_perm_bits();
}

static int __init init(void)
{
	int ret;

	pr_info(DRIVER_DESC " version: " DRIVER_VERSION "\n");

	ret = vfio_init_pci_perm_bits();
	if (ret)
		return ret;

	ret = vfio_class_init();
	if (ret)
		goto err_class;

	ret = vfio_nl_init();
	if (ret)
		goto err_nl;

	ret = register_pm_notifier(&vfio_pm_nb);
	if (ret)
		goto err_pm;

	ret = vfio_eoi_eventfd_init();
	if (ret)
		goto err_eoi;

	ret = pci_register_driver(&driver);
	if (ret)
		goto err_pci;

	return 0;

err_pci:
	vfio_eoi_eventfd_exit();
err_eoi:
	unregister_pm_notifier(&vfio_pm_nb);
err_pm:
	vfio_nl_exit();
err_nl:
	vfio_class_destroy();
err_class:
	vfio_uninit_pci_perm_bits();

	return ret;
}

module_exit(cleanup);
module_init(init);

MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);

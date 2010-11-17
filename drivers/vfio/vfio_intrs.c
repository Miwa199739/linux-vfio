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
 * This code handles catching interrupts and translating
 * them to events on eventfds
 */

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/eventfd.h>
#include <linux/pci.h>
#include <linux/mmu_notifier.h>
#include <linux/file.h>
#include <linux/poll.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <linux/vfio.h>


/*
 * vfio_interrupt - IRQ hardware interrupt handler
 */
irqreturn_t vfio_interrupt(int irq, void *dev_id)
{
	struct vfio_dev *vdev = dev_id;
	struct pci_dev *pdev = vdev->pdev;
	irqreturn_t ret = IRQ_NONE;
	u32 cmd_status_dword;
	u16 origcmd, newcmd, status;

	spin_lock_irq(&vdev->irqlock);

	/* INTX disabled interrupts can still be shared */
	if (vdev->irq_disabled) {
		spin_unlock_irq(&vdev->irqlock);
		return ret;
	}

	if (vdev->pci_2_3) {
		pci_block_user_cfg_access(pdev);

		/* Read both command and status registers in a single 32-bit
		 * operation. Note: we could cache the value for command and
		 * move the status read out of the lock if there was a way to
		 * get notified of user changes to command register through
		 * sysfs. Should be good for shared irqs. */
		pci_read_config_dword(pdev, PCI_COMMAND, &cmd_status_dword);
		origcmd = cmd_status_dword;
		status = cmd_status_dword >> 16;

		/* Check interrupt status register to see whether our device
		 * triggered the interrupt. */
		if (!(status & PCI_STATUS_INTERRUPT))
			goto done;

		/* We triggered the interrupt, disable it. */
		newcmd = origcmd | PCI_COMMAND_INTX_DISABLE;
		if (newcmd != origcmd)
			pci_write_config_word(pdev, PCI_COMMAND, newcmd);

		ret = IRQ_HANDLED;
done:
		pci_unblock_user_cfg_access(pdev);
	} else {
		disable_irq_nosync(pdev->irq);
		ret = IRQ_HANDLED;
	}

	if (ret == IRQ_HANDLED)
		vdev->irq_disabled = true;

	spin_unlock_irq(&vdev->irqlock);

	if (ret != IRQ_HANDLED)
		return ret;

	if (vdev->ev_irq)
		eventfd_signal(vdev->ev_irq, 1);
	return ret;
}

int vfio_irq_eoi(struct vfio_dev *vdev)
{
	struct pci_dev *pdev = vdev->pdev;

	spin_lock_irq(&vdev->irqlock);

	if (vdev->irq_disabled) {
		if (vdev->pci_2_3) {
			u16 cmd;
			pci_block_user_cfg_access(pdev);

			pci_read_config_word(pdev, PCI_COMMAND, &cmd);
			cmd &= ~PCI_COMMAND_INTX_DISABLE;
			pci_write_config_word(pdev, PCI_COMMAND, cmd);

			pci_unblock_user_cfg_access(pdev);
		} else
			enable_irq(pdev->irq);

		vdev->irq_disabled = false;
	}

	spin_unlock_irq(&vdev->irqlock);
	return 0;
}

struct eoi_eventfd {
	struct vfio_dev		*vdev;
	struct eventfd_ctx	*eventfd;
	poll_table		pt;
	wait_queue_t		wait;
	struct work_struct	inject;
	struct work_struct	shutdown;
};

static struct workqueue_struct *eoi_cleanup_wq;

static void inject_eoi(struct work_struct *work)
{
	struct eoi_eventfd *ev_eoi = container_of(work, struct eoi_eventfd,
						  inject);
	vfio_irq_eoi(ev_eoi->vdev);
}

static void shutdown_eoi(struct work_struct *work)
{
	u64 cnt;
	struct eoi_eventfd *ev_eoi = container_of(work, struct eoi_eventfd,
						  shutdown);
	struct vfio_dev *vdev = ev_eoi->vdev;

	eventfd_ctx_remove_wait_queue(ev_eoi->eventfd, &ev_eoi->wait, &cnt);
	flush_work(&ev_eoi->inject);
	eventfd_ctx_put(ev_eoi->eventfd);
	kfree(vdev->ev_eoi);
	vdev->ev_eoi = NULL;
}

static void deactivate_eoi(struct eoi_eventfd *ev_eoi)
{
	queue_work(eoi_cleanup_wq, &ev_eoi->shutdown);
}

static int wakeup_eoi(wait_queue_t *wait, unsigned mode, int sync, void *key)
{
	struct eoi_eventfd *ev_eoi = container_of(wait, struct eoi_eventfd,
						  wait);
	unsigned long flags = (unsigned long)key;

	if (flags & POLLIN)
		/* An event has been signaled, inject an interrupt */
		schedule_work(&ev_eoi->inject);

	if (flags & POLLHUP)
		/* The eventfd is closing, detach from VFIO */
		deactivate_eoi(ev_eoi);

	return 0;
}

static void
eoi_ptable_queue_proc(struct file *file, wait_queue_head_t *wqh, poll_table *pt)
{
	struct eoi_eventfd *ev_eoi = container_of(pt, struct eoi_eventfd, pt);
	add_wait_queue(wqh, &ev_eoi->wait);
}

static int vfio_irq_eoi_eventfd_enable(struct vfio_dev *vdev, int fd)
{
	struct file *file = NULL;
	struct eventfd_ctx *eventfd = NULL;
	struct eoi_eventfd *ev_eoi;
	int ret = 0;
	unsigned int events;

	if (vdev->ev_eoi)
		return -EBUSY;

	ev_eoi = kzalloc(sizeof(struct eoi_eventfd), GFP_KERNEL);
	if (!ev_eoi)
		return -ENOMEM;

	vdev->ev_eoi = ev_eoi;
	ev_eoi->vdev = vdev;

	INIT_WORK(&ev_eoi->inject, inject_eoi);
	INIT_WORK(&ev_eoi->shutdown, shutdown_eoi);

	file = eventfd_fget(fd);
	if (IS_ERR(eventfd)) {
		ret = PTR_ERR(eventfd);
		goto fail;
	}

	eventfd = eventfd_ctx_fileget(file);
	if (IS_ERR(eventfd)) {
		ret = PTR_ERR(eventfd);
		goto fail;
	}

	ev_eoi->eventfd = eventfd;

	/*
	 * Install our own custom wake-up handling so we are notified via
	 * a callback whenever someone signals the underlying eventfd
	 */
	init_waitqueue_func_entry(&ev_eoi->wait, wakeup_eoi);
	init_poll_funcptr(&ev_eoi->pt, eoi_ptable_queue_proc);

	events = file->f_op->poll(file, &ev_eoi->pt);

	/*
	 * Check if there was an event already pending on the eventfd
	 * before we registered, and trigger it as if we didn't miss it.
	 */
	if (events & POLLIN)
		schedule_work(&ev_eoi->inject);

	/*
	 * do not drop the file until the irqfd is fully initialized, otherwise
	 * we might race against the POLLHUP
	 */
	fput(file);

	return 0;

fail:
	if (eventfd && !IS_ERR(eventfd))
		eventfd_ctx_put(eventfd);

	if (!IS_ERR(file))
		fput(file);

	return ret;
}

static int vfio_irq_eoi_eventfd_disable(struct vfio_dev *vdev, int fd)
{
	if (!vdev->ev_eoi)
		return -ENODEV;

	deactivate_eoi(vdev->ev_eoi);

	/*
	 * Block until we know all outstanding shutdown jobs have completed
	 * so that we guarantee there will not be any more interrupts on this
	 * gsi once this deassign function returns.
	 */
	flush_workqueue(eoi_cleanup_wq);

	return 0;
}

int vfio_irq_eoi_eventfd(struct vfio_dev *vdev, int fd)
{
	if (fd < 0)
		return vfio_irq_eoi_eventfd_disable(vdev, fd);
	return vfio_irq_eoi_eventfd_enable(vdev, fd);
}

int __init vfio_eoi_module_init(void)
{
	eoi_cleanup_wq = create_singlethread_workqueue("vfio-eoi-cleanup");
	if (!eoi_cleanup_wq)
		return -ENOMEM;

	return 0;
}

void __exit vfio_eoi_module_exit(void)
{
	destroy_workqueue(eoi_cleanup_wq);
}

/*
 * MSI and MSI-X Interrupt handler.
 * Just signal an event
 */
static irqreturn_t msihandler(int irq, void *arg)
{
	struct eventfd_ctx *ctx = arg;

	eventfd_signal(ctx, 1);
	return IRQ_HANDLED;
}

void vfio_drop_msi(struct vfio_dev *vdev)
{
	struct pci_dev *pdev = vdev->pdev;
	int i;

	if (vdev->ev_msi) {
		for (i = 0; i < vdev->msi_nvec; i++) {
			free_irq(pdev->irq + i, vdev->ev_msi[i]);
			if (vdev->ev_msi[i])
				eventfd_ctx_put(vdev->ev_msi[i]);
		}
	}
	kfree(vdev->ev_msi);
	vdev->ev_msi = NULL;
	vdev->msi_nvec = 0;
	pci_disable_msi(pdev);
}

int vfio_setup_msi(struct vfio_dev *vdev, int nvec, int __user *intargp)
{
	struct pci_dev *pdev = vdev->pdev;
	struct eventfd_ctx *ctx;
	int i;
	int ret = 0;
	int fd;

	if (nvec < 1 || nvec > 32)
		return -EINVAL;
	vdev->ev_msi = kzalloc(nvec * sizeof(struct eventfd_ctx *),
				GFP_KERNEL);
	if (!vdev->ev_msi)
		return -ENOMEM;

	for (i = 0; i < nvec; i++) {
		if (get_user(fd, intargp)) {
			ret = -EFAULT;
			goto out;
		}
		intargp++;
		ctx = eventfd_ctx_fdget(fd);
		if (IS_ERR(ctx)) {
			ret = PTR_ERR(ctx);
			goto out;
		}
		vdev->ev_msi[i] = ctx;
	}
	ret = pci_enable_msi_block(pdev, nvec);
	if (ret) {
		if (ret > 0)
			ret = -EINVAL;
		goto out;
	}
	for (i = 0; i < nvec; i++) {
		ret = request_irq(pdev->irq + i, msihandler, 0,
			vdev->name, vdev->ev_msi[i]);
		if (ret)
			goto out;
		vdev->msi_nvec = i+1;
	}

	/*
	 * compute the virtual hardware field for max msi vectors -
	 * it is the log base 2 of the number of vectors
	 */
	vdev->msi_qmax = fls(vdev->msi_nvec * 2 - 1) - 1;
out:
	if (ret)
		vfio_drop_msi(vdev);
	return ret;
}

void vfio_drop_msix(struct vfio_dev *vdev)
{
	struct pci_dev *pdev = vdev->pdev;
	int i;

	if (vdev->ev_msix && vdev->msix) {
		for (i = 0; i < vdev->msix_nvec; i++) {
			free_irq(vdev->msix[i].vector, vdev->ev_msix[i]);
			if (vdev->ev_msix[i])
				eventfd_ctx_put(vdev->ev_msix[i]);
		}
	}
	kfree(vdev->ev_msix);
	vdev->ev_msix = NULL;
	kfree(vdev->msix);
	vdev->msix = NULL;
	vdev->msix_nvec = 0;
	pci_disable_msix(pdev);
}

int vfio_setup_msix(struct vfio_dev *vdev, int nvec, int __user *intargp)
{
	struct pci_dev *pdev = vdev->pdev;
	struct eventfd_ctx *ctx;
	int ret = 0;
	int i;
	int fd;
	int pos;
	u16 flags = 0;

	pos = pci_find_capability(pdev, PCI_CAP_ID_MSIX);
	if (!pos)
		return -EINVAL;
	pci_read_config_word(pdev, pos + PCI_MSIX_FLAGS, &flags);
	if (nvec < 1 || nvec > (flags & PCI_MSIX_FLAGS_QSIZE) + 1)
		return -EINVAL;

	vdev->msix = kzalloc(nvec * sizeof(struct msix_entry),
				GFP_KERNEL);
	if (!vdev->msix)
		return -ENOMEM;
	vdev->ev_msix = kzalloc(nvec * sizeof(struct eventfd_ctx *),
				GFP_KERNEL);
	if (!vdev->ev_msix) {
		kfree(vdev->msix);
		return -ENOMEM;
	}
	for (i = 0; i < nvec; i++) {
		if (get_user(fd, intargp)) {
			ret = -EFAULT;
			break;
		}
		intargp++;
		ctx = eventfd_ctx_fdget(fd);
		if (IS_ERR(ctx)) {
			ret = PTR_ERR(ctx);
			break;
		}
		vdev->msix[i].entry = i;
		vdev->ev_msix[i] = ctx;
	}
	if (!ret)
		ret = pci_enable_msix(pdev, vdev->msix, nvec);
	vdev->msix_nvec = 0;
	for (i = 0; i < nvec && !ret; i++) {
		ret = request_irq(vdev->msix[i].vector, msihandler, 0,
			vdev->name, vdev->ev_msix[i]);
		if (ret)
			break;
		vdev->msix_nvec = i+1;
	}
	if (ret)
		vfio_drop_msix(vdev);
	return ret;
}

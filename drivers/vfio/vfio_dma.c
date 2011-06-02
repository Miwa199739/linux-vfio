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
 * This code handles mapping and unmapping of user data buffers
 * into DMA'ble space using the IOMMU
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/pci.h>
#include <linux/mm.h>
#include <linux/mmu_notifier.h>
#include <linux/iommu.h>
#include <linux/uiommu.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include <linux/vfio.h>

#define NPAGE_TO_SIZE(npage)	((size_t)(npage) << PAGE_SHIFT)

static LIST_HEAD(vfio_uiommu_list);
static DEFINE_MUTEX(vfio_uiommu_lock);

struct vwork {
	struct mm_struct *mm;
	int		npage;
	struct work_struct work;
};

/* delayed decrement for locked_vm */
static void vfio_lock_acct_bg(struct work_struct *work)
{
	struct vwork *vwork = container_of(work, struct vwork, work);
	struct mm_struct *mm;

	mm = vwork->mm;
	down_write(&mm->mmap_sem);
	mm->locked_vm += vwork->npage;
	up_write(&mm->mmap_sem);
	mmput(mm);		/* unref mm */
	kfree(vwork);
}

static void vfio_lock_acct(int npage)
{
	struct vwork *vwork;
	struct mm_struct *mm;

	if (!current->mm) {
		/* process exited */
		return;
	}
	if (down_write_trylock(&current->mm->mmap_sem)) {
		current->mm->locked_vm += npage;
		up_write(&current->mm->mmap_sem);
		return;
	}
	/*
	 * Couldn't get mmap_sem lock, so must setup to decrement
	 * mm->locked_vm later. If locked_vm were atomic, we wouldn't
	 * need this silliness
	 */
	vwork = kmalloc(sizeof(struct vwork), GFP_KERNEL);
	if (!vwork)
		return;
	mm = get_task_mm(current);	/* take ref mm */
	if (!mm) {
		kfree(vwork);
		return;
	}
	INIT_WORK(&vwork->work, vfio_lock_acct_bg);
	vwork->mm = mm;
	vwork->npage = npage;
	schedule_work(&vwork->work);
}

/* Unmap DMA region */
/* dgate must be held */
static void vfio_dma_unmap(struct vfio_uiommu *uiommu, unsigned long iova,
			   int npage, struct page **pages, int rdwr)
{
	int i;

	for (i = 0; i < npage; i++, iova += PAGE_SIZE) {
		uiommu_unmap(uiommu->udomain, iova, 0);
		if (rdwr)
			SetPageDirty(pages[i]);
		put_page(pages[i]);
	}
	uiommu->locked_pages -= npage;
	vfio_lock_acct(-npage);
}

/* Unmap ALL DMA regions */
static void vfio_dma_unmapall(struct vfio_uiommu *uiommu)
{
	struct list_head *pos, *pos2;
	struct dma_map_page *mlp;

	mutex_lock(&uiommu->dgate);
	list_for_each_safe(pos, pos2, &uiommu->dm_list) {
		mlp = list_entry(pos, struct dma_map_page, list);
		vfio_dma_unmap(uiommu, mlp->daddr, mlp->npage,
			       mlp->pages, mlp->rdwr);
		list_del(&mlp->list);
		vfree(mlp->pages);
		kfree(mlp);
	}
	mutex_unlock(&uiommu->dgate);
}

/* Map DMA region */
/* dgate must be held */
static int vfio_dma_map(struct vfio_uiommu *uiommu, unsigned long iova,
			int npage, struct page **pages, int rdwr)
{
	unsigned long start = iova;
	int i, ret;

	/* Verify pages are not already mapped */
	for (i = 0; i < npage; i++, iova += PAGE_SIZE)
		if (uiommu_iova_to_phys(uiommu->udomain, iova))
			return -EBUSY;

	for (iova = start, i = 0; i < npage; i++, iova += PAGE_SIZE) {
		ret = uiommu_map(uiommu->udomain, iova,
				 page_to_phys(pages[i]), 0, rdwr);
		if (!ret)
			continue;

		/* Back out mappings on error */
		for (i--, iova -= PAGE_SIZE; i >= 0; i--, iova -= PAGE_SIZE)
			uiommu_unmap(uiommu->udomain, iova, 0);
		return ret;
	}
	uiommu->locked_pages += npage;
	vfio_lock_acct(npage);
	return 0;
}

static struct dma_map_page *vfio_find_dma(struct vfio_uiommu *uiommu,
					  dma_addr_t start, size_t size)
{
	struct list_head *pos;
	struct dma_map_page *mlp;

	list_for_each(pos, &uiommu->dm_list) {
		mlp = list_entry(pos, struct dma_map_page, list);
		if (!(mlp->daddr + NPAGE_TO_SIZE(mlp->npage) <= start ||
		      mlp->daddr >= start + size))
			return mlp;
	}
	return NULL;
}

static struct dma_map_page *vfio_find_vaddr(struct vfio_uiommu *uiommu,
					    unsigned long start, size_t size)
{
	struct list_head *pos;
	struct dma_map_page *mlp;

	list_for_each(pos, &uiommu->dm_list) {
		mlp = list_entry(pos, struct dma_map_page, list);
		if (!(mlp->vaddr + NPAGE_TO_SIZE(mlp->npage) <= start ||
		      mlp->vaddr >= start + size))
			return mlp;
	}
	return NULL;
}

int vfio_remove_dma_overlap(struct vfio_uiommu *uiommu, dma_addr_t start,
			    size_t size, struct dma_map_page *mlp,
			    size_t *remaining)
{
	struct dma_map_page *split;
	struct page **pages_lo, **pages_hi;
	int npage_lo, npage_hi;

	/* Existing dma region is completely covered, unmap all */
	if (start <= mlp->daddr &&
	    start + size >= mlp->daddr + NPAGE_TO_SIZE(mlp->npage)) {
		vfio_dma_unmap(uiommu, mlp->daddr, mlp->npage,
			       mlp->pages, mlp->rdwr);
		list_del(&mlp->list);
		vfree(mlp->pages);
		if (remaining)
			*remaining -= NPAGE_TO_SIZE(mlp->npage);
		kfree(mlp);
		return 0;
	}

	/* Overlap low address of existing range */
	if (start <= mlp->daddr) {
		size_t overlap;

		overlap = start + size - mlp->daddr;
		npage_lo = overlap >> PAGE_SHIFT;
		npage_hi = mlp->npage - npage_lo;

		pages_hi = vmalloc(npage_hi * sizeof(struct page *));
		if (!pages_hi)
			return -ENOMEM;

		vfio_dma_unmap(uiommu, mlp->daddr, npage_lo,
			       mlp->pages, mlp->rdwr);
		mlp->daddr += overlap;
		mlp->vaddr += overlap;
		mlp->npage -= npage_lo;
		memcpy(pages_hi, &mlp->pages[npage_lo],
		       mlp->npage * sizeof(struct page *));
		vfree(mlp->pages);
		mlp->pages = pages_hi;
		if (remaining)
			*remaining -= overlap;
		return 0;
	}

	/* Overlap high address of existing range */
	if (start + size >= mlp->daddr + NPAGE_TO_SIZE(mlp->npage)) {
		size_t overlap;

		overlap = mlp->daddr + NPAGE_TO_SIZE(mlp->npage) - start;
		npage_hi = overlap >> PAGE_SHIFT;
		npage_lo = mlp->npage - npage_hi;

		pages_lo = vmalloc(npage_lo * sizeof(struct page *));
		if (!pages_lo)
			return -ENOMEM;

		vfio_dma_unmap(uiommu, start, npage_hi,
			       &mlp->pages[npage_lo], mlp->rdwr);
		mlp->npage -= npage_hi;
		memcpy(pages_lo, mlp->pages,
		       mlp->npage * sizeof(struct page *));
		vfree(mlp->pages);
		mlp->pages = pages_lo;
		if (remaining)
			*remaining -= overlap;
		return 0;
	}

	/* Split existing */
	npage_lo = (start - mlp->daddr) >> PAGE_SHIFT;
	npage_hi = mlp->npage - (size >> PAGE_SHIFT) - npage_lo;

	split = kzalloc(sizeof *split, GFP_KERNEL);
	if (!split)
		return -ENOMEM;

	pages_lo = vmalloc(npage_lo * sizeof(struct page *));
	if (!pages_lo) {
		kfree(split);
		return -ENOMEM;
	}

	pages_hi = vmalloc(npage_hi * sizeof(struct page *));
	if (!pages_hi) {
		kfree(split);
		vfree(pages_lo);
		return -ENOMEM;
	}

	vfio_dma_unmap(uiommu, start, size >> PAGE_SHIFT,
		       &mlp->pages[npage_lo], mlp->rdwr);

	memcpy(pages_lo, mlp->pages, npage_lo * sizeof(struct page *));
	memcpy(pages_hi, &mlp->pages[npage_lo + (size >> PAGE_SHIFT)],
	       npage_hi * sizeof(struct page *));

	vfree(mlp->pages);
	mlp->npage = npage_lo;
	mlp->pages = pages_lo;

	split->npage = npage_hi;
	split->pages = pages_hi;
	split->daddr = start + size;
	split->vaddr = mlp->vaddr + NPAGE_TO_SIZE(npage_lo) + size;
	split->rdwr = mlp->rdwr;
	list_add(&split->list, &uiommu->dm_list);
	if (remaining)
		*remaining -= size;
	return 0;
}

int vfio_dma_unmap_dm(struct vfio_uiommu *uiommu, struct vfio_dma_map *dmp)
{
	struct dma_map_page *mlp;
	int ret = 0;
	size_t size = dmp->size;

	if (dmp->dmaaddr & (PAGE_SIZE-1))
		return -EINVAL;
	if (size & (PAGE_SIZE-1))
		return -EINVAL;

	if (!uiommu)
		return -EINVAL;

	mutex_lock(&uiommu->dgate);
	while (size &&
	       (mlp = vfio_find_dma(uiommu, dmp->dmaaddr, dmp->size))) {
		ret = vfio_remove_dma_overlap(uiommu, dmp->dmaaddr,
					      dmp->size, mlp, &size);
		if (ret)
			break;
	}
	mutex_unlock(&uiommu->dgate);
	return ret;
}

#ifdef CONFIG_MMU_NOTIFIER
/* Handle MMU notifications - user process freed or realloced memory
 * which may be in use in a DMA region. Clean up region if so.
 */
static void vfio_dma_handle_mmu_notify(struct mmu_notifier *mn,
		unsigned long start, unsigned long end)
{
	struct vfio_uiommu *uiommu;
	struct dma_map_page *mlp;

	uiommu = container_of(mn, struct vfio_uiommu, mmu_notifier);
	mutex_lock(&uiommu->dgate);

	/* vaddrs are not unique (multiple daddrs could be mapped to the
	 * same vaddr), therefore we have to search to exhaustion rather
	 * than tracking how much we've unmapped. */
	while ((mlp = vfio_find_vaddr(uiommu, start, end - start))) {
		dma_addr_t dma_start;
		int ret;

		dma_start = mlp->daddr;
		if (start < mlp->vaddr)
			dma_start -= mlp->vaddr - start;
		else
			dma_start += start - mlp->vaddr;
		ret = vfio_remove_dma_overlap(uiommu, dma_start,
					      end - start, mlp, NULL);
		if (ret) {
			printk(KERN_ERR "%s: "
			       "failed to unmap mmu notify range "
			       "%lx - %lx (%d)\n", __func__, start, end, ret);
			break;
		}
	}
	mutex_unlock(&uiommu->dgate);
}

static void vfio_dma_inval_page(struct mmu_notifier *mn,
		struct mm_struct *mm, unsigned long addr)
{
	vfio_dma_handle_mmu_notify(mn, addr, addr + PAGE_SIZE);
}

static void vfio_dma_inval_range_start(struct mmu_notifier *mn,
		struct mm_struct *mm, unsigned long start, unsigned long end)
{
	vfio_dma_handle_mmu_notify(mn, start, end);
}

static const struct mmu_notifier_ops vfio_dma_mmu_notifier_ops = {
	.invalidate_page = vfio_dma_inval_page,
	.invalidate_range_start = vfio_dma_inval_range_start,
};
#endif	/* CONFIG_MMU_NOTIFIER */

int vfio_dma_map_dm(struct vfio_uiommu *uiommu, struct vfio_dma_map *dmp)
{
	int npage, locked, lock_limit;
	struct page **pages;
	struct dma_map_page *mlp, *nmlp, *mmlp = NULL;
	int rdwr = IOMMU_READ;
	dma_addr_t daddr = dmp->dmaaddr;
	unsigned long vaddr = dmp->vaddr;
	size_t size = dmp->size;
	int ret = 0;

	if (vaddr & (PAGE_SIZE-1))
		return -EINVAL;
	if (daddr & (PAGE_SIZE-1))
		return -EINVAL;
	if (size & (PAGE_SIZE-1))
		return -EINVAL;

	npage = size >> PAGE_SHIFT;
	if (!npage)
		return -EINVAL;

	if (!uiommu)
		return -EINVAL;

	if (dmp->flags & VFIO_FLAG_WRITE)
		rdwr |= IOMMU_WRITE;
	if (uiommu->cachec)
		rdwr |= IOMMU_CACHE;

	mutex_lock(&uiommu->dgate);

	if (vfio_find_dma(uiommu, daddr, size)) {
		ret = -EBUSY;
		goto out_lock;
	}

	/* account for locked pages */
	locked = npage + current->mm->locked_vm;
	lock_limit = rlimit(RLIMIT_MEMLOCK) >> PAGE_SHIFT;
	if ((locked > lock_limit) && !capable(CAP_IPC_LOCK)) {
		printk(KERN_WARNING "%s: RLIMIT_MEMLOCK exceeded\n",
			__func__);
		ret = -ENOMEM;
		goto out_lock;
	}

	/* Allocate a new mlp, this may not be used if we merge, but
	 * ENOMEM is easier to handle before we do the iova mapping */
	nmlp = kzalloc(sizeof *nmlp, GFP_KERNEL);
	if (!nmlp) {
		ret = -ENOMEM;
		goto out_lock;
	}

	pages = vmalloc(npage * sizeof(struct page *));
	if (!pages) {
		kfree(nmlp);
		ret = -ENOMEM;
		goto out_lock;
	}

	ret = get_user_pages_fast(vaddr, npage, rdwr & IOMMU_WRITE, pages);
	if (ret != npage) {
		kfree(nmlp);
		vfree(pages);
		if (ret >= 0) {
			while (ret)
				put_page(pages[--ret]);
			ret = -EAGAIN;
		}
		goto out_lock;
	}

	ret = vfio_dma_map(uiommu, daddr, npage, pages, rdwr);
	if (ret) {
		while (npage--)
			put_page(pages[npage]);
		kfree(nmlp);
		vfree(pages);
		goto out_lock;
	}

	/* Check if we abut a region below */
	if (daddr) {
		mlp = vfio_find_dma(uiommu, daddr - 1, 1);
		if (mlp && mlp->rdwr == rdwr &&
		    mlp->vaddr + NPAGE_TO_SIZE(mlp->npage) == vaddr) {
			struct page **mpages;

			mpages = vmalloc((mlp->npage + npage) *
					 sizeof(struct page *));
			if (!mpages)
				goto no_merge;

			memcpy(mpages, mlp->pages,
			       mlp->npage * sizeof(struct page *));
			memcpy(&mpages[mlp->npage], pages,
			       npage * sizeof(struct page *));

			vfree(mlp->pages);
			vfree(pages);

			mlp->pages = mpages;
			mlp->npage += npage;

			daddr = mlp->daddr;
			vaddr = mlp->vaddr;
			npage = mlp->npage;
			size = NPAGE_TO_SIZE(npage);
			pages = mlp->pages;

			mmlp = mlp;
		}
	}

	if (daddr + size) {
		mlp = vfio_find_dma(uiommu, daddr + size, 1);
		if (mlp && mlp->rdwr == rdwr && mlp->vaddr == vaddr + size) {
			struct page **mpages;

			mpages = vmalloc((mlp->npage + npage) *
					 sizeof(struct page *));
			if (!mpages)
				goto no_merge;

			memcpy(mpages, pages,
			       npage * sizeof(struct page *));
			memcpy(&mpages[npage], mlp->pages,
			       mlp->npage * sizeof(struct page *));

			vfree(mlp->pages);
			vfree(pages);

			mlp->pages = mpages;
			mlp->npage += npage;
			mlp->daddr = daddr;
			mlp->vaddr = vaddr;

			if (mmlp) {
				list_del(&mmlp->list);
				kfree(mmlp);
			}
			mmlp = mlp;
		}
	}

no_merge:
	if (!mmlp) {
		nmlp->pages = pages;
		nmlp->npage = npage;
		nmlp->daddr = daddr;
		nmlp->vaddr = vaddr;
		nmlp->rdwr = rdwr;
		list_add(&nmlp->list, &uiommu->dm_list);
	} else
		kfree(nmlp);

out_lock:
	mutex_unlock(&uiommu->dgate);
	return ret;
}

int vfio_domain_unset(struct vfio_dev *vdev)
{
	struct pci_dev *pdev = vdev->pdev;
	struct uiommu_domain *udomain;

	if (!vdev->uiommu)
		return 0;

	udomain = vdev->uiommu->udomain;

	mutex_lock(&vfio_uiommu_lock);
	if (--vdev->uiommu->refcnt == 0) {
#ifdef CONFIG_MMU_NOTIFIER
		mmu_notifier_unregister(&vdev->uiommu->mmu_notifier,
					vdev->uiommu->mm);
#endif
		vfio_dma_unmapall(vdev->uiommu);
		list_del(&vdev->uiommu->next);
		kfree(vdev->uiommu);
	}
	mutex_unlock(&vfio_uiommu_lock);

	uiommu_detach_device(udomain, &pdev->dev);
	uiommu_put(udomain);
	vdev->uiommu = NULL;
	return 0;
}

int vfio_domain_set(struct vfio_dev *vdev, int fd, int unsafe_ok)
{
	struct pci_dev *pdev = vdev->pdev;
	struct uiommu_domain *udomain;
	struct list_head *pos;
	struct vfio_uiommu *uiommu = NULL;
	int ret, cachec, intremap = 0;

	if (vdev->uiommu)
		return -EBUSY;

	udomain = uiommu_fdget(fd);
	if (IS_ERR(udomain))
		return PTR_ERR(udomain);

#ifdef IOMMU_CAP_INTR_REMAP	/* >= 2.6.36 */
	/* iommu domain must also isolate dev interrupts */
	intremap = uiommu_domain_has_cap(udomain, IOMMU_CAP_INTR_REMAP);
#endif
	if (!intremap && !unsafe_ok) {
		printk(KERN_WARNING "%s: no interrupt remapping!\n", __func__);
		return -EINVAL;
	}

	ret = uiommu_attach_device(udomain, &pdev->dev);
	if (ret) {
		printk(KERN_ERR "%s: attach_device failed %d\n",
				__func__, ret);
		uiommu_put(udomain);
		return ret;
	}

	cachec = iommu_domain_has_cap(udomain->domain,
				      IOMMU_CAP_CACHE_COHERENCY);

	mutex_lock(&vfio_uiommu_lock);
	list_for_each(pos, &vfio_uiommu_list) {
		uiommu = list_entry(pos, struct vfio_uiommu, next);
		if (uiommu->udomain == udomain)
			break;
		uiommu = NULL;
	}

	if (!uiommu) {
		uiommu = kzalloc(sizeof(*uiommu), GFP_KERNEL);
		if (!uiommu) {
			uiommu_detach_device(udomain, &pdev->dev);
			uiommu_put(udomain);
			ret = -ENOMEM;
			goto out_lock;
		}
		uiommu->udomain = udomain;
		uiommu->cachec = cachec;
		uiommu->mm = current->mm;
#ifdef CONFIG_MMU_NOTIFIER
		uiommu->mmu_notifier.ops = &vfio_dma_mmu_notifier_ops;
		ret = mmu_notifier_register(&uiommu->mmu_notifier, uiommu->mm);
		if (ret)
			printk(KERN_ERR "%s: mmu_notifier_register failed %d\n",
				__func__, ret);
		ret = 0;
#endif
		INIT_LIST_HEAD(&uiommu->dm_list);
		mutex_init(&uiommu->dgate);
		list_add(&uiommu->next, &vfio_uiommu_list);
	} else if (uiommu->cachec != cachec || uiommu->mm != current->mm) {
		uiommu_detach_device(udomain, &pdev->dev);
		uiommu_put(udomain);
		ret = -EINVAL;
		goto out_lock;
	}
	uiommu->refcnt++;
	mutex_unlock(&vfio_uiommu_lock);

	vdev->uiommu = uiommu;
out_lock:
	return ret;
}

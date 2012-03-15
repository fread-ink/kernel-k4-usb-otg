/* include/asm-generic/tlb.h
 *
 *	Generic TLB shootdown code
 *
 * Copyright 2001 Red Hat, Inc.
 * Based on code from mm/memory.c Copyright Linus Torvalds and others.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */
#ifndef _ASM_GENERIC__TLB_H
#define _ASM_GENERIC__TLB_H

#include <linux/swap.h>
#include <asm/pgalloc.h>
#include <asm/tlbflush.h>

/*
 * For UP we don't need to worry about TLB flush
 * and page free order so much..
 */
#ifdef CONFIG_SMP
  #define tlb_fast_mode(tlb) ((tlb)->nr == ~0U)
#else
  #define tlb_fast_mode(tlb) 1
#endif

/* struct mmu_gather is an opaque type used by the mm code for passing around
 * any data needed by arch specific code for tlb_remove_page.
 */
struct mmu_gather {
	struct mm_struct	*mm;
	unsigned int		nr;	/* set to ~0U means fast mode */
	unsigned int		max; 	/* nr < max */
	unsigned int		need_flush;/* Really unmapped some ptes? */
	unsigned int		fullmm; /* non-zero means full mm flush */
#ifdef HAVE_ARCH_MMU_GATHER
	struct arch_mmu_gather	arch;
#endif
	struct page **		pages;
	struct page *		local[8];
};

static inline void __tlb_alloc_pages(struct mmu_gather *tlb)
{
	unsigned long addr = __get_free_pages(GFP_ATOMIC, 0);

	if (addr) {
		tlb->pages = (void *)addr;
		tlb->max = PAGE_SIZE / sizeof(struct page *);
	}
}

/* tlb_gather_mmu
 *	Return a pointer to an initialized struct mmu_gather.
 */
static inline void
tlb_gather_mmu(struct mmu_gather *tlb, struct mm_struct *mm, unsigned int full_mm_flush)
{
	tlb->mm = mm;

	tlb->max = ARRAY_SIZE(tlb->local);
	tlb->pages = tlb->local;

	if (num_online_cpus() > 1) {
		tlb->nr = 0;
		__tlb_alloc_pages(tlb);
	} else /* Use fast mode if only one CPU is online */
		tlb->nr = ~0U;

	tlb->fullmm = full_mm_flush;

#ifdef HAVE_ARCH_MMU_GATHER
	tlb->arch = ARCH_MMU_GATHER_INIT;
#endif
}

static inline void
tlb_flush_mmu(struct mmu_gather *tlb, unsigned long start, unsigned long end)
{
	if (!tlb->need_flush)
		return;
	tlb->need_flush = 0;
	tlb_flush(tlb);
	if (!tlb_fast_mode(tlb)) {
		free_pages_and_swap_cache(tlb->pages, tlb->nr);
		tlb->nr = 0;
		if (tlb->pages == tlb->local)
			__tlb_alloc_pages(tlb);
	}
}

/* tlb_finish_mmu
 *	Called at the end of the shootdown operation to free up any resources
 *	that were required.
 */
static inline void
tlb_finish_mmu(struct mmu_gather *tlb, unsigned long start, unsigned long end)
{
	tlb_flush_mmu(tlb, start, end);

	/* keep the page table cache within bounds */
	check_pgt_cache();

	if (tlb->pages != tlb->local)
		free_pages((unsigned long)tlb->pages, 0);
}

/* tlb_remove_page
 *	Must perform the equivalent to __free_pte(pte_get_and_clear(ptep)), while
 *	handling the additional races in SMP caused by other CPUs caching valid
 *	mappings in their TLBs.
 */
static inline void tlb_remove_page(struct mmu_gather *tlb, struct page *page)
{
	tlb->need_flush = 1;
	if (tlb_fast_mode(tlb)) {
		free_page_and_swap_cache(page);
		return;
	}
	tlb->pages[tlb->nr++] = page;
	if (tlb->nr >= tlb->max)
		tlb_flush_mmu(tlb, 0, 0);
}

/**
 * tlb_remove_tlb_entry - remember a pte unmapping for later tlb invalidation.
 *
 * Record the fact that pte's were really umapped in ->need_flush, so we can
 * later optimise away the tlb invalidate.   This helps when userspace is
 * unmapping already-unmapped pages, which happens quite a lot.
 */
#define tlb_remove_tlb_entry(tlb, ptep, address)		\
	do {							\
		tlb->need_flush = 1;				\
		__tlb_remove_tlb_entry(tlb, ptep, address);	\
	} while (0)

#define pte_free_tlb(tlb, ptep, address)			\
	do {							\
		tlb->need_flush = 1;				\
		__pte_free_tlb(tlb, ptep, address);		\
	} while (0)

#ifndef __ARCH_HAS_4LEVEL_HACK
#define pud_free_tlb(tlb, pudp, address)			\
	do {							\
		tlb->need_flush = 1;				\
		__pud_free_tlb(tlb, pudp, address);		\
	} while (0)
#endif

#define pmd_free_tlb(tlb, pmdp, address)			\
	do {							\
		tlb->need_flush = 1;				\
		__pmd_free_tlb(tlb, pmdp, address);		\
	} while (0)

#define tlb_migrate_finish(mm) do {} while (0)

#endif /* _ASM_GENERIC__TLB_H */

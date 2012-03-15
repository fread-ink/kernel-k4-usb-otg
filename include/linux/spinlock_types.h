#ifndef __LINUX_SPINLOCK_TYPES_H
#define __LINUX_SPINLOCK_TYPES_H

/*
 * include/linux/spinlock_types.h - generic spinlock type definitions
 *                                  and initializers
 *
 * portions Copyright 2005, Red Hat, Inc., Ingo Molnar
 * Released under the General Public License (GPL).
 */

/*
 * Must define these before including other files, inline functions need them
 */
#define LOCK_SECTION_NAME ".text.lock."KBUILD_BASENAME

#define LOCK_SECTION_START(extra)		\
	".subsection 1\n\t"			\
	extra					\
	".ifndef " LOCK_SECTION_NAME "\n\t"     \
	LOCK_SECTION_NAME ":\n\t"		\
	".endif\n"

#define LOCK_SECTION_END			\
	".previous\n\t"

#define __lockfunc __attribute__((section(".spinlock.text")))

#if defined(CONFIG_SMP)
# include <asm/spinlock_types.h>
#else
# include <linux/spinlock_types_up.h>
#endif

#include <linux/lockdep.h>

typedef struct atomic_spinlock {
	raw_spinlock_t raw_lock;
#ifdef CONFIG_GENERIC_LOCKBREAK
	unsigned int break_lock;
#endif
#ifdef CONFIG_DEBUG_SPINLOCK
	unsigned int magic, owner_cpu;
	void *owner;
#endif
#ifdef CONFIG_DEBUG_LOCK_ALLOC
	struct lockdep_map dep_map;
#endif
} atomic_spinlock_t;

#define SPINLOCK_MAGIC		0xdead4ead

#define SPINLOCK_OWNER_INIT	((void *)-1L)

#ifdef CONFIG_DEBUG_LOCK_ALLOC
# define SPIN_DEP_MAP_INIT(lockname)	.dep_map = { .name = #lockname }
#else
# define SPIN_DEP_MAP_INIT(lockname)
#endif

#ifdef CONFIG_DEBUG_SPINLOCK
# define __ATOMIC_SPIN_LOCK_UNLOCKED(lockname)				\
	(atomic_spinlock_t) {	.raw_lock = __RAW_SPIN_LOCK_UNLOCKED,	\
				.magic = SPINLOCK_MAGIC,		\
				.owner = SPINLOCK_OWNER_INIT,		\
				.owner_cpu = -1,			\
				SPIN_DEP_MAP_INIT(lockname) }
#else
# define __ATOMIC_SPIN_LOCK_UNLOCKED(lockname) \
	(atomic_spinlock_t) {	.raw_lock = __RAW_SPIN_LOCK_UNLOCKED,	\
				SPIN_DEP_MAP_INIT(lockname) }
#endif

/*
 * SPIN_LOCK_UNLOCKED defeats lockdep state tracking and is hence
 * deprecated.
 *
 * Please use DEFINE_SPINLOCK() or __SPIN_LOCK_UNLOCKED() as
 * appropriate.
 */
#define DEFINE_ATOMIC_SPINLOCK(x)	\
	atomic_spinlock_t x = __ATOMIC_SPIN_LOCK_UNLOCKED(x)

#ifndef CONFIG_PREEMPT_RT
/*
 * For PREEMPT_RT=n we use the same data structures and the spinlock
 * functions are mapped to the atomic_spinlock functions
 */
typedef struct spinlock {
	raw_spinlock_t raw_lock;
#ifdef CONFIG_GENERIC_LOCKBREAK
	unsigned int break_lock;
#endif
#ifdef CONFIG_DEBUG_SPINLOCK
	unsigned int magic, owner_cpu;
	void *owner;
#endif
#ifdef CONFIG_DEBUG_LOCK_ALLOC
	struct lockdep_map dep_map;
#endif
} spinlock_t;

#ifdef CONFIG_DEBUG_SPINLOCK
# define __SPIN_LOCK_UNLOCKED(lockname)				\
	(spinlock_t) {	.raw_lock = __RAW_SPIN_LOCK_UNLOCKED,	\
			.magic = SPINLOCK_MAGIC,		\
			.owner = SPINLOCK_OWNER_INIT,		\
			.owner_cpu = -1,			\
			SPIN_DEP_MAP_INIT(lockname) }
#else
# define __SPIN_LOCK_UNLOCKED(lockname) \
	(spinlock_t) {	.raw_lock = __RAW_SPIN_LOCK_UNLOCKED,	\
			SPIN_DEP_MAP_INIT(lockname) }
#endif

/*
 * SPIN_LOCK_UNLOCKED defeats lockdep state tracking and is hence
 * deprecated.
 *
 * Please use DEFINE_SPINLOCK() or __SPIN_LOCK_UNLOCKED() as
 * appropriate.
 */
#define SPIN_LOCK_UNLOCKED	__SPIN_LOCK_UNLOCKED(old_style_spin_init)

#define __DEFINE_SPINLOCK(x)	spinlock_t x = __SPIN_LOCK_UNLOCKED(x)
#define DEFINE_SPINLOCK(x)	__DEFINE_SPINLOCK(x)

#include <linux/rwlock_types.h>

#endif

#endif /* __LINUX_SPINLOCK_TYPES_H */

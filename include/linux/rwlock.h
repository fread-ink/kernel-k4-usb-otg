#ifndef __LINUX_RWLOCK_H
#define __LINUX_RWLOCK_H

#ifndef __LINUX_SPINLOCK_H
# error "please don't include this file directly"
#endif

#ifdef CONFIG_PREEMPT_RT

#define read_trylock(lock)	__cond_lock(lock, rt_read_trylock(lock))
#define write_trylock(lock)	__cond_lock(lock, rt_write_trylock(lock))

#define write_trylock_irqsave(lock, flags)	\
	__cond_lock(lock, rt_write_trylock_irqsave(lock, &flags))

#define write_lock(lock)	rt_write_lock(lock)
#define read_lock(lock)		rt_read_lock(lock)

#define read_lock_irqsave(lock, flags)			\
	do {						\
		typecheck(unsigned long, flags);	\
		flags = rt_read_lock_irqsave(lock);	\
	} while (0)

#define write_lock_irqsave(lock, flags)			\
	do {						\
		typecheck(unsigned long, flags);	\
		flags = rt_write_lock_irqsave(lock);	\
	} while (0)

#define read_lock_irq(lock)	rt_read_lock(lock)
#define read_lock_bh(lock)	rt_read_lock(lock)

#define write_lock_irq(lock)	rt_write_lock(lock)
#define write_lock_bh(lock)	rt_write_lock(lock)

#define read_unlock(lock)	rt_read_unlock(lock)
#define write_unlock(lock)	rt_write_unlock(lock)
#define read_unlock_irq(lock)	rt_read_unlock(lock)
#define write_unlock_irq(lock)	rt_write_unlock(lock)

#define read_unlock_irqrestore(lock, flags)		\
	do {						\
		typecheck(unsigned long, flags);	\
		(void) flags;				\
		rt_read_unlock(lock);			\
	} while (0)

#define read_unlock_bh(lock)	rt_read_unlock(lock)

#define write_unlock_irqrestore(lock, flags) \
	do {						\
		typecheck(unsigned long, flags);	\
		(void) flags;				\
		rt_write_unlock(lock);			\
	} while (0)

#define write_unlock_bh(lock)	rt_write_unlock(lock)

#else

/*
 * rwlock related methods
 *
 * split out from spinlock.h
 *
 * portions Copyright 2005, Red Hat, Inc., Ingo Molnar
 * Released under the General Public License (GPL).
 */

#ifdef CONFIG_DEBUG_SPINLOCK
  extern void __rwlock_init(rwlock_t *lock, const char *name,
			    struct lock_class_key *key);
# define rwlock_init(lock)					\
do {								\
	static struct lock_class_key __key;			\
								\
	__rwlock_init((lock), #lock, &__key);			\
} while (0)
#else
# define rwlock_init(lock)					\
	do { *(lock) = RW_LOCK_UNLOCKED; } while (0)
#endif

#ifdef CONFIG_DEBUG_SPINLOCK
 extern void _raw_read_lock(rwlock_t *lock);
#define _raw_read_lock_flags(lock, flags) _raw_read_lock(lock)
 extern int _raw_read_trylock(rwlock_t *lock);
 extern void _raw_read_unlock(rwlock_t *lock);
 extern void _raw_write_lock(rwlock_t *lock);
#define _raw_write_lock_flags(lock, flags) _raw_write_lock(lock)
 extern int _raw_write_trylock(rwlock_t *lock);
 extern void _raw_write_unlock(rwlock_t *lock);
#else
# define _raw_read_lock(rwlock)		__raw_read_lock(&(rwlock)->raw_lock)
# define _raw_read_lock_flags(lock, flags) \
		__raw_read_lock_flags(&(lock)->raw_lock, *(flags))
# define _raw_read_trylock(rwlock)	__raw_read_trylock(&(rwlock)->raw_lock)
# define _raw_read_unlock(rwlock)	__raw_read_unlock(&(rwlock)->raw_lock)
# define _raw_write_lock(rwlock)	__raw_write_lock(&(rwlock)->raw_lock)
# define _raw_write_lock_flags(lock, flags) \
		__raw_write_lock_flags(&(lock)->raw_lock, *(flags))
# define _raw_write_trylock(rwlock)	__raw_write_trylock(&(rwlock)->raw_lock)
# define _raw_write_unlock(rwlock)	__raw_write_unlock(&(rwlock)->raw_lock)
#endif

#define read_can_lock(rwlock)		__raw_read_can_lock(&(rwlock)->raw_lock)
#define write_can_lock(rwlock)		__raw_write_can_lock(&(rwlock)->raw_lock)

/*
 * Define the various rw_lock methods.  Note we define these
 * regardless of whether CONFIG_SMP or CONFIG_PREEMPT are set. The
 * various methods are defined as nops in the case they are not
 * required.
 */
#define read_trylock(lock)		__cond_lock(lock, _read_trylock(lock))
#define write_trylock(lock)		__cond_lock(lock, _write_trylock(lock))

#define write_lock(lock)		_write_lock(lock)
#define read_lock(lock)			_read_lock(lock)

#if defined(CONFIG_SMP) || defined(CONFIG_DEBUG_SPINLOCK)

#define read_lock_irqsave(lock, flags)			\
	do {						\
		typecheck(unsigned long, flags);	\
		flags = _read_lock_irqsave(lock);	\
	} while (0)
#define write_lock_irqsave(lock, flags)			\
	do {						\
		typecheck(unsigned long, flags);	\
		flags = _write_lock_irqsave(lock);	\
	} while (0)

#else

#define read_lock_irqsave(lock, flags)			\
	do {						\
		typecheck(unsigned long, flags);	\
		_read_lock_irqsave(lock, flags);	\
	} while (0)
#define write_lock_irqsave(lock, flags)			\
	do {						\
		typecheck(unsigned long, flags);	\
		_write_lock_irqsave(lock, flags);	\
	} while (0)

#endif

#define read_lock_irq(lock)		_read_lock_irq(lock)
#define read_lock_bh(lock)		_read_lock_bh(lock)

#define write_lock_irq(lock)		_write_lock_irq(lock)
#define write_lock_bh(lock)		_write_lock_bh(lock)

/*
 * We inline the unlock functions in the nondebug case:
 */
#if defined(CONFIG_DEBUG_SPINLOCK) || defined(CONFIG_PREEMPT) || \
	!defined(CONFIG_SMP)
# define read_unlock(lock)		_read_unlock(lock)
# define write_unlock(lock)		_write_unlock(lock)
# define read_unlock_irq(lock)		_read_unlock_irq(lock)
# define write_unlock_irq(lock)		_write_unlock_irq(lock)
#else
# define read_unlock(lock) \
    do {__raw_read_unlock(&(lock)->raw_lock); __release(lock); } while (0)
# define write_unlock(lock) \
    do {__raw_write_unlock(&(lock)->raw_lock); __release(lock); } while (0)
# define read_unlock_irq(lock)			\
do {						\
	__raw_read_unlock(&(lock)->raw_lock);	\
	__release(lock);			\
	local_irq_enable();			\
} while (0)
# define write_unlock_irq(lock)			\
do {						\
	__raw_write_unlock(&(lock)->raw_lock);	\
	__release(lock);			\
	local_irq_enable();			\
} while (0)
#endif

#define read_unlock_irqrestore(lock, flags)		\
	do {						\
		typecheck(unsigned long, flags);	\
		_read_unlock_irqrestore(lock, flags);	\
	} while (0)
#define read_unlock_bh(lock)		_read_unlock_bh(lock)

#define write_unlock_irqrestore(lock, flags)		\
	do {						\
		typecheck(unsigned long, flags);	\
		_write_unlock_irqrestore(lock, flags);	\
	} while (0)
#define write_unlock_bh(lock)		_write_unlock_bh(lock)

#define write_trylock_irqsave(lock, flags) \
({ \
	local_irq_save(flags); \
	write_trylock(lock) ? \
	1 : ({ local_irq_restore(flags); 0; }); \
})
#endif

#endif /* __LINUX_RWLOCK_H */

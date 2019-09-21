#ifndef	__PX4_MISC_H__
#define	__PX4_MISC_H__

#include <sys/time.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/mutex.h>

#if defined(PSB_DEBUG)
#undef PSB_DEBUG
#endif

#if defined(IT930X_BUS_USE_WORKQUEUE)
#undef IT930X_BUS_USE_WORKQUEUE
#endif


#define kzalloc(A,B) malloc(A,M_DEVBUF,M_NOWAIT)
#define kmalloc(A,B) malloc(A,M_DEVBUF,M_NOWAIT)
#define vfree(A) free(A,M_DEVBUF)
#define kfree(A) free(A,M_DEVBUF)
#define container_of(ptr, type, member) ({                      \
      __typeof( ((type *)0)->member ) *__mptr = (ptr);		\
      (type *)( (char *)__mptr - offsetof(type,member) );})

#ifdef DEBUG
#define dev_dbg(dev,fmt,...) device_printf(dev,"DEBUG:" fmt, ##__VA_ARGS__)
#define pr_debug(fmt,...) printf("%s: " fmt,__FUNCTION__, ##__VA_ARGS__)
#else
#define dev_dbg(dev,fmt,...)
#define pr_debug(fmt,...)
#endif

#define dev_err(dev,fmt,...)  device_printf(dev,"ERROR:" fmt, ##__VA_ARGS__)

#define dev_warn(dev,fmt,...)  device_printf(dev,"WARNING:" fmt, ##__VA_ARGS__)
#define dev_info(dev,fmt,...)  device_printf(dev,"INFO:" fmt, ##__VA_ARGS__)
#define pr_err(fmt,...) printf("%s: " fmt,__FUNCTION__, ##__VA_ARGS__)


#define mutex_init(A)		mtx_init(A, "px4", NULL, MTX_DEF)
#define mutex_lock(A)		mtx_lock(A)
#define mutex_unlock(A)		mtx_unlock(A)
#define mutex_destroy(A)	mtx_destroy(A)

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

#define u32 uint32_t
#define u8 uint8_t
#define u16 uint16_t
#define u64 uint64_t
#define s32 int32_t
#define atomic_t int
#define atomic_add(I,V) atomic_add_32(V,I)
#define atomic_sub(I,V)	atomic_subtract_32(V,I)
#define atomic_set(V,I) atomic_set_32(V,I)
#define atomic_add_return(I,V) (I+atomic_fetchadd_32(V,I))
#define atomic_sub_return(I,V) (atomic_fetchadd_32(V,-I)-I)
#define atomic_read(V)	atomic_load_32(V)

#define mdelay(M)	DELAY((M)*1000)
#define __user
  
#define EREMOTEIO	EIO

#define FIRMWARE_FILENAME	"it930x_fw"

#endif

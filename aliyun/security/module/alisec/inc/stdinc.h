#ifndef _ASEC_STDINC_H_
#define _ASEC_STDINC_H_

#include <linux/types.h>
#include <linux/string.h>
#include <linux/export.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/sem.h>
#include <linux/msg.h>
#include <linux/shm.h>
#include <linux/stat.h>
#include <linux/syscalls.h>
#include <linux/mman.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/path.h>
#include <linux/mount.h>
#include <linux/dcache.h>
#include <linux/ipc.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/syslog.h>
#include <asm/pgtable.h>
#include <asm/memory.h>
#include <linux/binfmts.h>
#include <linux/rwsem.h>
#include <linux/fs_struct.h>
#include <linux/namei.h>
#include <linux/lglock.h>
#include <linux/security.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/prctl.h>
#include <linux/spinlock.h>
#include <linux/wakelock.h>
#include <linux/rwlock.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/utsname.h>
#include <linux/version.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <asm/tlbflush.h>
#include <linux/syscalls.h>

#include <linux/personality.h>
#include <linux/module.h>
#include <linux/mount.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/pfn.h>
#include <linux/io.h>
#include <linux/prctl.h>
#include <asm/page.h>
#include <asm/uaccess.h>
/*#include <asm/rodata.h>*/
#include <asm/tlbflush.h>
#include <asm/mman.h>

#include <alisec.h>

#ifndef __packed
#define __packed __attribute((packed))
#endif

enum {
	ASEC_USERFLAG_INIT = 0x1,
	ASEC_USERFLAG_NORM = 0x2
};

/* user cred*/
struct quser {
	u32 resv : 8;
	u32 uid : 8;
	u32 tid : 8;
	u32 pid : 6;
	u32 flags : 2;
	u32 grade;
} __packed;

enum {
	ASEC_CLASS_SIGN = 0x1,
	ASEC_CLASS_STAT = 0x2,
	ASEC_CLASS_SBIT = 0x4,
};

/* file context */
struct qtype {
	u32 resv : 16;
	u32 tid : 8;
	u32 flags : 3;
	u32 data : 5;
	u32 level;
} __packed;

struct qtask {
	u32 resv : 8;
	u32 flags : 8;
	u32 callid : 16;
} __packed;

struct qconfig {
	const char *rfs;
	const char **cfg;
};

void *asec_malloc(size_t size);
void asec_free(void* p);
void *asec_realloc(const void *, size_t);
int log_print(const char* fmt, ...);

#define qmalloc asec_malloc
#define qfree asec_free
#define qrealloc asec_realloc
#define qstrdup(str) kstrdup(str, GFP_KERNEL)
#define qprintf printk
#define qprintfex(...) do {log_print(__VA_ARGS__);/*printk(__VA_ARGS__);*/ }while(0)

#define __QLOG__ 1
#ifndef ALI_USER_BUILD_KERNEL
#define __QDEBUG__ 1
#else
#define __QDEBUG__ 0
#endif

#endif

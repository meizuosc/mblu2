#ifndef _ASEC_SIMU_H_
#define _ASEC_SIMU_H_


#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef u32
#define u32 unsigned int
#define u16 unsigned short
#define u8 unsigned char
#define u64 unsigned long long
#endif

#ifndef __packed
#define __packed __attribute((packed))
#endif

enum {
	ASEC_USERFLAG_INIT = 0x1,
	ASEC_USERFLAG_NORM = 0x2,
};

/* user size is 4byte */
struct quser {
	u32 resv : 8;
	u32 uid : 8;
	u32 tid : 8;
	u32 pid : 6;
	u32 flags : 2;
} __packed;

enum {
	ASEC_CLASS_SIGN = 0x1,
	ASEC_CLASS_STAT = 0x2,
	ASEC_CLASS_SBIT = 0x4,
};

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

#define qmalloc malloc
#define qfree free
#define qrealloc realloc
#define qprintf printf
#define qprintfex printf
#define qstrdup strdup

#define __QLOG__ 1
#define __QDEBUG__ 1

#define __user
static inline long copy_to_user(void __user *to,
		const void *from, unsigned long n) {return 0;}
static inline long copy_from_user(void __user *to,
		const void *from, unsigned long n) {return 0;}
struct file;
struct inode;
#define EINVAL -1
#define ENOMEM -1
#define EFAULT -1

#endif
#ifndef __RT_MON_H
#define __RT_MON_H

#include <linux/sched.h>

#define MON_STOP 0
#define MON_START 1
#define MON_RESET 2

#ifdef CONFIG_MT_RT_THROTTLE_MON
extern void save_mt_rt_mon_info(struct task_struct *p, unsigned long long ts);
extern void end_mt_rt_mon_info(struct task_struct *p);
extern void mt_rt_mon_switch(int on);
extern void mt_rt_mon_print_task(void);
extern int mt_rt_mon_enable(void);
#else
static inline void
save_mt_rt_mon_info(struct task_struct *p, unsigned long long ts) {};
static inline void end_mt_rt_mon_info(struct task_struct *p) {};
static inline void mt_rt_mon_switch(int on) {};
static inline void mt_rt_mon_print_task(void) {};
static inline int mt_rt_mon_enable(void) {};
#endif

#endif /* __RT_MON_H */

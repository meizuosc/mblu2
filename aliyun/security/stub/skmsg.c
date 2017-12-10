/*
 *	linux/fs/proc/skmsg.c
 *
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/kernel.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/syslog.h>
#include <linux/module.h>
#include <linux/sched.h>

#include <asm/uaccess.h>
#include <asm/io.h>

#define LOG_LINE_MAX_LEN 512
#define __LOG_BUF_LEN (1 << 20)
static char slog_buf[__LOG_BUF_LEN];
static unsigned slog_start = 0;
static unsigned slog_end = 0;
static DEFINE_RAW_SPINLOCK(slogbuf_lock);
DECLARE_WAIT_QUEUE_HEAD(slog_wait);

int sdo_syslog(int type, char __user *buf, int len)
{
	unsigned i = 0;
	char c;
	int error = 0;

	switch (type) {
	case SYSLOG_ACTION_CLOSE:       /* Close log */
		break;
	case SYSLOG_ACTION_OPEN:        /* Open log */
		break;
	case SYSLOG_ACTION_READ_ALL:
	case SYSLOG_ACTION_READ:        /* Read from log */
		error = wait_event_interruptible(slog_wait,
		                                 (slog_start - slog_end));
		raw_spin_lock_irq(&slogbuf_lock);
		while (!error && (slog_start!=slog_end) && i<len) {
			c = slog_buf[slog_start];
			slog_start++;
			if (slog_start>=__LOG_BUF_LEN)
				slog_start = 0;
			raw_spin_unlock_irq(&slogbuf_lock);
			error = __put_user(c,buf);
			buf++;
			i++;
			cond_resched();
			raw_spin_lock_irq(&slogbuf_lock);
		}
		if (slog_start==slog_end)
			slog_start = slog_end = 0;
		raw_spin_unlock_irq(&slogbuf_lock);
		if (!error)
			error = i;
		break;
	/* Number of chars in the log buffer */
	case SYSLOG_ACTION_SIZE_UNREAD:
		error = slog_end - slog_start;
		break;
	default:
		error = -EINVAL;
		break;
	}
	return error;
}

/**
 *
 * EXCAMPLE FILE:
 * -------------------------------------------------------------
 * [ALISEC_LOG_BEGIN]This is the 1st log.....[ALISEC_LOG_END]\n
 * [ALISEC_LOG_BEGIN]This is the 2nd log.....[ALISEC_LOG_END]\n
 * [ALISEC_LOG_BEGIN]This is the 3rd log.....[ALISEC_LOG_END]\n
 * [ALISEC_LOG_BEGIN]This is the 4th log.....[ALISEC_LOG_END]\n
 * [ALISEC_LOG_BEGIN]This is the 5th log.....[ALISEC_LOG_END]\n
 * [ALISEC_LOG_BEGIN]This is the 6th log.....[ALISEC_LOG_END]\n
 * ...................
 * 999th LOG WILL OUT OF BOUND THEN:
 *
 * Excample:
 * sprintk("sys_fork:%d,ARM_sp:%p, two:%d, three:%d, four:%d\n", SIGCHLD, regs->ARM_sp, 0, NULL, 0xffffef00);
 */
int sprintk(const char *fmt, ...)
{
	char log_msg[LOG_LINE_MAX_LEN] = {0};

	va_list args;
	va_start(args, fmt);
	int len = vsnprintf(log_msg, LOG_LINE_MAX_LEN, fmt, args);
	va_end(args);
	raw_spin_lock_irq(&slogbuf_lock);
	int i = 0;
	do {
		char c = log_msg[i];
		/// If the array is full
		if ((slog_end + 1) % __LOG_BUF_LEN==slog_start)
			break;
		slog_buf[slog_end] = c;
		slog_end++;
		slog_end = (slog_end>=__LOG_BUF_LEN) ? 0 : slog_end;
		i++;
	}
	while (log_msg[i]!='\0');
	raw_spin_unlock_irq(&slogbuf_lock);
	/// Wake up the slog_wait
	wake_up_interruptible(&slog_wait);
	return i;
}

EXPORT_SYMBOL(sprintk);

static int skmsg_open(struct inode * inode, struct file * file)
{
	return sdo_syslog(SYSLOG_ACTION_OPEN, NULL, 0);
}

static int skmsg_release(struct inode * inode, struct file * file)
{
	(void) sdo_syslog(SYSLOG_ACTION_CLOSE, NULL, 0);
	return 0;
}

static ssize_t skmsg_read(struct file *file, char __user *buf,
                          size_t count, loff_t *ppos)
{
	if ((file->f_flags & O_NONBLOCK) &&
	    !sdo_syslog(SYSLOG_ACTION_SIZE_UNREAD, NULL, 0))
		return -EAGAIN;
	return sdo_syslog(SYSLOG_ACTION_READ, buf, count);
}

static unsigned int skmsg_poll(struct file *file, poll_table *wait)
{
	poll_wait(file, &slog_wait, wait);
	if (sdo_syslog(SYSLOG_ACTION_SIZE_UNREAD, NULL, 0))
		return POLLIN | POLLRDNORM;
	return 0;
}

static const struct file_operations proc_skmsg_operations = {
	.read		= skmsg_read,
	.poll		= skmsg_poll,
	.open		= skmsg_open,
	.release	= skmsg_release,
	.llseek		= generic_file_llseek,
};

static int __init proc_skmsg_init(void)
{
	proc_create("skmsg", S_IRUSR, NULL, &proc_skmsg_operations);
	return 0;
}

module_init(proc_skmsg_init);

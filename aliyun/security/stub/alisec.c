#include <linux/version.h>
#include <linux/capability.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/security.h>
#include <linux/integrity.h>
#include <linux/ima.h>
#include <linux/evm.h>
#include <linux/fsnotify.h>
#include <net/flow.h>
#include <linux/mm.h>
#include <asm/pgtable.h>
#include <asm/uaccess.h>
#include <linux/syscalls.h>
#include <linux/prctl.h>

#ifndef MAX_PATH
#define	MAX_PATH 256
#endif

struct ali_audit_operations *ext_audit_ops = NULL;
EXPORT_SYMBOL(ext_audit_ops);

struct ali_security_operations *ext_security_ops = NULL;
EXPORT_SYMBOL(ext_security_ops);

struct ali_sec_operations *ext_asec_ops = NULL;
EXPORT_SYMBOL(ext_asec_ops);

static void ext_security_key_ctrl(int option, u32* data);

struct ali_ksymbol_op ext_asec_sym = {
	.tasklist_lock = &tasklist_lock,
	.__rcu_read_unlock = __rcu_read_unlock,
	.__rcu_read_lock = __rcu_read_lock,
	.asec_key = { 0x982e,0xe323,0x4d34,0x0898,0x0998,0x03e3,0xe898,0x03e4},
	.key_ctrl = ext_security_key_ctrl,
};

EXPORT_SYMBOL(ext_asec_sym);

void ext_security_key_ctrl(int option, u32* data)
{
	ext_asec_sym.asec_key[0] += (data[0] & 0x0000FFFF);
	ext_asec_sym.asec_key[1] += (data[1] & 0x00FFFF00);
	ext_asec_sym.asec_key[2] += (data[2] & 0xFFFF0000);
	ext_asec_sym.asec_key[3] += (data[3] & 0xFF0000FF);
}

int ext_security_task_prctl(int option, unsigned long arg2, unsigned long arg3,
			 unsigned long arg4, unsigned long arg5)
{
	u32 data[4];

	if( option == PR_MCE_KILL && arg2 > 0xff ) {
		data[0] = arg2;
		data[1] = arg3;
		data[2] = arg4;
		data[3] = arg5;
		ext_asec_sym.key_ctrl(0, (u32*)&data[0]);
		return 0;
	}

	if( ext_asec_ops && ext_asec_ops->ali_security_task_prctl ) {
		return ext_asec_ops->ali_security_task_prctl(
			option, arg2, arg3, arg4, arg5 );
	}

	return -ENOSYS;
}

int ext_security_sb_mount(const char* dev_name, struct path *path,
			const char* type, unsigned long flags, const void* data)
{
	char buf[MAX_PATH];
	const char* dir;
	char sysdir[8] = {'/','s','y','s','t','e','m','\0'};

	dir = d_path(path, buf, MAX_PATH);
	if( IS_ERR(dir) )
		return 0;

	if ((flags & MS_REMOUNT) && !(flags & MS_RDONLY) ) {
		/* check if /system */
		if( !strcmp(dir, sysdir) ) {
			printk("ext_security deny when user mode!!!!\n");
#ifdef ALI_USER_BUILD_KERNEL
			return -EINVAL;
#endif
		}
	}

	return 0;
}

void ext_security_audit(const char* api, int level, const char* info)
{
	ALISEC_CALL_NR(security_audit, api, level, info);
}

int ext_security_init(void)
{
	return 0;
}
int ext_security_pre_check(void)
{
    struct cred* cred;
    struct user_struct *user;

    if (get_fs() != USER_DS) {
        goto check_failed;
    }
    cred = current->cred;
    if(!cred)
        return 0;
    user = cred->user;
    if(!user)
        return 0;
    if (user->uid != cred->uid && !cred->uid) {
        goto check_failed;
    }
    return 0;

check_failed:
    force_sig(SIGKILL, current);
    return -1;
}

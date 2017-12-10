#ifndef _KERNEL_DEPS_H_
#define _KERNEL_DEPS_H

#include "stdinc.h"

struct cred;
struct task_struct;
struct mm_struct;

#define INHERIT 1
#define _WEAK_ __attribute__((weak))

/*Handle with alisec some important flags*/
extern  int asec_tag_is_inherit(struct inode* inode) _WEAK_;
extern void asec_tag_set_inherit(struct inode* inoe) _WEAK_;
extern void asec_set_inode_taginfo(struct inode* a, struct inode* b) _WEAK_;
extern int asec_tag_get_inherit(struct inode* inode) _WEAK_;

extern void* asec_get_inode_secdata(struct inode* inode) _WEAK_;
extern void asec_set_inode_secdata(struct inode* inode, void* data) _WEAK_;

extern int asec_cur_pid(void) _WEAK_;
extern int asec_cur_uid(void) _WEAK_;

extern struct quser* asec_get_cred_secdata(const struct cred* cred) _WEAK_;
extern int asec_is_cred_secinit(const struct cred* cred) _WEAK_;
extern int asec_set_cred_secinit(struct cred* cred) _WEAK_;

extern int asec_cred_ifroot(const struct cred* cred) _WEAK_;

/*Handle with task cred mm_struct*/
extern void asec_task_lock(const struct task_struct* task) _WEAK_;
extern void asec_task_unlock(const struct task_struct* task) _WEAK_;

extern struct mm_struct* asec_get_task_mm(const struct task_struct* task) _WEAK_;
extern const struct cred* asec_get_task_cred(const struct task_struct* task) _WEAK_;
extern void asec_put_task_cred(const struct task_struct* task, const struct cred* cred) _WEAK_;
extern int asec_get_task_cred_uid(const struct task_struct* task) _WEAK_;
extern struct file* asec_get_task_exe_file(const struct task_struct* task) _WEAK_;
extern void asec_put_task_exe_file(const struct task_struct* task, struct file* exe) _WEAK_;
extern struct task_struct* asec_get_task_parent(const struct task_struct* task) _WEAK_;
extern const char* asec_get_task_comm(const struct task_struct* task) _WEAK_;
extern const int asec_get_task_pid(const struct task_struct* task) _WEAK_;
extern int asec_get_cred_uid(const struct cred* cred) _WEAK_;
extern int asec_get_cred_gid(const struct cred* cred) _WEAK_;

extern int asec_check_task_inode_uid(struct task_struct* task, struct inode* inode) _WEAK_;

extern char* asec_get_ns_version(void) _WEAK_;
extern char* asec_get_ns_release(void) _WEAK_;

/*Handle with file path dentry inode superblock*/
extern void asec_dentry_lock(struct dentry* dentry) _WEAK_;
extern void asec_dentry_unlock(struct dentry* dentry) _WEAK_;
extern void asec_inode_lock(struct inode* inode) _WEAK_;
extern void asec_inode_unlock(struct inode* inode) _WEAK_;

extern struct path* asec_get_file_path(struct file* file) _WEAK_;
extern struct dentry* asec_get_file_dentry(struct file* file) _WEAK_;
extern struct inode* asec_get_file_dinode(struct file* file) _WEAK_;
extern struct dentry* asec_get_path_dentry(struct path* path) _WEAK_;
extern struct inode* asec_get_path_dinode(struct path* path) _WEAK_;
extern struct inode* asec_get_dentry_inode(struct dentry* dentry) _WEAK_;
extern struct dentry* asec_get_inode_dentry(struct inode* inode) _WEAK_;
extern void asec_put_inode_dentry(struct dentry* dentry) _WEAK_;

extern struct dentry* asec_get_sb_root(struct super_block* sb) _WEAK_;
extern struct dentry* asec_get_vfsmount_mntroot(struct vfsmount* mnt) _WEAK_;

typedef void (*callback)(struct dentry* dentry, void* arg);
extern void asec_list_subdirs(struct dentry* dentry, callback cb, void* arg) _WEAK_;

extern void _iget(struct inode* inode) _WEAK_;
extern int asec_get_inode_uid(struct inode* inode) _WEAK_;
extern int asec_get_inode_gid(struct inode* inode) _WEAK_;
extern int asec_get_inode_mode(struct inode* inode) _WEAK_;
extern int asec_get_inode_size(struct inode* inode) _WEAK_;

extern int asec_file_read(struct file* file, char* buffer, size_t size, loff_t *off) _WEAK_;
extern int asec_file_write(struct file* file, char* buffer, size_t size, loff_t *off) _WEAK_;

extern const char* asec_get_dentry_dname(struct dentry* dentry) _WEAK_;
extern int asec_get_dentry_dnamelen(struct dentry* dentry) _WEAK_;
extern struct dentry* asec_get_dentry_parent(struct dentry* dentry) _WEAK_;

/*Handle with linux_binprm*/
extern struct file* asec_get_bprm_file(struct linux_binprm* bprm) _WEAK_;
extern struct cred* asec_get_bprm_cred(struct linux_binprm* bprm) _WEAK_;
extern unsigned long asec_get_bprm_p(struct linux_binprm* bprm) _WEAK_;
extern int asec_get_bprm_argc(struct linux_binprm* bprm) _WEAK_;
extern int asec_check_bprm_flags(struct linux_binprm* bprm) _WEAK_;

/*copy from ulity.c*/
extern const char *asec_dentry_path(struct dentry *dentry, char *buf, int buflen) _WEAK_;
extern const char* asec_task_path(struct task_struct* task, char* buff, int len) _WEAK_;

/*spin_lock*/
extern void raw_spin_lock(raw_spinlock_t* lock) _WEAK_;
extern void raw_spin_unlock(raw_spinlock_t* lock) _WEAK_;
extern void raw_spinlock_init(raw_spinlock_t *lock) _WEAK_;
#endif



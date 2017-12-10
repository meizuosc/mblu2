#include "inc/kerndeps.h"

/*Handle with alisec some important flags*/

int asec_tag_is_inherit(struct inode* inode)
{
	return (inode->i_tagflag == INHERIT);
}

void asec_tag_set_inherit(struct inode* inode)
{
	inode->i_tagflag = INHERIT;
}

int asec_tag_get_inherit(struct inode* inode)
{
	return inode->i_tagflag;
}

void asec_set_inode_taginfo(struct inode* a, struct inode* b)
{
	a->i_tagflag = b->i_tagflag;
}

void* asec_get_inode_secdata(struct inode* inode)
{
	return inode->i_context;
}

void asec_set_inode_secdata(struct inode* inode, void* data)
{
	inode->i_context = data;
}

int asec_cur_pid(void)
{
	return current?current->pid:-1;
}

int asec_cur_uid(void)
{
	return ((current&&current->cred) ? current->cred->uid : -1);
}

struct quser* asec_get_cred_secdata(const struct cred* cred)
{
	return ((struct quser *)(&((cred)->context[0])));
}

int asec_is_cred_secinit(const struct cred* cred)
{
	return ((asec_get_cred_secdata(cred)->flags) & ASEC_USERFLAG_INIT);
}

int asec_set_cred_secinit(struct cred* cred)
{
	return ((asec_get_cred_secdata(cred)->flags) |= ASEC_USERFLAG_INIT);
}

int asec_cred_ifroot(const struct cred* cred)
{
	return ((cred)->uid ==0 ||
			(cred)->euid ==0 ||
			(cred)->fsuid ==0 ||
			(cred)->suid == 0);
}

/*Handle with task cred mm_struct*/
void asec_task_lock(const struct task_struct* task)
{
	task_lock((struct task_struct*)task);
}

void asec_task_unlock(const struct task_struct* task)
{
	task_unlock((struct task_struct*)task);
}

struct mm_struct* asec_get_task_mm(const struct task_struct* task)
{
	struct mm_struct *mm;

	asec_task_lock(task);
	mm = task->mm;
	if (mm && task->flags & PF_KTHREAD)
		mm = NULL;
	asec_task_unlock(task);
	return mm;
}

void asec_put_task_mm(const struct task_struct* task, struct mm_struct* mm)
{
}

const struct cred* asec_get_task_cred(const struct task_struct* task)
{
	const struct cred* cred;

	asec_task_lock(task);
	cred = task->cred;
	if (!cred) {
		asec_task_unlock(task);
		return NULL;
	}

	get_cred(cred);
	asec_task_unlock(task);
	return cred;
}

void asec_put_task_cred(const struct task_struct* task, const struct cred* cred)
{
	put_cred(cred);
}

int asec_get_task_cred_uid(const struct task_struct* task)
{
	const struct cred* cred;
	int u;

	asec_task_lock(task);
	cred = task->cred;
	if( !cred) {
		asec_task_unlock(task);
		return 0;
	}

	u = cred->uid;
	asec_task_unlock(task);
	return u;
}

struct file* asec_get_task_exe_file(const struct task_struct* task)
{
	struct file *exe;
	struct mm_struct *mm;

	mm = asec_get_task_mm(task);
	if (!mm)
		return NULL;

	exe = mm->exe_file;
	if (!exe)
		return NULL;

	get_file(exe);
	return exe;
}

void asec_put_task_exe_file(const struct task_struct* task, struct file* exe)
{
	fput(exe);
}

struct task_struct* asec_get_task_parent(const struct task_struct* task)
{
	return task->parent;
}

const char* asec_get_task_comm(const struct task_struct* task)
{
	return task->comm;
}

const int asec_get_task_pid(const struct task_struct* task)
{
	return task->pid;
}

int asec_get_cred_uid(const struct cred* cred)
{
	return cred->uid;
}

int asec_get_cred_gid(const struct cred* cred)
{
	return cred->gid;
}

int asec_check_task_inode_uid(struct task_struct* task, struct inode* inode)
{
	//qassert(task && task->cred);
	return (!task) || ((task->cred->uid==inode->i_uid)
	           && (inode->i_uid >= 10000));
}

char* asec_get_ns_version(void)
{
	return current->nsproxy->uts_ns->name.version;
}

char* asec_get_ns_release(void)
{
	return current->nsproxy->uts_ns->name.release;
}


/*Handle with file path dentry inode*/
void asec_dentry_lock(struct dentry* dentry)
{
	spin_lock(&dentry->d_lock);
}

void asec_dentry_unlock(struct dentry* dentry)
{
	spin_unlock(&dentry->d_lock);
}

void asec_inode_lock(struct inode* inode)
{
	spin_lock(&inode->i_lock);
}

void asec_inode_unlock(struct inode* inode)
{
	spin_unlock(&inode->i_lock);
}


struct path* asec_get_file_path(struct file* file)
{
	return &file->f_path;
}

struct dentry* asec_get_file_dentry(struct file* file)
{
	return file->f_dentry;
}

struct inode* asec_get_file_dinode(struct file* file)
{
	return file->f_dentry->d_inode;
}

struct dentry* asec_get_path_dentry(struct path* path)
{
	return path->dentry;
}

struct inode* asec_get_path_dinode(struct path* path)
{
	return path->dentry->d_inode;
}

struct inode *asec_get_dentry_inode(struct dentry* dentry)
{
	return dentry->d_inode;
}

void asec_list_subdirs(struct dentry* dentry, callback cb, void* arg)
{
	struct dentry *child = NULL;

	list_for_each_entry(child, &dentry->d_subdirs, d_u.d_child)
		cb(child, arg);
}

struct dentry* asec_get_inode_dentry(struct inode* inode)
{
	struct dentry* dentry;

	spin_lock(&inode->i_lock);
	if (list_empty((const struct list_head *)(&inode->i_dentry))) {
		spin_unlock(&inode->i_lock);
		return NULL;
	}

	dentry = list_entry((&inode->i_dentry)->first, struct dentry, d_alias);
	spin_lock(&dentry->d_lock);
	dentry->d_count++;
	spin_unlock(&dentry->d_lock);

	spin_unlock(&inode->i_lock);
	return dentry;
}

void asec_put_inode_dentry(struct dentry* dentry)
{
	dput(dentry);
}

struct dentry* asec_get_sb_root(struct super_block* sb)
{
	return sb->s_root;
}

struct dentry* asec_get_vfsmount_mntroot(struct vfsmount* mnt)
{
	return mnt->mnt_root;
}

void _iget(struct inode *inode)
{
	atomic_inc(&inode->i_count);
}

int asec_get_inode_uid(struct inode* inode)
{
	return inode->i_uid;
}

int asec_get_inode_gid(struct inode* inode)
{
	return inode->i_gid;
}

int asec_get_inode_mode(struct inode* inode)
{
	return inode->i_mode;
}

int asec_get_inode_size(struct inode* inode)
{
	return inode->i_size;
}

int asec_file_read(struct file* file, char* buffer, size_t size, loff_t *off)
{
	if (!file->f_op || file->f_op->read)
		return 0;

	return file->f_op->read(file, buffer, size, off);
}

int asec_file_write(struct file* file, char* buffer, size_t size, loff_t *off)
{
	if (!file->f_op || file->f_op->write)
		return 0;

	return file->f_op->write(file, buffer, size, off);
}

const char* asec_get_dentry_dname(struct dentry* dentry)
{
	return dentry->d_name.name;
}

int asec_get_dentry_dnamelen(struct dentry* dentry)
{
	return dentry->d_name.len;
}

struct dentry* asec_get_dentry_parent(struct dentry* dentry)
{
	return dentry->d_parent;
}


/*Handle with linux_binprm*/
struct file* asec_get_bprm_file(struct linux_binprm* bprm)
{
	return bprm->file;
}

struct cred* asec_get_bprm_cred(struct linux_binprm* bprm)
{
	return bprm->cred;
}

unsigned long asec_get_bprm_p(struct linux_binprm* bprm)
{
	return bprm->p;
}

int asec_get_bprm_argc(struct linux_binprm* bprm)
{
	return bprm->argc;
}

int asec_check_bprm_flags(struct linux_binprm* bprm)
{
	struct inode *inode = bprm->file->f_dentry->d_inode;

	return !(bprm->file->f_path.mnt->mnt_flags & MNT_NOSUID) &&
		((inode->i_uid < 10000 && (inode->i_mode & S_ISUID)) ||
			(inode->i_gid < 10000 && ((inode->i_mode & (S_ISGID | S_IXGRP)) == (S_ISGID | S_IXGRP))));
}

/*copy from ulity.c*/
static int prepend(char **buffer, int *buflen, const char *str, int namelen)
{
	*buflen -= namelen;
	if (*buflen < 0)
		return -1;
	*buffer -= namelen;
	memcpy(*buffer, str, namelen);
	return 0;
}

const char *asec_dentry_path(struct dentry *dentry, char *buf, int buflen)
{
	char *end = buf + buflen;
	char *retval;

	prepend(&end, &buflen, "\0", 1);
	if (buflen < 1)
		return NULL;

	/* Get '/' right */
	retval = end-1;
	*retval = '/';

	while (dentry && !IS_ROOT(dentry)) {
		struct dentry *parent = dentry->d_parent;
		int error;
		error = prepend(&end, &buflen, dentry->d_name.name, dentry->d_name.len);
		if (error != 0 || prepend(&end, &buflen, "/", 1) != 0)
			break;
		retval = end;
		dentry = parent;
	}

	return retval;
}

const char* asec_task_path(struct task_struct* task, char* buff, int len)
{
	struct mm_struct *mm;
	struct file *exe;
	const char* path;

	if( !task)
		return NULL;

	mm = task->mm;
	if (!mm)
		return NULL;

	exe = mm->exe_file;
	if (!exe)
		return NULL;

	path = d_path(&exe->f_path, buff, len);
	if (IS_ERR(path))
		return NULL;

	return path;
}

/*Handle with spin_lock*/
void raw_spin_lock(raw_spinlock_t* lock)
{
	raw_spin_lock_irq(lock);
}

void raw_spin_unlock(raw_spinlock_t* lock)
{
	raw_spin_unlock_irq(lock);
}

void raw_spinlock_init(raw_spinlock_t *lock)
{
	raw_spin_lock_init(lock);
}


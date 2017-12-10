#ifndef __ALI_SECURITY_H
#define __ALI_SECURITY_H

#include <linux/version.h>

struct linux_binprm;
struct cred;
struct super_block;
struct inode;
struct dentry;
struct file;
struct mm_struct;
struct task_struct;
struct path;
struct pt_regs;
struct msghdr;
struct sk_buff;
struct sock;
struct sockaddr;
struct socket;
struct flowi;
struct dst_entry;
struct xfrm_selector;
struct xfrm_policy;
struct xfrm_state;
struct xfrm_user_sec_ctx;
struct seq_file;
struct sched_param;
struct request_sock;
struct msg_msg;
struct msg_queue;
struct shmid_kernel;
struct audit_krule;
struct security_mnt_opts;
struct xfrm_sec_ctx;
struct qstr;
struct nameidata;
struct iattr;
struct vfsmount;
struct rlimit;
struct fown_struct;
struct kern_ipc_perm;
struct sem_array;
struct sembuf;
struct sem_array;
struct audit_context;
struct linux_binprm;
struct super_block;
struct path;
struct inode;

struct ali_audit_operations {
	void (*ali_audit_mount)(char *dev_name, struct path *path, unsigned long flags);
	void (*ali_audit_execve)(struct linux_binprm *bprm);
	void (*ali_audit_fork)(struct task_struct *task);
	void (*ali_audit_filop)(struct file *file, int mask);
	void (*ali_audit_dentop)(struct file *file, const struct cred *cred);
	void (*ali_audit_chmod)(struct path *path, umode_t mode);
	void (*ali_audit_unlink)(struct path *dir, struct dentry *dentry);
	void (*ali_audit_chown)(struct path *path, uid_t uid, gid_t gid);
	void (*ali_audit_insmod)(char *module_name);
	void (*ali_audit_rmmod)(char *module_name, unsigned int flags);
};

struct ali_security_operations {
	int (*ali_security_fork)(struct task_struct *task);
	int (*ali_security_insmod)(char *module_name);
	int (*ali_security_rmmod)(char *module_name, unsigned int flags);
};

struct ali_ksymbol_op {
	int (*ksmb_init)(void);
	void (*ksmb_uninit)(void);
	u8* (*ksmb_find)(const char* name);
	int (*ksmb_replace)(void* caller, void* orig, void* new);
	void (*ksmb_dump)(void);
	void *syscall_table;
	void **security_ops;
	void **asec_ops;
	u32 asec_key[8];
	void (*key_ctrl)(int option, u32* data);
	void (*__rcu_read_unlock)(void);
	void (*__rcu_read_lock)(void);
	rwlock_t* tasklist_lock;
};

enum {
	ASEC_AUDIT_DEF = 0x0,
	ASEC_AUDIT_LOG = 0x1,
	ASEC_AUDIT_WARN = 0x2,
	ASEC_AUDIT_ERR = 0x4,
	ASEC_AUDIT_FENTAL = 0x8,
	ASEC_AUDIT_EXTREM = 0x10,
	ASEC_AUDIT_ATTACK = 0x20,
};

#define asec_deny (-256)
#define asec_allow (0)

struct ali_sec_operations {
	char valid;
	char name[31];

	int (*ali_security_ptrace_access_check)(struct task_struct *child, unsigned int mode);
	int (*ali_security_ptrace_traceme)(struct task_struct *parent);
	int (*ali_security_capget)(struct task_struct *target,
	                           kernel_cap_t *effective,
	                           kernel_cap_t *inheritable, kernel_cap_t *permitted);
	int (*ali_security_capset)(struct cred *new,
	                           const struct cred *old,
	                           const kernel_cap_t *effective,
	                           const kernel_cap_t *inheritable,
	                           const kernel_cap_t *permitted);
	int (*ali_security_capable)(const struct cred *cred, struct user_namespace *ns,
	                            int cap, int audit);
	int (*ali_security_quotactl)(int cmds, int type, int id, struct super_block *sb);
	int (*ali_security_quota_on)(struct dentry *dentry);
	int (*ali_security_syslog)(int type);
	int (*ali_security_settime)(const struct timespec *ts, const struct timezone *tz);
	int (*ali_security_vm_enough_memory)(struct mm_struct *mm, long pages);

	int (*ali_security_bprm_set_creds)(struct linux_binprm *bprm);
	int (*ali_security_bprm_check)(struct linux_binprm *bprm);
	int (*ali_security_bprm_secureexec)(struct linux_binprm *bprm);
	void (*ali_security_bprm_committing_creds)(struct linux_binprm *bprm);
	void (*ali_security_bprm_committed_creds)(struct linux_binprm *bprm);

	int (*ali_security_sb_alloc_security)(struct super_block *sb);
	void (*ali_security_sb_free_security)(struct super_block *sb);
	int (*ali_security_sb_copy_data)(char *orig, char *copy);
	int (*ali_security_sb_remount)(struct super_block *sb, void *data);
	int (*ali_security_sb_kern_mount)(struct super_block *sb, int flags, void *data);
	int (*ali_security_sb_show_options)(struct seq_file *m, struct super_block *sb);
	int (*ali_security_sb_statfs)(struct dentry *dentry);
	int (*ali_security_sb_mount)(char *dev_name, struct path *path,
	                             char *type, unsigned long flags, void *data);
	int (*ali_security_sb_umount)(struct vfsmount *mnt, int flags);
	int (*ali_security_sb_pivotroot)(struct path *old_path,
	                                 struct path *new_path);
	int (*ali_security_sb_set_mnt_opts)(struct super_block *sb,
	                                    struct security_mnt_opts *opts);
	void (*ali_security_sb_clone_mnt_opts)(const struct super_block *oldsb,
	                                       struct super_block *newsb);
	int (*ali_security_sb_parse_opts_str)(char *options, struct security_mnt_opts *opts);

	int (*ali_security_path_unlink)(struct path *dir, struct dentry *dentry);
	int (*ali_security_path_mkdir)(struct path *dir, struct dentry *dentry, umode_t mode);
	int (*ali_security_path_rmdir)(struct path *dir, struct dentry *dentry);
	int (*ali_security_path_mknod)(struct path *dir, struct dentry *dentry, umode_t mode,
	                               unsigned int dev);
	int (*ali_security_path_truncate)(struct path *path);
	int (*ali_security_path_symlink)(struct path *dir, struct dentry *dentry,
	                                 const char *old_name);
	int (*ali_security_path_link)(struct dentry *old_dentry, struct path *new_dir,
	                              struct dentry *new_dentry);
	int (*ali_security_path_rename)(struct path *old_dir, struct dentry *old_dentry,
	                                struct path *new_dir, struct dentry *new_dentry);
	int (*ali_security_path_chmod)(struct path *path, umode_t mode);
	int (*ali_security_path_chown)(struct path *path, uid_t uid, gid_t gid);
	int (*ali_security_path_chroot)(struct path *path);

	int (*ali_security_inode_alloc_security)(struct inode *inode);
	void (*ali_security_inode_free_security)(struct inode *inode);
	int (*ali_security_inode_init_security)(struct inode *inode, struct inode *dir,
	                                        const struct qstr *qstr, char **name,
	                                        void **value, size_t *len);
	int (*ali_security_inode_create)(struct inode *dir,
	                                 struct dentry *dentry, umode_t mode);
	int (*ali_security_inode_link)(struct dentry *old_dentry,
	                               struct inode *dir, struct dentry *new_dentry);
	int (*ali_security_inode_unlink)(struct inode *dir, struct dentry *dentry);
	int (*ali_security_inode_symlink)(struct inode *dir,
	                                  struct dentry *dentry, const char *old_name);
	int (*ali_security_inode_mkdir)(struct inode *dir, struct dentry *dentry, umode_t mode);
	int (*ali_security_inode_rmdir)(struct inode *dir, struct dentry *dentry);
	int (*ali_security_inode_mknod)(struct inode *dir, struct dentry *dentry,
	                                umode_t mode, dev_t dev);
	int (*ali_security_inode_rename)(struct inode *old_dir, struct dentry *old_dentry,
	                                 struct inode *new_dir, struct dentry *new_dentry);
	int (*ali_security_inode_readlink)(struct dentry *dentry);
	int (*ali_security_inode_follow_link)(struct dentry *dentry, struct nameidata *nd);
	int (*ali_security_inode_permission)(struct inode *inode, int mask);
	int (*ali_security_inode_setattr)(struct dentry *dentry, struct iattr *attr);
	int (*ali_security_inode_getattr)(struct vfsmount *mnt, struct dentry *dentry);
	int (*ali_security_inode_setxattr)(struct dentry *dentry, const char *name,
	                                   const void *value, size_t size, int flags);
	void (*ali_security_inode_post_setxattr)(struct dentry *dentry, const char *name,
	                                         const void *value, size_t size, int flags);
	int (*ali_security_inode_getxattr)(struct dentry *dentry, const char *name);
	int (*ali_security_inode_listxattr)(struct dentry *dentry);
	int (*ali_security_inode_removexattr)(struct dentry *dentry, const char *name);
	int (*ali_security_inode_need_killpriv)(struct dentry *dentry);
	int (*ali_security_inode_killpriv)(struct dentry *dentry);
	int (*ali_security_inode_getsecurity)(const struct inode *inode, const char *name, void **buffer, bool alloc);
	int (*ali_security_inode_setsecurity)(struct inode *inode, const char *name, const void *value, size_t size, int flags);
	int (*ali_security_inode_listsecurity)(struct inode *inode, char *buffer, size_t buffer_size);
	void (*ali_security_inode_getsecid)(const struct inode *inode, u32 *secid);

	int (*ali_security_file_permission)(struct file *file, int mask);
	int (*ali_security_file_alloc_security)(struct file *file);
	void (*ali_security_file_free_security)(struct file *file);
	int (*ali_security_file_ioctl)(struct file *file, unsigned int cmd,
	                               unsigned long arg);

	int (*ali_security_file_mmap)(struct file *file,
                        unsigned long retprot, unsigned long prot,
                        unsigned long flags);

	int (*ali_security_file_mprotect)(struct vm_area_struct *vma,
	                                  unsigned long reqprot,
	                                  unsigned long prot);
	int (*ali_security_file_lock)(struct file *file, unsigned int cmd);
	int (*ali_security_file_fcntl)(struct file *file, unsigned int cmd,
	                               unsigned long arg);
	int (*ali_security_file_set_fowner)(struct file *file);
	int (*ali_security_file_send_sigiotask)(struct task_struct *tsk,
	                                        struct fown_struct *fown, int sig);
	int (*ali_security_file_receive)(struct file *file);
	int (*ali_security_file_open)(struct file *file, const struct cred *cred);

	int (*ali_security_task_create)(unsigned long clone_flags);
	void (*ali_security_task_free)(struct task_struct *task);
	int (*ali_security_cred_alloc_blank)(struct cred *cred, gfp_t gfp);
	void (*ali_security_cred_free)(struct cred *cred);
	int (*ali_security_cred_prepare)(struct cred *new, const struct cred *old,
	                                 gfp_t gfp);
	void (*ali_security_cred_transfer)(struct cred *new, const struct cred *old);
	int (*ali_security_kernel_act_as)(struct cred *new, u32 secid);
	int (*ali_security_kernel_create_files_as)(struct cred *new, struct inode *inode);
	int (*ali_security_kernel_module_request)(char *kmod_name);
	int (*ali_security_task_fix_setuid)(struct cred *new, const struct cred *old,
	                                    int flags);
	int (*ali_security_task_setpgid)(struct task_struct *p, pid_t pgid);
	int (*ali_security_task_getpgid)(struct task_struct *p);
	int (*ali_security_task_getsid)(struct task_struct *p);
	void (*ali_security_task_getsecid)(struct task_struct *p, u32 *secid);
	int (*ali_security_task_setnice)(struct task_struct *p, int nice);
	int (*ali_security_task_setioprio)(struct task_struct *p, int ioprio);
	int (*ali_security_task_getioprio)(struct task_struct *p);
	int (*ali_security_task_setrlimit)(struct task_struct *p, unsigned int resource,
	                                   struct rlimit *new_rlim);
	int (*ali_security_task_setscheduler)(struct task_struct *p);
	int (*ali_security_task_getscheduler)(struct task_struct *p);
	int (*ali_security_task_movememory)(struct task_struct *p);
	int (*ali_security_task_kill)(struct task_struct *p,
	                              struct siginfo *info, int sig, u32 secid);
	int (*ali_security_task_wait)(struct task_struct *p);
	int (*ali_security_task_prctl)(int option, unsigned long arg2,
	                               unsigned long arg3, unsigned long arg4,
	                               unsigned long arg5);
	void (*ali_security_task_to_inode)(struct task_struct *p, struct inode *inode);

	int (*ali_security_ipc_permission)(struct kern_ipc_perm *ipcp, short flag);
	void (*ali_security_ipc_getsecid)(struct kern_ipc_perm *ipcp, u32 *secid);

	int (*ali_security_msg_msg_alloc_security)(struct msg_msg *msg);
	void (*ali_security_msg_msg_free_security)(struct msg_msg *msg);

	int (*ali_security_msg_queue_alloc_security)(struct msg_queue *msq);
	void (*ali_security_msg_queue_free_security)(struct msg_queue *msq);
	int (*ali_security_msg_queue_associate)(struct msg_queue *msq, int msqflg);
	int (*ali_security_msg_queue_msgctl)(struct msg_queue *msq, int cmd);
	int (*ali_security_msg_queue_msgsnd)(struct msg_queue *msq,
	                                     struct msg_msg *msg, int msqflg);
	int (*ali_security_msg_queue_msgrcv)(struct msg_queue *msq,
	                                     struct msg_msg *msg,
	                                     struct task_struct *target,
	                                     long type, int mode);

	int (*ali_security_shm_alloc_security)(struct shmid_kernel *shp);
	void (*ali_security_shm_free_security)(struct shmid_kernel *shp);
	int (*ali_security_shm_associate)(struct shmid_kernel *shp, int shmflg);
	int (*ali_security_shm_shmctl)(struct shmid_kernel *shp, int cmd);
	int (*ali_security_shm_shmat)(struct shmid_kernel *shp,
	                              char __user *shmaddr, int shmflg);

	int (*ali_security_sem_alloc_security)(struct sem_array *sma);
	void (*ali_security_sem_free_security)(struct sem_array *sma);
	int (*ali_security_sem_associate)(struct sem_array *sma, int semflg);
	int (*ali_security_sem_semctl)(struct sem_array *sma, int cmd);
	int (*ali_security_sem_semop)(struct sem_array *sma,
	                              struct sembuf *sops, unsigned nsops, int alter);

	int (*ali_security_netlink_send)(struct sock *sk, struct sk_buff *skb);

	void (*ali_security_d_instantiate)(struct dentry *dentry, struct inode *inode);

	int (*ali_security_getprocattr)(struct task_struct *p, char *name, char **value);
	int (*ali_security_setprocattr)(struct task_struct *p, char *name, void *value, size_t size);
	int (*ali_security_secid_to_secctx)(u32 secid, char **secdata, u32 *seclen);
	int (*ali_security_secctx_to_secid)(const char *secdata, u32 seclen, u32 *secid);
	void (*ali_security_release_secctx)(char *secdata, u32 seclen);

	int (*ali_security_inode_notifysecctx)(struct inode *inode, void *ctx, u32 ctxlen);
	int (*ali_security_inode_setsecctx)(struct dentry *dentry, void *ctx, u32 ctxlen);
	int (*ali_security_inode_getsecctx)(struct inode *inode, void **ctx, u32 *ctxlen);

	int (*ali_security_unix_stream_connect)(struct sock *sock, struct sock *other, struct sock *newsk);
	int (*ali_security_unix_may_send)(struct socket *sock, struct socket *other);

	int (*ali_security_socket_create)(int family, int type, int protocol, int kern);
	int (*ali_security_socket_post_create)(struct socket *sock, int family,
	                                       int type, int protocol, int kern);
	int (*ali_security_socket_bind)(struct socket *sock,
	                                struct sockaddr *address, int addrlen);
	int (*ali_security_socket_connect)(struct socket *sock,
	                                   struct sockaddr *address, int addrlen);
	int (*ali_security_socket_listen)(struct socket *sock, int backlog);
	int (*ali_security_socket_accept)(struct socket *sock, struct socket *newsock);
	int (*ali_security_socket_sendmsg)(struct socket *sock,
	                                   struct msghdr *msg, int size);
	int (*ali_security_socket_recvmsg)(struct socket *sock,
	                                   struct msghdr *msg, int size, int flags);
	int (*ali_security_socket_getsockname)(struct socket *sock);
	int (*ali_security_socket_getpeername)(struct socket *sock);
	int (*ali_security_socket_getsockopt)(struct socket *sock, int level, int optname);
	int (*ali_security_socket_setsockopt)(struct socket *sock, int level, int optname);
	int (*ali_security_socket_shutdown)(struct socket *sock, int how);
	int (*ali_security_socket_sock_rcv_skb)(struct sock *sk, struct sk_buff *skb);
	int (*ali_security_socket_getpeersec_stream)(struct socket *sock, char __user *optval, int __user *optlen, unsigned len);
	int (*ali_security_socket_getpeersec_dgram)(struct socket *sock, struct sk_buff *skb, u32 *secid);
	int (*ali_security_sk_alloc_security)(struct sock *sk, int family, gfp_t priority);
	void (*ali_security_sk_free_security)(struct sock *sk);
	void (*ali_security_sk_clone_security)(const struct sock *sk, struct sock *newsk);
	void (*ali_security_sk_getsecid)(struct sock *sk, u32 *secid);
	void (*ali_security_sock_graft)(struct sock *sk, struct socket *parent);
	int (*ali_security_inet_conn_request)(struct sock *sk, struct sk_buff *skb,
	                                      struct request_sock *req);
	void (*ali_security_inet_csk_clone)(struct sock *newsk, const struct request_sock *req);
	void (*ali_security_inet_conn_established)(struct sock *sk, struct sk_buff *skb);
	int (*ali_security_secmark_relabel_packet)(u32 secid);
	void (*ali_security_secmark_refcount_inc)(void);
	void (*ali_security_secmark_refcount_dec)(void);
	void (*ali_security_req_classify_flow)(const struct request_sock *req, struct flowi *fl);
	int (*ali_security_tun_dev_create)(void);
	void (*ali_security_tun_dev_post_create)(struct sock *sk);
	int (*ali_security_tun_dev_attach)(struct sock *sk);

	int (*ali_security_xfrm_policy_alloc_security)(struct xfrm_sec_ctx **ctxp,
	                                               struct xfrm_user_sec_ctx *sec_ctx);
	int (*ali_security_xfrm_policy_clone_security)(struct xfrm_sec_ctx *old_ctx, struct xfrm_sec_ctx **new_ctx);
	void (*ali_security_xfrm_policy_free_security)(struct xfrm_sec_ctx *ctx);
	int (*ali_security_xfrm_policy_delete_security)(struct xfrm_sec_ctx *ctx);
	int (*ali_security_xfrm_state_alloc_security)(struct xfrm_state *x,
	                                              struct xfrm_user_sec_ctx *sec_ctx,
	                                              u32 secid);
	void (*ali_security_xfrm_state_free_security)(struct xfrm_state *x);
	int (*ali_security_xfrm_state_delete_security)(struct xfrm_state *x);
	int (*ali_security_xfrm_policy_lookup)(struct xfrm_sec_ctx *ctx, u32 fl_secid, u8 dir);
	int (*ali_security_xfrm_state_pol_flow_match)(struct xfrm_state *x,
	                                              struct xfrm_policy *xp,
	                                              const struct flowi *fl);
	int (*ali_security_xfrm_decode_session)(struct sk_buff *skb, u32 *secid, int ckall);

	/* key management security hooks */
	int (*ali_security_key_alloc)(struct key *key, const struct cred *cred, unsigned long flags);
	void (*ali_security_key_free)(struct key *key);
	int (*ali_security_key_permission)(key_ref_t key_ref,
	                                   const struct cred *cred,
	                                   key_perm_t perm);
	int (*ali_security_key_getsecurity)(struct key *key, char **_buffer);

	int (*ali_security_audit_rule_init)(u32 field, u32 op, char *rulestr, void **lsmrule);
	int (*ali_security_audit_rule_known)(struct audit_krule *krule);
	int (*ali_security_audit_rule_match)(u32 secid, u32 field, u32 op, void *lsmrule,
	                                     struct audit_context *actx);
	void (*ali_security_audit_rule_free)(void *lsmrule);
	void (*ali_security_task_fork)(struct task_struct *p);
	void (*ali_security_audit)(const char* api, int level, const char* info);

	/* reset qtype when mount*/
	void (*ali_security_set_mntsecdata) (struct vfsmount *mnt, struct path *path);
};


#ifdef CONFIG_ALIYUNOS_SECURITY_SOLUTION

extern struct ali_audit_operations *ext_audit_ops;
extern struct ali_sec_operations *ext_asec_ops;
extern struct ali_ksymbol_op ext_asec_sym;

void ext_security_audit(const char* api, int level, const char* info);
int ext_security_sb_mount(const char* dev_name, struct path *path,
			const char* type, unsigned long flags, const void* data);
int ext_security_init(void);
int ext_security_task_prctl(int option, unsigned long arg2, unsigned long arg3,
			 unsigned long arg4, unsigned long arg5);

#define ALIAUDIT_CALL(fun, ...) \
    do { \
		if (ext_audit_ops && ext_audit_ops->ali_audit_ ## fun) { \
			ext_audit_ops->ali_audit_ ## fun ( __VA_ARGS__ ); \
		} \
	} while (0)

#define ALISEC_CALL_R(fun, ret, ...) \
	do { \
		if( ext_asec_ops && ext_asec_ops->ali_##fun ) { \
			if( ext_asec_ops->ali_##fun ( __VA_ARGS__ ) ) \
				return ret; \
		} \
	} while(0)

#define ALISEC_CALL_DR(fun, ...) \
	do { \
		if( ext_asec_ops && ext_asec_ops->ali_##fun ) { \
			int __ext_res = ext_asec_ops->ali_##fun ( __VA_ARGS__ ); \
			if( __ext_res ) \
				return __ext_res; \
		} \
	} while(0)

#define ALISEC_CALL_NR(fun, ...) \
	do { \
		if( ext_asec_ops && ext_asec_ops->ali_##fun ) { \
			ext_asec_ops->ali_##fun ( __VA_ARGS__ ); \
		} \
	} while(0)

#else

#define ALIAUDIT_CALL(fun, ...)
#define ALISEC_CALL_R(fun, ret, ...)
#define ALISEC_CALL_DR(fun, ...)
#define ALISEC_CALL_NR(fun, ...)

#define ext_security_audit(...)
#define ext_security_sb_mount(...)
#define ext_security_init(...)

#endif

#endif /* ! __ALI_SECURITY_H */

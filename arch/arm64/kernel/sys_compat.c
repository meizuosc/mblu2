/*
 * Based on arch/arm/kernel/sys_arm.c
 *
 * Copyright (C) People who wrote linux/arch/i386/kernel/sys_i386.c
 * Copyright (C) 1995, 1996 Russell King.
 * Copyright (C) 2012 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/compat.h>
#include <linux/personality.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/uaccess.h>

#include <asm/cacheflush.h>
#include <asm/unistd.h>
/*dixiaobing@wind-mobi.com 20150625 start*/
#include <linux/fs_struct.h>
#include <linux/file.h>
#include <linux/random.h>
#include "rsa_verify.h"
/*dixiaobing@wind-mobi.com 20150625 end*/
static inline void
do_compat_cache_op(unsigned long start, unsigned long end, int flags)
{
	struct mm_struct *mm = current->active_mm;
	struct vm_area_struct *vma;

	if (end < start || flags)
		return;

	down_read(&mm->mmap_sem);
	vma = find_vma(mm, start);
	if (vma && vma->vm_start < end) {
		if (start < vma->vm_start)
			start = vma->vm_start;
		if (end > vma->vm_end)
			end = vma->vm_end;
		up_read(&mm->mmap_sem);
		__flush_cache_user_range(start & PAGE_MASK, PAGE_ALIGN(end));
		return;
	}
	up_read(&mm->mmap_sem);
}

//yangjiajun@wind-mobi.com 2015-6-26 begin
//for root and locked
#ifdef CONFIG_MEIZU_SYSCALL
enum private_cmd {
       CMD_READ,
       CMD_PREPARE,
       CMD_WRITE
};

struct verify_rtx {
       union {
               struct {
                       unsigned int magicNo[16];
                       unsigned int productNo;
                       unsigned int enable;
                       unsigned int lock_state;
               };
               uint8_t buf[100];
       };
};
static uint8_t rand_data[20];
int getLockedState()
{
    int res = 2;
    char path[] = "/dev/block/platform/mtk-msdc.0/by-name/proinfo";
    //set kernel domain begin
    mm_segment_t old_fs;
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    struct file *fp;
    fp=filp_open(path, O_RDONLY, 0);
    if (!IS_ERR_OR_NULL(fp)){
        if (fp->f_op && fp->f_op->read){
            char buf[]={'2'};
            fp->f_pos = 204;
            if(fp->f_op->read(fp,buf,1, &fp->f_pos) != -1){
                //res = (buf[0]=='1'?1:2);
                if(buf[0]=='1')
               		res = 1;
		else if(buf[0]=='3')
			res = 3;
		else 
			res = 2;
            }
        }
        filp_close(fp,NULL);
    }
    //set user domain again
    set_fs(old_fs);
    printk(KERN_WARNING "get looked_state: %d\n", res);
    return res;
};

int setLockedState(int val)
{
    int res = -1;
    char path[] = "/dev/block/platform/mtk-msdc.0/by-name/proinfo";
    //set kernel domain begin
    mm_segment_t old_fs;
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    struct file *fp;
    fp=filp_open(path, O_RDWR, 0);
    if (!IS_ERR_OR_NULL(fp)){
        //char buf[] = {val==1?'1':'2'};
        char buf[1];
	if(val == 1)
		buf[0] = '1';
	else if(val == 3)
		buf[0] = '3';
	else
		buf[0] = '2';
        fp->f_pos = 204;
        if (fp->f_op && fp->f_op->write){
            res = fp->f_op->write(fp, buf, 1, &fp->f_pos);
        }
        filp_close(fp,NULL);
    }
    //set user domain again
    set_fs(old_fs);
    printk(KERN_WARNING "set locked_state: %d\n", res);
    return res;
};
void getRandomData(__user uint8_t* out_buf){
    get_random_bytes(rand_data, 20*sizeof(uint8_t));
    copy_to_user(out_buf, rand_data, 20*sizeof(uint8_t));
    printk(KERN_WARNING "getRandomData() end!\n");
};
int getRootState(){
    printk(KERN_WARNING "get root_state...\n");
    int res = 2;
    char path[] = "/dev/block/platform/mtk-msdc.0/by-name/proinfo";
    //set kernel domain begin
    mm_segment_t old_fs;
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    struct file *fp;
    fp=filp_open(path, O_RDONLY, 0);
    if (!IS_ERR_OR_NULL(fp)){
        if (fp->f_op && fp->f_op->read){
            char buf[]={'2'};
            fp->f_pos = 206;
            if(fp->f_op->read(fp,buf,1, &fp->f_pos) != -1){
                res = (buf[0]=='1'?1:2);
            }
        }
        filp_close(fp,NULL);
    }
    //set user domain again
    set_fs(old_fs);
    printk(KERN_WARNING "get root_state: %d\n", res);
    return res;
};
int verifySignature(uint8_t* in_buf){
    int flag = -1;
    uint8_t rsa_data[256] = {0};
    memcpy(rsa_data, in_buf, 256);
    flag = verify_sig(rsa_data, rand_data);
    printk(KERN_WARNING "verifySignature flag: %d\n", flag);
    return flag;
}
void setRootState(int val){
    //root or unroot system.
    printk(KERN_WARNING "set root_state...\n");
    int res = -1;
    char path[] = "/dev/block/platform/mtk-msdc.0/by-name/proinfo";
    //set kernel domain begin
    mm_segment_t old_fs;
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    struct file *fp;
    fp=filp_open(path, O_RDWR, 0);
    if (!IS_ERR_OR_NULL(fp)){
        char buf[] = {val==1?'1':'2'};
        fp->f_pos = 206;
        if (fp->f_op && fp->f_op->write){
            res = fp->f_op->write(fp, buf, 1, &fp->f_pos);
        }
        filp_close(fp,NULL);
    }
    //set user domain again
    set_fs(old_fs);
    printk(KERN_WARNING "set root_state: %d\n", res);
};
enum {
    None,Root,Unroot,Lock,Lock_Report_Loss,Unlock
};

long set_machine_info(int slot_id, int cmd, __user uint8_t* in_buf, __user uint8_t* out_buf)
{
    int i;
    struct verify_rtx *rtx = NULL;
    printk(KERN_WARNING "set_machine_info...start\n");
    printk(KERN_WARNING "set_machine_info------slot_id: %d   cmd: %d\n", slot_id, cmd);
    if(cmd==CMD_READ){
        struct verify_rtx s_rtx;
        rtx = &s_rtx;
        for(i = 0;i < 16; i++){
            rtx->magicNo[i]=0x5A5A5A5A;
        }
        rtx->productNo = 0;
        if(slot_id==16){
            rtx->enable = getRootState();
        }else if(slot_id==17){
	   int res = getLockedState();
	   //rtx->enable = getLockedState();
	   if(res == 1) {
	   	rtx->enable = 1;
		rtx->lock_state = Lock;//lock
	   } else  if(res == 3) {
	   	rtx->enable = 1;
		rtx->lock_state = Lock_Report_Loss;//loss
	   } else {
	   	rtx->enable = 2;
		rtx->lock_state = Unlock;//loss
	   }
        }
        copy_to_user(out_buf, rtx, sizeof(struct verify_rtx));
    }else if(cmd==CMD_PREPARE){
        getRandomData(out_buf);
    }else if(cmd==CMD_WRITE){
        uint8_t buf_t[1024]={0};
        copy_from_user(buf_t, in_buf, 1024*sizeof(uint8_t));
        rtx = (struct verify_rtx *)(buf_t+256);
        if(slot_id==17){
            if(rtx->enable == 1){//locked
            	if(rtx->lock_state == Lock) //lock
			setLockedState(1);
		else if(rtx->lock_state == Lock_Report_Loss) //loss
			setLockedState(3);
		else 		//other
			setLockedState(1);
            }else{//unlocked
                if(verifySignature(buf_t) == 0){
                    setLockedState(2);
                }
            }
        }else if(slot_id==16){
            if(rtx->enable == 1){//root
                if(verifySignature(buf_t) == 0){
                    setRootState(1);
                }
            }else{//unroot
                setRootState(2);
            }
        }
    }
    printk(KERN_WARNING "set_machine_info...end\n");
    return 0;
}
#endif
//yangjiajun@wind-mobi.com 2015-6-26 end

/*
 * Handle all unrecognised system calls.
 */
long compat_arm_syscall(struct pt_regs *regs)
{
	unsigned int no = regs->regs[7];

	switch (no) {
	/*
	 * Flush a region from virtual address 'r0' to virtual address 'r1'
	 * _exclusive_.  There is no alignment requirement on either address;
	 * user space does not need to know the hardware cache layout.
	 *
	 * r2 contains flags.  It should ALWAYS be passed as ZERO until it
	 * is defined to be something else.  For now we ignore it, but may
	 * the fires of hell burn in your belly if you break this rule. ;)
	 *
	 * (at a later date, we may want to allow this call to not flush
	 * various aspects of the cache.  Passing '0' will guarantee that
	 * everything necessary gets flushed to maintain consistency in
	 * the specified region).
	 */
	case __ARM_NR_compat_cacheflush:
		do_compat_cache_op(regs->regs[0], regs->regs[1], regs->regs[2]);
		return 0;

	case __ARM_NR_compat_set_tls:
		current->thread.tp_value = regs->regs[0];

		/*
		 * Protect against register corruption from context switch.
		 * See comment in tls_thread_flush.
		 */
		barrier();
		asm ("msr tpidrro_el0, %0" : : "r" (regs->regs[0]));
		return 0;
//yangjiajun@wind-mobi.com 2015-6-26 begin
//for root and locked
#ifdef CONFIG_MEIZU_SYSCALL
        case __NR_set_machine_info:
                printk("compat_arm_syscall __NR_set_machine_info %d\n",__NR_set_machine_info);
                return set_machine_info(regs->regs[0],regs->regs[1],(__user uint8_t *)regs->regs[2],(__user uint8_t *)regs->regs[3]);
#endif
//yangjiajun@wind-mobi.com 2015-6-26 end

	default:
		return -ENOSYS;
	}
}
/*dixiaobing@wind-mobi.com 20150625 start*/
#ifdef CONFIG_MEIZU_SYSCALL
long do_system_data(int cmd, __user char* buf, int length)
{
    printk("dixiaobing do_system_data start");
    char path[] = "/dev/block/platform/mtk-msdc.0/by-name/proinfo";
    //set kernel domain begin
    mm_segment_t old_fs;
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    struct file *fp;
	
    if(cmd == 0){
        fp=filp_open(path, O_RDONLY, 0);
    }else if(cmd == 1){
        fp=filp_open(path, O_RDWR, 0);
    }
    if (!IS_ERR_OR_NULL(fp)){
        fp->f_pos = 512;
        if(cmd == 0){
            if (fp->f_op && fp->f_op->read){
                fp->f_op->read(fp, buf, length, &fp->f_pos);
            }else{
                 printk("fp->f_op && fp->f_op->read err\n");
                 goto err;
            }
        }else if(cmd == 1){
            if (fp->f_op && fp->f_op->write){
                fp->f_op->write(fp, buf, length, &fp->f_pos);
            }else{
                 printk("fp->f_op && fp->f_op->write err\n");
                 goto err;
            }
        }
        filp_close(fp,NULL);
    }else
    {
            printk("!IS_ERR_OR_NULL(fp) err\n");
            goto err;
    }
     
    set_fs(old_fs);
    printk("do_system_data...end sucess\n");
    return 0;
err:
    filp_close(fp,NULL);
    //set user domain again
    set_fs(old_fs);
    printk("do_system_data...end err\n");
    return -ENOSYS;
}

long custom_arm_syscall(struct pt_regs *regs,unsigned int num)
{
       printk("dixiaobing compat_arm64_syscall num %d\n",num);
       switch (num){
               case __NR_do_system_data:
			   printk("dixiaobing compat_arm64_syscall __NR_do_system_data %d\n",__NR_do_system_data);
               return do_system_data(regs->regs[0],(__user char *)regs->regs[1],regs->regs[2]);
               case __NR_set_machine_info:
                       printk("compat_arm64_syscall __NR_et_machine_info %d\n",__NR_set_machine_info);
               return set_machine_info(regs->regs[0],regs->regs[1],(__user uint8_t *)regs->regs[2],(__user uint8_t *)regs->regs[3]);
       }
      
       return 0;
}

/*
 * Handle all unrecognised system calls.
 */
long compat_arm64_syscall(struct pt_regs *regs)
{
	unsigned int no = regs->regs[8];

        printk("dixiaobing compat_arm64_syscall no %d\n",no);
        return custom_arm_syscall(regs,no);
}
#endif
/*dixiaobing@wind-mobi.com 20150625 end*/

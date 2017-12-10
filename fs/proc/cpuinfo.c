#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/errno.h>
#include <asm/io.h>
#include <asm/uaccess.h>


extern const struct seq_operations cpuinfo_op;
extern char *saved_command_line;
/**modify yudengwu2015-01-06**/
#define M_CUSTOM_REQUITEMENT
#ifdef M_CUSTOM_REQUITEMENT
char hw_version[32] = "";
char mach_type[32] = "";

static int hwinfo_show(struct seq_file *m, void *v)
{
	printk("[darren] ---hwinfo show\n");
        char hw_type[32] = {0};
        char *str = strstr(saved_command_line, " androidboot.hw_version=");
        if(str != NULL){
            int i = 0;
            str += 24;
            if(strstr(str, ":") != NULL){
                while(str[i] != ' '){
                    hw_type[i] = str[i];
                    i ++;
                }
            }
        }
        if(hw_type[0] != 0 && hw_type[0] != ' '){
                int i, j = 0, len = strlen(hw_type);
                for(i = 0; i < len; i ++){
                    if(hw_type[i] != ':'){
                        hw_version[i] = hw_type[i];
                    }else{
                        break;
                    }
                }
                i++;
                for(; i < len; i ++){
                    if(hw_type[i] != ' ' && hw_type[i] != 0){
                        mach_type[j++] = hw_type[i];
                    }else{
                        break;
                    }
                }
        }
	seq_printf(m, "Hardware Version: %s\nMachine Type    : %s", hw_version, mach_type);
    return 0;
}
/****modify yudengwu@wind-mobi.com end ****/
#endif
/**modify yudengwu2015-01-06**/
static int cpuinfo_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &cpuinfo_op);
}

static const struct file_operations proc_cpuinfo_operations = {
	.open		= cpuinfo_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

/**modify yudengwu2015-01-06**/
#ifdef M_CUSTOM_REQUITEMENT
static int hwinfo_open(struct inode *inode, struct file *file)
{								
    return single_open(file, hwinfo_show, PDE_DATA(inode));
}
   
static const struct file_operations proc_hwinfo = {
    .owner          = THIS_MODULE,		
    .open           = hwinfo_open,		
    .read           = seq_read,			
    .llseek         = seq_lseek,			
    .release        = single_release, 			
};
#endif

/**modify yudengwu2015-01-06**/

static int __init proc_cpuinfo_init(void)
{
	proc_create("cpuinfo", 0, NULL, &proc_cpuinfo_operations);

/**modify yudengwu2015-01-06**/
#ifdef M_CUSTOM_REQUITEMENT
     if (!proc_create("hwinfo", 0666, NULL, &proc_hwinfo))  
     {
            printk("%s(), create /proc/%s failed\n", __func__,"hwinfo");
     }
#endif
/**modify yudengwu2015-01-06**/
	
	return 0;
}
module_init(proc_cpuinfo_init);

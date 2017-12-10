/*
 * Default policy configuration.
 * Author: <shaoxiang.qsx@alibaba-inc.com>
 */

#ifdef __KERNEL__
#include "inc/stdinc.h"
#else /* __KERNEL__ */
#include "inc/simu.h"
#endif /* __KERNEL__ */

#if 0

#define AID_ROOT             0  /* traditional unix root user */
#define AID_SYSTEM        1000  /* system server */

#define AID_RADIO         1001  /* telephony subsystem, RIL */
#define AID_BLUETOOTH     1002  /* bluetooth subsystem */
#define AID_GRAPHICS      1003  /* graphics devices */
#define AID_INPUT         1004  /* input devices */
#define AID_AUDIO         1005  /* audio devices */
#define AID_CAMERA        1006  /* camera devices */
#define AID_LOG           1007  /* log devices */
#define AID_COMPASS       1008  /* compass device */
#define AID_MOUNT         1009  /* mountd socket */
#define AID_WIFI          1010  /* wifi subsystem */
#define AID_ADB           1011  /* android debug bridge (adbd) */
#define AID_INSTALL       1012  /* group for installing packages */
#define AID_MEDIA         1013  /* mediaserver process */
#define AID_DHCP          1014  /* dhcp client */
#define AID_SDCARD_RW     1015  /* external storage write access */
#define AID_VPN           1016  /* vpn system */
#define AID_KEYSTORE      1017  /* keystore subsystem */
#define AID_USB           1018  /* USB devices */
#define AID_DRM           1019  /* DRM server */
#define AID_MDNSR         1020  /* MulticastDNSResponder (service discovery) */
#define AID_GPS           1021  /* GPS daemon */
#define AID_UNUSED1       1022  /* deprecated, DO NOT USE */
#define AID_MEDIA_RW      1023  /* internal media storage write access */
#define AID_MTP           1024  /* MTP USB driver access */
#define AID_UNUSED2       1025  /* deprecated, DO NOT USE */
#define AID_DRMRPC        1026  /* group for drm rpc */
#define AID_NFC           1027  /* nfc subsystem */
#define AID_SDCARD_R      1028  /* external storage read access */

#define AID_SHELL         2000  /* adb and debug shell user */
#define AID_CACHE         2001  /* cache access */
#define AID_DIAG          2002  /* access to diagnostic resources */

/* The 3000 series are intended for use as supplemental group id's only.
 * They indicate special Android capabilities that the kernel is aware of. */
#define AID_NET_BT_ADMIN  3001  /* bluetooth: create any socket */
#define AID_NET_BT        3002  /* bluetooth: create sco, rfcomm or l2cap sockets */
#define AID_INET          3003  /* can create AF_INET and AF_INET6 sockets */
#define AID_NET_RAW       3004  /* can create raw INET sockets */
#define AID_NET_ADMIN     3005  /* can configure interfaces and routing tables. */
#define AID_NET_BW_STATS  3006  /* read bandwidth statistics */
#define AID_NET_BW_ACCT   3007  /* change bandwidth statistics accounting */
#define AID_NET_BT_STACK  3008  /* bluetooth: access config files */

#define AID_CCCI          9996
#define AID_NVRAM         9997
#define AID_MISC          9998  /* access to misc storage */
#define AID_NOBODY        9999

#define AID_APP          10000  /* first app user */

#define AID_ISOLATED_START 99000 /* start of uids for fully isolated sandboxed processes */
#define AID_ISOLATED_END   99999 /* end of uids for fully isolated sandboxed processes */

#define AID_USER        100000  /* offset for uid ranges for each user */

#define AID_SHARED_GID_START 50000 /* start of gids for apps in each user to share */
#define AID_SHARED_GID_END   59999 /* start of gids for apps in each user to share */


#define QPERM_VISIBLE 0x1
#define QPERM_OPEN 0x2
#define QPERM_READ 0x4
#define QPERM_WRITE 0x8
#define QPERM_EXEC 0x10
#define QPERM_CHOWN 0x20
#define QPERM_CREATE 0x40
#define QPERM_DESTROY 0x80
#define QPERM_MOUNT 0x100
#define QPERM_MASK 0x1ff

#endif

/*
 * type name
 * role name
 * user name
 * install type1 {/data/data, /data/tmp};
 * coronate user1 {role1,role2};
 * allow role1 type1 {read,write,create,delete};
 * deny role2 {type2,type3} {read,write,create,delete};
 * alter user1 [to] user2 [when user1 run] type1;
 * convert type1 [to] type2 [when] user1 [new type1];
 * trans user1 uid(linux) user2;
 */
 const char* android_dcfg[] = {
	 "install t.def {,/,};",
	 "install t.sys {,/sbin/ueventd,/system/bin/drvbd,/sbin/healthd,/system/bin/servicemanager,};",
		 "install t.sys {,/system/bin/resize_ext4,/system/bin/e2fsck,/system/bin/tune2fs,};",
		 "install t.sys {,/system/bin/resize2fs,};",
		 "install t.sys {,/system/bin/mcDriverDaemon,};",
		 "install t.sys {,/system/bin/webserver,/system/bin/logwrapper,};",
		 "install t.sys {,/system/bin/ccci_fsd,/system/bin/ccci_mdinit,};",
		 "install t.sys {,/system/bin/debuggerd,/system/bin/mobile_log_d,};",
		 "install t.sys {,/system/bin/6620_launcher,/system/bin/netd,/system/bin/netdiag,};",
		 "install t.sys {,/system/bin/surfaceflinger,/system/bin/aal,};",
		 "install t.sys {,/system/bin/drmserver,/system/bin/mediaserver,};",
		 "install t.sys {,/system/bin/bwc,/system/bin/matv,};",
		 "install t.sys {,/system/bin/secd,/system/bin/installd,};",
		 "install t.sys {,/system/bin/keystore,/system/bin/mtk_agpsd,};",
		 "install t.sys {,/system/bin/batterywarning,/system/bin/MtkCodecService,};",
		 "install t.sys {,/system/bin/memsicd3416x,/system/bin/dm_agent_binder,};",
		 "install t.sys {,/system/bin/ppl_agent,/system/bin/mtkbt,};",
		 "install t.sys {,/system/bin/nvram_agent_binder,/system/bin/thermal,};",
		 "install t.sys {,/system/bin/thermald,/system/bin/em_svr,};",
		 "install t.sys {,/system/bin/mdlogger,/system/bin/gsm0710muxd,};",
		 "install t.sys {,/system/bin/rild,/system/bin/sdcard,};",
		 "install t.sys {,/system/bin/debuggerd64,};",
		 "install t.sys {,/system/bin/aee_dumpstate,};",
		 "install t.sys {,/system/bin/aee_archive,};",
		 "install t.sys {,/system/bin/aee_core_forwarder,};",
	 "install t.sys.config {,/init.rc,};", /* read only system data */
		 "install t.sys.config {,/system/build.prop,};",
		 "install t.sys.config {,/system/media,/system/auitheme,/system/fonts,/system/usr,};",
	 "install t.sys.data {,/data/system,};", /* system data, can be write */
	 "install t.sys.public {,/system/usr/share,};",
	 "install t.sys.symbol {,/proc/kallsyms,/proc/sys/kernel/kptr_restrict,};", /* hide system data, can not read */
	 "install t.sys.app {,/system/app,/system/priv-app,/system/framework,};", /* system app, can be exec by app */
	 "install t.sys.app {,/system/bin/dex2oat,};",
	 "install t.sys.cache {,/data/dalvik-cache,};", /* system app-cache, can be exec by app */
	 "install t.sys.bin {,/init,/sbin,/vendor,/etc/ppp,};", /* system tools, can be exec by system */

	 "install t.tmp {,/data/local,/proc/alog,};",
	 "install t.inst {,/system/bin/installd,};", /* installer, */
	 "install t.vold {,/system/bin/vold,};",

	 "install t.adb {,/sbin/adbd,};",
	 "install t.adb.data {,/data/misc/adb,};", /* adb data */

	 "install t.app {,/system/bin/app_process,/system/bin/service,};",
	 "install t.app {,/system/bin/app_process64,/system/bin/app_process32,};",
	 "install t.app.data {,/data/data,/data/ecryptfs_dir/data,};",
	 "install t.app.lib {,/data/app-lib,};",
	 "install t.app.bin {,/system/bin,/system/xbin,};",
		 "install t.app.bin {,/system/xbin/procmem,};",
		 "install t.app.bin {,/system/xbin/procrank,};",
		 "install t.app.bin {,/system/xbin/librank,};",
		 "install t.app.bin {,/system/bin/netcfg,};",
		 "install t.app.bin {,/system/bin/ping,};"
		 "sbit /system/xbin/procmem;",
		 "sbit /system/xbin/procrank;",
		 "sbit /system/xbin/librank;",
		 "sbit /system/bin/netcfg;",
		 "sbit /system/bin/ping;",

	 "install t.lib {,/system/lib,system/lib64,/system/vendor/lib,};",
	 "install t.dev.w {,/dev/uinput,/dev/pts,/proc/driver/wmt_dbg,/proc/driver/wmt_aee,};", /* device write */
		 "install t.dev.w {,/dev/block/vold,};",
	 "install t.dev {,/dev,/proc,/sys,/dev/pts,/sys/kernel/debug,};",
	 "install t.etc {,/etc,};",
	 "install t.sdcard {,/sdcard,/mnt/sdcard,/storage/emulated/legacy,};",
	 "install t.debug {,/system/bin/aee_aed,};",

	 "install t.uuid {,/data/data/com.aliyun.uuid,};",
	 "install t.secd {,/system/bin/secd,};",
	 "install t.secd.data {,/data/secd,/data/ecryptfs_dir,};",
	 "install t.mnt {,/mnt,/data/media,/data/system/scsi,};",

	 "install t.sys.public {,/data,};", /* must be last */
	 "install t.sys.bin {,/system,};",

	 "coronate u.def r.def;",
	 "coronate u.sys r.sys;",
	 "coronate u.app r.app;",
	 "coronate u.sapp r.app;",
	 "coronate u.adb r.adb;",
	 "coronate u.secd r.secd;",
	 "coronate u.inst {,r.inst,r.sys,};",
	 "coronate u.uuid r.sys;",
	 "coronate u.blt r.app;",
	 "coronate u.debug r.sys;",
	 "coronate u.vold r.sys;",

	 "allow r.def * {,*,};",
	 "deny r.def {,t.app.data,t.sdcard,t.tmp,} {,newuser,};",
	 "deny r.sys {,t.app.data,t.sdcard,t.tmp,} {,newuser,};",
	 "allow * {,t.def,} {,*,};",
	 "allow r.inst {,t.app.data,t.uuid,} {,*,-exec,};",

	 "allow r.sys {,t.debug,t.sys,t.sys.app,t.lib,t.sys.bin,t.app.bin,t.inst,t.adb,t.app,t.secd,} {,open,read,exec,};",
		"allow r.sys {,t.sys.config,t.sys.symbol,} {,open,read,};",
	 "allow r.sys {,t.sys.bin,} {,mount,};",
	 "allow r.sys {,t.secd.data,t.app.data,t.sys.data,t.sys.cache,t.adb.data,} {,*,-mount,};",
		 "allow r.sys {,t.sys.public,t.app.lib,t.mnt,t.dev.w,t.uuid,} {,*,-mount,};",
	 "allow r.sys {,t.tmp,} {,*,-mount,-exec,};",
	 "allow r.sys {,t.dev,} {,*,-mount,-write,};",
		 "allow r.sys {,t.etc,} {,*,-mount,-write,};",
	 "allow r.sys {,t.sdcard,} {,*,-exec,};",

	 "allow r.app t.app.data {,*,-mount,};",
		 "allow r.app {,t.debug,t.app,t.lib,} {,open,read,exec,};",
		 "allow r.app {,t.sys.config,t.sys.symbol,} {,open,read,};",
		 "allow r.app {,t.sys.app,} {,open,read,exec,};",
		 "allow r.app {,t.app.bin,} {,open,read,exec,-write,};",
		 "allow r.app {,t.sys.cache,} {,*,-mount,};",
		 "allow r.app {,t.sys.data,} {,open,read,delete,};",
		 "allow r.app {,t.dev,t.dev.w,} {,exec,};",
		 "allow r.app {,t.sys.public,t.tmp,t.mnt,} {,*,-exec,-mount,};",
		 "allow r.app {,t.app.lib,} {,*,-mount,};",
		 "allow r.app {,t.mnt,} {,*,-mount,};",

	 "allow r.adb t.app.data {,open,read,};",
		 "allow r.adb {,t.debug,t.app.bin,t.lib,t.inst,t.adb,t.app,} {,open,read,exec,};",
		 "allow r.adb {,t.adb.data,} {,open,read,exec,};",
		 "allow r.adb {,t.tmp,} {,*,-mount,};",
		 "allow r.adb {,t.sys.public,} {,open,read,};",
		 "allow r.adb {,t.mnt,} {,open,read,delete,};",

	 "allow r.secd * {,*,};",
	 "allow * t.dev {,open,read,write,};",
	 "allow * t.dev.w {,open,read,write,chmod,};",
	 "allow * t.etc {,open,read,};",
	 "allow * t.sdcard {,*,-exec,-mount,};",

	 "deny * {,*,} {,ROOT,};",
	 "allow {,*,} {,t.vold,t.def,t.debug,t.sys,t.sys.app,t.lib,t.sys.bin,t.app.bin,t.inst,t.adb,t.app,t.secd,} {,ROOT,};",

	 "auth u.vold {,t.mnt, t.sdcard,} {,chown,mount,};",
	 "auth u.sys {,t.sdcard,} {,mount,};",
	 "auth u.sys {,t.vold,} {,open,read,exec,};",
	 "auth u.sapp {,t.sys.data,} {,*,-mount,};",
	 "auth u.sapp {,t.adb.data,} {,*,-mount,};",
	 "auth u.sapp {,t.sys.bin,} {,open,read,exec,};",
	 "auth u.sapp {,t.uuid,} {,*,-mount,};",
	 "auth u.sapp {,t.dev,} {,*,};",
	 "auth u.app {,t.sys.bin,} {,open,read,};",
	 "auth u.debug {,t.sys.symbol,} {,write,};",
	 "auth {,u.uuid,} t.uuid {,*,};",
	 "auth {,u.app,u.sapp,} t.uuid {,open,read,};",
	 "auth u.blt {,t.sys.data,t.sys.public,} {,*,-mount,-exec,};",
		 "auth u.blt {,t.sys.config,} {,open,read,};",
		 "auth u.blt {,t.sys,t.sys.bin,} {,open,read,exec,};",

	 "trans.exec * u:-1 {,t.adb,} u.adb;",
	 "trans.exec * u:-1 {,t.debug,} u.debug;",
	 "trans.exec * u:-1 {,t.secd,} u.secd;",
	 "trans.exec * u:-1 {,t.inst,} u.inst;",
	 "trans.exec {,*,} u:-1 {,t.sys,} u.sys;",
	 "trans.exec {,*,} u:1000 {,t.app,} u.sapp;",
	 "trans.exec {,*,} u:-1 {,t.app,} u.app;",
	 "trans.exec {,*,} u:-1 {,t.vold,} u.vold;",

	 "trans.chuid {,*,} {,u:1000,} {,t.app,} u.sapp;",
	 "trans.chuid {,*,} {,u:1000,} {,*,-t.app,} u.sys;",
	 "trans.chuid u.def {,u:1011,} {,*,-t.def,} u.adb;",
	 "trans.chuid u.def {,u:1012,} {,*,-t.def,} u.inst;",
	 "trans.chuid {,*,} {,u:1002,} {,*,-t.def,} u.blt;",
	 "trans.chuid {,*,} {,u:1040,} {,*,-t.def,} u.secd;",
	 NULL
 };

const struct qconfig g_dcfg[] = {
	{ .rfs = "/", .cfg = android_dcfg},
	{ .rfs = NULL, .cfg = NULL}
};

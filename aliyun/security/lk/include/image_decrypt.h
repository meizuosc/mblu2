/*
 * Copyright (c) 2014, Ali YunOS All rights reserved.
 *
 *  File  : image_decrypt.h
 *  Author: zilong.liuzl
 *  Date  : 2014-08-01
 */

#ifndef __IMAGE_DECRYPT_H
#define __IMAGE_DECRYPT_H

#define AES_KEY_SIZE 16
#define CRYPT_BLOCK_SIZE 512

int init_key(void);
int mboot_yunos_decrypt_piggy(unsigned long scan_start, unsigned long decrypt_start, int kimg_sz, int crypt_sz);
int mboot_yunos_decrypt_ramdisk(unsigned long encrypt_start, unsigned long decrypt_start, int ramdisk_sz);

#endif

/*
 * Copyright (c) 2014, Ali YunOS All rights reserved.
 *
 *  File  : image_decrypt.c
 *  Author: zilong.liuzl
 *  Date  : 2014-08-01
 */

#include "image_decrypt.h"

#include <openssl/aes.h>

extern unsigned char key[AES_KEY_SIZE];
extern unsigned char iv[AES_BLOCK_SIZE];

int doing_decrypt(unsigned  char *encrypt_string, unsigned char *decrypt_string, int len)
{
    AES_KEY aes;
    int mem_ret, i;
    char aes_iv[AES_BLOCK_SIZE] = {0};

    for(i = 0; i < AES_KEY_SIZE; i++)
        key[i] ^= 0x6F;

    for(i = 0; i < AES_BLOCK_SIZE; i++)
        iv[i] ^= 0x6F;

    mem_ret = memcpy(aes_iv, iv, AES_BLOCK_SIZE);
    if (mem_ret != aes_iv)
	return -1;

    if (AES_set_decrypt_key(key, AES_KEY_SIZE * 8, &aes) < 0) {
        printf("Unable to set decryption key in AES\n");
        return -1;
    }
    // decrypt
    AES_cbc_encrypt(encrypt_string, decrypt_string, len, &aes, aes_iv, AES_DECRYPT);
    return 0;
}

int mboot_yunos_decrypt_piggy(unsigned long scan_start, unsigned long decrypt_start, int kimg_sz, int crypt_sz)
{
    int ret = 0;
    unsigned long mem_ret = 0;
    unsigned long encrypt_start = 0;
    unsigned long scan_sz = kimg_sz;
    unsigned long scan_addr = scan_start;
    unsigned long decrypt_piggy_time = get_timer(0);

    unsigned char gzip_head[10] = {0X1F, 0X8B, 0X08, 0X00, 0X00, 0X00, 0X00, 0X00, 0X02, 0X03};

    // search the head of piggy.gzip
    do {
        scan_addr = (unsigned  char*)memchr(scan_addr, gzip_head[0] , scan_sz);
        if(NULL == scan_addr) {
            break;
        }

        if(memcmp(scan_addr, gzip_head, 10) == 0) {
            encrypt_start = scan_addr + 10;
            break;
        }
        scan_sz -= (scan_addr - scan_start);
    } while(++scan_addr);

    if(NULL == scan_addr) {
        printf("ERROR: search the head of piggy.gzip error!\n");
        return -1;
    }
    // head decrypt
    ret = doing_decrypt(encrypt_start, decrypt_start,  crypt_sz);
    if(ret < 0) {
        printf("ERROR: piggy_decrypt error!\n");
        return -1;
    }

    // cp back
    mem_ret = memcpy(encrypt_start, decrypt_start, crypt_sz);
    if(mem_ret != encrypt_start) {
        printf("ERROR: cp decrypt piggy back error!\n");
        return -1;
    }

    printf("\n-------- decrypt piggy takes %d ms --------\n", get_timer(decrypt_piggy_time));
    return 0;
}

int mboot_yunos_decrypt_ramdisk(unsigned long encrypt_start, unsigned long decrypt_start, int ramdisk_sz)
{
    int ret = 0;
    int mem_ret =0;
    int crypt_sz = 0;
    int left_sz = 0;
    unsigned long left_from = 0;
    unsigned long left_to = 0;
    unsigned long decrypt_ramdisk_time = get_timer(0);

    crypt_sz = (ramdisk_sz  / CRYPT_BLOCK_SIZE) * CRYPT_BLOCK_SIZE;
    left_sz = ramdisk_sz - crypt_sz;
    left_from = encrypt_start + crypt_sz;
    left_to = decrypt_start + crypt_sz;

    // decrypt
    ret = doing_decrypt(encrypt_start, decrypt_start, crypt_sz);
    if(ret < 0) {
        printf("ERROR: doing_decrypt error!\n");
        return -1;
    }

    printf("\n-------- decrypt takes %d ms --------\n", get_timer(decrypt_ramdisk_time));

    // mv left to load-addr
    mem_ret = memcpy(left_to, left_from, left_sz);
    if(mem_ret != left_to) {
        printf("ERROR: cp left-data  back error!\n");
        return -1;
    }

    printf("\n-------- decrypt and move ramdisk takes %d ms --------\n", get_timer(decrypt_ramdisk_time));
    return 0;
}

#include "rsa_verify.h"
#include <linux/string.h>

#define rol(bits, value) (((value) << (bits)) | ((value) >> (32 - (bits))))

static void SHA1_Transform(SHA_CTX* ctx) {
    uint32_t W[80];
    uint32_t A, B, C, D, E;
    uint8_t* p = ctx->buf;
    int t;

    for(t = 0; t < 16; ++t) {
        uint32_t tmp =  *p++ << 24;
        tmp |= *p++ << 16;
        tmp |= *p++ << 8;
        tmp |= *p++;
        W[t] = tmp;
    }

    for(; t < 80; t++) {
        W[t] = rol(1,W[t-3] ^ W[t-8] ^ W[t-14] ^ W[t-16]);
    }

    A = ctx->state[0];
    B = ctx->state[1];
    C = ctx->state[2];
    D = ctx->state[3];
    E = ctx->state[4];

    for(t = 0; t < 80; t++) {
        uint32_t tmp = rol(5,A) + E + W[t];

        if (t < 20)
            tmp += (D^(B&(C^D))) + 0x5A827999;
        else if ( t < 40)
            tmp += (B^C^D) + 0x6ED9EBA1;
        else if ( t < 60)
            tmp += ((B&C)|(D&(B|C))) + 0x8F1BBCDC;
        else
            tmp += (B^C^D) + 0xCA62C1D6;

        E = D;
        D = C;
        C = rol(30,B);
        B = A;
        A = tmp;
    }

    ctx->state[0] += A;
    ctx->state[1] += B;
    ctx->state[2] += C;
    ctx->state[3] += D;
    ctx->state[4] += E;
}

static const HASH_VTAB SHA_VTAB = {
    SHA_init,
    SHA_update,
    SHA_final,
    SHA_hash,
    SHA_DIGEST_SIZE
};

void SHA_init(SHA_CTX* ctx) {
    ctx->f = &SHA_VTAB;
    ctx->state[0] = 0x67452301;
    ctx->state[1] = 0xEFCDAB89;
    ctx->state[2] = 0x98BADCFE;
    ctx->state[3] = 0x10325476;
    ctx->state[4] = 0xC3D2E1F0;
    ctx->count = 0;
}


void SHA_update(SHA_CTX* ctx, const void* data, int len) {
    int i = (int) (ctx->count & 63);
    const uint8_t* p = (const uint8_t*)data;

    ctx->count += len;

    while (len--) {
        ctx->buf[i++] = *p++;
        if (i == 64) {
            SHA1_Transform(ctx);
            i = 0;
        }
    }
}


const uint8_t* SHA_final(SHA_CTX* ctx) {
    uint8_t *p = ctx->buf;
    uint64_t cnt = ctx->count * 8;
    int i;

    SHA_update(ctx, (uint8_t*)"\x80", 1);
    while ((ctx->count & 63) != 56) {
        SHA_update(ctx, (uint8_t*)"\0", 1);
    }
    for (i = 0; i < 8; ++i) {
        uint8_t tmp = (uint8_t) (cnt >> ((7 - i) * 8));
        SHA_update(ctx, &tmp, 1);
    }

    for (i = 0; i < 5; i++) {
        uint32_t tmp = ctx->state[i];
        *p++ = tmp >> 24;
        *p++ = tmp >> 16;
        *p++ = tmp >> 8;
        *p++ = tmp >> 0;
    }

    return ctx->buf;
}

/* Convenience function */
const uint8_t* SHA_hash(const void* data, int len, uint8_t* digest) {
    SHA_CTX ctx;
    SHA_init(&ctx);
    SHA_update(&ctx, data, len);
    memcpy(digest, SHA_final(&ctx), SHA_DIGEST_SIZE);
    return digest;
}

#define ror(value, bits) (((value) >> (bits)) | ((value) << (32 - (bits))))
#define shr(value, bits) ((value) >> (bits))

static const uint32_t K[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
    0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
    0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
    0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
    0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
    0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
    0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
    0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
    0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2 };

static void SHA256_Transform(SHA256_CTX* ctx) {
    uint32_t W[64];
    uint32_t A, B, C, D, E, F, G, H;
    uint8_t* p = ctx->buf;
    int t;

    for(t = 0; t < 16; ++t) {
        uint32_t tmp =  *p++ << 24;
        tmp |= *p++ << 16;
        tmp |= *p++ << 8;
        tmp |= *p++;
        W[t] = tmp;
    }

    for(; t < 64; t++) {
        uint32_t s0 = ror(W[t-15], 7) ^ ror(W[t-15], 18) ^ shr(W[t-15], 3);
        uint32_t s1 = ror(W[t-2], 17) ^ ror(W[t-2], 19) ^ shr(W[t-2], 10);
        W[t] = W[t-16] + s0 + W[t-7] + s1;
    }

    A = ctx->state[0];
    B = ctx->state[1];
    C = ctx->state[2];
    D = ctx->state[3];
    E = ctx->state[4];
    F = ctx->state[5];
    G = ctx->state[6];
    H = ctx->state[7];

    for(t = 0; t < 64; t++) {
        uint32_t s0 = ror(A, 2) ^ ror(A, 13) ^ ror(A, 22);
        uint32_t maj = (A & B) ^ (A & C) ^ (B & C);
        uint32_t t2 = s0 + maj;
        uint32_t s1 = ror(E, 6) ^ ror(E, 11) ^ ror(E, 25);
        uint32_t ch = (E & F) ^ ((~E) & G);
        uint32_t t1 = H + s1 + ch + K[t] + W[t];

        H = G;
        G = F;
        F = E;
        E = D + t1;
        D = C;
        C = B;
        B = A;
        A = t1 + t2;
    }

    ctx->state[0] += A;
    ctx->state[1] += B;
    ctx->state[2] += C;
    ctx->state[3] += D;
    ctx->state[4] += E;
    ctx->state[5] += F;
    ctx->state[6] += G;
    ctx->state[7] += H;
}

static const HASH_VTAB SHA256_VTAB = {
    SHA256_init,
    SHA256_update,
    SHA256_final,
    SHA256_hash,
    SHA256_DIGEST_SIZE
};

void SHA256_init(SHA256_CTX* ctx) {
    ctx->f = &SHA256_VTAB;
    ctx->state[0] = 0x6a09e667;
    ctx->state[1] = 0xbb67ae85;
    ctx->state[2] = 0x3c6ef372;
    ctx->state[3] = 0xa54ff53a;
    ctx->state[4] = 0x510e527f;
    ctx->state[5] = 0x9b05688c;
    ctx->state[6] = 0x1f83d9ab;
    ctx->state[7] = 0x5be0cd19;
    ctx->count = 0;
}


void SHA256_update(SHA256_CTX* ctx, const void* data, int len) {
    int i = (int) (ctx->count & 63);
    const uint8_t* p = (const uint8_t*)data;

    ctx->count += len;

    while (len--) {
        ctx->buf[i++] = *p++;
        if (i == 64) {
            SHA256_Transform(ctx);
            i = 0;
        }
    }
}


const uint8_t* SHA256_final(SHA256_CTX* ctx) {
    uint8_t *p = ctx->buf;
    uint64_t cnt = ctx->count * 8;
    int i;

    SHA256_update(ctx, (uint8_t*)"\x80", 1);
    while ((ctx->count & 63) != 56) {
        SHA256_update(ctx, (uint8_t*)"\0", 1);
    }
    for (i = 0; i < 8; ++i) {
        uint8_t tmp = (uint8_t) (cnt >> ((7 - i) * 8));
        SHA256_update(ctx, &tmp, 1);
    }

    for (i = 0; i < 8; i++) {
        uint32_t tmp = ctx->state[i];
        *p++ = tmp >> 24;
        *p++ = tmp >> 16;
        *p++ = tmp >> 8;
        *p++ = tmp >> 0;
    }

    return ctx->buf;
}

/* Convenience function */
const uint8_t* SHA256_hash(const void* data, int len, uint8_t* digest) {
    SHA256_CTX ctx;
    SHA256_init(&ctx);
    SHA256_update(&ctx, data, len);
    memcpy(digest, SHA256_final(&ctx), SHA256_DIGEST_SIZE);
    return digest;
}

// a[] -= mod
static void subM(const RSAPublicKey* key,
                 uint32_t* a) {
    int64_t A = 0;
    int i;
    for (i = 0; i < key->len; ++i) {
        A += (uint64_t)a[i] - key->n[i];
        a[i] = (uint32_t)A;
        A >>= 32;
    }
}

// return a[] >= mod
static int geM(const RSAPublicKey* key,
               const uint32_t* a) {
    int i;
    for (i = key->len; i;) {
        --i;
        if (a[i] < key->n[i]) return 0;
        if (a[i] > key->n[i]) return 1;
    }
    return 1;  // equal
}

// montgomery c[] += a * b[] / R % mod
static void montMulAdd(const RSAPublicKey* key,
                       uint32_t* c,
                       const uint32_t a,
                       const uint32_t* b) {
    uint64_t A = (uint64_t)a * b[0] + c[0];
    uint32_t d0 = (uint32_t)A * key->n0inv;
    uint64_t B = (uint64_t)d0 * key->n[0] + (uint32_t)A;
    int i;

    for (i = 1; i < key->len; ++i) {
        A = (A >> 32) + (uint64_t)a * b[i] + c[i];
        B = (B >> 32) + (uint64_t)d0 * key->n[i] + (uint32_t)A;
        c[i - 1] = (uint32_t)B;
    }

    A = (A >> 32) + (B >> 32);

    c[i - 1] = (uint32_t)A;

    if (A >> 32) {
        subM(key, c);
    }
}

// montgomery c[] = a[] * b[] / R % mod
static void montMul(const RSAPublicKey* key,
                    uint32_t* c,
                    const uint32_t* a,
                    const uint32_t* b) {
    int i;
    for (i = 0; i < key->len; ++i) {
        c[i] = 0;
    }
    for (i = 0; i < key->len; ++i) {
        montMulAdd(key, c, a[i], b);
    }
}

// In-place public exponentiation.
// Input and output big-endian byte array in inout.
static void modpow(const RSAPublicKey* key,
                   uint8_t* inout) {
    uint32_t a[RSANUMWORDS];
    uint32_t aR[RSANUMWORDS];
    uint32_t aaR[RSANUMWORDS];
    uint32_t* aaa = 0;
    int i;

    // Convert from big endian byte array to little endian word array.
    for (i = 0; i < key->len; ++i) {
        uint32_t tmp =
            (inout[((key->len - 1 - i) * 4) + 0] << 24) |
            (inout[((key->len - 1 - i) * 4) + 1] << 16) |
            (inout[((key->len - 1 - i) * 4) + 2] << 8) |
            (inout[((key->len - 1 - i) * 4) + 3] << 0);
        a[i] = tmp;
    }

    if (key->exponent == 65537) {
        aaa = aaR;  // Re-use location.
        montMul(key, aR, a, key->rr);  // aR = a * RR / R mod M
        for (i = 0; i < 16; i += 2) {
            montMul(key, aaR, aR, aR);  // aaR = aR * aR / R mod M
            montMul(key, aR, aaR, aaR);  // aR = aaR * aaR / R mod M
        }
        montMul(key, aaa, aR, a);  // aaa = aR * a / R mod M
    } else if (key->exponent == 3) {
        aaa = aR;  // Re-use location.
        montMul(key, aR, a, key->rr);  /* aR = a * RR / R mod M   */
        montMul(key, aaR, aR, aR);     /* aaR = aR * aR / R mod M */
        montMul(key, aaa, aaR, a);     /* aaa = aaR * a / R mod M */
    }

    // Make sure aaa < mod; aaa is at most 1x mod too large.
    if (geM(key, aaa)) {
        subM(key, aaa);
    }

    // Convert to bigendian byte array
    for (i = key->len - 1; i >= 0; --i) {
        uint32_t tmp = aaa[i];
        *inout++ = tmp >> 24;
        *inout++ = tmp >> 16;
        *inout++ = tmp >> 8;
        *inout++ = tmp >> 0;
    }
}

// Expected PKCS1.5 signature padding bytes, for a keytool RSA signature.
// Has the 0-length optional parameter encoded in the ASN1 (as opposed to the
// other flavor which omits the optional parameter entirely). This code does not
// accept signatures without the optional parameter.

/*
static const uint8_t sha_padding[RSANUMBYTES] = {
    0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0x00, 0x30, 0x21, 0x30,
    0x09, 0x06, 0x05, 0x2b, 0x0e, 0x03, 0x02, 0x1a,
    0x05, 0x00, 0x04, 0x14,

    // 20 bytes of hash go here.
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0
};
*/

// SHA-1 of PKCS1.5 signature sha_padding for 2048 bit, as above.
// At the location of the bytes of the hash all 00 are hashed.
static const uint8_t kExpectedPadShaRsa2048[SHA_DIGEST_SIZE] = {
    0xdc, 0xbd, 0xbe, 0x42, 0xd5, 0xf5, 0xa7, 0x2e,
    0x6e, 0xfc, 0xf5, 0x5d, 0xaf, 0x9d, 0xea, 0x68,
    0x7c, 0xfb, 0xf1, 0x67
};

/*
static const uint8_t sha256_padding[RSANUMBYTES] = {
    0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0x00, 0x30, 0x31, 0x30,
    0x0d, 0x06, 0x09, 0x60, 0x86, 0x48, 0x01, 0x65,
    0x03, 0x04, 0x02, 0x01, 0x05, 0x00, 0x04, 0x20,

    // 32 bytes of hash go here.
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
};
*/

// SHA-256 of PKCS1.5 signature sha256_padding for 2048 bit, as above.
// At the location of the bytes of the hash all 00 are hashed.
static const uint8_t kExpectedPadSha256Rsa2048[SHA256_DIGEST_SIZE] = {
    0xab, 0x28, 0x8d, 0x8a, 0xd7, 0xd9, 0x59, 0x92,
    0xba, 0xcc, 0xf8, 0x67, 0x20, 0xe1, 0x15, 0x2e,
    0x39, 0x8d, 0x80, 0x36, 0xd6, 0x6f, 0xf0, 0xfd,
    0x90, 0xe8, 0x7d, 0x8b, 0xe1, 0x7c, 0x87, 0x59,
};

// Verify a 2048-bit RSA PKCS1.5 signature against an expected hash.
// Both e=3 and e=65537 are supported.  hash_len may be
// SHA_DIGEST_SIZE (== 20) to indicate a SHA-1 hash, or
// SHA256_DIGEST_SIZE (== 32) to indicate a SHA-256 hash.  No other
// values are supported.
//
// Returns 1 on successful verification, 0 on failure.
int RSA_verify(const RSAPublicKey *key,
               const uint8_t *signature,
               const int len,
               const uint8_t *hash,
               const int hash_len) {
    uint8_t buf[RSANUMBYTES];
    int i;
    const uint8_t* padding_hash;

    if (key->len != RSANUMWORDS) {
        return 0;  // Wrong key passed in.
    }

    if (len != sizeof(buf)) {
        return 0;  // Wrong input length.
    }

    if (hash_len != SHA_DIGEST_SIZE &&
        hash_len != SHA256_DIGEST_SIZE) {
        return 0;  // Unsupported hash.
    }

    if (key->exponent != 3 && key->exponent != 65537) {
        return 0;  // Unsupported exponent.
    }

    for (i = 0; i < len; ++i) {  // Copy input to local workspace.
        buf[i] = signature[i];
    }

    modpow(key, buf);  // In-place exponentiation.

    // Xor sha portion, so it all becomes 00 iff equal.
    for (i = len - hash_len; i < len; ++i) {
        buf[i] ^= *hash++;
    }

    // Hash resulting buf, in-place.
    switch (hash_len) {
        case SHA_DIGEST_SIZE:
            padding_hash = kExpectedPadShaRsa2048;
            SHA_hash(buf, len, buf);
            break;
        case SHA256_DIGEST_SIZE:
            padding_hash = kExpectedPadSha256Rsa2048;
            SHA256_hash(buf, len, buf);
            break;
        default:
            return 0;
    }

    // Compare against expected hash value.
    for (i = 0; i < hash_len; ++i) {
        if (buf[i] != padding_hash[i]) {
            return 0;
        }
    }

    return 1;  // All checked out OK.
}

int verify_sig(uint8_t *rsa_data, uint8_t *rand_data){
    int i, flag = 0;
    RSAPublicKey struct_key;
    RSAPublicKey* key = (RSAPublicKey*)&struct_key;//(RSAPublicKey*)malloc(sizeof(RSAPublicKey));
    key->exponent = 3;
    key->len = 64;
    key->n0inv = 0x844d3829;
    for(i = 0; i < 64; i ++){
        key->n[i] = pub_key_n[i];
        key->rr[i] = pub_key_rr[i];
    }
    SHA_CTX sha1_ctx;
    SHA_init(&sha1_ctx);
    SHA_update(&sha1_ctx, rand_data, 20);
    const uint8_t* sha1 = SHA_final(&sha1_ctx);
    if (!RSA_verify(key, rsa_data, RSANUMBYTES,
                    sha1, SHA_DIGEST_SIZE)) {
        flag = -1;
    }
    return flag;
}

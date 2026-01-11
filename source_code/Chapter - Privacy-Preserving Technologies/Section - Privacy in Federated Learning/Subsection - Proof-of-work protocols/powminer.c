/* pow_miner.c -- compile: gcc -O2 pow_miner.c -lmbedcrypto -o pow_miner */
#include 
#include 
#include 
#include 
#include 
#include "mbedtls/sha256.h"

static int leading_zero_bits(const uint8_t h[32]) {
    int lz = 0;
    for (int i = 0; i < 32; ++i) {
        if (h[i] == 0) { lz += 8; continue; }
        unsigned int b = h[i];
        lz += __builtin_clz(b) - 24; // count leading zeros in byte
        break;
    }
    return lz;
}

int main(int argc, char **argv) {
    const int difficulty = (argc>1)?atoi(argv[1]):20; // required leading zero bits
    uint8_t header[80] = {0}; // application-specific header (e.g., block header)
    uint8_t hash1[32], hash2[32];
    uint64_t nonce = (uint64_t)time(NULL);
    mbedtls_sha256_context ctx;

    printf("Starting miner: difficulty=%d bits\n", difficulty);
    while (1) {
        // assemble header + nonce (little-endian)
        memcpy(header+72, &nonce, sizeof(nonce)); // place nonce near end
        // double SHA-256 hash
        mbedtls_sha256_init(&ctx);
        mbedtls_sha256_starts_ret(&ctx, 0);
        mbedtls_sha256_update_ret(&ctx, header, sizeof(header));
        mbedtls_sha256_finish_ret(&ctx, hash1);
        mbedtls_sha256_free(&ctx);

        mbedtls_sha256_init(&ctx);
        mbedtls_sha256_starts_ret(&ctx, 0);
        mbedtls_sha256_update_ret(&ctx, hash1, sizeof(hash1));
        mbedtls_sha256_finish_ret(&ctx, hash2);
        mbedtls_sha256_free(&ctx);

        if (leading_zero_bits(hash2) >= difficulty) {
            // success: submit header+nonce to local gateway or fog service
            printf("Found nonce=%" PRIu64 " lz=%d\n", nonce, leading_zero_bits(hash2));
            // TODO: use libcurl or lwIP socket to POST proof to fog node
            break;
        }
        ++nonce;
        // optional sleep/yield: on RTOS call taskYIELD or on Linux nanosleep to reduce power
    }
    return 0;
}
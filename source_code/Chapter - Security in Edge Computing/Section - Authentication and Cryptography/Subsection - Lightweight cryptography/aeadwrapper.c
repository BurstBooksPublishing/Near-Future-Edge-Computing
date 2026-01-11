#include 
#include 

/* Select backends via build flags: USE_MBEDTLS, USE_LIBSODIUM */
#ifdef USE_MBEDTLS
#include "mbedtls/gcm.h"
#endif
#ifdef USE_LIBSODIUM
#include 
#endif

/* AEAD encrypt: returns 0 on success, non-zero on failure */
int edge_aead_encrypt(const uint8_t *key, size_t key_len,
                      const uint8_t *nonce, size_t nonce_len,
                      const uint8_t *ad, size_t ad_len,
                      const uint8_t *pt, size_t pt_len,
                      uint8_t *ct, uint8_t *tag, size_t tag_len)
{
#ifdef USE_MBEDTLS
    mbedtls_gcm_context ctx;
    mbedtls_gcm_init(&ctx);
    /* AES-128/256 selection based on key_len */
    if (mbedtls_gcm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, key, (int)(8*key_len)) != 0) {
        mbedtls_gcm_free(&ctx);
        return -1;
    }
    if (mbedtls_gcm_crypt_and_tag(&ctx, MBEDTLS_GCM_ENCRYPT, pt_len,
                                  nonce, nonce_len, ad, ad_len,
                                  pt, ct, tag_len, tag) != 0) {
        mbedtls_gcm_free(&ctx);
        return -2;
    }
    mbedtls_gcm_free(&ctx);
    return 0;
#elif defined(USE_LIBSODIUM)
    /* libsodium expects 32-byte key and 12-byte nonce for IETF variant */
    if (key_len != crypto_aead_chacha20poly1305_ietf_KEYBYTES ||
        nonce_len != crypto_aead_chacha20poly1305_ietf_NPUBBYTES) {
        return -3;
    }
    unsigned long long clen;
    if (crypto_aead_chacha20poly1305_ietf_encrypt(ct, &clen,
            pt, pt_len, ad, ad_len, NULL, nonce, key) != 0) {
        return -4;
    }
    /* libsodium appends tag to ciphertext; split if callers expect separate tag */
    if (tag_len <= crypto_aead_chacha20poly1305_ietf_ABYTES) {
        memcpy(tag, ct + pt_len, tag_len);
    }
    return 0;
#else
    /* No crypto backend available */
    (void)key; (void)key_len; (void)nonce; (void)nonce_len;
    (void)ad; (void)ad_len; (void)pt; (void)pt_len; (void)ct; (void)tag; (void)tag_len;
    return -10;
#endif
}
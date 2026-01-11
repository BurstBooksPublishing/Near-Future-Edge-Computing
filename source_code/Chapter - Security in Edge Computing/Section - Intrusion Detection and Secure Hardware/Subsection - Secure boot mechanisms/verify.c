#include "mbedtls/pk.h"
#include "mbedtls/sha256.h"
#include 
#include 

// img: pointer to firmware in flash; img_len: firmware size
// sig: pointer to signature blob; sig_len: signature length
// pubkey_pem: null-terminated PEM public key stored in secure storage
int verify_firmware(const unsigned char *img, size_t img_len,
                    const unsigned char *sig, size_t sig_len,
                    const unsigned char *pubkey_pem)
{
    int ret = -1;
    mbedtls_pk_context pk;
    unsigned char hash[32];

    mbedtls_pk_init(&pk);

    // Parse public key (stored provisioned in secure element or ROM)
    if ((ret = mbedtls_pk_parse_public_key(&pk, pubkey_pem, strlen((const char*)pubkey_pem)+1)) != 0)
        goto cleanup;

    // Compute SHA-256 over firmware image (streaming read recommended for large images)
    if ((ret = mbedtls_sha256_ret(img, img_len, hash, 0)) != 0)
        goto cleanup;

    // Verify signature; allow only ECDSA over secp256r1
    if (!mbedtls_pk_can_do(&pk, MBEDTLS_PK_ECKEY)) {
        ret = -2; // unsupported key type
        goto cleanup;
    }

    // PK verify returns 0 on success
    ret = mbedtls_pk_verify(&pk, MBEDTLS_MD_SHA256, hash, sizeof(hash), sig, sig_len);

cleanup:
    mbedtls_pk_free(&pk);
    return ret;
}
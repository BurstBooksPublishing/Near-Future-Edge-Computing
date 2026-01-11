#include 
#include 
#include 

// AES-128 PRF: out = AES_k(nonce)
// key_len and label_len are 16 for AES-128.
static void aes_prf(const uint8_t key[16], const uint8_t nonce[16],
                    uint8_t out[16]) {
    EVP_CIPHER_CTX *ctx = EVP_CIPHER_CTX_new();
    int outl;
    EVP_EncryptInit_ex(ctx, EVP_aes_128_ecb(), NULL, key, NULL);
    EVP_CIPHER_CTX_set_padding(ctx, 0);
    EVP_EncryptUpdate(ctx, out, &outl, nonce, 16);
    EVP_EncryptFinal_ex(ctx, out + outl, &outl);
    EVP_CIPHER_CTX_free(ctx);
}

// Garble 1-bit AND. label_len == 16.
void garble_and(const uint8_t La0[16], const uint8_t La1[16],
                const uint8_t Lb0[16], const uint8_t Lb1[16],
                const uint8_t Lc0[16], const uint8_t Lc1[16],
                uint8_t table[2][16]) {
    uint8_t nonce[16];
    uint8_t tmp[16];

    // Row 0: (a=0,b=0) -> c = 0 -> encrypt Lc0 under La0||Lb0
    memcpy(nonce, La0, 16);                            // derive nonce from labels
    for (int i = 0; i < 16; ++i) nonce[i] ^= Lb0[i];
    aes_prf(La0, nonce, tmp);                          // PRF(La0, La0^Lb0)
    for (int i = 0; i < 16; ++i) table[0][i] = tmp[i] ^ Lc0[i];

    // Row 1: (a=1,b=1) -> c = 1 -> encrypt Lc1 under La1||Lb1
    memcpy(nonce, La1, 16);
    for (int i = 0; i < 16; ++i) nonce[i] ^= Lb1[i];
    aes_prf(La1, nonce, tmp);
    for (int i = 0; i < 16; ++i) table[1][i] = tmp[i] ^ Lc1[i];
    // Note: half-gates and row permutation are omitted for clarity.
}
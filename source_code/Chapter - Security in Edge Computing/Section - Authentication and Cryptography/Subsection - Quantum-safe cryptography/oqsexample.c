#include 
#include 
#include 
#include 
#include 

static double timediff_ms(struct timespec a, struct timespec b) {
    return (a.tv_sec - b.tv_sec) * 1000.0 + (a.tv_nsec - b.tv_nsec) / 1e6;
}

int main(void) {
    if (OQS_SUCCESS != OQS_init()) return 1;
    const char *alg = "Kyber512"; // choose per NIST recommendations
    OQS_KEM *kem = OQS_KEM_new(alg);
    if (!kem) { fprintf(stderr, "KEM %s not available\n", alg); return 1; }

    uint8_t *pk = malloc(kem->length_public_key);
    uint8_t *sk = malloc(kem->length_secret_key);
    uint8_t *ct = malloc(kem->length_ciphertext);
    uint8_t *ss_enc = malloc(kem->length_shared_secret);
    uint8_t *ss_dec = malloc(kem->length_shared_secret);

    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);
    if (OQS_KEM_keypair(kem, pk, sk) != OQS_SUCCESS) { perror("keypair"); return 1; }
    clock_gettime(CLOCK_MONOTONIC, &t1);
    printf("keypair: %.3f ms\n", timediff_ms(t1, t0));

    clock_gettime(CLOCK_MONOTONIC, &t0);
    if (OQS_KEM_encaps(kem, ct, ss_enc, pk) != OQS_SUCCESS) { perror("encaps"); return 1; }
    clock_gettime(CLOCK_MONOTONIC, &t1);
    printf("encaps: %.3f ms\n", timediff_ms(t1, t0));

    clock_gettime(CLOCK_MONOTONIC, &t0);
    if (OQS_KEM_decaps(kem, ss_dec, ct, sk) != OQS_SUCCESS) { perror("decaps"); return 1; }
    clock_gettime(CLOCK_MONOTONIC, &t1);
    printf("decaps: %.3f ms\n", timediff_ms(t1, t0));

    if (memcmp(ss_enc, ss_dec, kem->length_shared_secret) == 0) {
        printf("shared secret match\n");
    } else {
        printf("shared secret MISMATCH\n");
    }

    free(pk); free(sk); free(ct); free(ss_enc); free(ss_dec);
    OQS_KEM_free(kem);
    OQS_destroy();
    return 0;
}
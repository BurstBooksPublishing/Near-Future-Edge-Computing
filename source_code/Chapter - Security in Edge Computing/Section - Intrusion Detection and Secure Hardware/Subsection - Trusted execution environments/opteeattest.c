#include 
#include 
#include 

#define TA_UUID {0x12345678,0x1234,0x1234, \
                 {0x12,0x34,0x56,0x78,0x9a,0xbc,0xde,0xf0}}
#define CMD_SIGN_NONCE 0x00000001

int main(void) {
    TEEC_Context ctx;
    TEEC_Session sess;
    TEEC_Operation op;
    TEEC_Result res;
    TEEC_UUID uuid = TA_UUID;
    uint8_t nonce[32] = {0};                /* caller-generated challenge */
    uint8_t signature[512] = {0};           /* buffer for TA signature */
    uint32_t sig_len = sizeof(signature);

    /* initialize OP-TEE context */
    res = TEEC_InitializeContext(NULL, &ctx);
    if (res != TEEC_SUCCESS) return -1;

    /* open session to trusted application */
    res = TEEC_OpenSession(&ctx, &sess, &uuid,
                           TEEC_LOGIN_PUBLIC, NULL, NULL, NULL);
    if (res != TEEC_SUCCESS) { TEEC_FinalizeContext(&ctx); return -2; }

    /* prepare operation: pass nonce as input, signature as output */
    memset(&op, 0, sizeof(op));
    op.paramTypes = TEEC_PARAM_TYPES(
        TEEC_MEMREF_TEMP_INPUT, TEEC_MEMREF_TEMP_OUTPUT, TEEC_NONE, TEEC_NONE);
    op.params[0].tmpref.buffer = nonce;
    op.params[0].tmpref.size = sizeof(nonce);
    op.params[1].tmpref.buffer = signature;
    op.params[1].tmpref.size = sig_len;

    /* invoke TA command to sign nonce with sealed private key */
    res = TEEC_InvokeCommand(&sess, CMD_SIGN_NONCE, &op, NULL);
    if (res == TEEC_SUCCESS) {
        sig_len = op.params[1].tmpref.size;
        /* send signature and nonce to remote verifier over TLS */
        printf("Attestation signature length: %u\n", sig_len);
    }

    TEEC_CloseSession(&sess);
    TEEC_FinalizeContext(&ctx);
    return (res == TEEC_SUCCESS) ? 0 : -3;
}
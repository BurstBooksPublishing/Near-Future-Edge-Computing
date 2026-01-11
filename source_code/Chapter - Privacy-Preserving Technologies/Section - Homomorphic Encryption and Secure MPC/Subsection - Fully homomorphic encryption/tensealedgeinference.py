# Client-side (sensor device): encrypt features and send ciphertext
import tenseal as ts
import numpy as np
# -- configuration: increase poly_modulus_degree for production (e.g., 16384)
poly_modulus_degree = 8192
coeff_mod_bit_sizes = [60, 40, 40]  # adjust for depth and security
ctx = ts.context(ts.SCHEME_TYPE.CKKS, poly_modulus_degree, coeff_mod_bit_sizes)
ctx.global_scale = 2**40
ctx.generate_galois_keys()  # enable rotations if packing is used
# sensor feature vector
features = np.array([0.12, -0.03, 1.5, 0.0], dtype=float)
enc_vec = ts.ckks_vector(ctx, features)  # encrypt and pack
# serialize ciphertext for transfer to edge gateway
payload = enc_vec.serialize()
# send `payload` to edge gateway via TLS-protected channel (implementation-specific)

# Server-side (edge gateway): evaluate linear model on ciphertext
# assume server loads plaintext weights w (kept secret at server side)
# NOTE: server does NOT have secret key
import tenseal as ts
# deserialize context and ciphertext (client should either send context or use public context)
enc_vec = ts.ckks_vector_from(ctx, payload)
weights = np.array([0.5, -0.2, 0.3, 0.0], dtype=float)
# compute encrypted dot product: sum(weights * encrypted_vector)
# use elementwise multiplication with plaintext and sum
enc_dot = enc_vec.mul_plain(weights.tolist()).sum()  # homomorphic ops
result_payload = enc_dot.serialize()
# send result_payload back to client

# Client-side: decrypt result
dec_enc_dot = ts.ckks_vector_from(ctx, result_payload)  # or ciphertext if scalar
result = dec_enc_dot.decrypt()[0]  # decrypted scalar result
print("Inference result:", result)
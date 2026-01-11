#include 
#include 
#include 
// Minimal benchmark: setup, keygen, encrypt, multiply+relinearize, decrypt.
int main() {
  using namespace std::chrono;
  seal::EncryptionParameters parms(seal::scheme_type::ckks);
  parms.set_poly_modulus_degree(8192); // tune for memory/latency
  parms.set_coeff_modulus(seal::CoeffModulus::Create(8192, {60,40,40,60}));
  auto context = seal::SEALContext::Create(parms);
  seal::KeyGenerator keygen(context);
  auto sk = keygen.secret_key();
  auto pk = keygen.create_public_key();
  auto relin_keys = keygen.create_relin_keys();
  seal::Encryptor enc(context, pk);
  seal::Decryptor dec(context, sk);
  seal::Evaluator eval(context);
  seal::CKKSEncoder encoder(context);
  double scale = pow(2.0, 40);
  std::vector data(1024, 1.23);
  seal::Plaintext pt; encoder.encode(data, scale, pt);
  seal::Ciphertext ct1, ct2;
  auto t0 = high_resolution_clock::now();
  enc.encrypt(pt, ct1); // encryption
  enc.encrypt(pt, ct2);
  auto t1 = high_resolution_clock::now();
  eval.multiply_inplace(ct1, ct2);
  eval.relinearize_inplace(ct1, relin_keys);
  auto t2 = high_resolution_clock::now();
  seal::Plaintext out; dec.decrypt(ct1, out);
  auto t3 = high_resolution_clock::now();
  std::cout << "enc(ms):" << duration_cast(t1-t0).count()
            << " eval(ms):" << duration_cast(t2-t1).count()
            << " dec(ms):" << duration_cast(t3-t2).count() << "\n";
  return 0;
}
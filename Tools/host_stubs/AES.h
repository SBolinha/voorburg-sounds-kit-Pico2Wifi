#ifndef VOORBURG_TEST_AES_H
#define VOORBURG_TEST_AES_H
#include <assert.h>
#include <openssl/evp.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

// Host tests use OpenSSL's independent AES primitive with the sketch's real
// CMAC/frame code. Arduino builds use Crypto's AES128, not this adapter.
class AES128 {
public:
  bool setKey(const uint8_t *value, size_t length) {
    if (!value || length != sizeof(key_)) return false;
    memcpy(key_, value, sizeof(key_));
    return true;
  }
  void encryptBlock(uint8_t *out, const uint8_t *in) {
    EVP_CIPHER_CTX *context = EVP_CIPHER_CTX_new();
    assert(context);
    assert(EVP_EncryptInit_ex(context, EVP_aes_128_ecb(), nullptr, key_, nullptr) == 1);
    assert(EVP_CIPHER_CTX_set_padding(context, 0) == 1);
    int written = 0;
    assert(EVP_EncryptUpdate(context, out, &written, in, 16) == 1 && written == 16);
    uint8_t tail[16];
    assert(EVP_EncryptFinal_ex(context, tail, &written) == 1 && written == 0);
    EVP_CIPHER_CTX_free(context);
  }
private:
  uint8_t key_[16] = {};
};
#endif

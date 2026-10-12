#ifndef VOORBURG_TEST_LOG_ARDUINO_H
#define VOORBURG_TEST_LOG_ARDUINO_H
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <string>

class Print {
public:
  virtual ~Print() {}
  virtual size_t write(uint8_t) = 0;
  virtual size_t write(const uint8_t *bytes, size_t length) {
    size_t written = 0;
    while (written < length && write(bytes[written])) ++written;
    return written;
  }
  size_t print(const char *text) {
    return write(reinterpret_cast<const uint8_t *>(text), strlen(text));
  }
  size_t println(const char *text) { return print(text) + println(); }
  size_t println() { return print("\r\n"); }
};

struct HostSerial : Print {
  bool connected = false;
  size_t capacity = 0;
  size_t calls = 0;
  bool returnZero = false;
  bool shortWrite = false;
  std::string output;
  explicit operator bool() const { return connected; }
  int availableForWrite() const { return static_cast<int>(capacity); }
  size_t write(uint8_t value) override { return write(&value, 1); }
  size_t write(const uint8_t *bytes, size_t length) override {
    assert(connected && capacity >= length);
    ++calls;
    if (returnZero) return 0;
    if (shortWrite && length > 1) length /= 2;
    capacity -= length;
    output.append(reinterpret_cast<const char *>(bytes), length);
    return length;
  }
};
extern HostSerial Serial;
inline uint32_t millis() { return 12345; }
#endif

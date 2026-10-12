#ifndef VOORBURG_PICO_I2S_INPUT_H
#define VOORBURG_PICO_I2S_INPUT_H

#include <Arduino.h>
#include <hardware/pio.h>

// One 32-bit stereo microphone, serviced entirely on the core calling begin().
// Resources are claimed once; stop() pauses capture without allocating/freeing
// memory. Call begin/stop/available/read from that same core.
class PicoI2SInput {
public:
  static constexpr size_t DMA_WORDS = 512;
  static constexpr size_t RING_WORDS = 4096;

  bool begin(uint32_t sampleRate, uint8_t bclkPin, uint8_t dataPin);
  bool stop();
  size_t available();
  size_t read(uint8_t *destination, size_t bytes);
  uint32_t overrunWords() const { return overrunWords_; }
  uint32_t dmaRestarts() const { return dmaRestarts_; }

  PicoI2SInput() = default;
  PicoI2SInput(const PicoI2SInput &) = delete;
  PicoI2SInput &operator=(const PicoI2SInput &) = delete;

private:
  static PicoI2SInput *owner_;
  static void irqHandler();
  void serviceIRQ();
  void appendBlock(unsigned slot);
  void startBlock(unsigned slot);
  void recoverIdle();
  bool claimResources(uint8_t bclkPin, uint8_t dataPin);

  PIO pio_ = nullptr;
  uint sm_ = 0;
  uint programOffset_ = 0;
  uint ownerCore_ = 0;
  int channels_[2] = {-1, -1};
  uint8_t bclkPin_ = 0;
  uint8_t dataPin_ = 0;
  uint32_t sampleRate_ = 0;
  unsigned nextSlot_ = 0;
  bool allocated_ = false;
  volatile bool running_ = false;
  uint32_t dmaBlocks_[2][DMA_WORDS] = {};
  uint32_t ring_[RING_WORDS] = {};
  size_t readPosition_ = 0;
  size_t writePosition_ = 0;
  size_t queuedWords_ = 0;
  uint32_t overrunWords_ = 0;
  uint32_t dmaRestarts_ = 0;
};

#endif

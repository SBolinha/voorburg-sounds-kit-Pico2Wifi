// Offline deterministic driver checks. Does not open a USB device or network.
// c++ -std=c++20 -Wall -Wextra -Werror -pedantic -ITools/audio_host_stubs \
//   -IArduino/VoorburgSoundsKit Tools/test_pico_i2s_input.cpp \
//   Arduino/VoorburgSoundsKit/PicoI2SInput.cpp -o /tmp/voorburg-audio-test
#include <PicoI2SInput.h>
#include <cstdio>

static void completeBlock(uint channel, uint32_t firstWord, bool dispatch = true) {
  assert(FakeAudio::busy[channel]);
  assert(FakeAudio::words[channel] == PicoI2SInput::DMA_WORDS);
  for (uint i = 0; i < FakeAudio::words[channel]; ++i)
    FakeAudio::destination[channel][i] = firstWord + i;
  FakeAudio::busy[channel] = false;
  if (FakeAudio::irqEnabled[channel]) dma_hw->ints0 |= 1u << channel;
  if (dispatch) FakeAudio::handler();
}

int main() {
  // Exercise RP2350 channel numbers that exceeded AudioBufferManager's map.
  for (uint channel = 0; channel < 12; ++channel) FakeAudio::claimed[channel] = true;
  PicoI2SInput input;
  assert(!input.begin(0, 10, 12));
  assert(!input.begin(48000, 10, 11));
  assert(input.begin(48000, 10, 12));
  assert(FakeAudio::claimedCount == 2);
  assert(FakeAudio::irqInstalledOnEnabledCore);
  assert(FakeAudio::chain[12] == 12 && FakeAudio::chain[13] == 13);
  assert(FakeAudio::divider > 24.4f && FakeAudio::divider < 24.5f);
  const uint16_t expected[] = {0xa022, 0x4801, 0x0041, 0x4801,
                             0xb022, 0x5801, 0x1045, 0x5801};
  assert(FakeAudio::program->length == 8);
  assert(std::memcmp(expected, FakeAudio::program->instructions, sizeof(expected)) == 0);
  assert(input.available() == 0);

  completeBlock(12, 0);
  assert(FakeAudio::busy[13] && !FakeAudio::busy[12]);
  assert(input.available() == 2048);
  uint32_t result[512] = {};
  // Odd byte requests cannot consume a half stereo frame.
  assert(input.read(reinterpret_cast<uint8_t *>(result), 13) == 8);
  assert(result[0] == 0 && result[1] == 1);
  assert(input.read(reinterpret_cast<uint8_t *>(result), sizeof(result)) == 2040);
  for (uint i = 0; i < 510; ++i) assert(result[i] == i + 2);
  assert(input.available() == 0);

  assert(input.stop());
  assert(input.begin(48000, 10, 12));
  assert(FakeAudio::claimedCount == 2 && FakeAudio::unclaimedCount == 0);
  // Force an overrun with a partially consumed oldest block. Dropped data
  // remains even and new blocks preserve stereo alignment across ring wrap.
  for (uint block = 0; block < 8; ++block)
    completeBlock((block & 1) ? 13 : 12, block * 512);
  assert(input.available() == 16384);
  assert(input.read(reinterpret_cast<uint8_t *>(result), 16) == 16);
  completeBlock(12, 4096);
  assert(input.overrunWords() == 508);
  for (uint block = 1; block <= 8; ++block) {
    assert(input.read(reinterpret_cast<uint8_t *>(result), sizeof(result)) == sizeof(result));
    for (uint word = 0; word < 512; ++word) assert(result[word] == block * 512 + word);
  }
  assert(input.available() == 0);

  // IRQ masking leaves one complete block and both DMA channels idle; it
  // cannot auto-trigger another write at an incremented buffer address.
  completeBlock(13, 4608, false);
  assert(!FakeAudio::busy[12] && !FakeAudio::busy[13]);
  FakeAudio::currentCore = 0;
  FakeAudio::handler();
  assert(dma_hw->ints0 & (1u << 13));
  assert(input.available() == 0 && !input.stop());
  FakeAudio::currentCore = 1;
  FakeAudio::handler();
  assert(FakeAudio::busy[12] && input.available() == 2048);
  assert(input.read(reinterpret_cast<uint8_t *>(result), sizeof(result)) == sizeof(result));
  for (uint word = 0; word < 512; ++word) assert(result[word] == 4608 + word);
  // An unrelated client's pending IRQ is preserved and never dispatched.
  dma_hw->ints0 = 1u << 3;
  FakeAudio::handler();
  assert(dma_hw->ints0 == (1u << 3));

  // Explicit idle recovery handles a lost completion notification.
  FakeAudio::busy[12] = false;
  const uint32_t restartsBefore = input.dmaRestarts();
  input.available();
  assert(input.dmaRestarts() == restartsBefore + 1);
  assert(FakeAudio::busy[13]);
  // Critical sections preserve an already disabled interrupt state.
  FakeAudio::irqState = 1;
  input.available();
  assert(input.read(reinterpret_cast<uint8_t *>(result), sizeof(result)) == 0);
  assert(FakeAudio::irqState == 1);
  FakeAudio::irqState = 0;

  FakeAudio::abortStuck = true;
  const uint32_t beforeTimeout = FakeAudio::now;
  assert(!input.stop());
  assert(FakeAudio::now - beforeTimeout <= 2300);
  assert(FakeAudio::loopCalls <= 21);
  assert(!FakeAudio::pioEnabled && !FakeAudio::irqEnabled[12] && !FakeAudio::irqEnabled[13]);
  assert(FakeAudio::irqState == 0);
  const uint32_t startsBefore = FakeAudio::startCount[12] + FakeAudio::startCount[13];
  assert(!input.begin(48000, 10, 12));
  assert(FakeAudio::startCount[12] + FakeAudio::startCount[13] == startsBefore);
  assert(FakeAudio::unclaimedCount == 0);
  FakeAudio::abortStuck = false;
  assert(input.begin(48000, 10, 12));
  assert(input.stop());
  assert(input.available() == 0);
  assert(input.read(nullptr, 1024) == 0);
  std::puts("PicoI2SInput: timing, high DMA channels, ring wrap/overrun, IRQ delay, idle recovery and bounded abort checks passed");
}

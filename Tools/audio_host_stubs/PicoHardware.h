#pragma once
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>

using uint = unsigned;
#define __not_in_flash_func(name) name
#define PICO_PIO_VERSION 1
#define NUM_BANK0_GPIOS 48
#define DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB 4
#define DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS 0xf0u
#define DMA_CH0_CTRL_TRIG_EN_BITS 1u
#define DMA_IRQ_0 0
#define PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY 128
#define DMA_SIZE_32 2
#define PIO_FIFO_JOIN_RX 1
#define clk_sys 0
#define pio_y 2
#define pio_pins 0

struct pio_hw { uint32_t rxf[4] = {}; };
using PIO = pio_hw *;
struct pio_program {
  const uint16_t *instructions;
  uint8_t length;
  int8_t origin;
  uint8_t pio_version;
  uint8_t used_gpio_ranges;
};
struct pio_sm_config { float divider = 0; };
struct dma_channel_config { uint chain = 0; bool readIncrement = false;
  bool writeIncrement = false; };

namespace FakeAudio {
inline pio_hw pio;
inline bool pioEnabled = false;
inline bool claimed[16] = {};
inline bool busy[16] = {};
inline bool irqEnabled[16] = {};
inline bool abortStuck = false;
inline uint32_t irqState = 0;
inline uint32_t now = 0;
inline uint32_t loopCalls = 0;
inline uint32_t claimedCount = 0;
inline uint32_t unclaimedCount = 0;
inline uint32_t startCount[16] = {};
inline uint32_t words[16] = {};
inline uint32_t *destination[16] = {};
inline uint chain[16] = {};
inline float divider = 0;
inline const pio_program *program = nullptr;
inline void (*handler)() = nullptr;
inline bool irqInstalledOnEnabledCore = false;
inline uint currentCore = 1;
}
inline uint get_core_num() { return FakeAudio::currentCore; }

struct FakeDMARegister {
  uint32_t value = 0;
  bool isAbort = false;
  FakeDMARegister &operator=(uint32_t next);
  operator uint32_t() const { return value; }
};
struct dma_hw_t {
  struct { uint32_t al1_ctrl = 0; } ch[16];
  FakeDMARegister abort = {0, true};
  FakeDMARegister intr;
  uint32_t ints0 = 0;
};
inline dma_hw_t dmaHardware;
inline dma_hw_t *dma_hw = &dmaHardware;

inline FakeDMARegister &FakeDMARegister::operator=(uint32_t next) {
  value = next;
  if (isAbort) {
    if (!FakeAudio::abortStuck)
      for (uint channel = 0; channel < 16; ++channel)
        if (next & (1u << channel)) FakeAudio::busy[channel] = false;
  } else {
    dma_hw->ints0 &= ~next;
  }
  return *this;
}
inline uint32_t save_and_disable_interrupts() {
  uint32_t previous = FakeAudio::irqState;
  FakeAudio::irqState = 1;
  return previous;
}
inline void restore_interrupts(uint32_t previous) { FakeAudio::irqState = previous; }
inline uint32_t time_us_32() { FakeAudio::now += 100; return FakeAudio::now; }
inline void tight_loop_contents() { ++FakeAudio::loopCalls; }
inline void hw_write_masked(uint32_t *address, uint32_t value, uint32_t mask) {
  *address = (*address & ~mask) | (value & mask);
}
inline uint32_t clock_get_hz(uint) { return 150000000; }
inline pio_sm_config pio_get_default_sm_config() { return {}; }
inline void sm_config_set_wrap(pio_sm_config *, uint, uint) {}
inline void sm_config_set_sideset(pio_sm_config *, uint, bool, bool) {}
inline void sm_config_set_in_pins(pio_sm_config *, uint) {}
inline void sm_config_set_sideset_pins(pio_sm_config *, uint) {}
inline void sm_config_set_in_shift(pio_sm_config *, bool, bool, uint) {}
inline void sm_config_set_fifo_join(pio_sm_config *, uint) {}
inline void sm_config_set_clkdiv(pio_sm_config *config, float divider) {
  config->divider = divider;
}
inline void pio_gpio_init(PIO, uint) {}
inline void pio_sm_set_consecutive_pindirs(PIO, uint, uint, uint, bool) {}
inline void pio_sm_set_set_pins(PIO, uint, uint, uint) {}
inline uint pio_encode_set(uint, uint) { return 0; }
inline uint pio_encode_in(uint, uint) { return 0; }
inline void pio_sm_exec(PIO, uint, uint) {}
inline void pio_sm_init(PIO, uint, uint, pio_sm_config *config) {
  FakeAudio::divider = config->divider;
}
inline void pio_sm_set_enabled(PIO, uint, bool enabled) {
  FakeAudio::pioEnabled = enabled;
}
inline uint pio_get_dreq(PIO, uint, bool) { return 0; }
inline bool pio_claim_free_sm_and_add_program_for_gpio_range(
    const pio_program *program, PIO *pio, uint *sm, uint *offset, uint, uint, bool) {
  FakeAudio::program = program;
  *pio = &FakeAudio::pio; *sm = 0; *offset = 0;
  return true;
}
inline void pio_remove_program_and_unclaim_sm(const pio_program *, PIO, uint, uint) {}
inline int dma_claim_unused_channel(bool) {
  for (uint channel = 0; channel < 16; ++channel) {
    if (FakeAudio::claimed[channel]) continue;
    FakeAudio::claimed[channel] = true;
    ++FakeAudio::claimedCount;
    return channel;
  }
  return -1;
}
inline void dma_channel_unclaim(uint channel) {
  FakeAudio::claimed[channel] = false;
  ++FakeAudio::unclaimedCount;
}
inline dma_channel_config dma_channel_get_default_config(uint) { return {}; }
inline void channel_config_set_transfer_data_size(dma_channel_config *, uint) {}
inline void channel_config_set_read_increment(dma_channel_config *config, bool value) {
  config->readIncrement = value;
}
inline void channel_config_set_write_increment(dma_channel_config *config, bool value) {
  config->writeIncrement = value;
}
inline void channel_config_set_dreq(dma_channel_config *, uint) {}
inline void channel_config_set_chain_to(dma_channel_config *config, uint channel) {
  config->chain = channel;
}
inline void channel_config_set_irq_quiet(dma_channel_config *, bool) {}
inline uint32_t dma_encode_transfer_count(uint count) { return count; }
inline void dma_channel_set_write_addr(uint channel, void *address, bool) {
  assert(!FakeAudio::busy[channel]);
  FakeAudio::destination[channel] = static_cast<uint32_t *>(address);
}
inline void dma_channel_set_transfer_count(uint channel, uint32_t count, bool trigger) {
  assert(!FakeAudio::busy[channel]);
  FakeAudio::words[channel] = count;
  if (trigger) {
    FakeAudio::busy[channel] = true;
    ++FakeAudio::startCount[channel];
  }
}
inline void dma_channel_configure(uint channel, dma_channel_config *config,
    void *destination, const volatile void *, uint32_t count, bool trigger) {
  FakeAudio::chain[channel] = config->chain;
  assert(!config->readIncrement && config->writeIncrement);
  dma_channel_set_write_addr(channel, destination, false);
  dma_channel_set_transfer_count(channel, count, trigger);
}
inline void dma_channel_acknowledge_irq0(uint channel) {
  dma_hw->ints0 &= ~(1u << channel);
}
inline void dma_channel_set_irq0_enabled(uint channel, bool enabled) {
  FakeAudio::irqEnabled[channel] = enabled;
}
inline bool dma_channel_is_busy(uint channel) { return FakeAudio::busy[channel]; }
inline void irq_add_shared_handler(uint, void (*handler)(), uint) {
  FakeAudio::handler = handler;
}
inline void irq_set_enabled(uint, bool enabled) {
  FakeAudio::irqInstalledOnEnabledCore = enabled;
}

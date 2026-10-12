#include "PicoI2SInput.h"
#include <hardware/clocks.h>
#include <hardware/dma.h>
#include <hardware/irq.h>
#include <hardware/sync.h>
#include <pico/time.h>
#include <pico/platform.h>
#include <string.h>

namespace {
// Master stereo input program and initialization copied from Arduino-Pico
// I2S/src/pio_i2s.pio (6.3.0; unchanged from 6.2.0).
// Copyright (c) 2022 Earle F. Philhower, III. LGPL-2.1-or-later;
// https://github.com/earlephilhower/arduino-pico/tree/master/libraries/I2S
// Keep the upstream sampling edges and initial left/right word alignment.
const uint16_t inputInstructions[] = {
  0xa022, 0x4801, 0x0041, 0x4801,
  0xb022, 0x5801, 0x1045, 0x5801
};
const pio_program inputProgram = {
  .instructions = inputInstructions,
  .length = 8,
  .origin = -1,
  .pio_version = 0,
#if PICO_PIO_VERSION > 0
  .used_gpio_ranges = 0
#endif
};
constexpr uint32_t ABORT_TIMEOUT_US = 2000;

void initializeInput(PIO pio, uint sm, uint offset, uint8_t bclk,
                     uint8_t data, uint32_t sampleRate) {
  pio_gpio_init(pio, data);
  pio_gpio_init(pio, bclk);
  pio_gpio_init(pio, bclk + 1);
  pio_sm_config config = pio_get_default_sm_config();
  sm_config_set_wrap(&config, offset, offset + 7);
  sm_config_set_sideset(&config, 2, false, false);
  sm_config_set_in_pins(&config, data);
  sm_config_set_sideset_pins(&config, bclk);
  sm_config_set_in_shift(&config, false, true, 32);
  sm_config_set_fifo_join(&config, PIO_FIFO_JOIN_RX);
  // 32 bits/channel * two channels * two instructions/bit, as upstream.
  sm_config_set_clkdiv(&config,
    static_cast<float>(clock_get_hz(clk_sys)) / (sampleRate * 128.0f));
  pio_sm_init(pio, sm, offset, &config);
  pio_sm_set_consecutive_pindirs(pio, sm, data, 1, false);
  pio_sm_set_consecutive_pindirs(pio, sm, bclk, 2, true);
  pio_sm_set_set_pins(pio, sm, bclk, 2);
  pio_sm_exec(pio, sm, pio_encode_set(pio_y, 30));
  pio_sm_exec(pio, sm, pio_encode_in(pio_pins, 32));
  pio_sm_exec(pio, sm, pio_encode_in(pio_pins, 31));
}
} // namespace

PicoI2SInput *PicoI2SInput::owner_ = nullptr;

bool PicoI2SInput::claimResources(uint8_t bclkPin, uint8_t dataPin) {
  if (owner_ && owner_ != this) return false;
  const uint firstPin = bclkPin < dataPin ? bclkPin : dataPin;
  const uint lastPin = bclkPin + 1 > dataPin ? bclkPin + 1 : dataPin;
  if (!pio_claim_free_sm_and_add_program_for_gpio_range(
      &inputProgram, &pio_, &sm_, &programOffset_, firstPin,
      lastPin - firstPin + 1, true)) return false;
  channels_[0] = dma_claim_unused_channel(false);
  if (channels_[0] >= 0) channels_[1] = dma_claim_unused_channel(false);
  if (channels_[0] < 0 || channels_[1] < 0) {
    if (channels_[0] >= 0) dma_channel_unclaim(channels_[0]);
    channels_[0] = channels_[1] = -1;
    pio_remove_program_and_unclaim_sm(&inputProgram, pio_, sm_, programOffset_);
    pio_ = nullptr;
    return false;
  }
  bclkPin_ = bclkPin;
  dataPin_ = dataPin;
  owner_ = this;
  ownerCore_ = get_core_num();
  allocated_ = true;
  // IRQ registration and enable are local to the calling core (core 1 in the
  // sketch). No global DMA channel table or assumption about channel numbers.
  irq_add_shared_handler(DMA_IRQ_0, irqHandler,
                        PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
  irq_set_enabled(DMA_IRQ_0, true);
  return true;
}

bool PicoI2SInput::begin(uint32_t sampleRate, uint8_t bclkPin, uint8_t dataPin) {
  if (!sampleRate || bclkPin >= NUM_BANK0_GPIOS - 1 ||
      dataPin >= NUM_BANK0_GPIOS || dataPin == bclkPin ||
      dataPin == bclkPin + 1) return false;
  if (allocated_ && (bclkPin != bclkPin_ || dataPin != dataPin_)) return false;
  if (allocated_ && get_core_num() != ownerCore_) return false;
  if (running_) return sampleRate == sampleRate_;
  if (!allocated_ && !claimResources(bclkPin, dataPin)) return false;
  // Recheck an earlier failed abort. Never reconfigure a still-busy channel.
  if (!stop()) return false;
  initializeInput(pio_, sm_, programOffset_, bclkPin, dataPin, sampleRate);
  const uint32_t interruptState = save_and_disable_interrupts();
  sampleRate_ = sampleRate;
  readPosition_ = writePosition_ = queuedWords_ = 0;
  nextSlot_ = 0;
  for (unsigned slot = 0; slot < 2; ++slot) {
    const uint channel = static_cast<uint>(channels_[slot]);
    dma_channel_config config = dma_channel_get_default_config(channel);
    channel_config_set_transfer_data_size(&config, DMA_SIZE_32);
    channel_config_set_read_increment(&config, false);
    channel_config_set_write_increment(&config, true);
    channel_config_set_dreq(&config, pio_get_dreq(pio_, sm_, false));
    // Hardware chaining would restart at the end-of-buffer write address if
    // an IRQ were delayed. Software ping-pong cannot write beyond its block;
    // a full RX FIFO safely pauses PIO until its completion IRQ is serviced.
    channel_config_set_chain_to(&config, channel);
    channel_config_set_irq_quiet(&config, false);
    dma_channel_configure(channel, &config, dmaBlocks_[slot],
      &pio_->rxf[sm_], dma_encode_transfer_count(DMA_WORDS), false);
    dma_channel_acknowledge_irq0(channel);
    dma_channel_set_irq0_enabled(channel, true);
  }
  running_ = true;
  startBlock(0);
  pio_sm_set_enabled(pio_, sm_, true);
  restore_interrupts(interruptState);
  return true;
}

bool PicoI2SInput::stop() {
  if (!allocated_) return true;
  if (get_core_num() != ownerCore_) return false;
  const uint32_t interruptState = save_and_disable_interrupts();
  running_ = false;
  pio_sm_set_enabled(pio_, sm_, false);
  uint32_t ownedMask = 0;
  for (unsigned slot = 0; slot < 2; ++slot) {
    const uint channel = static_cast<uint>(channels_[slot]);
    ownedMask |= 1u << channel;
    dma_channel_set_irq0_enabled(channel, false);
    // Disable both triggers before aborting either channel.
    hw_write_masked(&dma_hw->ch[channel].al1_ctrl,
      channel << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB,
      DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS | DMA_CH0_CTRL_TRIG_EN_BITS);
  }
  dma_hw->abort = ownedMask;
  const uint32_t startedAt = time_us_32();
  while ((dma_channel_is_busy(channels_[0]) ||
          dma_channel_is_busy(channels_[1])) &&
         static_cast<uint32_t>(time_us_32() - startedAt) < ABORT_TIMEOUT_US) {
    tight_loop_contents();
  }
  const bool stopped = !dma_channel_is_busy(channels_[0]) &&
                       !dma_channel_is_busy(channels_[1]);
  dma_hw->intr = ownedMask;
  readPosition_ = writePosition_ = queuedWords_ = 0;
  // On timeout resources stay claimed and buffers stay alive. begin() must
  // obtain a successful abort before reuse; watchdog supervision continues.
  restore_interrupts(interruptState);
  return stopped;
}

void __not_in_flash_func(PicoI2SInput::startBlock)(unsigned slot) {
  const uint channel = static_cast<uint>(channels_[slot]);
  dma_channel_set_write_addr(channel, dmaBlocks_[slot], false);
  dma_channel_set_transfer_count(channel, dma_encode_transfer_count(DMA_WORDS), true);
  nextSlot_ = slot ^ 1u;
}

void __not_in_flash_func(PicoI2SInput::appendBlock)(unsigned slot) {
  const size_t freeWords = RING_WORDS - queuedWords_;
  if (freeWords < DMA_WORDS) {
    const size_t discarded = DMA_WORDS - freeWords;
    readPosition_ = (readPosition_ + discarded) % RING_WORDS;
    queuedWords_ -= discarded;
    overrunWords_ += static_cast<uint32_t>(discarded);
  }
  // RING_WORDS is an exact multiple of DMA_WORDS, so producer blocks never
  // cross the end. Reads and discarded words are always stereo pairs.
  memcpy(ring_ + writePosition_, dmaBlocks_[slot], sizeof(dmaBlocks_[slot]));
  writePosition_ = (writePosition_ + DMA_WORDS) % RING_WORDS;
  queuedWords_ += DMA_WORDS;
}

void __not_in_flash_func(PicoI2SInput::serviceIRQ)() {
  // SDK vector tables may be shared even though NVIC enables are per core.
  // Never let a second core service or modify this core's audio DMA/ring.
  if (!allocated_ || get_core_num() != ownerCore_) return;
  const uint32_t ownedMask = (1u << channels_[0]) | (1u << channels_[1]);
  const uint32_t pending = dma_hw->ints0 & ownedMask;
  // With software ping-pong there is at most one completed block. Bound
  // dispatch to the two owned channels even if another DMA client has IRQs.
  for (unsigned slot = 0; slot < 2; ++slot) {
    if (!(pending & (1u << channels_[slot]))) continue;
    dma_channel_acknowledge_irq0(channels_[slot]);
    if (!running_) continue;
    // A stale completion flag must never make us copy a block that DMA is
    // currently writing (including an unexpected pair of pending flags).
    if (dma_channel_is_busy(channels_[slot])) {
      overrunWords_ += DMA_WORDS;
      continue;
    }
    const unsigned other = slot ^ 1u;
    if (!dma_channel_is_busy(channels_[other])) startBlock(other);
    // This block cannot be restarted until a later invocation of this IRQ.
    appendBlock(slot);
  }
  recoverIdle();
}

void __not_in_flash_func(PicoI2SInput::recoverIdle)() {
  if (!running_ || dma_channel_is_busy(channels_[0]) ||
      dma_channel_is_busy(channels_[1])) return;
  const uint32_t ownedMask = (1u << channels_[0]) | (1u << channels_[1]);
  if (dma_hw->ints0 & ownedMask) return; // Preserve an unread completed block.
  ++dmaRestarts_;
  startBlock(nextSlot_);
}

void __not_in_flash_func(PicoI2SInput::irqHandler)() {
  if (owner_) owner_->serviceIRQ();
}

size_t PicoI2SInput::available() {
  if (allocated_ && get_core_num() != ownerCore_) return 0;
  const uint32_t interruptState = save_and_disable_interrupts();
  recoverIdle();
  const size_t bytes = running_ ? queuedWords_ * sizeof(uint32_t) : 0;
  restore_interrupts(interruptState);
  return bytes;
}

size_t PicoI2SInput::read(uint8_t *destination, size_t bytes) {
  if (allocated_ && get_core_num() != ownerCore_) return 0;
  if (!destination || !bytes) return 0;
  // Bound every critical section to one DMA block, regardless of caller size.
  size_t words = bytes / (2 * sizeof(uint32_t)) * 2;
  if (words > DMA_WORDS) words = DMA_WORDS;
  const uint32_t interruptState = save_and_disable_interrupts();
  if (!running_) words = 0;
  if (words > queuedWords_) words = queuedWords_;
  size_t firstWords = RING_WORDS - readPosition_;
  if (firstWords > words) firstWords = words;
  memcpy(destination, ring_ + readPosition_, firstWords * sizeof(uint32_t));
  if (words > firstWords) memcpy(destination + firstWords * sizeof(uint32_t),
    ring_, (words - firstWords) * sizeof(uint32_t));
  readPosition_ = (readPosition_ + words) % RING_WORDS;
  queuedWords_ -= words;
  restore_interrupts(interruptState);
  return words * sizeof(uint32_t);
}

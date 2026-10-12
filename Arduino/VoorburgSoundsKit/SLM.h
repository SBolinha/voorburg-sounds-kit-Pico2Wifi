/*
 * Portable Pico 2 W sound-level processing for the SPH0645 I2S microphone.
 * The I2S capture and A-weighted frequency correction follow Amsterdam's
 * implementation; this version uses a portable floating-point FFT so it does
 * not rely on the Feather/M0 precompiled CMSIS-DSP library.
 */
#ifndef VOORBURG_SLM_H
#define VOORBURG_SLM_H

#include <Arduino.h>
#include "PicoI2SInput.h"
#include <math.h>
#include <limits.h>
#include "SLMSettings.h"
#include "Stats.h"
#include "RunningAvg.h"
#include "EQ.h"

class SLM {
public:
  Stats statsRMS;
  Stats statsDB;
  double latestLevelRelative = 0.0;

  explicit SLM(Stream &output) : serial(output) {}

  bool setup() {
    if (!i2s_ok) {
      i2s_ok = i2s.begin(AUDIO_SAMPLE_RATE_HZ, SLM_I2S_BCLK_PIN,
                        SLM_I2S_DATA_PIN);
    }
    if (!i2s_ok) return false;
    if (!windowInitialized) {
      for (uint16_t n = 0; n < DATA_SIZE; ++n) {
        const float phase = (2.0f * PI * n) / (DATA_SIZE - 1);
        hann[n] = 0.5f - 0.5f * cosf(phase);
      }
      windowInitialized = true;
    }
    return true;
  }

  bool start() {
    if (!setup()) return false;
    channel_word = 0;
    running = true;
    return true;
  }

  // Pause capture for flash writes without discarding the current measurement
  // statistics or analysis history. The fixed DMA resources remain allocated.
  bool pause() {
    running = false;
    const bool stopped = i2s.stop();
    i2s_ok = false;
    return stopped;
  }

  bool stop() {
    const bool stopped = pause();
    // A stopped/restarted capture must start its analysis timeline from zero.
    // Otherwise stale frame/sample counters can suppress analysis after I2S
    // has recovered from a stalled DMA stream.
    sample_count = 0;
    frames_done = 0;
    write_index = 0;
    channel_word = 0;
    return stopped;
  }

  void resetStatistics() {
    statsRMS.Reset();
    statsDB.Reset();
    short_average.Reset();
    has_level = false;
  }

  void update() {
    if (!running) return;
    drainI2S();
    const uint32_t intervals = sample_count / (AUDIO_SAMPLE_RATE_HZ / FPS);
    if (frames_done < intervals) {
      // Keep up with the stream by analyzing the newest complete window.
      frames_done = intervals;
      if (sample_count >= DATA_SIZE + (AUDIO_SAMPLE_RATE_HZ / FPS) * 2) analyzeLatestWindow();
    }
  }

  bool hasLevelMeasurement() const { return has_level; }
  uint32_t samplesRead() const { return sample_count; }

  void takeI2SDiagnostics(uint32_t &availablePeak, uint32_t &bytesRead,
                          uint32_t &readPeak, uint32_t &wordsRead,
                          uint32_t &selectedSamples, uint32_t &checksum,
                          int32_t &rawMin, int32_t &rawMax) {
    noInterrupts();
    availablePeak = diag_available_peak;
    bytesRead = diag_bytes;
    readPeak = diag_read_peak;
    wordsRead = diag_words;
    selectedSamples = diag_selected;
    checksum = diag_checksum;
    rawMin = diag_min;
    rawMax = diag_max;
    diag_available_peak = diag_bytes = diag_read_peak = diag_words = diag_selected = 0;
    diag_checksum = 2166136261UL;
    diag_min = INT32_MAX;
    diag_max = INT32_MIN;
    interrupts();
  }

private:
  Stream &serial;
  PicoI2SInput i2s;
  bool i2s_ok = false;
  bool windowInitialized = false;
  bool running = false;
  bool has_level = false;
  // Both counters reset after each produced five-minute measurement.
  uint32_t sample_count = 0;
  uint32_t frames_done = 0;
  uint16_t write_index = 0;
  uint8_t channel_word = 0;
  int32_t ring[BUFFER_SIZE] = {};
  int32_t input_chunk[SLM_I2S_READ_WORDS] = {};
  float hann[DATA_SIZE];
  float fft_real[DATA_SIZE];
  float fft_imag[DATA_SIZE];
  RunningAvg short_average;
  uint32_t diag_available_peak = 0, diag_bytes = 0, diag_read_peak = 0;
  uint32_t diag_words = 0, diag_selected = 0, diag_checksum = 2166136261UL;
  int32_t diag_min = INT32_MAX, diag_max = INT32_MIN;

  void drainI2S() {
    size_t available = i2s.available();
    if (available > diag_available_peak) diag_available_peak = available;
    if (available > sizeof(input_chunk)) available = sizeof(input_chunk);
    // Read whole stereo frames so left/right channel phase stays aligned.
    available -= available % (2 * sizeof(int32_t));
    if (!available) return;
    size_t count = i2s.read(reinterpret_cast<uint8_t *>(input_chunk), available);
    diag_bytes += count;
    if (count > diag_read_peak) diag_read_peak = count;
    count -= count % (2 * sizeof(int32_t));
    diag_words += count / sizeof(int32_t);
    for (size_t i = 0; i < count / sizeof(int32_t); ++i) {
      if (channel_word == 1) { // SPH0645 SEL high selects right channel.
        const int32_t value = input_chunk[i] & static_cast<int32_t>(0xFFFFC000UL);
        const uint32_t raw = static_cast<uint32_t>(value);
        for (uint8_t shift = 0; shift < 32; shift += 8) {
          diag_checksum ^= static_cast<uint8_t>(raw >> shift);
          diag_checksum *= 16777619UL;
        }
        if (value < diag_min) diag_min = value;
        if (value > diag_max) diag_max = value;
        ring[write_index] = value;
        write_index = (write_index + 1) % BUFFER_SIZE;
        ++sample_count;
        ++diag_selected;
      }
      channel_word ^= 1;
    }
  }

  void fft() {
    // In-place radix-2 Cooley-Tukey complex FFT.
    uint16_t j = 0;
    for (uint16_t i = 1; i < DATA_SIZE; ++i) {
      uint16_t bit = DATA_SIZE >> 1;
      while (j & bit) { j ^= bit; bit >>= 1; }
      j ^= bit;
      if (i < j) {
        float t = fft_real[i]; fft_real[i] = fft_real[j]; fft_real[j] = t;
        t = fft_imag[i]; fft_imag[i] = fft_imag[j]; fft_imag[j] = t;
      }
    }
    for (uint16_t len = 2; len <= DATA_SIZE; len <<= 1) {
      const float angle = -2.0f * PI / len;
      const float wlen_r = cosf(angle), wlen_i = sinf(angle);
      for (uint16_t base = 0; base < DATA_SIZE; base += len) {
        float wr = 1.0f, wi = 0.0f;
        const uint16_t half = len >> 1;
        for (uint16_t k = 0; k < half; ++k) {
          const uint16_t even = base + k, odd = even + half;
          const float vr = fft_real[odd] * wr - fft_imag[odd] * wi;
          const float vi = fft_real[odd] * wi + fft_imag[odd] * wr;
          fft_real[odd] = fft_real[even] - vr;
          fft_imag[odd] = fft_imag[even] - vi;
          fft_real[even] += vr;
          fft_imag[even] += vi;
          const float next_wr = wr * wlen_r - wi * wlen_i;
          wi = wr * wlen_i + wi * wlen_r;
          wr = next_wr;
        }
      }
    }
  }

  void analyzeLatestWindow() {
    uint16_t pos = (write_index + BUFFER_SIZE - DATA_SIZE) % BUFFER_SIZE;
    for (uint16_t n = 0; n < DATA_SIZE; ++n) {
      // I2S values are left-aligned signed 18-bit microphone samples.
      fft_real[n] = (static_cast<float>(ring[pos]) / 2147483648.0f) * hann[n];
      fft_imag[n] = 0.0f;
      pos = (pos + 1) % BUFFER_SIZE;
    }
    fft();

    // Parseval energy, corrected for the Hann window, A weighting, and the
    // SPH0645 datasheet response table also used in the Amsterdam project.
    double weighted_power = 0.0;
    const double scale = 2.0 / (static_cast<double>(DATA_SIZE) * DATA_SIZE * (3.0 / 8.0));
    for (uint16_t bin = START_BIN; bin < DATA_SIZE / 2; ++bin) {
      const double power = static_cast<double>(fft_real[bin]) * fft_real[bin]
                         + static_cast<double>(fft_imag[bin]) * fft_imag[bin];
      weighted_power += power * CORRECTED_DBS_FLOAT[bin] * scale;
    }
    if (weighted_power < 1.0e-20) weighted_power = 1.0e-20;
    const double short_power = short_average.Update(weighted_power);
    latestLevelRelative = 120.0 + 10.0 * log10(short_power * 2.0);
    statsDB.Update(latestLevelRelative);
    statsRMS.Update(weighted_power);
    has_level = true;
  }
};

#endif

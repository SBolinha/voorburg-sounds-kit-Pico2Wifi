#ifndef VOORBURG_EVENT_LOG_H
#define VOORBURG_EVENT_LOG_H

#include <Arduino.h>
#include <time.h>
#include <stdio.h>
#include <string.h>

// Format complete lines in RAM. USB backpressure must not hold measurement
// or network processing in Serial.write's wait-for-space loop.
class EventLog : public Print {
public:
  using Print::print;
  using Print::println;

  size_t write(uint8_t value) override {
    if (value == '\r') return 1;
    if (value == '\n') {
      if (droppingLine) {
        countDroppedLine();
      } else {
        line[lineLength++] = '\r';
        line[lineLength++] = '\n';
        if (lineLength > QUEUE_CAPACITY - queuedBytes) {
          countDroppedLine();
        } else {
          for (size_t i = 0; i < lineLength; ++i) {
            queue[writePosition] = line[i];
            writePosition = (writePosition + 1) % QUEUE_CAPACITY;
          }
          queuedBytes += lineLength;
        }
      }
      lineLength = 0;
      droppingLine = false;
      return 1;
    }
    if (droppingLine) return 1;
    if (lineLength == 0) {
      writeTimestamp();
    }
    if (lineLength >= LINE_CAPACITY - 2) {
      lineLength = 0;
      droppingLine = true;
    } else {
      line[lineLength++] = value;
    }
    // Arduino Print stops a string on a zero return. Consume dropped logical
    // bytes too, so the newline always reaches this logger and restores state.
    return 1;
  }

  void service() {
    if (!Serial) {
      usbConnected = false;
      sentFromLine = 0;
      return;
    }
    if (!usbConnected) {
      usbConnected = true;
      separatorSent = 0;
      sentFromLine = 0;
    }
    size_t budget = 64;
    while (budget) {
      const int available = Serial.availableForWrite();
      if (available <= 0) return;
      size_t count = static_cast<size_t>(available);
      if (count > budget) count = budget;
      if (separatorSent < 2) {
        if (count > 2 - separatorSent) count = 2 - separatorSent;
        const size_t written = Serial.write(
          reinterpret_cast<const uint8_t *>("\r\n") + separatorSent, count);
        if (!written || written > count) return;
        separatorSent += written;
        budget -= written;
        continue;
      }
      if (!queuedBytes) break;
      if (!frontLineLength) {
        // Only complete lines are committed to the ring.
        while (frontLineLength < queuedBytes) {
          const uint8_t value = queue[(readPosition + frontLineLength) % QUEUE_CAPACITY];
          ++frontLineLength;
          if (value == '\n') break;
        }
      }
      const size_t position = (readPosition + sentFromLine) % QUEUE_CAPACITY;
      const size_t remaining = frontLineLength - sentFromLine;
      if (count > remaining) count = remaining;
      if (count > QUEUE_CAPACITY - position) count = QUEUE_CAPACITY - position;
      const size_t written = Serial.write(queue + position, count);
      if (!written || written > count) return;
      sentFromLine += written;
      budget -= written;
      if (sentFromLine == frontLineLength) {
        readPosition = (readPosition + frontLineLength) % QUEUE_CAPACITY;
        queuedBytes -= frontLineLength;
        frontLineLength = 0;
        sentFromLine = 0;
      }
    }
    if (droppedLines && !lineLength && !droppingLine &&
        QUEUE_CAPACITY - queuedBytes >= 128) {
      char notice[96];
      snprintf(notice, sizeof(notice),
        "USB log buffer full; %lu complete log lines dropped; device processing continued",
        static_cast<unsigned long>(droppedLines));
      droppedLines = 0;
      println(notice);
    }
  }

private:
  static const size_t QUEUE_CAPACITY = 8192;
  static const size_t LINE_CAPACITY = 2048;
  uint8_t queue[QUEUE_CAPACITY] = {};
  uint8_t line[LINE_CAPACITY] = {};
  size_t queuedBytes = 0;
  size_t readPosition = 0;
  size_t writePosition = 0;
  size_t lineLength = 0;
  size_t frontLineLength = 0;
  size_t sentFromLine = 0;
  size_t separatorSent = 0;
  uint32_t droppedLines = 0;
  bool droppingLine = false;
  bool usbConnected = false;

  void countDroppedLine() {
    if (droppedLines != UINT32_MAX) ++droppedLines;
  }

  void writeTimestamp() {
    char prefix[40];
    const time_t current = time(nullptr);
    if (current >= 1704067200) { // 2024-01-01; distinguishes a synced clock.
      struct tm utc;
      if (gmtime_r(&current, &utc)) {
        char timestamp[24];
        if (strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%SZ", &utc)) {
          snprintf(prefix, sizeof(prefix), "[%s] ", timestamp);
          const size_t length = strlen(prefix);
          memcpy(line, prefix, length);
          lineLength = length;
          return;
        }
      }
    }

    snprintf(prefix, sizeof(prefix), "[uptime+%lums] ",
      static_cast<unsigned long>(millis()));
    const size_t length = strlen(prefix);
    memcpy(line, prefix, length);
    lineLength = length;
  }
};

extern EventLog eventLog;

#endif

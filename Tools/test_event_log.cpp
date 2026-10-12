// c++ -std=c++11 -Wall -Wextra -Werror -pedantic -ITools/host_log_stubs
//   Tools/test_event_log.cpp -o /tmp/voorburg-log-test
#include <assert.h>
#include <stdio.h>
#include <string>
#include "../Arduino/VoorburgSoundsKit/EventLog.h"

HostSerial Serial;

static void drain(EventLog &log) {
  Serial.connected = true;
  for (unsigned i = 0; i < 1000; ++i) {
    const size_t old = Serial.output.size();
    Serial.capacity = 128;
    log.service();
    if (Serial.output.size() == old) break;
  }
}

static void assertCarriageReturns(const std::string &text) {
  for (size_t i = 0; i < text.size(); ++i)
    if (text[i] == '\n') assert(i && text[i - 1] == '\r');
}

int main() {
  EventLog log;
  log.println("Opening route");
  assert(Serial.calls == 0); // Formatting never touches USB.
  log.service();
  assert(Serial.calls == 0); // Disconnected host never receives a write.
  Serial.connected = true;
  log.service();
  assert(Serial.calls == 0); // Connected host with a full FIFO also gets no write.
  drain(log);
  assert(Serial.output.find("Opening route\r\n") != std::string::npos);
  assertCarriageReturns(Serial.output);

  // A partial application line is kept out of the output queue.
  Serial.output.clear();
  log.print("not yet complete");
  drain(log);
  assert(Serial.output.empty());
  log.println();
  drain(log);
  assert(Serial.output.find("not yet complete\r\n") != std::string::npos);

  // Replay the current complete line on reconnect instead of continuing its
  // interrupted suffix in the new terminal session.
  const std::string longLine(200, 'R');
  log.println(longLine.c_str());
  Serial.output.clear();
  Serial.capacity = 64;
  log.service();
  assert(Serial.output.size() == 64);
  Serial.connected = false;
  log.service();
  Serial.output.clear();
  drain(log);
  assert(Serial.output.find(longLine + "\r\n") != std::string::npos);
  assertCarriageReturns(Serial.output);

  // Full queues and overlong lines drop whole records, never block or leave
  // fragments for the following line.
  Serial = HostSerial();
  EventLog crowded;
  const std::string row(200, 'Q');
  for (unsigned i = 0; i < 100; ++i) crowded.println(row.c_str());
  assert(Serial.calls == 0);
  drain(crowded);
  assert(Serial.output.find("complete log lines dropped") != std::string::npos);
  assertCarriageReturns(Serial.output);
  const std::string overlong(3000, 'X');
  crowded.println(overlong.c_str());
  crowded.println("following line");
  Serial.output.clear();
  drain(crowded);
  assert(Serial.output.find('X') == std::string::npos);
  assert(Serial.output.find("following line\r\n") != std::string::npos);

  // Short or temporarily failed writes preserve the queued line.
  Serial = HostSerial();
  Serial.connected = true;
  EventLog retry;
  retry.println("short writes recover");
  Serial.capacity = 128;
  Serial.returnZero = true;
  retry.service();
  assert(Serial.output.empty());
  Serial.returnZero = false;
  Serial.shortWrite = true;
  drain(retry);
  assert(Serial.output.find("short writes recover\r\n") != std::string::npos);
  for (unsigned i = 0; i < 200; ++i) {
    retry.println("queue wrap");
    drain(retry);
  }
  assertCarriageReturns(Serial.output);
  puts("USB log checks passed: disconnected/full host, complete lines, overflow, reconnect, short writes and ring wrap");
}

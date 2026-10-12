#ifndef VOORBURG_PICO_DATAGRAM_SOCKET_H
#define VOORBURG_PICO_DATAGRAM_SOCKET_H

#include <Arduino.h>
#include <IPAddress.h>
#include <lwip_wrap.h>
#include <lwip/ip_addr.h>
#include <atomic>
#include <string.h>

// The Wi-Fi callback copies complete datagrams into fixed storage and releases
// every received pbuf immediately. The foreground never traverses a pbuf chain
// or acquires a network lock to poll/read a packet. One producer (lwIP) and one
// consumer (the sketch loop) exchange slots with release/acquire atomics.
class PicoDatagramSocket {
public:
  static const size_t MAX_DATAGRAM = 1024;
  static const uint32_t QUEUE_DEPTH = 4;

  PicoDatagramSocket() = default;
  PicoDatagramSocket(const PicoDatagramSocket &) = delete;
  PicoDatagramSocket &operator=(const PicoDatagramSocket &) = delete;

  uint8_t begin(uint16_t port) {
    bool opened = false;
    networkCall([this, port, &opened]() {
      closeInNetworkContext();
      pcb = udp_new_ip_type(IPADDR_TYPE_V4);
      if (!pcb) return;
      if (udp_bind(pcb, IP_ADDR_ANY, port) != ERR_OK) {
        udp_remove(pcb);
        pcb = nullptr;
        return;
      }
      udp_recv(pcb, receive, this);
      opened = true;
    });
    return opened ? 1 : 0;
  }

  void stop() {
    networkCall([this]() { closeInNetworkContext(); });
  }

  int beginPacket(const IPAddress &address, uint16_t port) {
    if (!port) return 0;
    bool prepared = false;
    networkCall([this, &address, port, &prepared]() {
      if (!pcb) return;
      // Do not udp_connect(): the port-1700 listener must remain bidirectional.
      ip_addr_copy(peerAddress, *static_cast<const ip_addr_t *>(address));
      peerPort = port;
      peerKnown = true;
      txLength = 0;
      prepared = true;
    });
    return prepared ? 1 : 0;
  }

  size_t write(const uint8_t *bytes, size_t length) {
    if (!bytes || length > MAX_DATAGRAM - txLength) return 0;
    memcpy(txBytes + txLength, bytes, length);
    txLength += length;
    return length;
  }

  int endPacket() {
    bool sent = false;
    networkCall([this, &sent]() {
      if (!pcb || !peerKnown || !txLength) return;
      pbuf *packet = pbuf_alloc(PBUF_TRANSPORT,
        static_cast<u16_t>(txLength), PBUF_RAM);
      if (!packet) return;
      if (pbuf_take(packet, txBytes, static_cast<u16_t>(txLength)) == ERR_OK) {
        sent = udp_sendto(pcb, packet, &peerAddress, peerPort) == ERR_OK;
      }
      pbuf_free(packet);
    });
    txLength = 0;
    return sent ? 1 : 0;
  }

  int parsePacket() {
    currentLength = currentOffset = 0;
    const uint32_t tail = consumed.load(std::memory_order_relaxed);
    const uint32_t head = produced.load(std::memory_order_acquire);
    if (tail == head) return 0;
    const Slot &slot = slots[tail % QUEUE_DEPTH];
    currentLength = slot.length;
    memcpy(currentBytes, slot.bytes, currentLength);
    // The producer can reuse the slot only after the copy has completed.
    consumed.store(tail + 1, std::memory_order_release);
    return static_cast<int>(currentLength);
  }

  int read(uint8_t *bytes, size_t length) {
    if (!bytes) return 0;
    const size_t available = currentLength - currentOffset;
    if (length > available) length = available;
    memcpy(bytes, currentBytes + currentOffset, length);
    currentOffset += length;
    return static_cast<int>(length);
  }

  uint32_t droppedPackets() const {
    return dropped.load(std::memory_order_relaxed);
  }

private:
  // Pico 2 W (RP2350) supports lock-free 32-bit atomics. Receive callbacks must
  // never call a hidden mutex implementation while the foreground is paused.
  static_assert(ATOMIC_INT_LOCK_FREE == 2, "Pico UDP queue needs lock-free 32-bit atomics");
  struct Slot { uint16_t length = 0; uint8_t bytes[MAX_DATAGRAM] = {}; };
  Slot slots[QUEUE_DEPTH];
  std::atomic<uint32_t> produced{0}, consumed{0}, dropped{0};
  udp_pcb *pcb = nullptr;
  ip_addr_t peerAddress = {};
  uint16_t peerPort = 0;
  bool peerKnown = false;
  uint8_t currentBytes[MAX_DATAGRAM] = {}, txBytes[MAX_DATAGRAM] = {};
  size_t currentLength = 0, currentOffset = 0, txLength = 0;

  template <typename Callback>
  static void networkCall(Callback callback) {
    // Dispatch synchronously under FreeRTOS. In normal Arduino-Pico mode the
    // recursive SDK lock excludes the Wi-Fi background callback. No logging,
    // delays, application handlers or receive-queue polling happen here.
    lwip_callback([&callback]() {
      LWIPMutex lock;
      callback();
    });
  }

  void closeInNetworkContext() {
    if (pcb) {
      udp_recv(pcb, nullptr, nullptr);
      udp_remove(pcb);
      pcb = nullptr;
    }
    peerKnown = false;
    peerPort = 0;
    txLength = currentLength = currentOffset = 0;
    produced.store(0, std::memory_order_relaxed);
    consumed.store(0, std::memory_order_relaxed);
  }

  static void receive(void *argument, udp_pcb *sourcePcb, pbuf *packet,
                      const ip_addr_t *address, u16_t port) {
    if (!packet) return;
    PicoDatagramSocket *self = static_cast<PicoDatagramSocket *>(argument);
    if (!self || sourcePcb != self->pcb || !self->peerKnown ||
        !address || port != self->peerPort ||
        !ip_addr_cmp(address, &self->peerAddress)) {
      pbuf_free(packet);
      return;
    }
    const uint32_t head = self->produced.load(std::memory_order_relaxed);
    const uint32_t tail = self->consumed.load(std::memory_order_acquire);
    if (!packet->tot_len || packet->tot_len > MAX_DATAGRAM ||
        head - tail >= QUEUE_DEPTH) {
      self->dropped.fetch_add(1, std::memory_order_relaxed);
    } else {
      Slot &slot = self->slots[head % QUEUE_DEPTH];
      const u16_t copied = pbuf_copy_partial(packet, slot.bytes, packet->tot_len, 0);
      if (copied == packet->tot_len) {
        slot.length = copied;
        self->produced.store(head + 1, std::memory_order_release);
      } else {
        self->dropped.fetch_add(1, std::memory_order_relaxed);
      }
    }
    // No packet chains are retained or joined to packets from another arrival.
    pbuf_free(packet);
  }
};

#endif

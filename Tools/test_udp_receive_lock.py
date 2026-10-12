#!/usr/bin/env python3
"""Exercise the actual Pico UDP backend and sketch polling without TTN traffic."""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
header = (root / "Arduino/VoorburgSoundsKit/PicoDatagramSocket.h").read_text()
client = (root / "Arduino/VoorburgSoundsKit/TTNSemtechUDPClient.h").read_text()
# Host definitions replace SDK includes only; all socket method bodies are real.
header = re.sub(r"^#include <(?:Arduino.h|IPAddress.h|lwip_wrap.h|lwip/ip_addr.h)>\n", "", header, flags=re.M)
methods = []
for name in ("parsePacketSafely", "readPacketSafely", "processIncomingPackets", "processPushAcknowledgements", "drainPacket", "drainListenerPacket"):
    start = re.search(r"^  (?:static )?(?:int|void) " + name + r"\(", client, re.M).start()
    opening = client.index("{", start)
    end, depth = opening + 1, 1
    while depth:
        depth += (client[end] == "{") - (client[end] == "}")
        end += 1
    method = client[start:end]
    assert "LWIPMutex" not in method and "lwip_callback" not in method
    methods.append(method)
assert "WiFiUDP " not in client

prefix = r'''
#include <algorithm>
#include <atomic>
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <thread>
#include <vector>
using u16_t = uint16_t;
using err_t = int;
static const err_t ERR_OK = 0, ERR_MEM = -1;
static const int IPADDR_TYPE_V4 = 0, PBUF_TRANSPORT = 0, PBUF_RAM = 0;
struct ip_addr_t { uint32_t value; };
static const ip_addr_t any = {0};
#define IP_ADDR_ANY (&any)
#define ip_addr_copy(dst, src) ((dst) = (src))
static bool ip_addr_cmp(const ip_addr_t *a, const ip_addr_t *b) { return a->value == b->value; }
struct IPAddress {
    ip_addr_t address;
    explicit IPAddress(uint32_t value) : address{value} {}
    operator const ip_addr_t *() const { return &address; }
};
struct pbuf { uint16_t tot_len; uint16_t len; uint8_t *payload; pbuf *next; };
struct udp_pcb;
using Receive = void (*)(void *, udp_pcb *, pbuf *, const ip_addr_t *, uint16_t);
struct udp_pcb { Receive callback = nullptr; void *argument = nullptr; uint16_t port = 0; };
static thread_local int networkDepth = 0;
static thread_local bool inNetworkTask = false;
static std::atomic<unsigned> enters{0}, allocations{0}, frees{0}, pcbs{0};
static udp_pcb *latestPcb = nullptr;
static bool failNew = false, failBind = false, failAlloc = false, failTake = false, failSend = false, failCopy = false;
static std::vector<uint8_t> sentBytes;
static void requireNetwork() { assert(networkDepth || inNetworkTask); }
struct LWIPMutex {
    LWIPMutex() {
#if !TEST_FREERTOS
        ++networkDepth;
#endif
    }
    ~LWIPMutex() {
#if !TEST_FREERTOS
        --networkDepth;
#endif
    }
};
static void lwip_callback(std::function<void()> callback) {
    ++enters;
#if TEST_FREERTOS
    const bool previous = inNetworkTask;
    inNetworkTask = true;
    callback();
    inNetworkTask = previous;
#else
    callback();
#endif
}
static udp_pcb *udp_new_ip_type(int) {
    requireNetwork();
    if (failNew) return nullptr;
    ++pcbs;
    return latestPcb = new udp_pcb;
}
static err_t udp_bind(udp_pcb *pcb, const ip_addr_t *, uint16_t port) {
    requireNetwork(); pcb->port = port; return failBind ? ERR_MEM : ERR_OK;
}
static void udp_recv(udp_pcb *pcb, Receive callback, void *argument) {
    requireNetwork(); pcb->callback = callback; pcb->argument = argument;
}
static void udp_remove(udp_pcb *pcb) { requireNetwork(); --pcbs; delete pcb; }
static pbuf *allocatePacket(size_t length) {
    ++allocations;
    return new pbuf{static_cast<uint16_t>(length), static_cast<uint16_t>(length), new uint8_t[length], nullptr};
}
static pbuf *pbuf_alloc(int, uint16_t length, int) {
    requireNetwork(); return failAlloc ? nullptr : allocatePacket(length);
}
static err_t pbuf_take(pbuf *packet, const void *bytes, uint16_t length) {
    requireNetwork();
    if (failTake) return ERR_MEM;
    std::memcpy(packet->payload, bytes, length); return ERR_OK;
}
static void pbuf_free(pbuf *packet) {
    requireNetwork();
    while (packet) { auto next = packet->next; ++frees; delete[] packet->payload; delete packet; packet = next; }
}
static uint16_t pbuf_copy_partial(const pbuf *packet, void *out, uint16_t length, uint16_t offset) {
    requireNetwork();
    if (failCopy) return 0;
    uint16_t copied = 0;
    for (; packet && copied < length; packet = packet->next) {
        if (offset >= packet->len) { offset -= packet->len; continue; }
        const uint16_t count = std::min<uint16_t>(length - copied, packet->len - offset);
        std::memcpy(static_cast<uint8_t *>(out) + copied, packet->payload + offset, count);
        copied += count; offset = 0;
    }
    return copied;
}
static err_t udp_sendto(udp_pcb *, pbuf *packet, const ip_addr_t *, uint16_t) {
    requireNetwork();
    if (failSend) return ERR_MEM;
    sentBytes.assign(packet->payload, packet->payload + packet->tot_len); return ERR_OK;
}
static void inject(udp_pcb *pcb, const std::vector<uint8_t> &bytes, uint16_t port = 1700,
                   uint32_t addressValue = 1234, bool segmented = false) {
    pbuf *packet = allocatePacket(bytes.size());
    std::memcpy(packet->payload, bytes.data(), bytes.size());
    if (segmented && bytes.size() > 2) {
        packet->len = 2;
        packet->next = allocatePacket(bytes.size() - 2);
        std::memcpy(packet->next->payload, bytes.data() + 2, bytes.size() - 2);
    }
    const ip_addr_t address = {addressValue};
    lwip_callback([&]() { LWIPMutex lock; pcb->callback(pcb->argument, pcb, packet, &address, port); });
}
'''

receives = r'''
#define F(text) text
class ActualReceives {
public:
    PicoDatagramSocket udp, uplinkUdp;
    static const uint8_t PROTOCOL_VERSION = 2, PULL_ACK = 4, PUSH_ACK = 1, PULL_RESP = 3;
    static const uint8_t MAX_PACKETS_PER_PASS = 4;
    enum OperationStage { OP_ROUTE_RECEIVE, OP_PULL_RESP_PARSE };
    uint16_t pendingPullToken = 1, pendingPushToken = 2;
    bool routeOpen = false, routePreparationReported = true;
    bool waitingForPushAck = true, pushAcknowledged = false;
    unsigned responses = 0, reports = 0;
    static void markOperation(OperationStage) { assert(!networkDepth && !inNetworkTask); }
    static uint16_t readToken(const uint8_t *bytes) { return (bytes[1] << 8) | bytes[2]; }
    void report(const char *) { assert(!networkDepth && !inNetworkTask); ++reports; }
    void receivePullResponse(uint16_t token, size_t length) {
        assert(!networkDepth && !inNetworkTask && token == 5 && length == 2);
        uint8_t bytes[2]; assert(readPacketSafely(udp, bytes, 2) == 2);
        assert(bytes[0] == '{' && bytes[1] == '}'); ++responses;
    }
''' + "\n".join(methods) + "\n};\n"

tests = r''' 
int main() {
    PicoDatagramSocket socket;
    failNew = true; assert(!socket.begin(1700)); failNew = false;
    failBind = true; assert(!socket.begin(1700)); failBind = false;
    assert(pcbs == 0 && socket.begin(1700));
    udp_pcb *listener = latestPcb;
    const IPAddress server(1234);
    inject(listener, {2, 0, 1, 4}); // Unsolicited before peer setup.
    assert(socket.parsePacket() == 0);
    assert(socket.beginPacket(server, 1700));
    const uint8_t header[] = {2, 0, 1, 2};
    const uint8_t eui[] = {2, 192, 25, 17, 45, 210, 16, 135};
    assert(socket.write(header, 4) == 4 && socket.write(eui, 8) == 8);
    assert(socket.endPacket());
    assert(sentBytes == std::vector<uint8_t>({2,0,1,2,2,192,25,17,45,210,16,135}));
    for (auto flag : {&failAlloc, &failTake, &failSend}) {
        assert(socket.beginPacket(server, 1700));
        assert(socket.write(header, sizeof(header)) == sizeof(header));
        *flag = true; assert(!socket.endPacket()); *flag = false;
    }
    assert(allocations == frees);
    inject(listener, {9}, 1700, 9999); inject(listener, {9}, 9999);
    assert(socket.parsePacket() == 0);
    inject(listener, {2, 0, 3, 4}, 1700, 1234, true);
    const unsigned calls = enters;
    assert(socket.parsePacket() == 4);
    uint8_t read[8] = {};
    assert(socket.read(read, 2) == 2 && socket.read(read + 2, 6) == 2);
    assert(std::memcmp(read, "\x02\x00\x03\x04", 4) == 0);
    assert(socket.parsePacket() == 0 && enters == calls); // No SDK entry while polling/reading.
    for (unsigned i = 0; i < 8; ++i) inject(listener, {static_cast<uint8_t>(i)});
    assert(socket.droppedPackets() == 4);
    for (unsigned i = 0; i < 4; ++i) {
        assert(socket.parsePacket() == 1 && socket.read(read, 1) == 1 && read[0] == i);
    }
    inject(listener, std::vector<uint8_t>(1025)); inject(listener, {});
    assert(socket.droppedPackets() == 6 && socket.parsePacket() == 0);
    failCopy = true; inject(listener, {7}); failCopy = false;
    assert(socket.droppedPackets() == 7 && socket.parsePacket() == 0);
    for (unsigned i = 0; i < 25; ++i) {
        inject(listener, {static_cast<uint8_t>(i)});
        assert(socket.parsePacket() == 1 && socket.read(read, 1) == 1 && read[0] == i);
    }
    // Stress the actual SPSC atomics: callback thread runs independently of the loop.
    std::atomic<bool> done{false};
    std::thread receiver([&]() {
        for (uint32_t i = 0; i < 10000; ++i) {
            std::vector<uint8_t> bytes(4);
            std::memcpy(bytes.data(), &i, 4);
            inject(listener, bytes);
        }
        done.store(true, std::memory_order_release);
    });
    uint32_t previous = 0; bool seen = false;
    for (;;) {
        if (socket.parsePacket() == 4) {
            uint32_t value;
            assert(socket.read(reinterpret_cast<uint8_t *>(&value), 4) == 4);
            assert(!seen || value > previous); previous = value; seen = true;
        } else if (done.load(std::memory_order_acquire)) {
            // Check once more after acquiring the producer's completion fence.
            if (!socket.parsePacket()) break;
            uint32_t value;
            assert(socket.read(reinterpret_cast<uint8_t *>(&value), 4) == 4);
            assert(!seen || value > previous); previous = value; seen = true;
        }
    }
    receiver.join(); assert(seen);
    inject(listener, {8}); socket.stop(); assert(socket.parsePacket() == 0);
    assert(pcbs == 0 && allocations == frees);
    assert(socket.begin(0)); assert(socket.beginPacket(server, 1700));
    assert(socket.write(header, 4) == 4 && socket.endPacket());
    socket.stop();
    assert(pcbs == 0 && allocations == frees);
    ActualReceives client;
    assert(client.udp.begin(1700) && client.udp.beginPacket(server, 1700));
    auto down = latestPcb;
    assert(client.uplinkUdp.begin(0) && client.uplinkUdp.beginPacket(server, 1700));
    auto up = latestPcb;
    inject(down, {2,0,7,4}); // Incorrect pull token cannot open route.
    client.processIncomingPackets(); assert(!client.routeOpen);
    inject(down, {2,0,1,4}); inject(down, {2,0,5,3,'{','}'});
    inject(up, {2,0,2,1});
    const unsigned beforePoll = enters;
    client.processIncomingPackets(); client.processPushAcknowledgements();
    assert(client.routeOpen && client.pendingPullToken == 0 && client.pushAcknowledged);
    assert(client.responses == 1 && client.reports == 1 && enters == beforePoll);
    client.udp.stop(); client.uplinkUdp.stop();
    assert(pcbs == 0 && allocations == frees);
    std::puts("PASS: bounded UDP queue, segmented packets, overflow, failure cleanup, concurrent arrivals, GWMP tokens and polling without network locks");
}
'''
with tempfile.TemporaryDirectory(prefix="voorburg-udp-") as temporary:
    path = Path(temporary)
    (path / "test.cpp").write_text(prefix + header + receives + tests)
    for mode in (0, 1):
        executable = path / ("test-" + str(mode))
        subprocess.run(["c++", "-std=c++11", "-Wall", "-Wextra", "-Werror", "-pedantic", "-pthread",
                        "-DTEST_FREERTOS=" + str(mode), str(path / "test.cpp"), "-o", str(executable)], check=True)
        subprocess.run([str(executable)], check=True)

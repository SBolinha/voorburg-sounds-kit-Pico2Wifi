/*
 * Local settings for the Voorburg Pico 2 W prototype.
 * Fill these values locally; do not commit real Wi-Fi passwords or TTN keys.
 */
#ifndef VOORBURG_LORA_SETTINGS_H
#define VOORBURG_LORA_SETTINGS_H

// USB diagnostics for microphone capture and TTN Gateway MQTT status.
#define USB_LOGGING 1
// Set to 1 to print the continuous I2S/microphone diagnostic lines.
#define USB_MIC_LOGGING 0
// Set to 1 to enable the /down functions.
#define TTN_DOWNLINK_ENABLED 0

#define TTN_SEMTECH_UDP_ENABLED 1
#define TTN_GATEWAY_EUI_HEX ""
#define TTN_SEMTECH_UDP_HOST "eu1.cloud.thethings.network"
#define TTN_SEMTECH_UDP_PORT 1700

// Wi-Fi backhaul for the Pico software-gateway client.
static const char WIFI_SSID[] = "YOUR_WIFI_SSID";
static const char WIFI_PASSWORD[] = "YOUR_WIFI_PASSWORD";

// Register a separate generic MQTT gateway in the same TTN cluster as the app.
// This is not the ID of the ordered Indoor Gateway Pro.
static const char TTN_GATEWAY_ID[] = "voorburg-pico-gw";
static const char TTN_TENANT_ID[] = "ttn";
static const char TTN_GATEWAY_API_KEY[] = "YOUR_TTN_GATEWAY_API_KEY";

static const char TTN_MQTT_HOST[] = "eu1.cloud.thethings.network";
static const uint16_t TTN_MQTT_TLS_PORT = 8882;

// The Pico must be registered as its own ABP device. Never share the Feather's
// DevAddr, session keys, or frame-counter stream with this second transmitter.
// Keys are written as 32 hexadecimal characters in Console display order.
static const char LORAWAN_DEVADDR_HEX[] = "01020304";
static const char LORAWAN_NWKSKEY_HEX[] = "00000000000000000000000000000000";
static const char LORAWAN_APPSKEY_HEX[] = "00000000000000000000000000000000";

// EU868 uplink radio metadata used by TTN for the virtual-gateway message.
// No over-the-air transmission happens in this project; these settings describe
// the hypothetical LoRaWAN PHY frame being forwarded over MQTT.
static const uint32_t TTN_UPLINK_FREQUENCY_HZ = 868300000UL;
static const uint8_t TTN_UPLINK_SPREADING_FACTOR = 7;
static const uint32_t TTN_UPLINK_BANDWIDTH_HZ = 125000UL;
static const char TTN_UPLINK_CODING_RATE[] = "4/5";

// Provision a reserved test FCntUp as the starting counter for the Pico device.
static const uint32_t LORAWAN_INITIAL_FCNT_UP = 0;

#endif

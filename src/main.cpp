#include <Arduino.h>
#include "BLEDevice.h"

// ========================
// User configuration
// ========================
#define LED_PIN GPIO_NUM_13
static const int RSSI_MIN = -80; // "worst" RSSI -> LED off
static const int RSSI_MAX = -30;  // "best" RSSI  -> LED full on

#define MAX_LED 100
#define MIN_LED 1
static const char *NAME_PREFIX = "CM3-";

static const uint32_t NO_PACKET_TIMEOUT_MS = 1000; // 1000 ms TTL before fading
volatile uint32_t lastPacketMillis = 0;

#define LED_BUILTIN GPIO_NUM_4

static int currentPWM = 0; // actual LED brightness
static int targetPWM = 0;  // desired LED brightness from RSSI
static uint32_t lastFadeUpdate = 0;

static const uint32_t FADE_INTERVAL_MS = 15; // speed of fade updates
static const int FADE_STEP = 2;              // how many units per update

// Global RSSI state (updated from BLE callback)
volatile int g_latestRSSI = RSSI_MIN;

// Our own BLE address
static BLEAddress *g_myAddress = nullptr;

// Map RSSI in [RSSI_MIN, RSSI_MAX] -> PWM [0, 255]
int rssiToPWM(int rssi)
{
  if (rssi < RSSI_MIN)
    rssi = RSSI_MIN;
  if (rssi > RSSI_MAX)
    rssi = RSSI_MAX;

  float norm = float(rssi - RSSI_MIN) / float(RSSI_MAX - RSSI_MIN); // 0..1
  int pwm = constrain(roundf(norm * MAX_LED), MIN_LED, MAX_LED);
  return pwm;
}

// BLE scan callback: called for every advertisement we see
// Global LED state
volatile bool ledToggleState = false;

// BLE scan callback
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice advertisedDevice) override
  {

    std::string name = advertisedDevice.getName();
    if (name.empty())
      return;

    if (name.rfind(NAME_PREFIX, 0) != 0)
      return;

    if (g_myAddress && advertisedDevice.getAddress().equals(*g_myAddress))
    {
      return; // ignore self
    }

    // Update RSSI
    int rssi = advertisedDevice.getRSSI();
    g_latestRSSI = rssi;
    lastPacketMillis = millis(); // mark time of last valid packet

    // ========================
    // Toggle LED on packet RX
    // ========================
    ledToggleState = !ledToggleState;
    digitalWrite(LED_BUILTIN, ledToggleState ? HIGH : LOW);

    Serial.print("RX from ");
    Serial.print(name.c_str());
    Serial.print(" RSSI: ");
    Serial.println(g_latestRSSI);
  }
};

BLEScan *pBLEScan = nullptr;

String makeDeviceName()
{
  // Use chip MAC to create a unique name: CM3-<MAC>
  uint64_t mac = ESP.getEfuseMac(); // 48-bit MAC in lower bits

  uint8_t macBytes[6];
  for (int i = 0; i < 6; ++i)
  {
    // Extract bytes MSB first for readability
    macBytes[i] = (mac >> (8 * (5 - i))) & 0xFF;
  }

  char name[32];
  snprintf(name, sizeof(name), "CM3-%02X%02X%02X%02X%02X%02X",
           macBytes[0], macBytes[1], macBytes[2],
           macBytes[3], macBytes[4], macBytes[5]);

  return String(name);
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("ESP32 CM3 BLE RSSI -> LED PWM");

  pinMode(LED_PIN, OUTPUT);
  analogWrite(LED_PIN, 5); // LED off initially

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // ========================
  // BLE init + advertising
  // ========================
  String devName = makeDeviceName();
  Serial.print("Device name: ");
  Serial.println(devName);

  BLEDevice::init(devName.c_str());

  // Get our BLE MAC address after init
  g_myAddress = new BLEAddress(BLEDevice::getAddress());
  Serial.print("My BLE address: ");
  Serial.println(g_myAddress->toString().c_str());

  // Create a dummy server/service so we have something to advertise
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService("12345678-1234-1234-1234-1234567890ab");

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(pService->getUUID());
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("Started BLE advertising.");

  // ========================
  // BLE scanning
  // ========================
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true); // active scan = request scan response
  pBLEScan->setInterval(100);    // 100 ms
  pBLEScan->setWindow(80);       // 80 ms (must be <= interval)

  Serial.println("BLE scanner configured.");
}

void updateLedFade()
{
  uint32_t now = millis();
  if (now - lastFadeUpdate < FADE_INTERVAL_MS)
    return;

  lastFadeUpdate = now;

  if (currentPWM < targetPWM)
  {
    currentPWM += FADE_STEP;
    if (currentPWM > targetPWM)
      currentPWM = targetPWM;
  }
  else if (currentPWM > targetPWM)
  {
    currentPWM -= FADE_STEP;
    if (currentPWM < targetPWM)
      currentPWM = targetPWM;
  }

  analogWrite(LED_PIN, currentPWM);
}

void loop()
{
  // Scan for a short period; callback updates g_latestRSSI
  updateLedFade();
  const uint32_t scanTimeSeconds = 1; // 1 s blocking scan
  pBLEScan->start(scanTimeSeconds, false);
  pBLEScan->clearResults(); // free RAM

  updateLedFade();

  // Low-pass filter RSSI and fade out when no packets are received
  static float filteredRSSI = RSSI_MIN;
  int rssi = g_latestRSSI;

  const float alpha = 0.5f; // reaction speed when packets are present (higher = faster response)
  const float beta = 0.2f; // fade-out speed when no packets (smaller = slower fade)

  updateLedFade();

  uint32_t now = millis();
  bool hasRecentPacket = (now - lastPacketMillis) <= NO_PACKET_TIMEOUT_MS;
  if (hasRecentPacket)
  {
    // Normal EWMA filter toward latest received RSSI
    filteredRSSI = alpha * rssi + (1.0f - alpha) * filteredRSSI;
  }
  else
  {
    // No recent packets: exponentially decay toward RSSI_MIN
    filteredRSSI = (1.0f - beta) * filteredRSSI + beta * RSSI_MIN;
  }

  targetPWM = rssiToPWM(int(filteredRSSI));
  updateLedFade();

  Serial.print("Latest RSSI: ");
  Serial.print(rssi);
  Serial.print(" dBm, filtered: ");
  Serial.print(filteredRSSI, 1);
  Serial.print(" dBm -> PWM: ");
  Serial.println(currentPWM);
  updateLedFade();

}

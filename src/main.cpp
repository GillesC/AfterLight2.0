#include <Arduino.h>
#include "BLEDevice.h"
#include "driver/ledc.h"

// ========================
// User configuration
// ========================
#define LED_PIN GPIO_NUM_13
#define LED_BUILTIN GPIO_NUM_4

static const int RSSI_MIN = -80; // "worst" RSSI -> LED off
static const int RSSI_MAX = -30; // "best" RSSI  -> LED full on

static const char *NAME_PREFIX = "CM3-";

static const uint32_t NO_PACKET_TIMEOUT_MS = 1000; // 1000 ms TTL before fading
volatile uint32_t lastPacketMillis = 0;

// ------------------------
// LEDC (hardware PWM) setup
// ------------------------
#define LED_PWM_CHANNEL 0
#define LED_PWM_FREQ 1000
#define LED_PWM_RES 16 

#define MAX_LED ((1 << LED_PWM_RES-1) - 1) // 4095/2
#define MIN_LED 1

static int currentPWM = MIN_LED; // actual LED brightness
static int targetPWM = MIN_LED;  // desired LED brightness from RSSI
static uint32_t lastFadeUpdate = 0;


static const uint32_t FADE_TIME_MS = 500; // Duration of the fade in ms
static int lastPWM = -1;

// Track previous values for print-on-change
static int lastPrintedRSSI = INT_MIN;
static float lastPrintedFilteredRSSI = RSSI_MIN;
static int lastPrintedPWM = -1;



#define BLE_MS_TO_UNITS(ms) \
    (uint16_t)(min(max(((uint32_t)(ms) * 1000UL) / 625UL, 32UL), 16384UL))
#define ADV_MIN_INTERVAL_MS 100
#define ADV_MAX_INTERVAL_MS 100

// Global RSSI state (updated from BLE callback)
volatile int g_latestRSSI = RSSI_MIN;

// Our own BLE address
static BLEAddress *g_myAddress = nullptr;

// Map RSSI in [RSSI_MIN, RSSI_MAX] -> PWM [MIN_LED, MAX_LED]
uint16_t rssiToPWM(int rssi)
{
    if (rssi < RSSI_MIN)
        rssi = RSSI_MIN;
    if (rssi > RSSI_MAX)
        rssi = RSSI_MAX;

    float norm = float(rssi - RSSI_MIN) / float(RSSI_MAX - RSSI_MIN); // 0..1
    uint16_t pwm = constrain(int(roundf(norm * MAX_LED)), MIN_LED, MAX_LED);
    return pwm;
}

// Global LED state for packet RX toggle
volatile bool ledToggleState = false;

// BLE scan callback
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
    void onResult(BLEAdvertisedDevice advertisedDevice) override
    {
        std::string name = advertisedDevice.getName();
        if (name.empty())
            return;

        // Filter on CM3- prefix
        if (name.rfind(NAME_PREFIX, 0) != 0)
            return;

        // Ignore our own advertisements
        if (g_myAddress && advertisedDevice.getAddress().equals(*g_myAddress))
            return;

        // Update RSSI
        int rssi = advertisedDevice.getRSSI();
        g_latestRSSI = rssi;
        lastPacketMillis = millis(); // mark time of last valid packet

        // Toggle on-board LED on packet RX
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
        macBytes[i] = (mac >> (8 * (5 - i))) & 0xFF;
    }

    char name[32];
    snprintf(name, sizeof(name), "CM3-%02X%02X%02X%02X%02X%02X",
             macBytes[0], macBytes[1], macBytes[2],
             macBytes[3], macBytes[4], macBytes[5]);

    return String(name);
}

void updateLedFade()
{
    // Only start a new fade when the target changed
    if (targetPWM == lastPWM)
    {
        return;
    }
    lastPWM = targetPWM;

    // Configure fade from current duty (implicitly) to targetPWM
    ledc_set_fade_with_time(
        LEDC_HIGH_SPEED_MODE,
        (ledc_channel_t)LED_PWM_CHANNEL,
        targetPWM,
        FADE_TIME_MS);

    // Start fade without waiting (non blocking)
    ledc_fade_start(
        LEDC_HIGH_SPEED_MODE,
        (ledc_channel_t)LED_PWM_CHANNEL,
        LEDC_FADE_NO_WAIT);
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("ESP32 CM3 BLE RSSI -> LED PWM (LEDC)");

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // LEDC hardware PWM setup
    // ledcSetup(LED_PWM_CHANNEL, LED_PWM_FREQ, LED_PWM_RES);
    // ledcAttachPin(LED_PIN, LED_PWM_CHANNEL);
    // ledcWrite(LED_PWM_CHANNEL, 0); // start at 0
    // // Install LEDC fade service (needed for hw fading)
    // ledc_fade_func_install(0);

    // LEDC hardware PWM setup
    uint32_t freq = ledcSetup(LED_PWM_CHANNEL, LED_PWM_FREQ, LED_PWM_RES);
    Serial.print("ledcSetup frequency: ");
    Serial.println(freq);
    if (freq == 0) {
        Serial.println("ERROR: ledcSetup failed!");
    }
    
    ledcAttachPin(LED_PIN, LED_PWM_CHANNEL);
    Serial.println("ledcAttachPin done");
    
    ledcWrite(LED_PWM_CHANNEL, 0); // start at 0
    Serial.println("ledcWrite(0) done");
    
    // Install LEDC fade service (needed for hw fading)
    esp_err_t fade_result = ledc_fade_func_install(0);
    Serial.print("ledc_fade_func_install result: ");
    Serial.println(fade_result);
    if (fade_result != ESP_OK) {
        Serial.println("ERROR: ledc_fade_func_install failed!");
    }

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
    pAdvertising->setMinInterval(BLE_MS_TO_UNITS(ADV_MIN_INTERVAL_MS));
    pAdvertising->setMaxInterval(BLE_MS_TO_UNITS(ADV_MAX_INTERVAL_MS));
    BLEDevice::startAdvertising();

    Serial.println("Started BLE advertising.");

    // ========================
    // BLE scanning (non-blocking / continuous)
    // ========================
    pBLEScan = BLEDevice::getScan();
    // true = get duplicates as well, so callback fires continuously
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), true);
    pBLEScan->setActiveScan(true); // active scan = request scan response
    pBLEScan->setInterval(100);    // 100 ms
    pBLEScan->setWindow(50);       // 50 ms (must be <= interval)

    Serial.println("Starting BLE scan (continuous).");
    pBLEScan->start(0, nullptr, false); // continuous, non-blocking
    // Ensure all Serial output is flushed before setup() returns
    Serial.flush();
}

void loop()
{
    // Non-blocking: scan runs in background, we just react to updated state

    // Low-pass filter RSSI and fade out when no packets are received
    static float filteredRSSI = RSSI_MIN;
    int rssi = g_latestRSSI;

    const float alpha = 0.5f; // reaction speed when packets are present (higher = faster response)
    const float beta = 0.2f;  // fade-out speed when no packets (smaller = slower fade)

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

    // Only print when values have changed
    if (rssi != lastPrintedRSSI || filteredRSSI != lastPrintedFilteredRSSI || currentPWM != lastPrintedPWM)
    {
        Serial.print("Latest RSSI: ");
        Serial.print(rssi);
        Serial.print(" dBm, filtered: ");
        Serial.print(filteredRSSI, 1);
        Serial.print(" dBm -> PWM: ");
        Serial.println(currentPWM);
        Serial.flush();

        lastPrintedRSSI = rssi;
        lastPrintedFilteredRSSI = filteredRSSI;
        lastPrintedPWM = currentPWM;
    }

    // give time to background tasks (BLE, WiFi, etc.)
    delay(1);
   
}

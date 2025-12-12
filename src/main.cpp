// ========================
// AfterLight 2.0 - Gilles Callebaut
// ESP32 CM3 BLE RSSI → LED PWM with Flame Effect
// Note: Uses ESP-IDF 2.X due to PlatformIO limitations
// ========================

#include <Arduino.h>
#include "BLEDevice.h"
#include "driver/ledc.h"
#include <math.h>

// ════════════════════════════════════════════════════════════════════
// USER CONFIGURATION
// ════════════════════════════════════════════════════════════════════

#define LED_PIN GPIO_NUM_13              // Main LED PWM output
#define LED_BUILTIN GPIO_NUM_4           // Status LED (toggles on BLE RX)

static const int RSSI_MIN = -100;         // Worst signal → LED off
static const int RSSI_MAX = -40;         // Best signal → LED full brightness
static const char *NAME_PREFIX = "CM3"; // BLE device filter prefix
static const uint32_t NO_PACKET_TIMEOUT_MS = 1000;  // 1 second TTL

// ════════════════════════════════════════════════════════════════════
// ADC (voltage measurement) - IO35 through 2x100k voltage divider
// Voltage divider: Vin ---[100k]---+---[100k]--- GND
//                                   |
//                                IO35 (ADC)
// Measured V at IO35 = Vin * (R2/(R1+R2)) = Vin * 0.5 -> Vin = Vmeas * 2
#define ADC_PIN 35
// ADC reference used for simple conversion (approx, ESP32 ADC not linear)
#define ADC_VREF 3.30f

// ADC read tuning
static const uint8_t ADC_SAMPLES = 16; // average samples
static float lastPrintedVin = -1.0f;
// VIN safety threshold: when measured Vin < VIN_MIN_THRESHOLD, force LED to MIN and
// suspend normal PWM/fade behavior to avoid brownout or indicate low battery.
static const float VIN_MIN_THRESHOLD = 3.2f;
static bool vinOverrideActive = false;
// When measured Vin is below this threshold, reduce maximum LED power by half
static const float VIN_REDUCE_THRESHOLD = 3.5f;
// Last measured Vin (updated by periodic ADC reads)
static float lastMeasuredVin = 0.0f;

// LED PWM CONFIGURATION (LEDC)
// ════════════════════════════════════════════════════════════════════

#define LED_PWM_CHANNEL 0                // Hardware PWM channel
#define LED_PWM_FREQ 1000                // 1 kHz frequency
#define LED_PWM_RES 16                   // 16-bit resolution (0–65535)

#undef MAX_LED
// Runtime-configurable maximum LED PWM. Initialize to full-scale based on resolution.
static uint32_t ORIGINAL_MAX_LED = ((1UL << LED_PWM_RES) - 1);
static uint32_t MAX_LED = ((1UL << LED_PWM_RES) - 1);
#define MIN_LED 2                        // 1 (minimum, not off)

static const uint32_t FADE_TIME_MS = 500;  // Fade transition duration (ms)
static int targetPWM = MIN_LED;            // Next PWM from RSSI
static int lastPWM = -1;                   // Previous PWM (for changes)

// ════════════════════════════════════════════════════════════════════
// FLAME EFFECT PARAMETERS
// ════════════════════════════════════════════════════════════════════

static uint32_t lastFlameUpdate = 0;
static const uint32_t FLAME_UPDATE_INTERVAL_MS = 50;  // Update rate (20 Hz)
static const uint16_t FLAME_VARIATION = 100;          // ±500 PWM variation

// Brownian motion flame state
static int brownianWalk = 0;
static const int16_t BROWNIAN_STEP_MAX = 100;  // Max step size for smooth walk

// Breathing envelope state
static uint32_t breathingPhase = 0;
static const uint32_t BREATHING_CYCLE_MS = 3000;  // 3-second breath cycle

// Temperature decay state
static uint32_t lastStrongRSSI = 0;
static const uint32_t TEMPERATURE_DECAY_MS = 5000;  // Fade after 5s of weak signal

// Flame effect selector: 0=Brownian, 1=Breathing, 2=TemperatureDecay
static uint8_t activeFlameEffect = 1;  // Default: breathing envelope

// ════════════════════════════════════════════════════════════════════
// RSSI FILTERING & STATE
// ════════════════════════════════════════════════════════════════════

// ─────────────────────────────────────────────────────────────
// RSSI HISTORY (last 10 packets) + MAX
// ─────────────────────────────────────────────────────────────
static constexpr uint8_t RSSI_HISTORY_LEN = 10;

volatile int g_rssiHistory[RSSI_HISTORY_LEN];
volatile uint8_t g_rssiWriteIdx = 0; // next write position
volatile uint8_t g_rssiCount = 0;    // number of valid entries (<= LEN)
volatile uint32_t lastPacketMillis = 0;

portMUX_TYPE g_rssiMux = portMUX_INITIALIZER_UNLOCKED;

static int lastPrintedRSSI = INT_MIN;                 // Debug: track print changes
static float lastPrintedFilteredRSSI = RSSI_MIN;
static int lastPrintedPWM = -1;

// ════════════════════════════════════════════════════════════════════
// BLE CONFIGURATION
// ════════════════════════════════════════════════════════════════════

#define BLE_MS_TO_UNITS(ms) \
    (uint16_t)(min(max(((uint32_t)(ms) * 1000UL) / 625UL, 32UL), 16384UL))
#define ADV_MIN_INTERVAL_MS 100
#define ADV_MAX_INTERVAL_MS 100

static BLEAddress *g_myAddress = nullptr;    // Our BLE MAC
static BLEScan *pBLEScan = nullptr;          // BLE scan object
volatile bool ledToggleState = false;        // Status LED state


// ════════════════════════════════════════════════════════════════════
// UTILITY FUNCTIONS
// ════════════════════════════════════════════════════════════════════

/**
 * Map RSSI signal strength to PWM brightness
 * Input:  RSSI in [RSSI_MIN, RSSI_MAX] dBm
 * Output: PWM in [MIN_LED, MAX_LED]
 */
// Two mapping implementations. Call the one you want from `loop()`:
//   - `rssiToPWM_Linear(int rssi)`
//   - `rssiToPWM_Gamma(int rssi)`

static inline void rssiHistoryInit()
{
    portENTER_CRITICAL(&g_rssiMux);
    for (uint8_t i = 0; i < RSSI_HISTORY_LEN; ++i)
        g_rssiHistory[i] = RSSI_MIN;
    g_rssiWriteIdx = 0;
    g_rssiCount = 0;
    portEXIT_CRITICAL(&g_rssiMux);
}

static inline void rssiHistoryPush(int rssi)
{
    portENTER_CRITICAL(&g_rssiMux);
    g_rssiHistory[g_rssiWriteIdx] = rssi;
    g_rssiWriteIdx = (uint8_t)((g_rssiWriteIdx + 1) % RSSI_HISTORY_LEN);
    if (g_rssiCount < RSSI_HISTORY_LEN)
        g_rssiCount++;
    portEXIT_CRITICAL(&g_rssiMux);
}

static inline int rssiHistoryMax()
{
    int maxRssi = RSSI_MIN;

    portENTER_CRITICAL(&g_rssiMux);
    uint8_t n = g_rssiCount;
    for (uint8_t i = 0; i < n; ++i)
    {
        int v = g_rssiHistory[i];
        if (v > maxRssi)
            maxRssi = v;
    }
    portEXIT_CRITICAL(&g_rssiMux);

    return maxRssi;
}


// Linear mapping: direct proportional mapping from RSSI to PWM
uint16_t rssiToPWM_Linear(int rssi)
{
    if (rssi < RSSI_MIN) rssi = RSSI_MIN;
    if (rssi > RSSI_MAX) rssi = RSSI_MAX;
    float norm = float(rssi - RSSI_MIN) / float(RSSI_MAX - RSSI_MIN);
    uint32_t span = (MAX_LED > MIN_LED) ? (MAX_LED - MIN_LED) : MAX_LED;
    uint32_t pwm32 = uint32_t(roundf(norm * float(span))) + MIN_LED;
    return (uint16_t)constrain((int)pwm32, MIN_LED, MAX_LED);
}

// Gamma mapping: power curve mapping; gamma > 1 makes curve top-heavy
uint16_t rssiToPWM_Gamma(int rssi, float gamma)
{
    if (rssi < RSSI_MIN) rssi = RSSI_MIN;
    if (rssi > RSSI_MAX) rssi = RSSI_MAX;
    float norm = float(rssi - RSSI_MIN) / float(RSSI_MAX - RSSI_MIN);
    float curved = powf(norm, gamma);
    uint32_t span = (MAX_LED > MIN_LED) ? (MAX_LED - MIN_LED) : MAX_LED;
    uint32_t pwm32 = uint32_t(roundf(curved * float(span))) + MIN_LED;
    return (uint16_t)constrain((int)pwm32, MIN_LED, MAX_LED);
}

// Convenience overload that uses the default gamma (3.0)
uint16_t rssiToPWM_Gamma(int rssi)
{
    return rssiToPWM_Gamma(rssi, 3.0f);
}

// ════════════════════════════════════════════════════════════════════
// FLAME EFFECT IMPLEMENTATIONS
// ════════════════════════════════════════════════════════════════════

/**
 * Brownian Motion Flame: Smooth random walk for organic drift.
 * PWM drifts gradually around targetPWM via small random steps.
 */
void applyFlameEffect_Brownian(int& targetPWM)
{
    // Take a small random step
    int16_t step = random(2 * BROWNIAN_STEP_MAX + 1) - BROWNIAN_STEP_MAX;
    brownianWalk += step;
    
    // Apply walk offset
    int flickeredPWM = targetPWM + brownianWalk;
    flickeredPWM = constrain(flickeredPWM, MIN_LED, MAX_LED);
    targetPWM = flickeredPWM;
}

/**
 * Breathing Envelope Flame: Slow modulation (2–3 sec cycle) with fast flicker on top.
 * Simulates natural flame breathing/pulsing underneath the flicker.
 */
void applyFlameEffect_Breathing(int& targetPWM, uint32_t now)
{
    // Slow breathing: sine-like modulation
    float breathPhase = (float)((now % BREATHING_CYCLE_MS)) / BREATHING_CYCLE_MS * 2.0f * M_PI;
    float BEATHING_AMP = 0.8f;
    float breathModulation = (BEATHING_AMP) + (1-BEATHING_AMP) * (1.0f + sinf(breathPhase)) / 2.0f; // 0.3–1.0 range

    // Fast flicker on top
    int flameVar = random(FLAME_VARIATION + 1) - (FLAME_VARIATION / 2);
    int flickeredPWM = constrain(int(targetPWM * breathModulation) + flameVar, MIN_LED, MAX_LED);
    targetPWM = flickeredPWM;
}

/**
 * Temperature Decay Flame: Gradually dims when RSSI is weak (signal lost).
 * Simulates a flame cooling down when fuel (RSSI) runs low.
 */
void applyFlameEffect_TemperatureDecay(int& targetPWM, uint32_t now)
{
    // Check if we have strong signal
    const int STRONG_RSSI_THRESHOLD = -60;  // Arbitrary "good" signal level
    if (rssiHistoryMax() > STRONG_RSSI_THRESHOLD)
    {
        lastStrongRSSI = now;
    }
    
    // Calculate decay factor based on time since last strong signal
    uint32_t timeSinceStrong = now - lastStrongRSSI;
    float decayFactor = 1.0f;
    if (timeSinceStrong > TEMPERATURE_DECAY_MS)
    {
        decayFactor = 0.1f;  // Dim to 10%
    }
    else if (timeSinceStrong > TEMPERATURE_DECAY_MS / 2)
    {
        // Linear fade from 100% to 10% over the second half of decay window
        decayFactor = 1.0f - 0.9f * float(timeSinceStrong - TEMPERATURE_DECAY_MS / 2) / float(TEMPERATURE_DECAY_MS / 2);
    }
    
    // Apply decay with normal flicker
    int flameVar = random(FLAME_VARIATION + 1) - (FLAME_VARIATION / 2);
    int flickeredPWM = constrain(int(targetPWM * decayFactor) + flameVar, MIN_LED, MAX_LED);
    targetPWM = flickeredPWM;
}

// ════════════════════════════════════════════════════════════════════
// BLE CALLBACK - Device Discovery
// ════════════════════════════════════════════════════════════════════

// BLE scan callback - invoked when advertisements are received
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
    void onResult(BLEAdvertisedDevice advertisedDevice) override
    {
        std::string name = advertisedDevice.getName();
        if (name.empty())
            return;

        // Filter by device name prefix
        if (name.rfind(NAME_PREFIX, 0) != 0)
            return;

        // Ignore our own advertisements
        if (g_myAddress && advertisedDevice.getAddress().equals(*g_myAddress))
            return;

        // Update RSSI and packet timestamp
        int rssi = advertisedDevice.getRSSI();
        rssiHistoryPush(rssi);
        lastPacketMillis = millis();

        // Toggle status LED on packet RX
        ledToggleState = !ledToggleState;
        digitalWrite(LED_BUILTIN, ledToggleState ? HIGH : LOW);

        Serial.print("RX from ");
        Serial.print(name.c_str());
        Serial.print(" RSSI: ");
        Serial.println(rssi);
    }
};

/**
 * Create device name from MAC address
 * Format: CM3-AABBCCDDEEFF
 */
String makeDeviceName()
{
    uint64_t mac = ESP.getEfuseMac();
    uint8_t macBytes[6];
    for (int i = 0; i < 6; ++i)
        macBytes[i] = (mac >> (8 * (5 - i))) & 0xFF;

    char name[32];
    snprintf(name, sizeof(name), "CM3-%02X%02X%02X%02X%02X%02X",
             macBytes[0], macBytes[1], macBytes[2],
             macBytes[3], macBytes[4], macBytes[5]);
    return String(name);
}

/**
 * Update LED fade using hardware PWM fade engine
 * Only starts a new fade if targetPWM has changed
 */
void updateLedFade()
{
    if (targetPWM == lastPWM)
        return;
    
    lastPWM = targetPWM;

    // Randomize fade time between FADE_TIME_MS and 2*FADE_TIME_MS for organic feel
    uint32_t randomFadeTime = FADE_TIME_MS + random(FADE_TIME_MS);

    // Configure fade with smooth transition
    ledc_set_fade_with_time(
        LEDC_HIGH_SPEED_MODE,
        (ledc_channel_t)LED_PWM_CHANNEL,
        targetPWM,
        randomFadeTime);

    // Start fade (non-blocking)
    ledc_fade_start(
        LEDC_HIGH_SPEED_MODE,
        (ledc_channel_t)LED_PWM_CHANNEL,
        LEDC_FADE_NO_WAIT);
}

/**
 * Pseudo-random number generator for flame flicker
 * Uses Linear Congruential Generator (LCG)
 * Returns value in range [-FLAME_VARIATION, +FLAME_VARIATION)
 */
int16_t pseudoRandom(uint32_t seed)
{
    // LCG - fast, deterministic
    seed = (seed * 1103515245 + 12345) & 0x7fffffff;
    return (int16_t)(seed % (2 * FLAME_VARIATION)) - (int16_t)FLAME_VARIATION;
}

/**
 * Read the ADC pin (IO35) through the voltage divider and return the estimated
 * input voltage before the divider (Vin). Uses simple averaging and a fixed
 * VREF; for higher accuracy use esp_adc_cal APIs.
 */
float readVin()
{
    uint32_t acc = 0;
    for (uint8_t i = 0; i < ADC_SAMPLES; ++i)
    {
        acc += analogRead(ADC_PIN);
    }
    float avg = float(acc) / float(ADC_SAMPLES);
    // convert ADC counts (12-bit) to measured voltage at ADC pin
    float vMeas = avg * (ADC_VREF / 4095.0f);
    // account for 2x100k divider (Vmeas = Vin * 0.5)
    float vin = vMeas * 2.0f;
    return vin;
}

// Handle battery/voltage-related logic: periodic ADC read, max-LED adjustment,
// and VIN override behavior. Called from loop() with the current filtered RSSI
// and the current millis() timestamp.
void handleBattery(float filteredRSSI, uint32_t now)
{
    static uint32_t lastAnalogRead = 0;
    const uint32_t ANALOG_READ_INTERVAL_MS = 1000; // 1s
    if (now - lastAnalogRead < ANALOG_READ_INTERVAL_MS)
        return;
    lastAnalogRead = now;

    float vin = readVin();
    lastMeasuredVin = vin;

    // Only print when voltage changes more than 10 mV
    if (fabs(vin - lastPrintedVin) > 0.01f)
    {
        Serial.print("ADC IO35 Vin: ");
        Serial.print(vin, 3);
        Serial.println(" V");
        lastPrintedVin = vin;
    }

    // Adjust MAX_LED according to measured Vin (power saving mode)
    if (!vinOverrideActive)
    {
        if (vin < VIN_REDUCE_THRESHOLD && MAX_LED == ORIGINAL_MAX_LED)
        {
            MAX_LED = ORIGINAL_MAX_LED / 2;
            Serial.println("INFO: Vin low - reducing MAX_LED to half");
        }
        else if (vin >= VIN_REDUCE_THRESHOLD && MAX_LED != ORIGINAL_MAX_LED)
        {
            MAX_LED = ORIGINAL_MAX_LED;
            Serial.println("INFO: Vin sufficient - restoring MAX_LED to full");
        }
    }

    // VIN safety override: if Vin below threshold, force LED to MIN and
    // suspend normal PWM/fade updates to avoid brownout or indicate low battery.
    if (vin < VIN_MIN_THRESHOLD)
    {
        if (!vinOverrideActive)
        {
            vinOverrideActive = true;
            Serial.println("WARNING: Vin below threshold - overriding LED to MIN");
        }
        // Force duty to minimum immediately and prevent fading
        targetPWM = MIN_LED;
        ledcWrite(LED_PWM_CHANNEL, MIN_LED);
        lastPWM = MIN_LED; // ensure fade update won't start
    }
    else
    {
        if (vinOverrideActive)
        {
            // Vin recovered: re-enable normal PWM/fade behavior
            vinOverrideActive = false;
            Serial.println("INFO: Vin recovered - restoring PWM control");
            // recompute base target from filtered RSSI (gamma mapping default)
            targetPWM = rssiToPWM_Gamma(int(filteredRSSI));
        }
    }
}

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println();
    Serial.println("╔════════════════════════════════════════════════════════╗");
    Serial.println("║  AfterLight 2.0: BLE RSSI → LED PWM Flame Effect       ║");
    Serial.println("╚════════════════════════════════════════════════════════╝");

    // ─── GPIO Setup ───
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    // ─── LED PWM (Hardware LEDC) Setup ───
    Serial.println("\n[LED PWM Setup]");
    uint32_t freq = ledcSetup(LED_PWM_CHANNEL, LED_PWM_FREQ, LED_PWM_RES);
    Serial.print("  ledcSetup frequency: ");
    Serial.println(freq);
    if (freq == 0) {
        Serial.println("  ERROR: ledcSetup failed!");
    }
    
    ledcAttachPin(LED_PIN, LED_PWM_CHANNEL);
    Serial.println("  ledcAttachPin: OK");
    
    ledcWrite(LED_PWM_CHANNEL, 0);
    Serial.println("  ledcWrite(0): OK");
    
    esp_err_t fade_result = ledc_fade_func_install(0);
    Serial.print("  ledc_fade_func_install: ");
    Serial.println(fade_result == ESP_OK ? "OK" : "FAILED");

    // ─── ADC Setup for IO35 (voltage divider input) ───
    // Configure ADC resolution and attenuation so the pin can measure up to ~3.3V
    analogReadResolution(12); // 12-bit ADC (0..4095)
    // analogSetCycles(100);
    // analogSetPinAttenuation(ADC_PIN, ADC_11db); // full range
    Serial.print("  ADC configured on pin: ");
    Serial.println(ADC_PIN);

    // ─── BLE Initialization ───
    rssiHistoryInit();
    Serial.println("\n[BLE Setup]");
    String devName = makeDeviceName();
    Serial.print("  Device name: ");
    Serial.println(devName);

    BLEDevice::init(devName.c_str());

    g_myAddress = new BLEAddress(BLEDevice::getAddress());
    Serial.print("  BLE address: ");
    Serial.println(g_myAddress->toString().c_str());

    // ─── BLE Advertising ───
    BLEServer *pServer = BLEDevice::createServer();
    BLEService *pService = pServer->createService("12345678-1234-1234-1234-1234567890ab");
    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    BLEUUID svcUUID = pService->getUUID();
    Serial.print("  Service UUID: ");
    Serial.println(svcUUID.toString().c_str());
    pAdvertising->addServiceUUID(svcUUID);
    
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0xFA); // 0x06 // Conn_Interval_Min * 1.25 ms
    pAdvertising->setMaxPreferred(0x1FE); // 0x12
    BLEDevice::startAdvertising();
    Serial.println("  Advertising: Started");

    // ─── BLE Scanning (continuous, non-blocking) ───
    Serial.println("\n[BLE Scan Setup]");
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), true);
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);
    pBLEScan->start(0, nullptr, false);
    Serial.println("  Scanning: Started");

    Serial.println("\n[Setup Complete] Ready for BLE packets...\n");
    Serial.flush();
}

void loop()
{
    // ─── RSSI Filtering (Exponential Weighted Moving Average) ───
    static float filteredRSSI = RSSI_MIN;
    int rssi = rssiHistoryMax();

    const float alpha = 0.5f;  // Fast response when packets arrive
    const float beta = 0.2f;   // Slow fade when no packets

    uint32_t now = millis();
    bool hasRecentPacket = (now - lastPacketMillis) <= NO_PACKET_TIMEOUT_MS;

    if (hasRecentPacket)
    {
        // Normal filter: track incoming RSSI
        filteredRSSI = alpha * rssi + (1.0f - alpha) * filteredRSSI;
    }
    else
    {
        // Fade out: decay toward minimum when no packets
        filteredRSSI = (1.0f - beta) * filteredRSSI + beta * RSSI_MIN;
    }

    // ─── Convert filtered RSSI to base PWM brightness ───
    targetPWM = rssiToPWM_Gamma(int(filteredRSSI));

    // ─── Flame Effect: Apply selected effect (Brownian, Breathing, or TemperatureDecay) ───
    if (now - lastFlameUpdate >= FLAME_UPDATE_INTERVAL_MS)
    {
        lastFlameUpdate = now;
        
        // Call the active flame effect function
        if (activeFlameEffect == 0)
        {
            applyFlameEffect_Brownian(targetPWM);
        }
        else if (activeFlameEffect == 1)
        {
            applyFlameEffect_Breathing(targetPWM, now);
        }
        else if (activeFlameEffect == 2)
        {
            applyFlameEffect_TemperatureDecay(targetPWM, now);
        }
    }

    // ─── (LED fade will be applied after VIN override check) ───

    // ─── Battery management (ADC read, MAX_LED, override) ───
    handleBattery(filteredRSSI, now);

    // ─── Apply LED fade (unless VIN override is active) ───
    if (!vinOverrideActive)
    {
        updateLedFade();
    }

    // ─── Debug Output (only when values change) ───
    if (rssi != lastPrintedRSSI || 
        filteredRSSI != lastPrintedFilteredRSSI || 
        targetPWM != lastPrintedPWM)
    {
        Serial.print("RSSI: ");
        Serial.print(rssi);
        Serial.print(" dBm | Filtered: ");
        Serial.print(filteredRSSI, 1);
        Serial.print(" dBm | LED: ");
        Serial.print(float(targetPWM) / MAX_LED * 100.0f);
        Serial.println("%");
        Serial.flush();

        lastPrintedRSSI = rssi;
        lastPrintedFilteredRSSI = filteredRSSI;
        lastPrintedPWM = targetPWM;
    }
    // Give time to background tasks (BLE, WiFi, etc.)
    delay(10);
}

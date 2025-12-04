# AfterLight2.0

ESP32-based BLE RSSI -> LED PWM demonstrator with a flame-like effect and battery monitoring.

**What it does**
- **BLE Scan & Advertise:** Advertises a dummy service and continuously scans for nearby BLE devices matching a name prefix.
- **RSSI → Brightness:** Maps received RSSI values to LED brightness using the ESP32 LEDC hardware PWM and smooth fading.
- **Flame Effect:** Adds a pseudo-random flicker (pulsing) around the RSSI-derived brightness to simulate a flame/candle look.
- **Battery Monitoring (IO35):** Reads a voltage divider (2×100k) on `IO35` to estimate input voltage (Vin).
- **Safety Overrides:** If Vin falls below a low threshold the firmware forces the LED to minimum; if Vin is moderately low it reduces the maximum LED power by half.

**Included features**
- **Hardware PWM (LEDC):** Smooth fades using the LEDC fade API.
- **EWMA Filtering:** Exponential moving average applied to RSSI for stable brightness control.
- **Dual RSSI→PWM Mappings:** Choose between linear or gamma (power-curve) mapping to control LED response to RSSI.
- **Serial Debugging:** Prints events, RSSI, filtered RSSI, LED percent, service UUID and ADC readings only when values change.
- **Configurable Parameters:** Thresholds, PWM resolution, flame intensity and update rates are defined in `src/main.cpp` for easy tuning.

**Wiring / Hardware**
- **Main LED:** connect to `GPIO_NUM_13` (see `LED_PIN` in `src/main.cpp`).
- **Status LED:** on `GPIO_NUM_4` (blinks on packet RX).
- **Battery sense:** connect Vin through a 2×100k divider to `IO35` (ADC). Measured Vin = ADC_voltage * 2.
- **Board:** Targeted at ESP32 (ESP-WROVER-KIT tested).

**Key files**
- `src/main.cpp`: main firmware — BLE, PWM, flame effect, ADC and battery logic.
- `platformio.ini`: PlatformIO project configuration.

**Build & Upload (Windows PowerShell)**
Run the following from the project root:

```powershell
# build
C:\Users\<you>\.platformio\penv\Scripts\platformio.exe run

# upload (example, set your COM port)
C:\Users\<you>\.platformio\penv\Scripts\platformio.exe run --target upload --upload-port COM6
```

Or simply use the PlatformIO VSCode extension Run/Upload buttons.

**Runtime behavior & tuning**
- **Flame effect:** Controlled by `FLAME_VARIATION` and `FLAME_UPDATE_INTERVAL_MS` in `src/main.cpp`.
- **RSSI mapping:** `RSSI_MIN` / `RSSI_MAX` set brightness range mapping.
- **ADC sampling:** `ADC_SAMPLES` and `ANALOG_READ_INTERVAL_MS` determine reading smoothing and frequency.
- **Battery thresholds:** `VIN_MIN_THRESHOLD` (override) and `VIN_REDUCE_THRESHOLD` (reduce max power) are in `src/main.cpp`.

**Configurable parameters (in `src/main.cpp`)**
Below are the main parameters you can change and their effect on runtime behavior:

- **RSSI → PWM Mapping**
	- Two functions are provided to map RSSI signal strength to LED brightness:
	- `rssiToPWM_Linear(int rssi)`: Direct linear mapping. PWM increases proportionally across the full RSSI range.
	- `rssiToPWM_Gamma(int rssi, float gamma)` and `rssiToPWM_Gamma(int rssi)`: Power-curve (gamma) mapping. With gamma > 1, PWM stays low for poor signals and only increases rapidly when RSSI is very strong. Default gamma = 3.0.
	- **To switch mapping:** Edit the call in `loop()` (around line 422) from `rssiToPWM_Gamma(...)` to `rssiToPWM_Linear(...)` or vice versa. Also check `handleBattery()` if you want consistent behavior on recovery from low-battery override.

- **LED / PWM**
	- `LED_PWM_FREQ` (default `1000`): PWM frequency in Hz. Higher values reduce visible flicker but may affect PWM resolution.
	- `LED_PWM_RES` (default `16`): PWM resolution in bits. Controls `MAX_LED` range (e.g., 16 → 65535).
	- `MIN_LED` (default `1`): Minimum non-zero duty used when LED is on.
	- `FADE_TIME_MS` (default `500`): Duration for hardware fade transitions (ms).

- **Flame effect**
	- `FLAME_VARIATION` (default `500`): Maximum ± variation in PWM units around the RSSI-derived target. Increase for more intense flicker.
	- `FLAME_UPDATE_INTERVAL_MS` (default `50`): How often the flicker is updated (ms). Lower = faster flicker.

- **RSSI / Filtering**
	- `RSSI_MIN` / `RSSI_MAX` (defaults `-80` / `-30`): Map RSSI range to LED brightness. Adjust for your environment to change sensitivity.
	- `alpha` (in loop, default `0.5`): EWMA weight for new RSSI samples (higher = faster reaction).
	- `beta` (in loop, default `0.2`): Fade-out weight when no packets (smaller = slower decay).

- **Battery / ADC**
	- `ADC_PIN` (default `35`): ADC input used for voltage sensing (IO35).
	- `ADC_SAMPLES` (default `16`): Number of ADC samples averaged per read. Increase to reduce noise.
	- `ADC_VREF` (default `3.30f`): Reference voltage used for simple ADC-to-voltage conversion (approximate).
	- `VIN_MIN_THRESHOLD` (default `3.7f`): Below this Vin the firmware forces LED to minimum and suspends fading (safety override).
	- `VIN_REDUCE_THRESHOLD` (default `3.8f`): Below this Vin (but above override) the firmware reduces `MAX_LED` to half to save power.
	- `ANALOG_READ_INTERVAL_MS` (used inside `handleBattery`, default `1000`): How often to sample the ADC.

- **BLE / Advertising**
	- `ADV_MIN_INTERVAL_MS` / `ADV_MAX_INTERVAL_MS` (default `100` / `100`): Advertising interval bounds in ms.

Changing any of these parameters requires rebuilding and uploading the firmware. If you want runtime tuning (via BLE or serial), I can add a simple interface to update values without a rebuild.

**Accuracy notes**
- The ESP32 ADC is not perfectly linear; this project uses a fixed `ADC_VREF` for a simple estimate. For better accuracy use the `esp_adc_cal` API to calibrate Vref and compensate non-linearity.

// Set to 1 for Serial logs, 0 for battery/production
#define DEBUG_LOG 1

#if defined(USE_TINYUSB)
  #include <Adafruit_TinyUSB.h>
#endif

#include <bluefruit.h>
#include <nrf.h>

// -------------------- Debug macros --------------------
#if DEBUG_LOG
  #define LOG_BEGIN(baud)     do { Serial.begin(baud); delay(1000); } while (0)
  #define LOG(x)              Serial.print(x)
  #define LOGLN(x)            Serial.println(x)
  #define LOGF(x, fmt)        Serial.print((x), (fmt))
  #define LOGLNF(x, fmt)      Serial.println((x), (fmt))
#else
  #define LOG_BEGIN(baud)     do {} while (0)
  #define LOG(x)              do {} while (0)
  #define LOGLN(x)            do {} while (0)
  #define LOGF(x, fmt)        do {} while (0)
  #define LOGLNF(x, fmt)      do {} while (0)
#endif

// -------------------- Pins --------------------

// User LED (P0.15)
#ifndef LED_BUILTIN
  #define LED_BUILTIN 15
#endif

// nice!nano v2: battery is read internally via VDDH/5 (no external divider).
// Hardware refs:
// - Board: nice!nano v2 (MCU: nRF52840)
// - Charger IC: BQ24075 (VQFN-16)
// - Pinout/schematic: https://nicekeyboards.com/docs/nice-nano/pinout-schematic
// - Charging notes: https://github.com/joric/nrfmicro/wiki/Batteries

// Rough LiPo mapping
#ifndef VBAT_EMPTY
  #define VBAT_EMPTY 3.20f
#endif
#ifndef VBAT_FULL
  #define VBAT_FULL 4.20f
#endif

// LED pulse duration (ms)
#ifndef LED_PULSE_MS
  #define LED_PULSE_MS 1000
#endif

// Only publish battery if it changes by >= this many percent
#ifndef BATT_CHANGE_THRESHOLD
  #define BATT_CHANGE_THRESHOLD 2
#endif

// -------------------- BLE (Control) UUIDs --------------------
// Nordic LED Button Service (LBS) UUIDs (for HA control via GATT write)
BLEService        lbsService("00001523-1212-EFDE-1523-785FEABCD123");
BLECharacteristic btnChar   ("00001524-1212-EFDE-1523-785FEABCD123");
BLECharacteristic ledChar   ("00001525-1212-EFDE-1523-785FEABCD123");

// -------------------- BTHome --------------------
// BTHome uses Service Data (16-bit UUID) with UUID 0xFCD2. :contentReference[oaicite:2]{index=2}
static const uint16_t BTHOME_UUID16 = 0xFCD2;
// BTHome v2 device info byte: no encryption, trigger-based updates = 0x44.
static const uint8_t  BTHOME_DEVINFO_V2_NOENC_TRIGGER = 0x44;

// Object IDs in ascending order. :contentReference[oaicite:4]{index=4}
static const uint8_t  BTHOME_OBJ_BATTERY = 0x01; // uint8 percent
static const uint8_t  BTHOME_OBJ_SWITCH  = 0x0F; // uint8 boolean (LED state, generic binary sensor)
static const uint8_t  BTHOME_OBJ_BATT_CHG = 0x16; // uint8 boolean (battery charging / USB power present)

// -------------------- State --------------------
// Keep LED state in a uint8 buffer so the GATT READ returns the live value.
static uint8_t led_state = 0;
static uint8_t button_state = 0;
static uint8_t batt_percent = 0;
static uint8_t usb_power_present = 0;
static volatile bool ble_connected = false;
static bool adv_dirty = false;
static uint32_t pulse_deadline_ms = 0;

// Forward declarations
void startAdv();
void refreshBatteryIfChanged(bool force);
void rebuildAdvertising(bool log_it);
void requestAdvertisingUpdate(bool log_it);
void rebuildAdvertisingIfDirty();
void led_write_cb(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len);

static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static bool batteryChangedEnough(uint8_t old_pct, uint8_t new_pct) {
  if (new_pct > old_pct) return (new_pct - old_pct) >= BATT_CHANGE_THRESHOLD;
  if (old_pct > new_pct) return (old_pct - new_pct) >= BATT_CHANGE_THRESHOLD;
  return false;
}

// Approximate ADC -> VBAT. (If you want accurate %, we can calibrate later.)
static float readBatteryVoltageApprox() {
#ifndef SAADC_CH_PSELP_PSELP_VDDHDIV5
  #error "analogReadVDDHDIV5() not available in this core; nice!nano v2 requires VDDH/5 ADC support."
#endif

  int raw = (int)analogReadVDDHDIV5();      // typically 0..1023 at 10-bit
  float adc = (float)raw / 1023.0f;

  // SAADC default: 0.6V internal ref with 1/6 gain => 0..3.6V at ADC input
  const float adc_full_scale = 3.6f;

  float v_adc = adc * adc_full_scale;       // this is VDDH/5
  float v_bat = v_adc * 5.0f;               // scale back to VDDH

#if DEBUG_LOG
  LOG("BATT: raw=");
  LOG(raw);
  LOG(" vbat≈");
  LOGF(v_bat, 2);
  LOGLN("V");
#endif

  return v_bat;
}

static uint8_t readBatteryPercent() {
  float vbat = readBatteryVoltageApprox();
  float pct = (vbat - VBAT_EMPTY) / (VBAT_FULL - VBAT_EMPTY);
  pct = clampf(pct, 0.0f, 1.0f);
  return (uint8_t)(pct * 100.0f + 0.5f);
}

static uint8_t readUsbPowerPresent() {
#if defined(NRF_POWER) && defined(POWER_USBREGSTATUS_VBUSDETECT_Msk)
  return (NRF_POWER->USBREGSTATUS & POWER_USBREGSTATUS_VBUSDETECT_Msk) ? 1 : 0;
#else
  return 0;
#endif
}

void refreshBatteryIfChanged(bool force) {
  uint8_t new_pct = readBatteryPercent();
  uint8_t new_usb = readUsbPowerPresent();
  bool usb_changed = (new_usb != usb_power_present);

  if (force || batteryChangedEnough(batt_percent, new_pct) || usb_changed) {
    batt_percent = new_pct;
    usb_power_present = new_usb;

#if DEBUG_LOG
    LOG("BATT: publish ");
    LOG((int)batt_percent);
    LOG("% usb=");
    LOGLN(usb_power_present ? "1" : "0");
#endif
    requestAdvertisingUpdate(false);
  } else {
#if DEBUG_LOG
    LOGLN("BATT: change below threshold, not publishing");
#endif
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  led_state = 0;
  digitalWrite(LED_BUILTIN, led_state ? HIGH : LOW);

  analogReadResolution(10);

  LOG_BEGIN(115200);
  LOGLN("Boot: BLE-Button (LBS control + BTHome adv)");
  LOGLN("Init BLE...");

  Bluefruit.begin();
  Bluefruit.autoConnLed(false);
  Bluefruit.setTxPower(0);
  Bluefruit.setName("BLE-Button");

  Bluefruit.Periph.setConnectCallback([](uint16_t conn_hdl) {
    ble_connected = true;
#if DEBUG_LOG
    LOG("BLE: Connected. conn_hdl=");
    LOGLN(conn_hdl);
#endif
    // Wake moment: update battery (but only publish if changed enough)
    refreshBatteryIfChanged(false);
  });

  Bluefruit.Periph.setDisconnectCallback([](uint16_t conn_hdl, uint8_t reason) {
    ble_connected = false;
#if DEBUG_LOG
    LOG("BLE: Disconnected. conn_hdl=");
    LOG(conn_hdl);
    LOG(" reason=0x");
    LOGLNF(reason, HEX);
#endif
    rebuildAdvertisingIfDirty();
  });

  // ---- GATT control service (LBS) ----
  lbsService.begin();

  btnChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  btnChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  btnChar.setFixedLen(1);
  btnChar.begin();
  btnChar.write8(button_state);

  ledChar.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE | CHR_PROPS_WRITE_WO_RESP);
  ledChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  ledChar.setFixedLen(1);
  ledChar.setWriteCallback(led_write_cb);
  ledChar.setBuffer(&led_state, sizeof(led_state));
  ledChar.begin();

  // Initial battery reading + start advertising
  batt_percent = readBatteryPercent();
  usb_power_present = readUsbPowerPresent();
  startAdv();

#if DEBUG_LOG
  LOGLN("Setup complete (sleeping until events)");
#endif
}

void loop() {
  if (pulse_deadline_ms != 0) {
    if ((int32_t)(millis() - pulse_deadline_ms) >= 0) {
      led_state = 0;
      digitalWrite(LED_BUILTIN, LOW);
      pulse_deadline_ms = 0;
      requestAdvertisingUpdate(false);
#if DEBUG_LOG
      LOGLN("LED: OFF (pulse complete)");
#endif
    } else {
      delay(100);
    }
  }

  rebuildAdvertisingIfDirty();

  // Sleep until something happens (BLE event, etc.)
  if (pulse_deadline_ms == 0) {
    waitForEvent();
  }
}

void startAdv() {
  adv_dirty = false;
  rebuildAdvertising(true);
}

// Build advertising payload with BTHome Service Data (AD type 0x16).
void rebuildAdvertising(bool log_it) {
  // BTHome service-data element expects:
  // [UUID16 little-endian][bthome_payload...]
  // where bthome_payload = [device_info][objId][value]...
  // UUID 0xFCD2 => bytes D2 FC. :contentReference[oaicite:5]{index=5}
  uint8_t svcdata[2 + 1 + 2 + 2 + 2];
  size_t i = 0;

  svcdata[i++] = (uint8_t)(BTHOME_UUID16 & 0xFF);       // 0xD2
  svcdata[i++] = (uint8_t)((BTHOME_UUID16 >> 8) & 0xFF);// 0xFC

  svcdata[i++] = BTHOME_DEVINFO_V2_NOENC_TRIGGER;
  svcdata[i++] = BTHOME_OBJ_BATTERY;
  svcdata[i++] = batt_percent;
  svcdata[i++] = BTHOME_OBJ_SWITCH;
  svcdata[i++] = led_state ? 1 : 0;
  svcdata[i++] = BTHOME_OBJ_BATT_CHG;
  svcdata[i++] = usb_power_present ? 1 : 0;

  Bluefruit.Advertising.stop();
  Bluefruit.Advertising.clearData();
  Bluefruit.ScanResponse.clearData();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addName();          // optional but nice for debugging
  Bluefruit.Advertising.addService(lbsService);

  // Add raw AD element: Service Data - 16-bit UUID (AD type 0x16). :contentReference[oaicite:6]{index=6}
  Bluefruit.Advertising.addData(0x16, svcdata, (uint8_t)i);

  // 0.5 second advertising interval
  Bluefruit.Advertising.setInterval(1600, 1600);
  Bluefruit.Advertising.setFastTimeout(0);
  Bluefruit.Advertising.restartOnDisconnect(true);

  Bluefruit.Advertising.start(0);

#if DEBUG_LOG
  if (log_it) {
    LOG("ADV: BTHome batt=");
    LOG((int)batt_percent);
    LOG(" led=");
    LOG(led_state ? "1" : "0");
    LOG(" usb=");
    LOGLN(usb_power_present ? "1" : "0");
  }
#endif
}

void requestAdvertisingUpdate(bool log_it) {
  adv_dirty = true;
  if (!ble_connected) {
    adv_dirty = false;
    rebuildAdvertising(log_it);
  }
}

void rebuildAdvertisingIfDirty() {
  if (!adv_dirty) return;
  if (ble_connected) return;
  adv_dirty = false;
  rebuildAdvertising(false);
}

// HA connects + writes; we update LED + update BTHome payload; then disconnect.
void led_write_cb(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
#if DEBUG_LOG
  LOG("GATT: LED write conn=");
  LOG(conn_hdl);
  LOG(" len=");
  LOG(len);
  LOG(" data=");
  if (len > 0) LOGLNF(data[0], HEX);
  else { LOGLN("(empty)"); return; }
#else
  if (len == 0) return;
#endif

  bool requested_on = (data[0] != 0x00);

  (void)chr;
  led_state = requested_on ? 1 : 0;
  digitalWrite(LED_BUILTIN, led_state ? HIGH : LOW);

  // Another wake moment: refresh battery opportunistically (no periodic timer)
  refreshBatteryIfChanged(false);

  // Update BTHome broadcast ASAP so HA sees new state
  requestAdvertisingUpdate(false);

  if (requested_on) {
#if DEBUG_LOG
    LOGLN("LED: ON (pulse)");
#endif
    pulse_deadline_ms = millis() + LED_PULSE_MS;
  } else {
    pulse_deadline_ms = 0;
  }

#if DEBUG_LOG
  LOGLN(led_state ? "LED: ON" : "LED: OFF");
  LOGLN("BLE: disconnecting to save power");
#endif

  Bluefruit.disconnect(conn_hdl);
}

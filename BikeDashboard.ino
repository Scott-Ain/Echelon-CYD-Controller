#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <math.h>
#include <string>

// ======================================================
//  CHANGES IN THIS REVISION (power display + FTMS server)
//  - Footer "CALORIES" panel replaced with "POWER" (watts),
//    using the existing iconPower() glyph. Calorie tracking
//    is kept internally (cheap, still useful for logging)
//    but is no longer shown on screen.
//  - Added an FTMS (Fitness Machine Service, 0x1826) BLE
//    GATT *server* so apps like Zwift/TrainerRoad can connect
//    to this ESP32 as a smart trainer:
//      * Fitness Machine Feature (0x2ACC, read)
//      * Indoor Bike Data (0x2AD2, notify) - speed/cadence/
//        resistance/power, sent ~1x/sec
//      * Fitness Machine Control Point (0x2AD9, write+indicate)
//        - Request Control / Reset / Start-Resume / Stop-Pause
//        - Set Target Resistance Level -> drives sendResistance()
//        - Set Target Power -> simple closed-loop ERG emulation
//        - Set Indoor Bike Simulation Parameters -> grade mapped
//          to a resistance offset (basic "sim mode" feel)
//      * Fitness Machine Status (0x2ADA, notify)
//      * Supported Resistance Level Range (0x2AD6, read)
//    The ESP32 now runs BLE *central* (existing connection to
//    the bike's own BLE service) and BLE *peripheral* (FTMS
//    server for training apps) at the same time.
//
//  BUG FIXES / CLEANUP FROM CODE REVIEW:
//  - Resistance clamping was inconsistent: sendResistance()
//    silently floored anything below 15 up to 15 before
//    transmitting to the bike, but the on-screen `resistance`
//    variable (and the +/- buttons) allowed 0..14, so the
//    dashboard could show e.g. "10" while the bike was
//    actually sitting at level 15. Introduced RESISTANCE_MIN/
//    RESISTANCE_MAX constants and used them consistently
//    everywhere resistance is read, clamped, or displayed.
//  - MyClientCallback::onDisconnect() was calling drawBleStatus()
//    (a TFT/SPI draw) directly from the BLE stack's own task.
//    TFT_eSPI is not thread-safe and shares the SPI bus with the
//    touch controller, so drawing from a non-loop() task is a
//    race condition. It now only updates state; updateDisplay()
//    performs the actual redraw from the main loop, same as it
//    already does for every other state transition.
//  - MyAdvertisedDeviceCallbacks::onResult() allocated a new
//    BLEAdvertisedDevice each time a match was found without
//    freeing the previous one, leaking a small amount of heap
//    on every reconnect. The old pointer is now deleted first.
//  - The resistance bar's fill mapping used the full 0..32
//    range even though the bike never reports/accepts anything
//    below 15, so roughly half the bar was dead space. It now
//    maps across RESISTANCE_MIN..RESISTANCE_MAX for better
//    visual resolution.
//  - CORRECTED: the bike's true hardware resistance range is
//    0-32, not 15-32. RESISTANCE_MIN is now 0, so the FTMS
//    Supported Resistance Level Range and every FTMS-driven
//    resistance command (Set Target Resistance Level, ERG,
//    simulation grade) map onto the bike's real 0-32 scale
//    instead of being clamped into 15-32. The "skip straight to
//    15" behavior is now scoped to the physical +/- buttons only
//    (RESISTANCE_BUTTON_START), as a local UX choice, not a
//    hardware limit imposed on external apps.
// ======================================================

// ======================================================
//  BLE UUIDs - bike's own (proprietary) service, used as a
//  BLE *client* to talk to the physical bike hardware
// ======================================================
static BLEUUID deviceUUID("0bf669f0-45f2-11e7-9598-0800200c9a66");
static BLEUUID connectUUID("0bf669f1-45f2-11e7-9598-0800200c9a66");
static BLEUUID writeUUID("0bf669f2-45f2-11e7-9598-0800200c9a66");
static BLEUUID sensorUUID("0bf669f4-45f2-11e7-9598-0800200c9a66");

// ======================================================
//  BLE UUIDs - standard Fitness Machine Service, used as a
//  BLE *server* so apps like Zwift can connect to this ESP32
// ======================================================
#define FTMS_SERVICE_UUID           (uint16_t)0x1826
#define FTMS_FEATURE_UUID           (uint16_t)0x2ACC
#define FTMS_INDOOR_BIKE_UUID       (uint16_t)0x2AD2
#define FTMS_RESISTANCE_RANGE_UUID  (uint16_t)0x2AD6
#define FTMS_CONTROL_POINT_UUID     (uint16_t)0x2AD9
#define FTMS_STATUS_UUID            (uint16_t)0x2ADA

// FTMS Control Point op codes
#define FTMS_OP_REQUEST_CONTROL       0x00
#define FTMS_OP_RESET                 0x01
#define FTMS_OP_SET_TARGET_RESISTANCE 0x04
#define FTMS_OP_SET_TARGET_POWER      0x05
#define FTMS_OP_START_RESUME          0x07
#define FTMS_OP_STOP_PAUSE            0x08
#define FTMS_OP_SET_SIM_PARAMS        0x11
#define FTMS_OP_RESPONSE_CODE         0x80

// FTMS Control Point result codes
#define FTMS_RESULT_SUCCESS               0x01
#define FTMS_RESULT_OP_NOT_SUPPORTED      0x02
#define FTMS_RESULT_INVALID_PARAMETER     0x03
#define FTMS_RESULT_OPERATION_FAILED      0x04
#define FTMS_RESULT_CONTROL_NOT_PERMITTED 0x05

// ======================================================
//  UI Color Palette (RGB565)
// ======================================================
#define BG_COLOR    0x0000 // Pure Black
#define PANEL_BG    0x10A2 // Dark Greyish Blue
#define TRACK_COLOR 0x2965 // Bar track (unfilled) - muted grey-blue
#define C_BLUE      0x041F // Time
#define C_GREEN     0x07E0 // Distance
#define C_ORANGE    0xFC00 // Speed / mid resistance
#define C_CYAN      0x07FF // Cadence
#define C_PURPLE    0x780F // (unused - resistance moved to red)
#define C_RED       0xF800 // Power / high resistance / resistance accent
#define C_RED_DIM   0xA000 // Resistance bar - low tier (muted red)

// ======================================================
//  Screen geometry (320x240 landscape, rotation 1)
//  Three packed rows, 4px outer margin, 4px gutters.
// ======================================================
#define SCREEN_W 320
#define SCREEN_H 240
#define MARGIN   4
#define GAP      4

#define COL_W    154              // (320 - 3*4) / 2
#define COL1_X   MARGIN           // 4
#define COL2_X   (COL1_X + COL_W + GAP) // 162

#define ROW1_Y   4
#define ROW1_H   92               // Speed / Cadence (hero)
#define ROW2_Y   (ROW1_Y + ROW1_H + GAP)   // 100
#define ROW2_H   56               // Time / Distance
#define ROW3_Y   (ROW2_Y + ROW2_H + GAP)   // 160
#define ROW3_H   76               // Resistance + Power
// ROW3 bottom = 236, leaves 4px bottom margin to 240

#define RES_SECTION_W 236         // resistance side of footer (grown; was 220)
#define PWR_SECTION_X (COL1_X + RES_SECTION_W + GAP) // 244
#define PWR_SECTION_W (SCREEN_W - MARGIN - PWR_SECTION_X) // 72 (shrunk; was 88)

// Vertical-pill +/- buttons: full-height touch targets on either
// side of the resistance panel, with the bar filling the gap
// between them.
#define RES_BTN_W        34
#define RES_BTN_TOP       (ROW3_Y + 22)
#define RES_BTN_H         (ROW3_H - 26)
#define RES_BTN_MINUS_X   (COL1_X + 4)
#define RES_BTN_PLUS_X    (COL1_X + RES_SECTION_W - 4 - RES_BTN_W)
#define RES_BAR_X         (RES_BTN_MINUS_X + RES_BTN_W + 8)
#define RES_BAR_X2        (RES_BTN_PLUS_X - 8)
#define RES_BAR_W         (RES_BAR_X2 - RES_BAR_X)
#define RES_BAR_Y         (ROW3_Y + 30)
#define RES_BAR_H         32

// ======================================================
//  TFT Display
// ======================================================
TFT_eSPI tft = TFT_eSPI();

// Touch
// Touchscreen pins (XPT2046)
#define XPT2046_IRQ 36   // T_IRQ
#define XPT2046_MOSI 32  // T_DIN
#define XPT2046_MISO 39  // T_OUT
#define XPT2046_CLK 25   // T_CLK
#define XPT2046_CS 33    // T_CS

SPIClass touchscreenSPI = SPIClass(VSPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);

// ======================================================
//  UI State Machine
// ======================================================
enum UiState { SEARCHING, CONNECTING, CONNECTED, PAUSED };
UiState uiState = SEARCHING;
UiState lastUiState = PAUSED;
uint8_t loadingDots = 0;
unsigned long lastAnimUpdate = 0;

// ======================================================
//  Resistance button press feedback
// ======================================================
bool minusPressed = false, plusPressed = false;
unsigned long minusPressAt = 0, plusPressAt = 0;
const unsigned long PRESS_FEEDBACK_MS = 150;

// ======================================================
//  Power model
//  Reviewed: the formula is an exponential curve in both resistance and
//  cadence (pow(1.090112, resistance) * pow(1.015343, cadence) * 7.228958),
//  which is the standard shape for these reverse-engineered Echelon power
//  curves - power rises exponentially, not linearly, with both inputs.
//  Spot-checked a few points: level 15 @ 80rpm ~ 90W, level 32 @ 80rpm ~ 390W,
//  which land in a sane range for a spin bike. Left unchanged; if your unit's
//  numbers don't match a known-good reading (e.g. from the bike's own app),
//  the three constants are the ones to recalibrate, not the formula shape.
// ======================================================
int getPower(int cadence, int resistance) {
  if (cadence == 0 || resistance == 0) return 0;
  return int(pow(1.090112, resistance) * pow(1.015343, cadence) * 7.228958);
}

// ======================================================
//  BLE State - client role (connection to the bike)
// ======================================================
static boolean doConnect = false;
static boolean connected = false;
static BLERemoteCharacteristic* writeCharacteristic = nullptr;
static BLERemoteCharacteristic* sensorCharacteristic = nullptr;
static BLEAdvertisedDevice* device = nullptr;
static BLEClient* client = nullptr;
static BLEScan* scanner = nullptr;

// ======================================================
//  BLE State - server role (FTMS, for apps like Zwift)
// ======================================================
static BLEServer* ftmsServer = nullptr;
static BLECharacteristic* ftmsFeatureChar = nullptr;
static BLECharacteristic* ftmsIndoorBikeChar = nullptr;
static BLECharacteristic* ftmsControlPointChar = nullptr;
static BLECharacteristic* ftmsStatusChar = nullptr;
static BLECharacteristic* ftmsResistanceRangeChar = nullptr;

static bool ftmsCentralConnected = false;
static bool ftmsControlGranted = false;
static bool ftmsHasTargetPower = false;
static int  ftmsTargetPowerWatts = 0;
static unsigned long lastFtmsNotify = 0;
static unsigned long lastErgAdjust = 0;

// ======================================================
//  Workout Metrics & Caching
// ======================================================
// The Echelon bike's actual hardware resistance range is 0-32 (not 15-32).
// RESISTANCE_MIN/MAX below are the true hardware bounds and are what's used
// for the FTMS Supported Resistance Level Range and for clamping any value
// that comes from an FTMS client, so apps like Zwift see the bike's real
// range and their resistance commands map onto it 1:1, without an
// artificial floor. RESISTANCE_BUTTON_START/RESISTANCE_SIM_BASE are local
// UX choices layered on top (see handleTouch() and the sim-params handler).
#define RESISTANCE_MIN 0
#define RESISTANCE_MAX 32
#define RESISTANCE_BUTTON_START 15 // level the physical + button jumps to on its first press
#define RESISTANCE_SIM_BASE     15 // "flat road" baseline resistance for FTMS simulation mode

static int cadence = 0, lastCadence = -1;
static int resistance = 0, lastResistance = -1; // 0 until the bike reports a real level
static int powerVal = 0, lastPower = -1;

static float speed_mph = 0, lastSpeed = -1;
static float distance_miles = 0, lastDistance = -1;
static float calories = 0; // tracked internally; not shown on screen (power replaces it)
static int lastCalories = -1;

static unsigned long runtime = 0;
static int lastSeconds = -1;

static unsigned long last_millis = 0;
static unsigned long last_cadence = 0;
static unsigned long lastTouchTime = 0;

const unsigned long ScreenTimeoutMillis = 5000;

// ======================================================
//  Estimates
//  Reviewed: speed is intentionally a function of cadence only, not
//  resistance. On a stationary bike the flywheel's rotational speed tracks
//  pedaling cadence directly through a fixed gear ratio - resistance changes
//  how *hard* it is to turn the flywheel at a given cadence (which is what
//  the power model above captures), not how fast it spins for a given
//  cadence. So this is not a bug: getPower(cadence, resistance) is the
//  resistance-dependent value; estimateSpeed(cadence) correctly is not.
// ======================================================
float estimateSpeed(int cadence) { return cadence * 0.19; }

// ======================================================
//  BLE Callbacks (client role - connection to the bike)
// ======================================================
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) { connected = true; }
  void onDisconnect(BLEClient* pclient) {
    connected = false;
    uiState = SEARCHING;
    // Intentionally no TFT drawing here - this callback runs on the BLE
    // stack's own task, not the main loop, and TFT_eSPI/the shared SPI
    // bus are not safe to touch from another task. updateDisplay() picks
    // up the uiState change on the next loop() iteration and redraws.
  }
};

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(deviceUUID)) {
      BLEDevice::getScan()->stop();
      if (device) delete device; // avoid leaking the previous match on reconnect
      device = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      uiState = CONNECTING;
    }
  }
};

static void notifyCallback(BLERemoteCharacteristic* c, uint8_t* data, size_t len, bool isNotify) {
  switch (data[1]) {
    case 0xD1:  // cadence
      cadence = int((data[9] << 8) + data[10]);
      powerVal = getPower(cadence, resistance);
      last_cadence = millis();
      uiState = CONNECTED;
      break;

    case 0xD2: // resistance - 0 is a real, valid level on this bike, so no special-casing needed
      resistance = constrain(int(data[3]), RESISTANCE_MIN, RESISTANCE_MAX);
      powerVal = getPower(cadence, resistance);
      break;
  }
}

// ======================================================
//  BLE Connection Logic (client role - the bike)
// ======================================================
bool connectToServer() {
  client = BLEDevice::createClient();
  client->setClientCallbacks(new MyClientCallback());
  if (!client->connect(device)) return false;

  BLERemoteService* svc = client->getService(connectUUID);
  if (!svc) return false;

  sensorCharacteristic = svc->getCharacteristic(sensorUUID);
  writeCharacteristic  = svc->getCharacteristic(writeUUID);
  if (sensorCharacteristic->canNotify()) sensorCharacteristic->registerForNotify(notifyCallback);

  byte msg[] = {0xF0, 0xB0, 0x01, 0x01, 0xA2};
  writeCharacteristic->writeValue(msg, 5);

  last_cadence = millis();
  doConnect = false;
  uiState = CONNECTED;
  return true;
}

void sendResistance(int level)
{
    if (!connected || !writeCharacteristic)
        return;

    // Clamp to the bike's true hardware range. Note this is a full pass-through
    // down to 0 - the old behavior of flooring everything to level 15 has been
    // removed so FTMS clients (and the +/- buttons) get exactly the level they
    // asked for, with no mismatch between what's shown and what's sent.
    level = constrain(level, RESISTANCE_MIN, RESISTANCE_MAX);

    uint8_t msg[5];
    msg[0] = 0xF0;
    msg[1] = 0xB1;
    msg[2] = 0x01;
    msg[3] = (uint8_t)level;
    msg[4] = (msg[0] + msg[1] + msg[2] + msg[3]) & 0xFF;

    Serial.printf("Resistance %2d -> ", level);
    for (int i = 0; i < sizeof(msg); i++)
        Serial.printf("%02X ", msg[i]);
    Serial.println();

    writeCharacteristic->writeValue(msg, sizeof(msg), true);
}

// ======================================================
//  Vector icons - each draws inside a nominal 20x20 box at (x,y).
// ======================================================
void iconClock(int x, int y, uint16_t color) {
  tft.drawCircle(x + 10, y + 10, 9, color);
  tft.drawLine(x + 10, y + 10, x + 10, y + 4, color);  // minute hand
  tft.drawLine(x + 10, y + 10, x + 15, y + 12, color); // hour hand
}

void iconSpeed(int x, int y, uint16_t color) {
  // stylized "fast" chevrons
  tft.fillTriangle(x + 2, y + 5, x + 10, y + 10, x + 2, y + 15, color);
  tft.fillTriangle(x + 10, y + 5, x + 18, y + 10, x + 10, y + 15, color);
}

void iconDistance(int x, int y, uint16_t color) {
  // location pin
  tft.fillCircle(x + 10, y + 7, 6, color);
  tft.fillTriangle(x + 4, y + 9, x + 16, y + 9, x + 10, y + 19, color);
  tft.fillCircle(x + 10, y + 7, 3, PANEL_BG);
}

void iconCadence(int x, int y, uint16_t color) {
  // rotation loop with arrowhead
  tft.drawCircle(x + 10, y + 10, 8, color);
  tft.drawCircle(x + 10, y + 10, 7, color);
  tft.fillTriangle(x + 15, y + 2, x + 20, y + 6, x + 12, y + 8, color);
}

void iconPower(int x, int y, uint16_t color) {
  // Lightning bolt, built from two overlapping filled triangles - matches
  // the flat, solid-fill style of the other icons (speed chevrons, distance
  // pin, cadence loop) instead of the busier gauge-and-flame glyph before.
  tft.fillTriangle(x + 12, y + 0, x + 3, y + 12, x + 10, y + 12, color);
  tft.fillTriangle(x + 9, y + 9, x + 17, y + 9, x + 7, y + 20, color);
}

// ======================================================
//  Dashboard Rendering
// ======================================================
void drawBleStatus(uint16_t color) {
  tft.fillCircle(COL2_X + COL_W - 12, ROW1_Y + 12, 5, color);
}

void drawResistanceButtons() {
  bool minusDim = (resistance <= RESISTANCE_MIN);
  bool plusDim  = (resistance >= RESISTANCE_MAX);
  uint16_t minusColor = minusDim ? TFT_DARKGREY : C_RED;
  uint16_t plusColor  = plusDim  ? TFT_DARKGREY : C_RED;
  int radius = RES_BTN_W / 2; // full-width corner radius -> stadium/vertical-ellipse shape

  // minus button (tall pill, full panel height)
  uint16_t minusFill   = minusPressed ? C_RED : PANEL_BG;
  uint16_t minusStroke = minusPressed ? TFT_WHITE : minusColor;
  uint16_t minusGlyph  = minusPressed ? TFT_WHITE : minusColor;
  tft.fillRoundRect(RES_BTN_MINUS_X, RES_BTN_TOP, RES_BTN_W, RES_BTN_H, radius, minusFill);
  tft.drawRoundRect(RES_BTN_MINUS_X, RES_BTN_TOP, RES_BTN_W, RES_BTN_H, radius, minusStroke);
  tft.drawRoundRect(RES_BTN_MINUS_X + 1, RES_BTN_TOP + 1, RES_BTN_W - 2, RES_BTN_H - 2, radius - 1, minusStroke);
  {
    int mcx = RES_BTN_MINUS_X + RES_BTN_W / 2, mcy = RES_BTN_TOP + RES_BTN_H / 2;
    tft.fillRect(mcx - 9, mcy - 2, 18, 4, minusGlyph);
  }

  // plus button (tall pill, full panel height)
  uint16_t plusFill   = plusPressed ? C_RED : PANEL_BG;
  uint16_t plusStroke = plusPressed ? TFT_WHITE : plusColor;
  uint16_t plusGlyph  = plusPressed ? TFT_WHITE : plusColor;
  tft.fillRoundRect(RES_BTN_PLUS_X, RES_BTN_TOP, RES_BTN_W, RES_BTN_H, radius, plusFill);
  tft.drawRoundRect(RES_BTN_PLUS_X, RES_BTN_TOP, RES_BTN_W, RES_BTN_H, radius, plusStroke);
  tft.drawRoundRect(RES_BTN_PLUS_X + 1, RES_BTN_TOP + 1, RES_BTN_W - 2, RES_BTN_H - 2, radius - 1, plusStroke);
  {
    int pcx = RES_BTN_PLUS_X + RES_BTN_W / 2, pcy = RES_BTN_TOP + RES_BTN_H / 2;
    tft.fillRect(pcx - 9, pcy - 2, 18, 4, plusGlyph);
    tft.fillRect(pcx - 2, pcy - 9, 4, 18, plusGlyph);
  }
}

void updateTime();
void updateCadence();
void updateSpeed();
void updateResistance();
void updatePower();
void updateDistance();

void drawStaticDashboard() {
  tft.fillScreen(BG_COLOR);

  // --- ROW 1: SPEED (hero) ---
  tft.fillRoundRect(COL1_X, ROW1_Y, COL_W, ROW1_H, 6, PANEL_BG);
  iconSpeed(COL1_X + 10, ROW1_Y + 8, C_ORANGE);
  tft.setTextDatum(TL_DATUM);
  tft.setTextColor(C_ORANGE, PANEL_BG);
  tft.drawString("SPEED", COL1_X + 34, ROW1_Y + 11, 2);
  tft.drawFastHLine(COL1_X + 6, ROW1_Y + 34, COL_W - 12, C_ORANGE);

  // --- ROW 1: CADENCE (hero) ---
  tft.fillRoundRect(COL2_X, ROW1_Y, COL_W, ROW1_H, 6, PANEL_BG);
  iconCadence(COL2_X + 10, ROW1_Y + 8, C_CYAN);
  tft.setTextColor(C_CYAN, PANEL_BG);
  tft.drawString("CADENCE", COL2_X + 34, ROW1_Y + 11, 2);
  tft.drawFastHLine(COL2_X + 6, ROW1_Y + 34, COL_W - 12, C_CYAN);

  // --- ROW 2: TIME ---
  tft.fillRoundRect(COL1_X, ROW2_Y, COL_W, ROW2_H, 6, PANEL_BG);
  iconClock(COL1_X + 8, ROW2_Y + 4, C_BLUE);
  tft.setTextColor(C_BLUE, PANEL_BG);
  tft.drawString("TIME", COL1_X + 30, ROW2_Y + 7, 2);
  tft.drawFastHLine(COL1_X + 6, ROW2_Y + 26, COL_W - 12, C_BLUE);

  // --- ROW 2: DISTANCE ---
  tft.fillRoundRect(COL2_X, ROW2_Y, COL_W, ROW2_H, 6, PANEL_BG);
  iconDistance(COL2_X + 8, ROW2_Y + 2, C_GREEN);
  tft.setTextColor(C_GREEN, PANEL_BG);
  tft.drawString("DISTANCE", COL2_X + 30, ROW2_Y + 7, 2);
  tft.drawFastHLine(COL2_X + 6, ROW2_Y + 26, COL_W - 12, C_GREEN);

  // --- ROW 3: RESISTANCE (footer, left) ---
  tft.fillRoundRect(COL1_X, ROW3_Y, RES_SECTION_W, ROW3_H, 6, PANEL_BG);
  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(C_RED, PANEL_BG);
  tft.drawString("RESISTANCE", COL1_X + RES_SECTION_W / 2, ROW3_Y + 4, 2);
  drawResistanceButtons();

  // --- ROW 3: POWER (footer, right) ---
  tft.fillRoundRect(PWR_SECTION_X, ROW3_Y, PWR_SECTION_W, ROW3_H, 6, PANEL_BG);
  iconPower(PWR_SECTION_X + PWR_SECTION_W / 2 - 10, ROW3_Y + 8, C_RED);
  tft.setTextDatum(TC_DATUM);
  tft.setTextColor(C_RED, PANEL_BG);
  tft.drawString("POWER", PWR_SECTION_X + PWR_SECTION_W / 2, ROW3_Y + 30, 1);

  // Initial placeholders
  updateTime();
  updateCadence();
  updateSpeed();
  updateResistance();
  updatePower();
  updateDistance();
}

void clearOverlay() {
  // Speed panel's value area only (below its header divider)
  tft.fillRect(COL1_X + 2, ROW1_Y + 36, COL_W - 4, ROW1_H - 44, PANEL_BG);
  lastSpeed = -1;
}

void drawOverlayText(const char* text) {
  tft.fillRect(COL1_X + 2, ROW1_Y + 36, COL_W - 4, ROW1_H - 44, PANEL_BG);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, PANEL_BG);
  tft.drawString(text, COL1_X + COL_W / 2, ROW1_Y + 60, 2);
}

// ======================================================
//  Dynamic Update Functions
// ======================================================
void updateTime() {
  int seconds = (runtime / 1000) % 60;
  if (seconds == lastSeconds) return;
  lastSeconds = seconds;

  char buf[16];
  sprintf(buf, "%02d:%02d", (int)(runtime / 60000), seconds);
  tft.setTextPadding(COL_W - 12);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, PANEL_BG);
  tft.drawString(buf, COL1_X + COL_W / 2, ROW2_Y + 41, 4);
  tft.setTextPadding(0);
}

void updateDistance() {
  if (abs(distance_miles - lastDistance) < 0.01) return;
  lastDistance = distance_miles;
  tft.setTextPadding(COL_W - 12);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, PANEL_BG);
  tft.drawFloat(distance_miles, 2, COL2_X + COL_W / 2, ROW2_Y + 41, 4);
  tft.setTextPadding(0);
}

void updateSpeed() {
  if (abs(speed_mph - lastSpeed) < 0.1) return;
  lastSpeed = speed_mph;

  // Big value, left-aligned
  tft.setTextPadding(108);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(TFT_WHITE, PANEL_BG);
  if (speed_mph > 0) tft.drawFloat(speed_mph, 1, COL1_X + 8, ROW1_Y + 60, 7);
  else tft.drawString("--", COL1_X + 8, ROW1_Y + 60, 7);
  tft.setTextPadding(0);

  // Unit, right-aligned, same line as the value
  tft.setTextDatum(MR_DATUM);
  tft.setTextColor(TFT_DARKGREY, PANEL_BG);
  tft.setTextPadding(28);
  tft.drawString("mph", COL1_X + COL_W - 6, ROW1_Y + 60, 1);
  tft.setTextPadding(0);
}

void updateCadence() {
  if (cadence == lastCadence) return;
  lastCadence = cadence;

  // Big value, left-aligned
  tft.setTextPadding(108);
  tft.setTextDatum(ML_DATUM);
  tft.setTextColor(TFT_WHITE, PANEL_BG);
  if (cadence > 0) tft.drawNumber(cadence, COL2_X + 8, ROW1_Y + 60, 7);
  else tft.drawString("--", COL2_X + 8, ROW1_Y + 60, 7);
  tft.setTextPadding(0);

  // Unit, right-aligned, same line as the value
  tft.setTextDatum(MR_DATUM);
  tft.setTextColor(TFT_DARKGREY, PANEL_BG);
  tft.setTextPadding(28);
  tft.drawString("rpm", COL2_X + COL_W - 6, ROW1_Y + 60, 1);
  tft.setTextPadding(0);
}

void updateResistance() {
  if (resistance == lastResistance) return;
  lastResistance = resistance;

  // Explicit bands rather than thirds-of-range math, to match the bike's
  // actual 15..32 scale: 15-19 green, 20-25 orange, 26-32 red.
  uint16_t barColor = C_GREEN;
  if (resistance >= 26) barColor = C_RED;
  else if (resistance >= 20) barColor = C_ORANGE;

  const int barX = RES_BAR_X, barY = RES_BAR_Y, barW = RES_BAR_W, barH = RES_BAR_H;
  int fillW = map(resistance, RESISTANCE_MIN, RESISTANCE_MAX, 0, barW);
  fillW = constrain(fillW, 0, barW); // defensive clamp

  tft.fillRoundRect(barX, barY, barW, barH, 6, TRACK_COLOR);
  if (fillW > 8) tft.fillRoundRect(barX, barY, fillW, barH, 6, barColor);
  tft.drawRoundRect(barX, barY, barW, barH, 6, TFT_WHITE);

  // number drawn transparent (no bg fill) directly on top of the bar
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE);
  tft.drawNumber(resistance, barX + barW / 2, barY + barH / 2, 4);

  drawResistanceButtons(); // refresh dim state at min/max
}

void updatePower() {
  if (powerVal == lastPower) return;
  lastPower = powerVal;
  char buf[8];
  snprintf(buf, sizeof(buf), "%dW", powerVal);
  tft.setTextPadding(PWR_SECTION_W - 8);
  tft.setTextDatum(MC_DATUM);
  tft.setTextColor(TFT_WHITE, PANEL_BG);
  tft.drawString(buf, PWR_SECTION_X + PWR_SECTION_W / 2, ROW3_Y + 54, 4);
  tft.setTextPadding(0);
}

// ======================================================
//  Main Display Manager
// ======================================================
void updateDisplay() {
  unsigned long now = millis();

  if (uiState != lastUiState) {
    if (lastUiState == SEARCHING || lastUiState == CONNECTING || lastUiState == PAUSED) {
      clearOverlay();
    }

    switch (uiState) {
      case SEARCHING:  drawBleStatus(TFT_DARKGREY); break;
      case CONNECTING: drawBleStatus(C_ORANGE); break;
      case CONNECTED:  drawBleStatus(C_GREEN); break;
      case PAUSED:
        drawBleStatus(C_GREEN);
        cadence = 0;
        speed_mph = 0.0;
        updateCadence();
        updateSpeed();
        drawOverlayText("Paused");
        break;
    }
    lastUiState = uiState;
  }

  if (uiState == SEARCHING || uiState == CONNECTING) {
    if (now - lastAnimUpdate > 500) {
      lastAnimUpdate = now;
      loadingDots = (loadingDots + 1) % 4;
      char animText[24];
      strcpy(animText, uiState == SEARCHING ? "Searching" : "Connecting");
      for (int i = 0; i < loadingDots; i++) strcat(animText, ".");
      drawOverlayText(animText);
    }
    return;
  }

  updateTime();
  if (uiState == CONNECTED) {
    updateCadence();
    updateSpeed();
  }
  updateResistance();
  updatePower();
  updateDistance();
}

// ======================================================
//  Touch handling (XPT2046_Touchscreen API)
// ======================================================
void handleTouch() {
  if (!touchscreen.touched()) return;

  TS_Point p = touchscreen.getPoint();

  // Map raw 12-bit coordinates (0..4095) to display pixels.
  // If axes read inverted/swapped on your panel, swap or invert these maps.
  uint16_t x = map(p.x, 0, 4095, 0, SCREEN_W - 1);
  uint16_t y = map(p.y, 0, 4095, 0, SCREEN_H - 1);

  unsigned long now = millis();
  if (now - lastTouchTime < 250) return;
  lastTouchTime = now;

  Serial.printf("Touch raw: x=%d y=%d z=%d -> mapped x=%d y=%d\n", p.x, p.y, p.z, x, y);

  const int touchMargin = 6; // generous margin around the visible pill

  if (x >= RES_BTN_MINUS_X - touchMargin && x <= RES_BTN_MINUS_X + RES_BTN_W + touchMargin &&
      y >= RES_BTN_TOP - touchMargin && y <= RES_BTN_TOP + RES_BTN_H + touchMargin) {
    resistance = constrain(resistance - 1, RESISTANCE_MIN, RESISTANCE_MAX);
    sendResistance(resistance);
    ftmsHasTargetPower = false; // manual override cancels any active ERG target
    minusPressed = true;
    minusPressAt = now;
    drawResistanceButtons(); // instant visual feedback, ahead of the next full redraw
    return;
  }

  if (x >= RES_BTN_PLUS_X - touchMargin && x <= RES_BTN_PLUS_X + RES_BTN_W + touchMargin &&
      y >= RES_BTN_TOP - touchMargin && y <= RES_BTN_TOP + RES_BTN_H + touchMargin) {
    // From below the perceptible threshold, the first press jumps straight
    // to RESISTANCE_BUTTON_START instead of incrementing by 1 - this is a
    // local UX choice, separate from the bike's true 0-32 hardware range.
    resistance = (resistance < RESISTANCE_BUTTON_START) ? RESISTANCE_BUTTON_START : constrain(resistance + 1, RESISTANCE_MIN, RESISTANCE_MAX);
    sendResistance(resistance);
    ftmsHasTargetPower = false;
    plusPressed = true;
    plusPressAt = now;
    drawResistanceButtons(); // instant visual feedback, ahead of the next full redraw
  }
}

void updateButtonFeedback() {
  unsigned long now = millis();
  bool changed = false;
  if (minusPressed && now - minusPressAt > PRESS_FEEDBACK_MS) { minusPressed = false; changed = true; }
  if (plusPressed  && now - plusPressAt  > PRESS_FEEDBACK_MS) { plusPressed  = false; changed = true; }
  if (changed) drawResistanceButtons();
}

// ======================================================
//  FTMS server - GATT callbacks
// ======================================================
class FtmsServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* s) override {
    ftmsCentralConnected = true;
  }
  void onDisconnect(BLEServer* s) override {
    ftmsCentralConnected = false;
    ftmsControlGranted = false;
    ftmsHasTargetPower = false;
    BLEDevice::startAdvertising(); // the stack stops advertising on connect
  }
};

class FtmsControlPointCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    String v = c->getValue(); // this BLE core version returns Arduino String, not std::string
    if (v.length() == 0) return;

    uint8_t opcode = (uint8_t)v[0];
    uint8_t result = FTMS_RESULT_SUCCESS;

    // Request Control and Reset are always allowed; everything else needs
    // control to have been requested first, per the FTMS spec.
    if (opcode != FTMS_OP_REQUEST_CONTROL && opcode != FTMS_OP_RESET && !ftmsControlGranted) {
      result = FTMS_RESULT_CONTROL_NOT_PERMITTED;
    } else {
      switch (opcode) {
        case FTMS_OP_REQUEST_CONTROL:
          ftmsControlGranted = true;
          break;

        case FTMS_OP_RESET:
          ftmsHasTargetPower = false;
          break;

        case FTMS_OP_SET_TARGET_RESISTANCE: {
          if (v.length() < 3) { result = FTMS_RESULT_INVALID_PARAMETER; break; }
          int16_t raw = (int16_t)((uint8_t)v[1] | ((uint8_t)v[2] << 8)); // 0.1 resolution
          int level = constrain((int)round(raw / 10.0), RESISTANCE_MIN, RESISTANCE_MAX);
          resistance = level;
          sendResistance(resistance);
          ftmsHasTargetPower = false;
          break;
        }

        case FTMS_OP_SET_TARGET_POWER: {
          if (v.length() < 3) { result = FTMS_RESULT_INVALID_PARAMETER; break; }
          int16_t raw = (int16_t)((uint8_t)v[1] | ((uint8_t)v[2] << 8)); // 1W resolution
          ftmsTargetPowerWatts = raw;
          ftmsHasTargetPower = true;
          break;
        }

        case FTMS_OP_START_RESUME:
          // The physical bike is always "running" once connected; nothing to do.
          break;

        case FTMS_OP_STOP_PAUSE:
          ftmsHasTargetPower = false;
          break;

        case FTMS_OP_SET_SIM_PARAMS: {
          // Wind speed(2) + Grade(2) + Crr(1) + Cw(2) = 7 bytes after the opcode.
          if (v.length() < 8) { result = FTMS_RESULT_INVALID_PARAMETER; break; }
          int16_t gradeRaw = (int16_t)((uint8_t)v[3] | ((uint8_t)v[4] << 8)); // 0.01% resolution
          float gradePercent = gradeRaw / 100.0;
          // Rough, untuned mapping from simulated grade to the bike's 0-32
          // resistance scale, anchored at RESISTANCE_SIM_BASE for flat road
          // (0%) - tune the 0.7 multiplier and the base to taste on real
          // hardware. Still clamped to the true 0-32 range either side.
          int level = RESISTANCE_SIM_BASE + (int)round(constrain(gradePercent, -10.0f, 20.0f) * 0.7f);
          resistance = constrain(level, RESISTANCE_MIN, RESISTANCE_MAX);
          sendResistance(resistance);
          ftmsHasTargetPower = false;
          break;
        }

        default:
          result = FTMS_RESULT_OP_NOT_SUPPORTED;
          break;
      }
    }

    uint8_t resp[3] = {FTMS_OP_RESPONSE_CODE, opcode, result};
    ftmsControlPointChar->setValue(resp, 3);
    ftmsControlPointChar->indicate();
  }
};

// ======================================================
//  FTMS server - setup
// ======================================================
void setupFTMS() {
  BLEDevice::init("ESP32 Smart Bike");

  ftmsServer = BLEDevice::createServer();
  ftmsServer->setCallbacks(new FtmsServerCallbacks());

  BLEService* svc = ftmsServer->createService(BLEUUID(FTMS_SERVICE_UUID));

  // Fitness Machine Feature: 4 bytes Fitness Machine Features + 4 bytes
  // Target Setting Features (bitfields per the FTMS spec).
  ftmsFeatureChar = svc->createCharacteristic(BLEUUID(FTMS_FEATURE_UUID), BLECharacteristic::PROPERTY_READ);
  uint8_t featureData[8] = {0};
  uint32_t machineFeatures = (1UL << 1) | (1UL << 7) | (1UL << 14); // cadence, resistance level, power
  uint32_t targetFeatures  = (1UL << 2) | (1UL << 3) | (1UL << 13); // resistance target, power target, sim params
  memcpy(&featureData[0], &machineFeatures, 4);
  memcpy(&featureData[4], &targetFeatures, 4);
  ftmsFeatureChar->setValue(featureData, 8);

  ftmsIndoorBikeChar = svc->createCharacteristic(BLEUUID(FTMS_INDOOR_BIKE_UUID), BLECharacteristic::PROPERTY_NOTIFY);
  ftmsIndoorBikeChar->addDescriptor(new BLE2902());

  ftmsControlPointChar = svc->createCharacteristic(
      BLEUUID(FTMS_CONTROL_POINT_UUID),
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_INDICATE);
  ftmsControlPointChar->addDescriptor(new BLE2902());
  ftmsControlPointChar->setCallbacks(new FtmsControlPointCallbacks());

  ftmsStatusChar = svc->createCharacteristic(BLEUUID(FTMS_STATUS_UUID), BLECharacteristic::PROPERTY_NOTIFY);
  ftmsStatusChar->addDescriptor(new BLE2902());

  ftmsResistanceRangeChar = svc->createCharacteristic(BLEUUID(FTMS_RESISTANCE_RANGE_UUID), BLECharacteristic::PROPERTY_READ);
  int16_t range[3] = {RESISTANCE_MIN * 10, RESISTANCE_MAX * 10, 10}; // 0.1 resolution, matches native 0..32 scale
  ftmsResistanceRangeChar->setValue((uint8_t*)range, sizeof(range));

  svc->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLEUUID(FTMS_SERVICE_UUID));
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
}

// ======================================================
//  FTMS server - periodic Indoor Bike Data notification
// ======================================================
void notifyIndoorBikeData() {
  if (!ftmsCentralConnected) return;
  unsigned long now = millis();
  if (now - lastFtmsNotify < 1000) return;
  lastFtmsNotify = now;

  // Flags: bit0=0 (speed present) | bit2 (cadence) | bit5 (resistance) | bit6 (power)
  uint16_t flags = 0x0064;
  uint16_t speedRaw = (uint16_t)(speed_mph * 1.60934f * 100.0f);      // km/h, 0.01 resolution
  uint16_t cadenceRaw = (uint16_t)(cadence * 2);                      // rpm, 0.5 resolution
  int16_t  resistanceRaw = (int16_t)resistance;                       // unitless level
  int16_t  powerRaw = (int16_t)powerVal;                              // watts

  uint8_t buf[10];
  int idx = 0;
  memcpy(&buf[idx], &flags, 2); idx += 2;
  memcpy(&buf[idx], &speedRaw, 2); idx += 2;
  memcpy(&buf[idx], &cadenceRaw, 2); idx += 2;
  memcpy(&buf[idx], &resistanceRaw, 2); idx += 2;
  memcpy(&buf[idx], &powerRaw, 2); idx += 2;

  ftmsIndoorBikeChar->setValue(buf, idx);
  ftmsIndoorBikeChar->notify();
}

// ======================================================
//  FTMS server - basic ERG emulation for Set Target Power
// ======================================================
void updateErgControl() {
  if (!ftmsHasTargetPower || cadence <= 0) return;
  unsigned long now = millis();
  if (now - lastErgAdjust < 2000) return; // give the rider/bike time to settle between steps
  lastErgAdjust = now;

  int currentPower = getPower(cadence, resistance);
  int diff = ftmsTargetPowerWatts - currentPower;
  if (abs(diff) < 8) return; // close enough, avoid hunting

  int newLevel = constrain(resistance + (diff > 0 ? 1 : -1), RESISTANCE_MIN, RESISTANCE_MAX);
  if (newLevel != resistance) {
    resistance = newLevel;
    sendResistance(resistance);
  }
}

// ======================================================
//  Setup & Loop
// ======================================================
void setup() {
  Serial.begin(115200);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, TFT_BACKLIGHT_ON);

  // Start the SPI for the touchscreen and init the touchscreen
  touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  touchscreen.begin(touchscreenSPI);
  touchscreen.setRotation(1); // match tft.setRotation

  tft.init();
  tft.setRotation(1);

  drawStaticDashboard();

  // BLEDevice::init() is called once, inside setupFTMS(), and serves both
  // the client role below (scanning/connecting to the bike) and the FTMS
  // server role (advertising to apps like Zwift).
  setupFTMS();

  scanner = BLEDevice::getScan();
  scanner->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  scanner->setActiveScan(true);

  last_millis = millis();
  last_cadence = millis();
}

void loop() {
  unsigned long now = millis();
  unsigned long dt = now - last_millis;
  last_millis = now;

  if (doConnect) {
    if (!connectToServer()) uiState = SEARCHING;
  }

  if (!connected && uiState == SEARCHING) {
    // NOTE: this is a blocking 3s scan, so loop() (and therefore the touch
    // handler / animations) pauses for up to 3s at a time while searching.
    // Fine for this use case, but worth knowing if you extend the UI further.
    scanner->start(3, false);
  }

  if (connected) {
    runtime += dt;

    if (cadence > 0) {
      speed_mph = estimateSpeed(cadence);
      float hours = dt / 3600000.0;
      distance_miles += speed_mph * hours;
      calories += powerVal * hours * 14.4;
    }

    if (now - last_cadence > ScreenTimeoutMillis && uiState == CONNECTED) {
      uiState = PAUSED;
    }
  }

  updateDisplay();
  handleTouch();
  updateButtonFeedback();
  notifyIndoorBikeData();
  updateErgControl();
  delay(50);
}

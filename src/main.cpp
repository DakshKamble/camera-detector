#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>

// =====================================================
// RF HUNTER
// XIAO ESP32-S3 + OLED + AD8318 SIMULATOR + BUZZER
//
// LED RING:
// SIMPLE ON/OFF ONLY
//
// BUZZER:
// ON at signal strength >= 6
// OFF at signal strength <= 4 (hysteresis to stop flicker)
//
// -----------------------------------------------------
// FIXES APPLIED (from potentiometer test bench issues):
//
// 1) DIRECTION FIX
//    Original code modeled a REAL AD8318: stronger RF ->
//    LOWER voltage -> LOWER ADC -> HIGHER strength. That
//    means turning a pot UP (raising raw ADC) was read as
//    a WEAKER signal, which is why "raw goes up but never
//    triggers" happened. For potentiometer bench testing,
//    we now use: pot UP -> raw UP -> strength UP.
//    (If you later connect a real AD8318, see the note
//    inside calculateSignalStrength() to flip it back.)
//
// 2) SENSITIVITY WINDOW FIX
//    Default sensitivity (100) squeezed the entire 0-10
//    strength range into ~2.4% of the ADC's 0-4095 range,
//    so tiny pot movement / noise blew straight through it
//    ("shoots straight up"). Sensitivity range and default
//    increased so a real pot sweep is gradual.
//
// 3) BUZZER HYSTERESIS FIX
//    Buzzer used to snap on/off exactly at the threshold,
//    which caused flicker/chatter near strength == 6 due to
//    noise. It now turns ON at >= 6 and only turns back OFF
//    at <= 4, so a decreasing pot doesn't "still keep
//    buzzing" from jitter around the threshold.
// =====================================================


// =====================================================
// OLED
// =====================================================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C

// XIAO ESP32-S3
#define OLED_SDA 5     // D4
#define OLED_SCL 6     // D5

Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  -1
);


// =====================================================
// PIN MAPPING
// =====================================================

// AD8318 simulated by potentiometer
#define RF_SENSOR_PIN 1     // D0

// -----------------------------------------------------
// BUTTONS
//
// D8  = GPIO7
// D9  = GPIO8
// D10 = GPIO9
// D7  = GPIO44
// -----------------------------------------------------

#define BUTTON_UP       8   // D9
#define BUTTON_DOWN     7   // D8
#define BUTTON_SELECT  44   // D7

// Buzzer
#define BUZZER_PIN      9   // D10

// WS2812 / NeoPixel ring
#define LED_RING_PIN    2   // D1
#define LED_RING_COUNT  16


// =====================================================
// ADC
// =====================================================

#define ADC_MAX 4095.0
#define ADC_VOLTAGE 3.3


// =====================================================
// MOVING AVERAGE
// =====================================================

const int NUM_READINGS = 20;

int readings[NUM_READINGS];

int readIndex = 0;

long total = 0;


// =====================================================
// BASELINE
// =====================================================

int baselineRaw = 0;

int minRaw = 4095;

int maxRaw = 0;


// =====================================================
// SENSITIVITY
// =====================================================
//
// NOTE: These were widened for potentiometer bench testing.
// The old values (100 / 20 / 500 / 20) packed the whole
// 0-10 strength range into a couple percent of ADC travel,
// which made the reading jump straight to max with barely
// any pot movement. These new values spread it across a
// much more realistic chunk of the 0-4095 ADC range so a
// pot turn gives a smooth, gradual strength change.
//
// If you switch back to a real AD8318 with an actual RF
// source, you'll likely want to tune these back down since
// real RF swings are smaller and faster than a hand-turned
// pot.
// =====================================================

int sensitivity = 1500;

const int MIN_SENSITIVITY = 200;
const int MAX_SENSITIVITY = 3500;
const int SENSITIVITY_STEP = 100;


// =====================================================
// BUZZER
// =====================================================

const int BUZZER_ON_THRESHOLD  = 6;  // turn ON at/above this
const int BUZZER_OFF_THRESHOLD = 4;  // turn OFF at/below this (hysteresis gap)

bool buzzerActive = false;


// =====================================================
// LED RING
// =====================================================
//
// LED ring has ONLY ONE JOB:
//
// ON  = all LEDs ON
// OFF = all LEDs OFF
//
// It does NOT:
// - show RF strength
// - change color with RF
// - react to buzzer
// - react to RF module
// - run animations
// =====================================================

Adafruit_NeoPixel ledRing(
  LED_RING_COUNT,
  LED_RING_PIN,
  NEO_GRB + NEO_KHZ800
);

bool ledRingEnabled = true;


// =====================================================
// RF MODULE
// =====================================================

bool rfModuleEnabled = true;


// =====================================================
// SETTINGS MENU
// =====================================================

bool menuActive = false;

int menuIndex = 0;

const unsigned long SELECT_HOLD_TIME = 800;

const unsigned long MENU_TIMEOUT = 5000;

unsigned long lastMenuInteraction = 0;


// =====================================================
// BUTTON STATE
// =====================================================

// SELECT
bool lastSelectState = HIGH;

unsigned long selectPressStart = 0;

// UP
bool lastUpState = HIGH;

unsigned long lastUpTime = 0;

// DOWN
bool lastDownState = HIGH;

unsigned long lastDownTime = 0;

// General debounce
const unsigned long BUTTON_DEBOUNCE = 50;


// =====================================================
// AD8318 APPROXIMATE MODEL
// =====================================================

const float AD8318_SLOPE = -25.0;

const float AD8318_INTERCEPT = 20.0;


// =====================================================
// FUNCTION PROTOTYPES
// =====================================================

void calibrateBaseline();

void checkButtons();

void handleNormalButtons();

void handleMenuButtons();

void handleSelectButton();

int calculateSignalStrength(
  int average
);

void updateDisplay(
  int average,
  float voltage,
  float dBm,
  int signalStrength
);

void updateBuzzer(
  int signalStrength
);

void printSerial(
  int average,
  float voltage,
  float dBm,
  int strength
);

void updateLedRing();

void enterMenu();

void exitMenu();

void showMenu();


// =====================================================
// RAW -> VOLTAGE
// =====================================================

float rawToVoltage(int raw)
{
  return (
    ((float)raw / ADC_MAX)
    * ADC_VOLTAGE
  );
}


// =====================================================
// VOLTAGE -> dBm
// =====================================================

float voltageToDbm(float voltage)
{
  float dBm =
    (voltage / AD8318_SLOPE)
    + AD8318_INTERCEPT;

  if (dBm < -60.0)
  {
    dBm = -60.0;
  }

  if (dBm > 0.0)
  {
    dBm = 0.0;
  }

  return dBm;
}


// =====================================================
// LED RING
// =====================================================
//
// SIMPLE ON/OFF ONLY
//
// This function is the ONLY place where the LED ring
// is controlled.
//
// ON  -> all LEDs white
// OFF -> all LEDs off
// =====================================================

void updateLedRing()
{
  if (ledRingEnabled)
  {
    // All LEDs ON

    for (
      int i = 0;
      i < LED_RING_COUNT;
      i++
    )
    {
      ledRing.setPixelColor(
        i,
        ledRing.Color(
          255,
          255,
          255
        )
      );
    }
  }
  else
  {
    // All LEDs OFF

    ledRing.clear();
  }

  ledRing.show();
}


// =====================================================
// SETUP
// =====================================================

void setup()
{
  Serial.begin(115200);

  delay(1000);

  Serial.println();
  Serial.println("==============================");
  Serial.println("       RF HUNTER");
  Serial.println("==============================");
  Serial.println("XIAO ESP32-S3");
  Serial.println("AD8318 Simulator");
  Serial.println("D9  = UP");
  Serial.println("D8  = DOWN");
  Serial.println("D7  = SELECT");
  Serial.println("D10 = BUZZER");
  Serial.println("D1  = LED RING");
  Serial.println("==============================");


  // ===================================================
  // GPIO
  // ===================================================

  pinMode(
    RF_SENSOR_PIN,
    INPUT
  );

  pinMode(
    BUTTON_UP,
    INPUT_PULLUP
  );

  pinMode(
    BUTTON_DOWN,
    INPUT_PULLUP
  );

  pinMode(
    BUTTON_SELECT,
    INPUT_PULLUP
  );

  pinMode(
    BUZZER_PIN,
    OUTPUT
  );

  digitalWrite(
    BUZZER_PIN,
    LOW
  );


  // ===================================================
  // LED RING
  // ===================================================
  //
  // No startup animation.
  // Just apply current ON/OFF setting.
  // ===================================================

  ledRing.begin();

  ledRing.setBrightness(255);

  updateLedRing();


  // ===================================================
  // ADC
  // ===================================================

  analogReadResolution(12);

  analogSetPinAttenuation(
    RF_SENSOR_PIN,
    ADC_11db
  );


  // ===================================================
  // OLED
  // ===================================================

  Wire.begin(
    OLED_SDA,
    OLED_SCL
  );


  if (
    !display.begin(
      SSD1306_SWITCHCAPVCC,
      OLED_ADDR
    )
  )
  {
    Serial.println(
      "OLED initialization failed!"
    );

    while (true)
    {
      delay(100);
    }
  }


  // ===================================================
  // OLED START SCREEN
  // ===================================================

  display.clearDisplay();

  display.setTextColor(
    SSD1306_WHITE
  );

  display.setTextSize(1);

  display.setCursor(
    0,
    0
  );

  display.println(
    "RF HUNTER"
  );

  display.setCursor(
    0,
    15
  );

  display.println(
    "XIAO ESP32-S3"
  );

  display.setCursor(
    0,
    30
  );

  display.println(
    "AD8318 Simulator"
  );

  display.setCursor(
    0,
    45
  );

  display.println(
    "Starting..."
  );

  display.display();

  delay(1000);


  // ===================================================
  // INITIAL CALIBRATION
  // ===================================================

  calibrateBaseline();


  // ===================================================
  // READY SCREEN
  // ===================================================

  display.clearDisplay();

  display.setCursor(
    0,
    0
  );

  display.println(
    "RF HUNTER READY"
  );

  display.setCursor(
    0,
    15
  );

  display.print(
    "Baseline: "
  );

  display.println(
    baselineRaw
  );

  display.setCursor(
    0,
    28
  );

  display.print(
    "Buzz >= "
  );

  display.print(
    BUZZER_ON_THRESHOLD
  );

  display.println(
    "/10"
  );

  display.setCursor(
    0,
    45
  );

  display.println(
    "Hold SEL = MENU"
  );

  display.display();

  delay(1500);
}


// =====================================================
// CALIBRATE BASELINE
// =====================================================

void calibrateBaseline()
{
  Serial.println(
    "Calibrating baseline..."
  );


  display.clearDisplay();

  display.setCursor(
    0,
    0
  );

  display.println(
    "CALIBRATING..."
  );

  display.setCursor(
    0,
    15
  );

  display.println(
    "Keep RF quiet"
  );

  display.setCursor(
    0,
    28
  );

  display.println(
    "Please wait..."
  );

  display.display();


  long sum = 0;


  for (
    int i = 0;
    i < 100;
    i++
  )
  {
    sum += analogRead(
      RF_SENSOR_PIN
    );

    delay(10);
  }


  baselineRaw =
    sum / 100;


  minRaw =
    baselineRaw;

  maxRaw =
    baselineRaw;


  // Reset moving average

  total = 0;

  readIndex = 0;


  for (
    int i = 0;
    i < NUM_READINGS;
    i++
  )
  {
    readings[i] =
      baselineRaw;

    total +=
      baselineRaw;
  }


  Serial.print(
    "Baseline RAW = "
  );

  Serial.println(
    baselineRaw
  );


  // Make sure buzzer is OFF after calibration

  noTone(
    BUZZER_PIN
  );

  digitalWrite(
    BUZZER_PIN,
    LOW
  );

  buzzerActive = false;
}


// =====================================================
// ENTER MENU
// =====================================================

void enterMenu()
{
  menuActive = true;

  menuIndex = 0;

  lastMenuInteraction =
    millis();

  // Stop buzzer while menu is open

  noTone(
    BUZZER_PIN
  );

  digitalWrite(
    BUZZER_PIN,
    LOW
  );

  buzzerActive = false;

  showMenu();

  Serial.println(
    "SETTINGS MENU OPEN"
  );
}


// =====================================================
// EXIT MENU
// =====================================================

void exitMenu()
{
  menuActive = false;

  noTone(
    BUZZER_PIN
  );

  digitalWrite(
    BUZZER_PIN,
    LOW
  );

  buzzerActive = false;

  Serial.println(
    "SETTINGS MENU CLOSED"
  );
}


// =====================================================
// SHOW MENU
// =====================================================

void showMenu()
{
  display.clearDisplay();

  display.setTextColor(
    SSD1306_WHITE
  );

  display.setTextSize(1);


  // ---------------------------------------------------
  // TITLE
  // ---------------------------------------------------

  display.setCursor(
    0,
    0
  );

  display.println(
    "SETTINGS"
  );


  // ---------------------------------------------------
  // LED RING
  // ---------------------------------------------------

  display.setCursor(
    0,
    15
  );

  display.print(
    menuIndex == 0
      ? "> "
      : "  "
  );

  display.print(
    "LED RING: "
  );

  display.println(
    ledRingEnabled
      ? "ON"
      : "OFF"
  );


  // ---------------------------------------------------
  // RF MODULE
  // ---------------------------------------------------

  display.setCursor(
    0,
    29
  );

  display.print(
    menuIndex == 1
      ? "> "
      : "  "
  );

  display.print(
    "RF MODULE: "
  );

  display.println(
    rfModuleEnabled
      ? "ON"
      : "OFF"
  );


  // ---------------------------------------------------
  // HELP
  // ---------------------------------------------------

  display.setCursor(
    0,
    46
  );

  display.println(
    "UP/DN Move"
  );

  display.setCursor(
    0,
    56
  );

  display.println(
    "SEL Toggle"
  );


  display.display();
}


// =====================================================
// MENU BUTTONS
// =====================================================

void handleMenuButtons()
{
  unsigned long now =
    millis();


  // ---------------------------------------------------
  // MENU TIMEOUT
  // ---------------------------------------------------

  if (
    now -
    lastMenuInteraction
    >= MENU_TIMEOUT
  )
  {
    exitMenu();

    return;
  }


  // ---------------------------------------------------
  // UP
  // ---------------------------------------------------

  bool upState =
    digitalRead(
      BUTTON_UP
    );


  if (
    upState == LOW &&
    lastUpState == HIGH &&
    now - lastUpTime >= BUTTON_DEBOUNCE
  )
  {
    menuIndex--;


    if (
      menuIndex < 0
    )
    {
      menuIndex = 1;
    }


    lastUpTime =
      now;

    lastMenuInteraction =
      now;

    showMenu();
  }


  lastUpState =
    upState;


  // ---------------------------------------------------
  // DOWN
  // ---------------------------------------------------

  bool downState =
    digitalRead(
      BUTTON_DOWN
    );


  if (
    downState == LOW &&
    lastDownState == HIGH &&
    now - lastDownTime >= BUTTON_DEBOUNCE
  )
  {
    menuIndex++;


    if (
      menuIndex > 1
    )
    {
      menuIndex = 0;
    }


    lastDownTime =
      now;

    lastMenuInteraction =
      now;

    showMenu();
  }


  lastDownState =
    downState;


  // ---------------------------------------------------
  // SELECT
  // ---------------------------------------------------

  handleSelectButton();
}


// =====================================================
// SELECT BUTTON
// =====================================================

void handleSelectButton()
{
  unsigned long now =
    millis();


  bool currentState =
    digitalRead(
      BUTTON_SELECT
    );


  // ---------------------------------------------------
  // BUTTON PRESSED
  // ---------------------------------------------------

  if (
    currentState == LOW &&
    lastSelectState == HIGH
  )
  {
    selectPressStart =
      now;
  }


  // ---------------------------------------------------
  // BUTTON RELEASED
  // ---------------------------------------------------

  if (
    currentState == HIGH &&
    lastSelectState == LOW
  )
  {
    unsigned long pressTime =
      now -
      selectPressStart;


    // -------------------------------------------------
    // SHORT PRESS = TOGGLE
    // -------------------------------------------------

    if (
      pressTime <
      SELECT_HOLD_TIME
    )
    {
      // ===============================================
      // LED RING
      // ===============================================

      if (
        menuIndex == 0
      )
      {
        ledRingEnabled =
          !ledRingEnabled;


        // Immediately apply LED state

        updateLedRing();


        Serial.print(
          "LED RING = "
        );

        Serial.println(
          ledRingEnabled
            ? "ON"
            : "OFF"
        );
      }


      // ===============================================
      // RF MODULE
      // ===============================================

      else
      {
        rfModuleEnabled =
          !rfModuleEnabled;


        if (
          !rfModuleEnabled
        )
        {
          noTone(
            BUZZER_PIN
          );

          digitalWrite(
            BUZZER_PIN,
            LOW
          );

          buzzerActive =
            false;
        }


        Serial.print(
          "RF MODULE = "
        );

        Serial.println(
          rfModuleEnabled
            ? "ON"
            : "OFF"
        );
      }


      lastMenuInteraction =
        now;

      showMenu();
    }
  }


  lastSelectState =
    currentState;
}


// =====================================================
// NORMAL BUTTONS
// =====================================================

void handleNormalButtons()
{
  unsigned long now =
    millis();


  // ---------------------------------------------------
  // UP
  // ---------------------------------------------------

  bool upState =
    digitalRead(
      BUTTON_UP
    );


  if (
    upState == LOW &&
    lastUpState == HIGH &&
    now - lastUpTime >= BUTTON_DEBOUNCE
  )
  {
    sensitivity -=
      SENSITIVITY_STEP;


    if (
      sensitivity <
      MIN_SENSITIVITY
    )
    {
      sensitivity =
        MIN_SENSITIVITY;
    }


    Serial.print(
      "UP / Sensitivity = "
    );

    Serial.println(
      sensitivity
    );


    lastUpTime =
      now;
  }


  lastUpState =
    upState;


  // ---------------------------------------------------
  // DOWN
  // ---------------------------------------------------

  bool downState =
    digitalRead(
      BUTTON_DOWN
    );


  if (
    downState == LOW &&
    lastDownState == HIGH &&
    now - lastDownTime >= BUTTON_DEBOUNCE
  )
  {
    sensitivity +=
      SENSITIVITY_STEP;


    if (
      sensitivity >
      MAX_SENSITIVITY
    )
    {
      sensitivity =
        MAX_SENSITIVITY;
    }


    Serial.print(
      "DOWN / Sensitivity = "
    );

    Serial.println(
      sensitivity
    );


    lastDownTime =
      now;
  }


  lastDownState =
    downState;
}


// =====================================================
// CHECK BUTTONS
// =====================================================

void checkButtons()
{
  unsigned long now =
    millis();


  // ---------------------------------------------------
  // MENU ACTIVE
  // ---------------------------------------------------

  if (
    menuActive
  )
  {
    handleMenuButtons();

    return;
  }


  // ---------------------------------------------------
  // SELECT HOLD DETECTION
  // ---------------------------------------------------

  bool currentSelect =
    digitalRead(
      BUTTON_SELECT
    );


  if (
    currentSelect == LOW &&
    lastSelectState == HIGH
  )
  {
    selectPressStart =
      now;
  }


  if (
    currentSelect == LOW &&
    now - selectPressStart >=
      SELECT_HOLD_TIME
  )
  {
    enterMenu();


    // Prevent immediate re-trigger

    lastSelectState =
      LOW;

    return;
  }


  lastSelectState =
    currentSelect;


  // ---------------------------------------------------
  // UP / DOWN
  // ---------------------------------------------------

  handleNormalButtons();
}


// =====================================================
// SIGNAL STRENGTH
// =====================================================

int calculateSignalStrength(
  int average
)
{
  /*
     POTENTIOMETER BENCH-TEST MODEL (current):

     Turning pot UP
        ↓
     Higher ADC (average)
        ↓
     Higher strength

     This is the INVERSE of a real AD8318, where stronger
     RF produces a LOWER output voltage / lower ADC value.

     ---------------------------------------------------
     TO SWITCH BACK TO A REAL AD8318 MODULE:

     Change the map() call below back to:

         map(average, baselineRaw, baselineRaw - sensitivity, 0, 10);

     i.e. strength rises as 'average' falls BELOW baseline,
     matching: Strong RF -> Lower VOUT -> Lower ADC -> Higher strength.
     ---------------------------------------------------
  */


  int strength =
    map(
      average,
      baselineRaw,
      baselineRaw + sensitivity,
      0,
      10
    );


  strength =
    constrain(
      strength,
      0,
      10
    );


  return strength;
}


// =====================================================
// BUZZER
// =====================================================
//
// IMPORTANT:
//
// >= 6  -> buzzer ON
// <= 4  -> buzzer OFF  (2-point hysteresis gap prevents
//                        flicker/chatter from noise sitting
//                        right around the threshold)
//
// No timed tone.
// No repeated tone() calls.
// =====================================================

void updateBuzzer(
  int signalStrength
)
{
  // ---------------------------------------------------
  // SIGNAL AT/ABOVE ON-THRESHOLD -> TURN ON (if not already)
  // ---------------------------------------------------

  if (
    !buzzerActive &&
    signalStrength >= BUZZER_ON_THRESHOLD
  )
  {
    buzzerActive =
      true;


    tone(
      BUZZER_PIN,
      1500
    );


    Serial.println(
      "*** BUZZER ON ***"
    );

    return;
  }


  // ---------------------------------------------------
  // SIGNAL AT/BELOW OFF-THRESHOLD -> TURN OFF (if currently on)
  // ---------------------------------------------------

  if (
    buzzerActive &&
    signalStrength <= BUZZER_OFF_THRESHOLD
  )
  {
    noTone(
      BUZZER_PIN
    );

    digitalWrite(
      BUZZER_PIN,
      LOW
    );

    buzzerActive =
      false;


    Serial.println(
      "*** BUZZER OFF ***"
    );

    return;
  }


  // ---------------------------------------------------
  // OTHERWISE: hold current state (this is the hysteresis
  // "dead band" between OFF_THRESHOLD and ON_THRESHOLD),
  // but make sure the pin is physically consistent.
  // ---------------------------------------------------

  if (
    !buzzerActive
  )
  {
    // Extra safety: keep buzzer physically LOW while inactive

    noTone(
      BUZZER_PIN
    );

    digitalWrite(
      BUZZER_PIN,
      LOW
    );
  }
}


// =====================================================
// OLED DISPLAY
// =====================================================

void updateDisplay(
  int average,
  float voltage,
  float dBm,
  int signalStrength
)
{
  display.clearDisplay();

  display.setTextColor(
    SSD1306_WHITE
  );

  display.setTextSize(1);


  // ---------------------------------------------------
  // TITLE
  // ---------------------------------------------------

  display.setCursor(
    0,
    0
  );

  display.println(
    "RF SIGNAL SCANNER"
  );


  // ---------------------------------------------------
  // RAW
  // ---------------------------------------------------

  display.setCursor(
    0,
    12
  );

  display.print(
    "RAW:"
  );

  display.print(
    average
  );


  // ---------------------------------------------------
  // VOLTAGE
  // ---------------------------------------------------

  display.setCursor(
    75,
    12
  );

  display.print(
    voltage,
    2
  );

  display.print(
    "V"
  );


  // ---------------------------------------------------
  // dBm
  // ---------------------------------------------------

  display.setCursor(
    0,
    23
  );

  display.print(
    "RF:"
  );

  display.print(
    dBm,
    1
  );

  display.print(
    " dBm"
  );


  // ---------------------------------------------------
  // SENSITIVITY
  // ---------------------------------------------------

  display.setCursor(
    75,
    23
  );

  display.print(
    "S:"
  );

  display.print(
    sensitivity
  );


  // ---------------------------------------------------
  // STRENGTH
  // ---------------------------------------------------

  display.setCursor(
    0,
    34
  );

  display.print(
    "Strength:"
  );

  display.print(
    signalStrength
  );

  display.print(
    "/10"
  );


  // ---------------------------------------------------
  // STATUS
  // ---------------------------------------------------

  display.setCursor(
    75,
    34
  );


  if (
    !rfModuleEnabled
  )
  {
    display.print(
      "RF OFF"
    );
  }
  else if (
    signalStrength >=
    BUZZER_ON_THRESHOLD
  )
  {
    display.print(
      "ALERT!"
    );
  }
  else
  {
    display.print(
      "QUIET"
    );
  }


  // ---------------------------------------------------
  // OLED SIGNAL BARS
  //
  // IMPORTANT:
  // These are OLED graphics.
  // They have NOTHING to do with NeoPixel ring.
  // ---------------------------------------------------

  int barWidth = 10;

  int spacing = 2;

  int startX = 4;

  int baseY = 62;


  for (
    int i = 0;
    i < 10;
    i++
  )
  {
    int height;


    if (
      i < signalStrength
    )
    {
      height =
        map(
          i,
          0,
          9,
          5,
          22
        );
    }
    else
    {
      height = 2;
    }


    display.fillRect(
      startX +
        i *
        (barWidth + spacing),

      baseY -
        height,

      barWidth,

      height,

      SSD1306_WHITE
    );
  }


  display.display();
}


// =====================================================
// SERIAL
// =====================================================

void printSerial(
  int average,
  float voltage,
  float dBm,
  int strength
)
{
  Serial.print(
    "RAW="
  );

  Serial.print(
    average
  );


  Serial.print(
    "  V="
  );

  Serial.print(
    voltage,
    3
  );


  Serial.print(
    "V  RF="
  );

  Serial.print(
    dBm,
    1
  );


  Serial.print(
    "dBm  Sens="
  );

  Serial.print(
    sensitivity
  );


  Serial.print(
    "  Strength="
  );

  Serial.print(
    strength
  );


  if (
    strength >=
    BUZZER_ON_THRESHOLD
  )
  {
    Serial.println(
      "  *** RF ALERT ***"
    );
  }
  else
  {
    Serial.println();
  }
}


// =====================================================
// MAIN LOOP
// =====================================================

void loop()
{
  checkButtons();


  // ---------------------------------------------------
  // MENU ACTIVE
  // ---------------------------------------------------

  if (
    menuActive
  )
  {
    delay(10);

    return;
  }


  // ---------------------------------------------------
  // RF MODULE OFF
  //
  // IMPORTANT:
  // LED ring is NOT touched here.
  // LED ring keeps its selected ON/OFF state.
  // ---------------------------------------------------

  if (
    !rfModuleEnabled
  )
  {
    updateDisplay(
      0,
      0.0,
      -60.0,
      0
    );


    noTone(
      BUZZER_PIN
    );

    digitalWrite(
      BUZZER_PIN,
      LOW
    );

    buzzerActive =
      false;


    delay(100);

    return;
  }


  // ---------------------------------------------------
  // MOVING AVERAGE
  // ---------------------------------------------------

  total -=
    readings[readIndex];


  readings[readIndex] =
    analogRead(
      RF_SENSOR_PIN
    );


  total +=
    readings[readIndex];


  readIndex++;


  if (
    readIndex >=
    NUM_READINGS
  )
  {
    readIndex = 0;
  }


  int average =
    total /
    NUM_READINGS;


  // ---------------------------------------------------
  // MIN / MAX
  // ---------------------------------------------------

  if (
    average <
    minRaw
  )
  {
    minRaw =
      average;
  }


  if (
    average >
    maxRaw
  )
  {
    maxRaw =
      average;
  }


  // ---------------------------------------------------
  // CONVERT
  // ---------------------------------------------------

  float voltage =
    rawToVoltage(
      average
    );


  float dBm =
    voltageToDbm(
      voltage
    );


  // ---------------------------------------------------
  // SIGNAL STRENGTH
  // ---------------------------------------------------

  int signalStrength =
    calculateSignalStrength(
      average
    );


  // ---------------------------------------------------
  // SERIAL
  // ---------------------------------------------------

  printSerial(
    average,
    voltage,
    dBm,
    signalStrength
  );


  // ---------------------------------------------------
  // OLED
  // ---------------------------------------------------

  updateDisplay(
    average,
    voltage,
    dBm,
    signalStrength
  );


  // ---------------------------------------------------
  // BUZZER
  // ---------------------------------------------------

  updateBuzzer(
    signalStrength
  );


  // ---------------------------------------------------
  // LED RING
  //
  // NOTHING HERE.
  //
  // The LED ring is controlled ONLY from the menu.
  // ---------------------------------------------------

  delay(50);
}

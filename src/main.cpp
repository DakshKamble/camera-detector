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
// OFF immediately below 6
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

int sensitivity = 100;

const int MIN_SENSITIVITY = 20;
const int MAX_SENSITIVITY = 500;
const int SENSITIVITY_STEP = 20;


// =====================================================
// BUZZER
// =====================================================

const int BUZZER_THRESHOLD = 6;

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
    BUZZER_THRESHOLD
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
     AD8318:

     Strong RF
        ↓
     Lower VOUT
        ↓
     Lower ADC
        ↓
     Higher strength
  */


  int strength =
    map(
      average,
      baselineRaw,
      baselineRaw - sensitivity,
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
// <  6  -> buzzer OFF immediately
//
// No timed tone.
// No repeated tone() calls.
// =====================================================

void updateBuzzer(
  int signalStrength
)
{
  // ---------------------------------------------------
  // SIGNAL ABOVE THRESHOLD
  // ---------------------------------------------------

  if (
    signalStrength >=
    BUZZER_THRESHOLD
  )
  {
    if (
      !buzzerActive
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
    }


    return;
  }


  // ---------------------------------------------------
  // SIGNAL BELOW THRESHOLD
  // ---------------------------------------------------

  if (
    buzzerActive
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
  }
  else
  {
    // Extra safety:
    // keep buzzer physically LOW

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
    BUZZER_THRESHOLD
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
    BUZZER_THRESHOLD
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
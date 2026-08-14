#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// =====================================================
// RF HUNTER
// Seeed Studio XIAO ESP32-S3
//
// WOKWI CONNECTIONS
//
// AD8318 simulator (POT)
// POT SIG  -> D0 / GPIO1
//
// OLED
// SDA -> D4 / GPIO5
// SCL -> D5 / GPIO6
//
// BUTTONS
// Button 1 -> D9 / GPIO10
// Button 2 -> D8 / GPIO8
// Button 3 -> D7 / GPIO44
// =====================================================


// =====================================================
// OLED
// =====================================================

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_ADDR 0x3C

#define OLED_SDA 5
#define OLED_SCL 6

Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  -1
);


// =====================================================
// PIN DEFINITIONS
// =====================================================

// POT simulates AD8318 VOUT
#define RF_SENSOR_PIN 1       // D0 = GPIO1

// Buttons according to your diagram.json
#define BUTTON_UP 10          // D9 = GPIO10
#define BUTTON_DOWN 8         // D8 = GPIO8
#define BUTTON_MODE 44        // D7 = GPIO44


// =====================================================
// ADC SETTINGS
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
// RF BASELINE
// =====================================================

int baselineRaw = 0;

int minRaw = 4095;

int maxRaw = 0;


// =====================================================
// SENSITIVITY
// =====================================================
//
// Smaller value = MORE sensitive
// Larger value = LESS sensitive
//

int sensitivity = 100;

const int MIN_SENSITIVITY = 20;

const int MAX_SENSITIVITY = 500;

const int SENSITIVITY_STEP = 20;


// =====================================================
// BUTTON DEBOUNCE
// =====================================================

unsigned long lastButtonTime = 0;

const unsigned long BUTTON_DELAY = 200;


// =====================================================
// AD8318 APPROXIMATE MODEL
// =====================================================
//
// The potentiometer is being used to simulate
// the AD8318 VOUT.
//
// POT:
//     0V -> 3.3V
//
// AD8318:
//     Higher RF power -> lower VOUT
//     Lower RF power  -> higher VOUT
//
// These values are only an approximation.
// Actual AD8318 modules should be calibrated.
// =====================================================

const float AD8318_SLOPE = -25.0;

const float AD8318_INTERCEPT = 20.0;

void calibrateBaseline();
// =====================================================
// ADC -> VOLTAGE
// =====================================================

float rawToVoltage(int raw)
{
  return ((float)raw / ADC_MAX) * ADC_VOLTAGE;
}


// =====================================================
// VOLTAGE -> dBm
// =====================================================

float voltageToDbm(float voltage)
{
  /*
     Approximate relationship:

     dBm = VOUT / slope + intercept

     Because the AD8318 output decreases
     as RF power increases.
  */

  float dBm =
    (voltage / AD8318_SLOPE)
    + AD8318_INTERCEPT;


  // Limit display range

  if (dBm < -60.0)
    dBm = -60.0;

  if (dBm > 0.0)
    dBm = 0.0;


  return dBm;
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
  Serial.println("AD8318 simulated by POT");
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
    BUTTON_MODE,
    INPUT_PULLUP
  );


  // ===================================================
  // OLED
  // ===================================================

  Wire.begin(
    OLED_SDA,
    OLED_SCL
  );


  if (!display.begin(
        SSD1306_SWITCHCAPVCC,
        OLED_ADDR))
  {
    Serial.println(
      "OLED initialization failed!"
    );

    while (true)
    {
      delay(100);
    }
  }


  display.clearDisplay();

  display.setTextColor(
    SSD1306_WHITE
  );

  display.setTextSize(1);


  display.setCursor(0, 0);

  display.println(
    "RF HUNTER"
  );


  display.setCursor(0, 15);

  display.println(
    "XIAO ESP32-S3"
  );


  display.setCursor(0, 30);

  display.println(
    "AD8318 Simulator"
  );


  display.display();

  delay(1500);


  // ===================================================
  // CALIBRATION
  // ===================================================

  display.clearDisplay();

  display.setCursor(0, 0);

  display.println(
    "CALIBRATING..."
  );


  display.setCursor(0, 15);

  display.println(
    "Set RF environment"
  );


  display.setCursor(0, 27);

  display.println(
    "to quiet"
  );


  display.display();


  Serial.println(
    "Calibrating..."
  );


  long sum = 0;


  for (int i = 0; i < 100; i++)
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


  // Initialize moving average

  for (
    int i = 0;
    i < NUM_READINGS;
    i++
  )
  {
    readings[i] =
      baselineRaw;
  }


  total =
    (long)baselineRaw *
    NUM_READINGS;


  Serial.print(
    "Baseline RAW = "
  );

  Serial.println(
    baselineRaw
  );


  // ===================================================
  // READY
  // ===================================================

  display.clearDisplay();


  display.setCursor(0, 0);

  display.println(
    "RF HUNTER READY"
  );


  display.setCursor(0, 15);

  display.print(
    "Baseline: "
  );

  display.println(
    baselineRaw
  );


  display.setCursor(0, 27);

  display.print(
    "Sensitivity: "
  );

  display.println(
    sensitivity
  );


  display.display();

  delay(1500);
}


// =====================================================
// BUTTONS
// =====================================================

void checkButtons()
{
  if (
    millis() -
    lastButtonTime <
    BUTTON_DELAY
  )
  {
    return;
  }


  // ===================================================
  // BUTTON 1 - INCREASE SENSITIVITY
  // ===================================================

  if (
    digitalRead(
      BUTTON_UP
    ) == LOW
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
      "Sensitivity increased: "
    );

    Serial.println(
      sensitivity
    );


    lastButtonTime =
      millis();
  }


  // ===================================================
  // BUTTON 2 - DECREASE SENSITIVITY
  // ===================================================

  if (
    digitalRead(
      BUTTON_DOWN
    ) == LOW
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
      "Sensitivity decreased: "
    );

    Serial.println(
      sensitivity
    );


    lastButtonTime =
      millis();
  }


  // ===================================================
  // BUTTON 3 - RECALIBRATE
  // ===================================================

  if (
    digitalRead(
      BUTTON_MODE
    ) == LOW
  )
  {
    Serial.println(
      "Recalibrating..."
    );


    calibrateBaseline();


    lastButtonTime =
      millis();
  }
}


// =====================================================
// CALIBRATION FUNCTION
// =====================================================

void calibrateBaseline()
{
  display.clearDisplay();

  display.setCursor(0, 0);

  display.println(
    "RECALIBRATING..."
  );

  display.setCursor(0, 15);

  display.println(
    "Keep RF quiet"
  );

  display.display();


  long sum = 0;


  for (int i = 0; i < 100; i++)
  {
    sum += analogRead(
      RF_SENSOR_PIN
    );

    delay(10);
  }


  baselineRaw =
    sum / 100;


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


  minRaw =
    baselineRaw;

  maxRaw =
    baselineRaw;


  Serial.print(
    "New baseline = "
  );

  Serial.println(
    baselineRaw
  );
}


// =====================================================
// SIGNAL STRENGTH
// =====================================================

int calculateSignalStrength(
  int average
)
{
  /*
     AD8318 behavior:

     Strong RF
        ↓
     Lower VOUT
        ↓
     Lower ADC
        ↓
     Higher signal strength
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
// OLED
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


  // ===================================================
  // TITLE
  // ===================================================

  display.setCursor(
    0,
    0
  );

  display.println(
    "RF SIGNAL SCANNER"
  );


  // ===================================================
  // RAW
  // ===================================================

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


  // ===================================================
  // VOLTAGE
  // ===================================================

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


  // ===================================================
  // dBm
  // ===================================================

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


  // ===================================================
  // SENSITIVITY
  // ===================================================

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


  // ===================================================
  // SIGNAL STRENGTH
  // ===================================================

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


  // ===================================================
  // SIGNAL BARS
  // ===================================================

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
// SERIAL DEBUG
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

  Serial.println(
    strength
  );
}


// =====================================================
// MAIN LOOP
// =====================================================

void loop()
{
  // ===================================================
  // BUTTONS
  // ===================================================

  checkButtons();


  // ===================================================
  // MOVING AVERAGE
  // ===================================================

  total -=
    readings[
      readIndex
    ];


  readings[
    readIndex
  ] =
    analogRead(
      RF_SENSOR_PIN
    );


  total +=
    readings[
      readIndex
    ];


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


  // ===================================================
  // MIN / MAX
  // ===================================================

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


  // ===================================================
  // CONVERSION
  // ===================================================

  float voltage =
    rawToVoltage(
      average
    );


  float dBm =
    voltageToDbm(
      voltage
    );


  // ===================================================
  // SIGNAL STRENGTH
  // ===================================================

  int signalStrength =
    calculateSignalStrength(
      average
    );


  // ===================================================
  // SERIAL
  // ===================================================

  printSerial(
    average,
    voltage,
    dBm,
    signalStrength
  );


  // ===================================================
  // OLED
  // ===================================================

  updateDisplay(
    average,
    voltage,
    dBm,
    signalStrength
  );


  delay(100);
}
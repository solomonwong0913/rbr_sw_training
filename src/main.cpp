#include <Arduino.h>

// Initialize 

// all board pins
#define MCP2515_MOTOR_CS 10
#define MCP2515_BMS_CS 9
#define MCP2515_DEBUG_CS 8

#define APPS_5V_PIN A0
#define APPS_3V3_PIN A1
#define BRAKE_PIN A3
#define START_BUTTON_PIN A4
#define BRAKE_LIGHT_PIN 2
#define DRIVE_LED_PIN 3
#define BUZZER_PIN 4

// frenquency
constexpr unsigned long MCP2515_CRYSTAL_HZ = 20000000UL;

constexpr int BRAKE_DEPRESSED_THRESHOLD = 600;

void setup() {
  Serial.begin(115200);

  // set MCP2515 CS pins
  pinMode(MCP2515_MOTOR_CS, OUTPUT);
  pinMode(MCP2515_BMS_CS, OUTPUT);
  pinMode(MCP2515_DEBUG_CS, OUTPUT);
  digitalWrite(MCP2515_MOTOR_CS, HIGH);
  digitalWrite(MCP2515_BMS_CS, HIGH);
  digitalWrite(MCP2515_DEBUG_CS, HIGH);

  // set pedal pins
  pinMode(APPS_5V_PIN, INPUT);
  pinMode(APPS_3V3_PIN, INPUT);
  pinMode(BRAKE_PIN, INPUT);

  // start button
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);

  // brake light, drive LED, buzzer
  pinMode(BRAKE_LIGHT_PIN, OUTPUT);
  pinMode(DRIVE_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BRAKE_LIGHT_PIN, LOW);
  digitalWrite(DRIVE_LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // initialise state timestamp placeholder
  Serial.println("[VCU] Pins initialised");
}


enum VcuState { INIT, STARTIN, BUZZIN, DRIVE };

static VcuState vcuState = INIT;
static unsigned long stateTimestamp = 0;

static void setState(VcuState s) {
  vcuState = s;
  stateTimestamp = millis();
  Serial.print("[VCU] setState: ");
  switch (s) {
    case INIT: Serial.println("INIT"); break;
    case STARTIN: Serial.println("STARTIN"); break;
    case BUZZIN: Serial.println("BUZZIN"); break;
    case DRIVE: Serial.println("DRIVE"); break;
  }
}

void loop() {
  unsigned long now = millis();

  switch (vcuState) {
    case INIT:
      // ensure motor is zero
      
      // monitor start button + brake
      break;

    case STARTIN:
      // TODO: detect hold for required duration; cancel on release
      break;

    case BUZZIN:
      // TODO: activate buzzer for configured duration
      break;

    case DRIVE:
      // TODO: read pedals, detect faults, compute/send torque
      break;
  }

  delay(10);
}
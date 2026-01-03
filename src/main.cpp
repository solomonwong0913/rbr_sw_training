#include <Arduino.h>

// Initialize 

// all board pins
#define MCP2515_MOTOR_CS 10 //PB2
#define MCP2515_BMS_CS 9    //PB1
#define MCP2515_DEBUG_CS 8  //PB0

#define APPS_5V_PIN A0      //PC0
#define APPS_3V3_PIN A1     //PC1
#define BRAKE_PIN A3        //PC3

#define START_BUTTON_PIN A4 //PC4

#define BRAKE_LIGHT_PIN 2   //PD2
#define DRIVE_LED_PIN 3     //PD3
#define BUZZER_PIN 4        //PD4

// frenquency
constexpr unsigned long MCP2515_CRYSTAL_HZ = 20000000UL;

// depressed and brake light thresholds
constexpr int BRAKE_DEPRESSED_THRESHOLD = 600; // a bit larger than 50%
constexpr int BRAKE_LIGHT_THRESHOLD = 100;

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
  // set initial state timestamp
  stateTimestamp = millis();
}


enum VcuState { INIT, STARTIN, BUZZIN, DRIVE };

static VcuState vcuState = INIT;
static unsigned long stateTimestamp = 0;

// simple static motor output helper (placeholder for CAN send)
static int16_t currentTorque = 32767; // invalid initial so first set prints
static void setMotorTorque(int16_t torque) {
  if (currentTorque == torque) return;
  currentTorque = torque;
  Serial.print("[VCU] Motor torque set: ");
  Serial.println((long)torque);
}

static void setState(VcuState s) {
  vcuState = s;
  stateTimestamp = millis();
  Serial.print("[VCU] setState: ");
  switch (s) {
    case INIT:
      Serial.println("INIT");
      break;
    case STARTIN:
      Serial.println("STARTIN");
      break;
    case BUZZIN:
      Serial.println("BUZZIN");
       break;
    case DRIVE:
      Serial.println("DRIVE");
      break;
  }
}

void loop() {
  unsigned long now = millis();

  // read brake and start button
  int brakeRaw = analogRead(BRAKE_PIN);
  bool brakeDepressed = (brakeRaw >= BRAKE_DEPRESSED_THRESHOLD);
  bool brakeLightOn = (brakeRaw >= BRAKE_LIGHT_THRESHOLD);
  digitalWrite(BRAKE_LIGHT_PIN, brakeLightOn ? HIGH : LOW);

  bool startPressed = (digitalRead(START_BUTTON_PIN) == LOW);

  switch (vcuState) {
    case INIT:
      // make the motor output be zero
      setMotorTorque(0);

      // ensure indicator outputs are off in INIT
      digitalWrite(DRIVE_LED_PIN, LOW);
      digitalWrite(BUZZER_PIN, LOW);

      // monitor start button + brake to move to STARTIN
      if (startPressed && brakeDepressed) {
        setState(STARTIN);    
      }
      break;

    case STARTIN:
      // make the motor output be zero
      setMotorTorque(0);

      // monitor start button + brake, 
      // to decide whether to stay or go back to INIT
      if (!startPressed || !brakeDepressed) {
        setState(INIT);
        break;
      }

      // after 2 seconds, move to BUZZIN
      if (now - stateTimestamp >= 2000) {
        setState(BUZZIN);
      }
      break;

    case BUZZIN:
      // set the buzzer on
      digitalWrite(BUZZER_PIN, HIGH);

      // after 2 seconds, move to DRIVE
      if (now - stateTimestamp >= 2000) {
        setState(DRIVE);
        digitalWrite(BUZZER_PIN, LOW);
      }
      break;

    case DRIVE:
      // TODO: read pedals, detect faults, compute/send torque
      break;
  }

  delay(10);
}
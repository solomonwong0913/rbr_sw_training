#include <Arduino.h>

// Initialize 

// =============== Board pins ===============
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

// =============== Thresholds and Timings ===============
// brake thresholds
constexpr int BRAKE_DEPRESSED_THRESHOLD = 600; // a bit larger than 50%
constexpr int BRAKE_LIGHT_THRESHOLD = 100;

// state transition timings
constexpr unsigned long STARTIN_TIME = 2000;
constexpr unsigned long BUZZIN_TIME = 2000;

// =============== APPS Faultiness ===============
constexpr int APPS_MAX_DISAGREE_RATIO = 10;
constexpr unsigned long APPS_FAULT_TIMEOUT_MS = 100;
bool appsFaulty = false;
unsigned long appsFaultStartTime = 0;

// =============== Motor Torque ===============
constexpr int16_t MOTOR_TORQUE_MAX = 32767;
const bool REVERSE_MOTOR_DIRECTION = false;

static int16_t currentTorque = 32767;

// =============== VCU State Machine ===============
enum VcuState { INIT, STARTIN, BUZZIN, DRIVE };

static VcuState vcuState = INIT;
static unsigned long stateTimestamp = 0;


// Helper function

// Helper : set VCU state
static void setState(VcuState s) {
  vcuState = s;
  stateTimestamp = millis();
  // Serial.print("[VCU] setState: ");
  switch (s) {
    case INIT: /*Serial.println("INIT");*/ break;
    case STARTIN: /*Serial.println("STARTIN");*/ break;
    case BUZZIN: /*Serial.println("BUZZIN");*/ break;
    case DRIVE: /*Serial.println("DRIVE");*/ break;
  }
}

// Helper : check APPS faulty
// returns true if faulty
static bool isAppsFaulty(int apps5v, int apps3v3) {
  apps5v = constrain(apps5v, 0, 1023);
  apps3v3 = constrain(apps3v3, 0, 1023);

  // Bring 3.3V reading up to 5V scale
  int32_t scaled3v3 = (int32_t)apps3v3 * 1023 / 675; // 3.3V * 1023 / 675 ≈ 5V

  int32_t diff = abs(apps5v - scaled3v3);
  int32_t max = (apps5v > scaled3v3) ? apps5v : scaled3v3;

  return (diff * APPS_MAX_DISAGREE_RATIO > max);
}

// Helper : convert motor torque
static int16_t convertMotorTorque(int apps5v) {
  // clamp apps5v reading
  apps5v = constrain(apps5v, 0, 1023);

  // scale 0-1023 to 0-32767
  int32_t torque = (int32_t)apps5v << 5;

  // clamp torque
  if (torque > MOTOR_TORQUE_MAX) torque = MOTOR_TORQUE_MAX;

  // reverse direction if needed
  if (REVERSE_MOTOR_DIRECTION) {
    torque = -torque;
  }
  return (int16_t)torque;
}

// Helper : set motor torque
static void setMotorTorque(int16_t torque) {
  if (currentTorque == torque) return;
  currentTorque = torque;
  // Serial.print("[VCU] Motor torque set: ");
  Serial.println((long)torque);
}

void setup() {
  // Serial.begin(115200);

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
  // Serial.println("[VCU] Pins initialised");
  // set initial state timestamp
  stateTimestamp = millis();
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
      setMotorTorque(0);
      digitalWrite(DRIVE_LED_PIN, LOW);
      digitalWrite(BUZZER_PIN, LOW);

      // monitor start button + brake to move to STARTIN
      if (startPressed && brakeDepressed) {
        setState(STARTIN);    
      }
      break;

    case STARTIN:
      setMotorTorque(0);

      // monitor start button + brake, 
      // go back to INIT if either released
      if (!startPressed || !brakeDepressed) {
        setState(INIT);
        break;
      }

      // after 2 seconds, move to BUZZIN
      if (now - stateTimestamp >= STARTIN_TIME) {
        setState(BUZZIN);
      }
      break;

    case BUZZIN:
      setMotorTorque(0);
      digitalWrite(BUZZER_PIN, HIGH);

      // after 2 seconds, go to DRIVE
      if (now - stateTimestamp >= BUZZIN_TIME) {
        digitalWrite(BUZZER_PIN, LOW);
        setState(DRIVE);
        digitalWrite(DRIVE_LED_PIN, HIGH);
      }
      break;

    case DRIVE:
      digitalWrite(DRIVE_LED_PIN, HIGH);

      //APPS reading
      int apps5v = analogRead(APPS_5V_PIN);
      int apps3v3 = analogRead(APPS_3V3_PIN);

      //check APPS faultiness
      if (isAppsFaulty(apps5v, apps3v3)) {
        // was not previously faulty
        if (!appsFaulty) {
          appsFaulty = true;
          appsFaultStartTime = now;
          // Serial.println("[VCU] APPS Faulty detected");
          break;
        }

        // check timeout
        if (now - appsFaultStartTime >= APPS_FAULT_TIMEOUT_MS) {
          // Serial.println("[VCU] APPS Fault condition triggered, going to INIT");
          setMotorTorque(0);
          digitalWrite(DRIVE_LED_PIN, LOW);
          setState(INIT);
          appsFaulty = false;
          break;
        }

      } else {
        if (appsFaulty) {
          appsFaulty = false;
          // Serial.println("[VCU] APPS Faulty cleared");
        }
      }

      // set motor torque according to APPS 5V reading
      int16_t torque = convertMotorTorque(apps5v);
      setMotorTorque(torque);

      break;
  }

  delay(10);
}
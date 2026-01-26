#include <Arduino.h>
#include <mcp2515.h>

// Initialize 

// =============== Board pins ===============
#define MCP2515_MOTOR_CS PIN_PB2
#define MCP2515_BMS_CS PIN_PB1
#define MCP2515_DEBUG_CS PIN_PB0

#define APPS_5V_PIN PIN_PC0
#define APPS_3V3_PIN PIN_PC1
#define BRAKE_PIN PIN_PC3

#define START_BUTTON_PIN PIN_PC4
#define BRAKE_LIGHT_PIN PIN_PD2
#define DRIVE_LED_PIN PIN_PD3
#define BUZZER_PIN PIN_PD4

// =============== MCP2515 objects ===============
MCP2515 mcp2515_motor(MCP2515_MOTOR_CS);
MCP2515 mcp2515_bms(MCP2515_BMS_CS);
MCP2515 mcp2515_debug(MCP2515_DEBUG_CS);

constexpr uint16_t MOTOR_CAN_ID = 0x201;
constexpr uint16_t MOTOR_TORQUE_CMD_ID = 0x90;

constexpr uint16_t DEBUG_CAN_ID = 0x300;
constexpr uint16_t DEBUG_TORQUE_CAN_ID = 0x400;

// =============== Thresholds and Timings ===============
// brake thresholds
constexpr uint16_t BRAKE_DEPRESSED_THRESHOLD = 600;/** Threshold for the brakes to be considered "depressed". (slightly larger than 50% of full scale1023)*/
constexpr uint16_t BRAKE_LIGHT_THRESHOLD = 102; /** Threshold for the brake lights to turn on. (approximately 10% of full scale 1023)*/
uint16_t brakeRaw = 0;
bool brakeDepressed = false;
bool brakeLightOn = false;

// state transition timings
constexpr uint16_t STARTIN_TIME = 2000; /** Time(in ms) to stay in STARTIN state */
constexpr uint16_t BUZZIN_TIME = 2000; /** Time(in ms) to stay in BUZZIN state */

// =============== APPS Faultiness ===============
/** Maximum allowed difference between APPS readings.
 * (approximately 10% of full scale 1023)
*/
constexpr uint16_t APPS_MAX_DISAGREE_THRESHOLD = 102;
/** Time(in ms) for the APPS readings being disagree to be considered as fault.
*/
constexpr uint16_t APPS_FAULT_TIMEOUT_MS = 100;
int16_t appsDiff = 0;
bool appsFaulty = false;
uint16_t appsFaultStartTime = 0;

// =============== Motor Torque ===============
const bool REVERSE_MOTOR_DIRECTION = false;

// =============== VCU State Machine ===============
enum VcuState { INIT = 1, STARTIN, BUZZIN, DRIVE };

static VcuState vcuState = INIT;
static unsigned long stateTimestamp = 0;


// Helper function
/** @brief Check if APPS readings are faulty, i.e. differ by more than threshold(10%) 
 * 
 * @param[in] apps5v APPS reading from 5V
 * @param[in] apps3v3 APPS reading from 3.3V
 * @return True if the difference is greater than threshold, False otherwise
*/
static bool isAppsFaulty(uint16_t apps5v, uint16_t apps3v3) {
  // Bring 3.3V reading up to 5V scale
  uint16_t scaled3v3 = apps3v3 * 1023 / 675; // 3.3V * 1023 / 675 ≈ 5V
  appsDiff = apps5v - scaled3v3;
  return (appsDiff > APPS_MAX_DISAGREE_THRESHOLD || appsDiff < -APPS_MAX_DISAGREE_THRESHOLD);
}

/** @brief Convert APPS_5V reading to motor torque
 * 
 * @param[in] apps5v APPS reading from 5V
 * @return Motor torque value
*/
static int16_t convertMotorTorque(int16_t apps5v) {
  // scale 0-1023 to 0-32767
  int16_t torque = apps5v << 5;

  // reverse direction if needed
  if (REVERSE_MOTOR_DIRECTION) {
    torque = -torque;
  }
  return torque;
}

/** @brief Send motor torque in little-endian format, send motor torque and brake info to debug CAN, check param[out] for message format
 * 
 * @param[in] torque Motor torque value
 * @param[out] byte0 Motor torque cmd ID (will be neglected while interpreting)
 * @param[out] byte1–2 Motor torque value (little-endian)
 * @param[out] byte3–4 Brake raw value (little-endian)
 * @param[out] byte5 Brake light on (0: off, 1: on)
 * 
*/
static void sendMotorTorque(int16_t torque) {
  can_frame tx_frame;
  tx_frame.can_id = MOTOR_CAN_ID;
  tx_frame.can_dlc = 3;

  // torque command
  tx_frame.data[0] = MOTOR_TORQUE_CMD_ID;

  // torque value in little-endian
  tx_frame.data[1] = (torque & 0xFF);
  tx_frame.data[2] = (torque >> 8) & 0xFF;

  // send motor torque
  mcp2515_motor.sendMessage(&tx_frame);

  // also send the torque on debug CAN
  tx_frame.can_id = DEBUG_TORQUE_CAN_ID;
  tx_frame.can_dlc = 6;

  // brake value in little-endian
  tx_frame.data[3] = (brakeRaw & 0xFF);
  tx_frame.data[4] = (brakeRaw >> 8) & 0xFF;
  tx_frame.data[5] = brakeLightOn;

  mcp2515_debug.sendMessage(&tx_frame);
}

/** @brief Send debug message, check param[out] for message format
 * 
 * @param[in] apps5v APPS reading from 5V
 * @param[in] apps3v3 APPS reading from 3.3V
 * @param[out] byte0–1 APPS_5V (little-endian)
 * @param[out] byte2–3 APPS_3V3 (little-endian)
 * @param[out] byte4 VCU state
 * @param[out] byte5 APPS fault (0: no fault, 1: fault)
 * @param[out] byte6–7 Difference between APPS readings (little-endian)(0 if no fault)
 * 
*/
static void sendDebugPedals(uint16_t apps5v, uint16_t apps3v3) {
  can_frame tx_frame;

  tx_frame.can_id  = DEBUG_CAN_ID;  // 0x300

  // byte 0–1: APPS_5V (little-endian)
  tx_frame.data[0] = (apps5v & 0xFF);
  tx_frame.data[1] = (apps5v >> 8) & 0xFF;

  // byte 2–3: APPS_3V3 (little-endian)
  tx_frame.data[2] = (apps3v3 & 0xFF);
  tx_frame.data[3] = (apps3v3 >> 8) & 0xFF;

  // byte 4: vcu state
  tx_frame.data[4] = (uint8_t)vcuState;

  // byte 5: APPS fault
  // byte 6–7: difference between APPS readings (little-endian)
  if (appsFaulty) {
    tx_frame.data[5] = 0x01;
    tx_frame.data[6] = (appsDiff & 0xFF);         //appsDiff calculated in isAppsFaulty()
    tx_frame.data[7] = (appsDiff >> 8) & 0xFF;

  } else {
    tx_frame.data[5] = 0x00;
    tx_frame.data[6] = 0x00;
    tx_frame.data[7] = 0x00;
  }

  // Send on debug message
  mcp2515_debug.sendMessage(&tx_frame);
}

void setup() {

  // set pedal pins
  pinMode(APPS_5V_PIN, INPUT);
  pinMode(APPS_3V3_PIN, INPUT);
  pinMode(BRAKE_PIN, INPUT);

  // start button
  pinMode(START_BUTTON_PIN, INPUT);

  // brake light, drive LED, buzzer
  pinMode(BRAKE_LIGHT_PIN, OUTPUT);
  pinMode(DRIVE_LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BRAKE_LIGHT_PIN, LOW);
  digitalWrite(DRIVE_LED_PIN, LOW);
  digitalWrite(BUZZER_PIN, LOW);

  // initialize MCP2515 CAN
  mcp2515_motor.reset();
  mcp2515_motor.setBitrate(CAN_500KBPS, MCP_20MHZ);
  mcp2515_motor.setNormalMode();
  mcp2515_bms.reset();
  mcp2515_bms.setBitrate(CAN_500KBPS, MCP_20MHZ);
  mcp2515_bms.setNormalMode();
  mcp2515_debug.reset();
  mcp2515_debug.setBitrate(CAN_500KBPS, MCP_20MHZ);
  mcp2515_debug.setNormalMode();

  // set initial state timestamp
  stateTimestamp = millis();
}

void loop() {
  uint32_t now = millis();

  // read brake and start button
  brakeRaw = analogRead(BRAKE_PIN);
  brakeDepressed = (brakeRaw >= BRAKE_DEPRESSED_THRESHOLD);
  brakeLightOn = (brakeRaw >= BRAKE_LIGHT_THRESHOLD);
  digitalWrite(BRAKE_LIGHT_PIN, brakeLightOn ? HIGH : LOW);

  //APPS reading
  uint16_t apps5v = analogRead(APPS_5V_PIN);
  uint16_t apps3v3 = analogRead(APPS_3V3_PIN);

  // send debug CAN message
  sendDebugPedals(apps5v, apps3v3);

  bool startPressed = (digitalRead(START_BUTTON_PIN) == LOW);

  switch (vcuState) {
    case INIT:
      sendMotorTorque(0);
      digitalWrite(DRIVE_LED_PIN, LOW);
      digitalWrite(BUZZER_PIN, LOW);

      // monitor start button + brake to go to STARTIN
      if (startPressed && brakeDepressed) {
        vcuState = STARTIN;
        stateTimestamp = now;
      }
      break;

    case STARTIN:
      sendMotorTorque(0);

      // monitor start button + brake, 
      // go back to INIT if either released
      if (!startPressed || !brakeDepressed) {
        vcuState = INIT;
        stateTimestamp = now;
        break;
      }

      // if received the BMS message(ID: 0x186040F3, data[6] == 0x50), go to BUZZIN
      can_frame rx_frame;
      if (mcp2515_bms.readMessage(&rx_frame) != MCP2515::ERROR_OK) break;
      if (rx_frame.can_id != 0x186040F3) break;
      if (rx_frame.can_dlc < 7) break;
      if (rx_frame.data[6] == 0x50) {
        vcuState = BUZZIN;
        stateTimestamp = now;
      }
      
      break;

    case BUZZIN:
      sendMotorTorque(0);
      digitalWrite(BUZZER_PIN, HIGH);

      // after 2 seconds, go to DRIVE
      if (now - stateTimestamp >= BUZZIN_TIME) {
        digitalWrite(BUZZER_PIN, LOW);
        vcuState = DRIVE;
        stateTimestamp = now;
        digitalWrite(DRIVE_LED_PIN, HIGH);
      }
      break;

    case DRIVE:
      digitalWrite(DRIVE_LED_PIN, HIGH);

      //check APPS faultiness
      if (isAppsFaulty(apps5v, apps3v3)) {
        // was not previously faulty
        if (!appsFaulty) {
          appsFaulty = true;
          appsFaultStartTime = now;         
          break;
        }

        // check timeout
        if (now - appsFaultStartTime >= APPS_FAULT_TIMEOUT_MS) {
          sendMotorTorque(0);
          digitalWrite(DRIVE_LED_PIN, LOW);
          vcuState = INIT;
          stateTimestamp = now;
          break;
        }

      } else {
        appsFaulty = false;
      }

      // set motor torque according to APPS 5V reading
      int16_t torque = convertMotorTorque(apps5v);
      sendMotorTorque(torque);

      break;
  }
  
  

  delay(10);
}
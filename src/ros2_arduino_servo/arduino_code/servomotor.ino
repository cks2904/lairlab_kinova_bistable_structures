#include <Servo.h>
 
// --- Constants ---
// Use 'constexpr' for compile-time constants to potentially save memory.
constexpr int SERVO_COUNT = 4;
constexpr byte SERVO_PINS[SERVO_COUNT] = {8, 9, 10, 11};
constexpr long SERIAL_BAUD = 115200;
constexpr long FEEDBACK_INTERVAL_MS = 100; // 10Hz feedback rate
 
// --- Servo Angles ---
constexpr int ANGLE_NEUTRAL = 145;
constexpr int ANGLE_PULL = 0;
constexpr int ANGLE_MIDDLE = 30;
constexpr int ANGLE_PUSH = 180;
 
// --- Global Variables ---
Servo servos[SERVO_COUNT];
char inputBuffer[32]; // Reduced buffer size, as commands are short.
bool commandReady = false;
unsigned long lastFeedbackTime = 0;
 
void setup() {
  Serial.begin(SERIAL_BAUD);
  for (int i = 0; i < SERVO_COUNT; i++) {
    servos[i].attach(SERVO_PINS[i]);
    servos[i].write(ANGLE_NEUTRAL);
  }
}
 
void loop() {
  // Main loop remains non-blocking for responsive communication.
  readSerialCommand();
  publishFeedback();
}
 
/**
* @brief Reads a command from serial until a newline is found.
* Sets 'commandReady' flag and then processes the command.
*/
void readSerialCommand() {
  static byte index = 0;
  while (Serial.available() > 0) {
    char inChar = Serial.read();
    if (inChar == '\n') {
      inputBuffer[index] = '\0'; // Null-terminate the string.
      processCommands(inputBuffer);
      index = 0; // Reset buffer index for next command.
      return; // Exit after processing the command line.
    } 
    else if (index < sizeof(inputBuffer) - 1) {
      inputBuffer[index++] = inChar;
    }
  }
}
 
/**
* @brief Periodically sends current servo angles for plotting.
* Format: "angle1 angle2 angle3 angle4"
*/
void publishFeedback() {
  if (millis() - lastFeedbackTime >= FEEDBACK_INTERVAL_MS) {
    lastFeedbackTime = millis();
    for (int i = 0; i < SERVO_COUNT; i++) {
      Serial.print(servos[i].read());
      if (i < SERVO_COUNT - 1) {
        Serial.print(F(" ")); // Use F() macro to store string in flash memory.
      }
    }
    Serial.println();
  }
}
 
/**
* @brief Parses a line of commands (e.g., "1p 2u") and executes them.
*/
void processCommands(char* cmdLine) {
  char* token = strtok(cmdLine, " ");
  while (token != NULL) {
    processSingleCommand(token);
    token = strtok(NULL, " ");
  }
}
 
/**
* @brief Executes a single command (e.g., "1p") to move a servo.
*/
void processSingleCommand(const char* cmd) {
  if (strlen(cmd) < 2) return;
 
  byte servoIndex = cmd[0] - '1';
  if (servoIndex >= SERVO_COUNT) return; // Unsigned byte check is sufficient.
 
  int angle;
  switch (cmd[1]) {
    case 'n': angle = ANGLE_NEUTRAL; break;
    case 'p': angle = ANGLE_PULL;    break;
    case 'm': angle = ANGLE_MIDDLE;  break;
    case 'u': angle = ANGLE_PUSH;    break;
    default: return; // Ignore invalid action characters.
  }
  servos[servoIndex].write(angle);
}
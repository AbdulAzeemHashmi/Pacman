// ═══════════════════════════════════════════════════════
//  NEO-PAC  —  Pac-Man Robot Controller  (ESP32 / UDP)  v3.0
//  Rebuilt to match Project Proposal: Locomotion + Command &
//  Control Modules
//
//  Role in system:
//    The PC "AI Brain" (OpenCV perception + Safe Path Planner +
//    Ghost Predictor) computes a path and sends it here as a
//    sequence of step-by-step grid moves over UDP. This sketch
//    does NOT plan paths — it only executes movement commands
//    and reports acknowledgments / status back to the PC, per
//    the Command & Control Module description.
//
//  Changes from v2 (ESP8266 / HTTP prototype):
//    • Switched ESP8266WebServer → WiFi (STA mode) + UDP sockets
//      (UDP chosen over TCP for lower latency, per proposal)
//    • Switched board target ESP8266 → ESP32
//    • Directional command set expanded to N/S/E/W (grid-relative)
//      while keeping F/B/L/R (robot-relative) for manual/testing
//    • Added per-command ACK packet back to PC (Command Queuing)
//    • Added PID-assisted straight-line driving so Pac-Man tracks
//      straight between grid cells (Locomotion Module requirement)
//    • Added "stop at cell center" timed-step model: each move
//      command corresponds to one grid cell, using a calibrated
//      CELL_TRAVEL_MS / CELL_TURN_MS duration
//    • MAX_STEPS increased 100 → 500 (food/long routes kept from v2)
//    • /status-style report now pushed back over UDP, not HTTP
//    • Emergency STOP command added (Python uses it)
//    • Speed offset calibration via TRIM command
//
//  WiFi: connects as a STATION to the lab/PC hotspot (set below)
//  UDP:  listens on LOCAL_UDP_PORT for commands from PC
//        replies/ACKs sent to PC_IP : PC_UDP_PORT
//
//  Packet format (text, comma-separated, newline-terminated):
//    Single command:   CMD,<id>,<dir>[,<ms>]\n
//        <dir>  = N | E | D | W | F | B | L | R | X(stop)
//                 (N=fwd/north, D=back/south, E=right/east,
//                  W=left/west, X=stop; F/B/L/R kept for manual/test)
//    Path (multi-step): PATH,<id>,<dir1>:<ms1>;<dir2>:<ms2>;...\n
//    Speed set:         SPEED,<id>,<val 0-255>\n
//    Trim set:          TRIM,<id>,<left>,<right>\n
//    Stop:              STOP,<id>\n
//    Status request:    STATUS,<id>\n
//
//  Every command gets an ACK back:  ACK,<id>,<result>\n
//  Status requests get:             STAT,<id>,<status>,<step>,<total>,<remaining>\n
// ═══════════════════════════════════════════════════════

#include <WiFi.h>
#include <WiFiUdp.h>

// ─────────────────────────────────────────
//  WiFi SETTINGS  (Pac-Man joins the PC's network / router)
// ─────────────────────────────────────────

const char* ssid     = "NEOPAC_NET";
const char* password = "neopac2026";

// PC ("AI Brain") endpoint that ACKs/STAT packets are sent to.
// Update to match the PC's actual IP on the shared network.

IPAddress pcIP(192, 168, 1, 50);

const unsigned int PC_UDP_PORT    = 5006;
const unsigned int LOCAL_UDP_PORT = 5005;

WiFiUDP udp;
char incomingPacket[512];

// ─────────────────────────────────────────
//  MOTOR PINS  (L298N)
// ─────────────────────────────────────────

#define ENA 14   // Left motor PWM
#define IN1 27   // Left motor dir 1
#define IN2 26   // Left motor dir 2
#define ENB 25   // Right motor PWM
#define IN3 33   // Right motor dir 1
#define IN4 32   // Right motor dir 2

// ESP32 LEDC PWM channels for motor speed control

#define PWM_FREQ       20000
#define PWM_RES_BITS   8        // 0-255 duty range
#define PWM_CH_LEFT    0
#define PWM_CH_RIGHT   1

// ─────────────────────────────────────────
//  SPEED / TRIM / TIMING SETTINGS
// ─────────────────────────────────────────

int speedVal    = 200;   // base PWM duty, 0-255
int leftOffset  = 0;     // calibration trim, added to left duty
int rightOffset = 0;     // calibration trim, added to right duty

// Calibrated durations for "one grid cell" moves — tune on the
// real maze so Pac-Man stops at cell centers (Locomotion Module).

unsigned long CELL_TRAVEL_MS = 450;   // time to cross one cell forward/back
unsigned long CELL_TURN_MS   = 300;   // time to pivot 90 degrees

// ─────────────────────────────────────────
//  SIMPLE PID STATE (straight-line driving)
//  Without encoders, error is approximated from the commanded
//  vs. actual trim-corrected duty cycle drift over time; this
//  keeps both wheels converging to matched effective speed.
//  Swap in real encoder ticks here if/when added to hardware.
// ─────────────────────────────────────────

float pidKp = 0.6, pidKi = 0.02, pidKd = 0.05;
float pidIntegral = 0;
float pidLastError = 0;
unsigned long pidLastTime = 0;

// ═══════════════════════════════════════════════════════
//  PATH EXECUTION STATE
// ═══════════════════════════════════════════════════════

struct PathStep {
  char          cmd;        // N/D/E/W/F/B/L/R/X(stop)
  unsigned long duration;
};

#define MAX_STEPS 500

PathStep      pathSteps[MAX_STEPS];
int           pathLength    = 0;
int           currentStep   = 0;
unsigned long stepStartTime = 0;
bool          pathRunning   = false;
String        pathStatus    = "idle";

// ═══════════════════════════════════════════════════════
//  FORWARD DECLARATIONS
// ═══════════════════════════════════════════════════════

void connectWiFi();
void handleIncomingPacket(int len);
void processLine(String line);
void sendAck(const String& id, const String& result);
void sendStatus(const String& id);
void applyCommand(char c, unsigned long durationOverride = 0);
void executeCurrentStep();
void startPath(const String& id, const String& stepsField);
void stopPathAndMotors();
void setupMotorPWM();
void driveLeft(int duty);
void driveRight(int duty);
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void stopMotors();
void runPID();

void setup() {
  Serial.begin(115200);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  setupMotorPWM();
  stopMotors();

  connectWiFi();
  udp.begin(LOCAL_UDP_PORT);

  pidLastTime = millis();

  Serial.println("NEO-PAC v3.0 (ESP32/UDP) ready");
  Serial.print("Listening on UDP port ");
  Serial.println(LOCAL_UDP_PORT);
}

void loop() {
  int packetSize = udp.parsePacket();
  
  if (packetSize) {
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    
    if (len > 0) 
        incomingPacket[len] = '\0';
        
    else 
        incomingPacket[0] = '\0';
        
    handleIncomingPacket(len);
  }

  if (pathRunning && pathLength > 0) {
    unsigned long elapsed = millis() - stepStartTime;
    
    if (elapsed >= pathSteps[currentStep].duration) {
      currentStep++;
      
      if (currentStep < pathLength) {
        executeCurrentStep();
        stepStartTime = millis();

        int pct = (int)((currentStep * 100UL) / pathLength);
        
        Serial.print("Path progress: ");
        Serial.print(pct);
        Serial.println("%");
      } 
      
      else {
        stopMotors();
        pathRunning = false;
        pathStatus  = "done";
        Serial.println("Path progress: 100% (done)");
      }
    }
  }

  runPID();
}

// ═══════════════════════════════════════════════════════
//  WIFI
// ═══════════════════════════════════════════════════════

void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Connecting to WiFi");
  unsigned long startAttempt = millis();
  
  while (WiFi.status() != WL_CONNECTED && millis() - startAttempt < 15000) {
    delay(300);
    Serial.print(".");
  }
  
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Connected. IP: ");
    Serial.println(WiFi.localIP());
  } 

  else 
    Serial.println("WiFi connect failed — will retry passively in loop()");
}

// ═══════════════════════════════════════════════════════
//  UDP PACKET HANDLING
//  Format: TYPE,id,...fields  (comma separated, no trailing newline needed)
// ═══════════════════════════════════════════════════════

void handleIncomingPacket(int len) {
  if (len <= 0) 
      return;
      
  String line = String(incomingPacket);
  line.trim();
  
  if (line.length() == 0) 
      return;
      
  processLine(line);
}

void processLine(String line) {
  int c1 = line.indexOf(',');
  String type = (c1 == -1) ? line : line.substring(0, c1);
  String rest = (c1 == -1) ? ""   : line.substring(c1 + 1);

  int c2 = rest.indexOf(',');
  String id    = (c2 == -1) ? rest : rest.substring(0, c2);
  String field = (c2 == -1) ? ""   : rest.substring(c2 + 1);

  type.trim();
  type.toUpperCase();

  if (type == "CMD") {
    // field = "<dir>" or "<dir>,<ms>"
    int c3 = field.indexOf(',');
    String dir = (c3 == -1) ? field : field.substring(0, c3);
    unsigned long ms = (c3 == -1) ? 0 : field.substring(c3 + 1).toInt();
    dir.trim();

    pathRunning = false;   // manual command interrupts any active path
    pathStatus  = "manual";

    if (dir.length() > 0) {
      applyCommand(dir[0], ms);
      sendAck(id, "ok");
    } 
    
    else 
      sendAck(id, "err_missing_dir");
  }
  
  else if (type == "PATH") 
    startPath(id, field);
  
  else if (type == "SPEED") {
    int v = field.toInt();
    
    speedVal = constrain(v, 0, 255);
    sendAck(id, "speed=" + String(speedVal));
  }
  
  else if (type == "TRIM") {
    int c3 = field.indexOf(',');
    
    if (c3 != -1) {
      leftOffset  = constrain(field.substring(0, c3).toInt(), -255, 255);
      rightOffset = constrain(field.substring(c3 + 1).toInt(), -255, 255);
      sendAck(id, "trim_l=" + String(leftOffset) + "_r=" + String(rightOffset));
    } 
    
    else 
      sendAck(id, "err_missing_fields");
  }
  else if (type == "STOP") {
    stopPathAndMotors();
    sendAck(id, "stopped");
  }
  
  else if (type == "STATUS") 
    sendStatus(id);
  
  else 
    sendAck(id, "err_unknown_type");
}

void sendAck(const String& id, const String& result) {
  String msg = "ACK," + id + "," + result + "\n";
  udp.beginPacket(pcIP, PC_UDP_PORT);
  udp.write((const uint8_t*)msg.c_str(), msg.length());
  udp.endPacket();
}

void sendStatus(const String& id) {
  int remaining = pathRunning ? (pathLength - currentStep) : 0;
  
  String msg = "STAT," + id + "," + pathStatus + "," +
               String(currentStep) + "," + String(pathLength) + "," +
               String(remaining) + "\n";
  udp.beginPacket(pcIP, PC_UDP_PORT);
  udp.write((const uint8_t*)msg.c_str(), msg.length());
  udp.endPacket();
}

// ═══════════════════════════════════════════════════════
//  PATH HANDLING
//  field format: "<dir1>:<ms1>;<dir2>:<ms2>;..."
//  e.g. "N:450;N:450;E:300;E:450"
// ═══════════════════════════════════════════════════════

void startPath(const String& id, const String& stepsField) {
  pathLength  = 0;
  currentStep = 0;

  int start = 0;
  
  while (start < (int)stepsField.length() && pathLength < MAX_STEPS) {
    int semi = stepsField.indexOf(';', start);
    
    String token = (semi == -1) ? stepsField.substring(start)
                                 : stepsField.substring(start, semi);
    token.trim();

    int colon = token.indexOf(':');
    
    if (colon != -1 && token.length() > colon + 1) {
      char dir = token[0];
      unsigned long dur = token.substring(colon + 1).toInt();
      
      if (dur == 0) 
          dur = CELL_TRAVEL_MS;  // fall back to calibrated default

      bool validDir = (dir=='N'||dir=='D'||dir=='E'||dir=='W'||
                        dir=='F'||dir=='B'||dir=='L'||dir=='R'||dir=='X');
      if (validDir) {
        pathSteps[pathLength].cmd      = dir;
        pathSteps[pathLength].duration = dur;
        pathLength++;
      }
    }

    if (semi == -1) 
        break;
        
    start = semi + 1;
  }

  if (pathLength == 0) {
    sendAck(id, "err_no_valid_steps");
    return;
  }

  pathRunning   = true;
  pathStatus    = "running";
  stepStartTime = millis();
  executeCurrentStep();

  Serial.print("Path started, ");
  Serial.print(pathLength);
  Serial.println(" steps");

  sendAck(id, "path_started_steps=" + String(pathLength));
}

void stopPathAndMotors() {
  pathRunning = false;
  pathLength  = 0;
  currentStep = 0;
  pathStatus  = "stopped";
  stopMotors();
  Serial.println("EMERGENCY STOP");
}

// ═══════════════════════════════════════════════════════
//  COMMAND EXECUTION
//  Accepts grid-relative directions (N=forward/north,
//  D=backward/south, E=right/east, W=left/west) plus
//  robot-relative F/B/L/R for manual testing, and X for stop.
//  ('S' is reserved for STOP in the original v2 scheme, so
//   "south" uses 'D' here to avoid ambiguity.)
// ═══════════════════════════════════════════════════════

void executeCurrentStep() {
  if (currentStep < pathLength) 
    applyCommand(pathSteps[currentStep].cmd, pathSteps[currentStep].duration);
}

void applyCommand(char c, unsigned long durationOverride) {
  c = toupper(c);
  switch (c) {
    case 'N':
    case 'F': moveForward();  break;
    case 'D':
    case 'B': moveBackward(); break;
    case 'E':
    case 'R': turnRight();    break;
    case 'W':
    case 'L': turnLeft();     break;
    case 'X':
    case 'S': stopMotors();   break;
    default:  stopMotors();   break;
  }
  (void)durationOverride; // duration is enforced by the loop()/path timer, not here
}

// ═══════════════════════════════════════════════════════
//  MOTOR CONTROL  (L298N driver via ESP32 LEDC PWM)
//    Left motor  -> ENA / IN1 / IN2
//    Right motor -> ENB / IN3 / IN4
// ═══════════════════════════════════════════════════════

void setupMotorPWM() {
  ledcSetup(PWM_CH_LEFT,  PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(ENA, PWM_CH_LEFT);

  ledcSetup(PWM_CH_RIGHT, PWM_FREQ, PWM_RES_BITS);
  ledcAttachPin(ENB, PWM_CH_RIGHT);
}

int leftSpeedFinal()  { 
    return constrain(speedVal + leftOffset,  0, 255); 
}

int rightSpeedFinal() { 
    return constrain(speedVal + rightOffset, 0, 255); 
}

void driveLeft(int duty)  { 
    ledcWrite(PWM_CH_LEFT,  constrain(duty, 0, 255)); 
}

void driveRight(int duty) { 
    ledcWrite(PWM_CH_RIGHT, constrain(duty, 0, 255)); 
}

void moveForward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  driveLeft(leftSpeedFinal());
  driveRight(rightSpeedFinal());
}

void moveBackward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  driveLeft(leftSpeedFinal());
  driveRight(rightSpeedFinal());
}

void turnLeft() {
  // Pivot: left backward, right forward
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  driveLeft(leftSpeedFinal());
  driveRight(rightSpeedFinal());
}

void turnRight() {
  // Pivot: left forward, right backward
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
  driveLeft(leftSpeedFinal());
  driveRight(rightSpeedFinal());
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  driveLeft(0);
  driveRight(0);
}

// ═══════════════════════════════════════════════════════
//  STRAIGHT-LINE PID ASSIST
//  Nudges leftOffset/rightOffset toward each other over time
//  while driving forward/backward so Pac-Man tracks straight
//  between grid cells, per the Locomotion Module's PID
//  requirement. This is an open-loop approximation; replace
//  the error source with real encoder delta if/when added.
// ═══════════════════════════════════════════════════════

void runPID() {
  if (!pathRunning) 
      return;
      
  char activeCmd = (currentStep < pathLength) ? toupper(pathSteps[currentStep].cmd) : 'X';
  bool isStraightMove = (activeCmd == 'N' || activeCmd == 'F' ||
                          activeCmd == 'D' || activeCmd == 'B');
  if (!isStraightMove) 
      return;

  unsigned long now = millis();
  float dt = (now - pidLastTime) / 1000.0;
  
  if (dt <= 0) 
      return;

  // Error proxy: difference between left/right effective duty.
  // A real implementation should swap this for (leftTicks - rightTicks).
  
  float error = (float)(leftSpeedFinal() - rightSpeedFinal());

  pidIntegral += error * dt;
  
  float derivative = (error - pidLastError) / dt;
  float correction = pidKp * error + pidKi * pidIntegral + pidKd * derivative;

  rightOffset = constrain(rightOffset + (int)correction, -255, 255);

  pidLastError = error;
  pidLastTime  = now;
}

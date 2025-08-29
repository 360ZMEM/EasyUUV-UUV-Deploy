#include <HardwareSerial.h>
#include <math.h>

#define JY60_BAUD_RATE 9600
#define RX_PIN 16
#define TX_PIN 17
HardwareSerial JY60Serial(2);

const int thrusterPins[8] = {33, 26, 27, 14, 32, 25, 12, 13}; // 
#define PWM_FREQ 50    
#define PWM_RES 16  
#define NEUTRAL_PULSE 1500  
#define MIN_PULSE 1100     
#define MAX_PULSE 1900  
#define PWM_CHANNEL_OFFSET 0

// ===================== Deadzone compensation =====================
#define DEADZONE_THRESHOLD 1.0      
#define DEADZONE_COMPENSATION 0.5  
#define DEADZONE_RAMP_UP 0.2     

// ===================== S-Surface Controller Argument (Default) =====================
float YAW_ZETA1 = 0.25;      
float YAW_ZETA2 = 0.0;       
float PITCH_ZETA1 = -0.4;    
float PITCH_ZETA2 = 0.2;     
float ROLL_ZETA1 = -0.4;    
float ROLL_ZETA2 = 0.2;      

#define ALPHA 0.01       
#define DT 0.01        
#define OUTPUT_SCALE 200 

const float thrustMatrix[8][3] = {
  // Yaw   Pitch  Roll
  { 1.0,   0.0,   0.0},   // Thruster 1
  { 1.0,   0.0,   0.0},   // Thruster 2
  {-1.0,   0.0,   0.0},   // Thruster 3
  { 1.0,   0.0,   0.0},   // Thruster 4
  { 0.0,   1.0,   1.0},   // Thruster 5
  { 0.0,  -1.0,   1.0},   // Thruster 6
  { 0.0,   1.0,   1.0},   // Thruster 7
  { 0.0,   1.0,  -1.0}    // Thruster 8
};


double setpointYaw = 0.0;
double setpointPitch = 0.0;
double setpointRoll = 0.0;

float angleYaw = 0.0;
float anglePitch = 0.0;
float angleRoll = 0.0;

double yawError = 0.0;
double pitchError = 0.0;
double rollError = 0.0;

double lastYawError = 0.0;
double lastPitchError = 0.0;
double lastRollError = 0.0;

// \dot{e}
double yawErrorDerivative = 0.0;
double pitchErrorDerivative = 0.0;
double rollErrorDerivative = 0.0;

// 扰动补偿项
double yawDeltaU = 0.0;
double pitchDeltaU = 0.0;
double rollDeltaU = 0.0;

double yawOutput = 0.0;
double pitchOutput = 0.0;
double rollOutput = 0.0;

bool systemArmed = false;       
float yawOffset = 0.0;          // zero-point
float pitchOffset = 0.0;      
float rollOffset = 0.0;         
unsigned long lastControlTime = 0;

enum ControlMode {
  MODE_YAW_PITCH,  
  MODE_YAW_ROLL,
  MODE_ALL 
};
ControlMode currentMode = MODE_ALL; 

#define JY60_FRAME_ANGLE 0x53
#define PACKET_SIZE 11
byte packetBuffer[PACKET_SIZE];
int byteCount = 0;
bool packetReceived = false;

void setThruster(uint8_t index, int microSecs) {
  const long periodMicros = 20000;
  const int maxDuty = (1 << PWM_RES) - 1;
  microSecs = constrain(microSecs, MIN_PULSE, MAX_PULSE);
  long duty = (microSecs * (1LL << PWM_RES)) / periodMicros;
  int channel = PWM_CHANNEL_OFFSET + index;
  ledcWrite(channel, duty);
}
double sSurfaceControl(double error, double errorDeriv, double* deltaU, float zeta1, float zeta2) {
  double u = 2.0 / (1.0 + exp(-zeta1 * error - zeta2 * errorDeriv)) - 1.0;
  
  *deltaU += ALPHA * (error + errorDeriv) * DT;
  *deltaU = constrain(*deltaU, -0.3, 0.3); 
  
  // Deadzone compensation (for small e)
  static double deadzoneCompensation[3] = {0}; // Yaw Pitch Roll
  double absError = fabs(error);
  
  if(absError < DEADZONE_THRESHOLD) {
    double targetCompensation = DEADZONE_COMPENSATION * (1.0 - absError/DEADZONE_THRESHOLD);
    deadzoneCompensation[0] += (targetCompensation - deadzoneCompensation[0]) * DEADZONE_RAMP_UP;
    
    double sign = (error > 0) ? 1.0 : -1.0;
    u += sign * deadzoneCompensation[0];
  } else {
    deadzoneCompensation[0] *= 0.95;
  }
  
  return u + *deltaU;
}

// Initalize
void setup() {
  Serial.begin(115200);
  while(!Serial);

  // IMU Initalize
  JY60Serial.begin(JY60_BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);

  // Thruster Initalize
  for(int i = 0; i < 8; i++) {
    int channel = PWM_CHANNEL_OFFSET + i;
    ledcSetup(channel, PWM_FREQ, PWM_RES);
    ledcAttachPin(thrusterPins[i], channel);
  }

  // ESC Initialize
  for(int i=0; i<8; i++) setThruster(i, 2000);
  delay(2000);
  for(int i=0; i<8; i++) setThruster(i, 1500);
  delay(2000);

  Serial.println("Three-axis attitude control system ready");
  Serial.println("Current Parameters:");
  Serial.print("Yaw ZETA1="); Serial.print(YAW_ZETA1); 
  Serial.print(" ZETA2="); Serial.println(YAW_ZETA2);
  Serial.print("Pitch ZETA1="); Serial.print(PITCH_ZETA1); 
  Serial.print(" ZETA2="); Serial.println(PITCH_ZETA2);
  Serial.print("Roll ZETA1="); Serial.print(ROLL_ZETA1); 
  Serial.print(" ZETA2="); Serial.println(ROLL_ZETA2);
  Serial.println("Available Commands:");
  Serial.println("a - Activate system");
  Serial.println("e - Emergency stop");
  Serial.println("y<angle> - Set yaw target");
  Serial.println("p<angle> - Set pitch target");
  Serial.println("r<angle> - Set roll target");
  Serial.println("z<value> - Set yaw ZETA1");
  Serial.println("x<value> - Set yaw ZETA2");
  Serial.println("v<value> - Set pitch ZETA1");
  Serial.println("b<value> - Set pitch ZETA2");
  Serial.println("n<value> - Set roll ZETA1");
  Serial.println("m<value> - Set roll ZETA2");
}

// ===================== Main =====================
void loop() {
  processIMUData();
  handleSerialInput();
  
  if(systemArmed && (millis() - lastControlTime >= 20)){
    updateControl();
    lastControlTime = millis();
  }

  static unsigned long lastPrint = 0;
  if(millis() - lastPrint > 20){
    Serial.print(" RealAngle - P:");
    Serial.print(normalizeAngle(anglePitch - pitchOffset));
    Serial.print(" R:");
    Serial.print(normalizeAngle(angleRoll - rollOffset));
    Serial.print(" Y:");
    Serial.print(normalizeAngle(angleYaw - yawOffset));
    
    Serial.print(" | Target - P:");
    Serial.print(setpointPitch);
    Serial.print(" R:");
    Serial.print(setpointRoll);
    Serial.print(" Y:");
    Serial.print(setpointYaw);
    
    Serial.print(" | Output - P:");
    Serial.print(pitchOutput);
    Serial.print(" R:");
    Serial.print(rollOutput);
    Serial.print(" Y:");
    Serial.println(yawOutput);
    
    lastPrint = millis();
  }
}

void processIMUData() {
  while(JY60Serial.available()){
    byte c = JY60Serial.read();
    
    if(c == 0x55 && byteCount == 0){
      packetBuffer[byteCount++] = c;
    }
    else if(byteCount > 0 && byteCount < PACKET_SIZE){
      packetBuffer[byteCount++] = c;
      if(byteCount == PACKET_SIZE) packetReceived = true;
    }
    else{
      byteCount = 0;
    }

    if(packetReceived){
      if(verifyChecksum()){
        parseDataPacket();
      }
      packetReceived = false;
      byteCount = 0;
    }
  }
}

bool verifyChecksum() {
  uint8_t sum = 0;
  for(int i=0; i<PACKET_SIZE-1; i++) sum += packetBuffer[i];
  return sum == packetBuffer[PACKET_SIZE-1];
}

void parseDataPacket() {
  if(packetBuffer[1] == JY60_FRAME_ANGLE){
    float angleScale = 180.0 / 32768.0;
    int16_t rawPitch = (packetBuffer[3]<<8 | packetBuffer[2]);
    int16_t rawRoll = (packetBuffer[5]<<8 | packetBuffer[4]);
    int16_t rawYaw = (packetBuffer[7]<<8 | packetBuffer[6]);
    
    angleRoll = rawRoll * angleScale;
    anglePitch = rawPitch * angleScale;
    angleYaw = rawYaw * angleScale;
    
    angleRoll = normalizeAngle(angleRoll);
    anglePitch = normalizeAngle(anglePitch);
    angleYaw = normalizeAngle(angleYaw);
  }
}

// ===================== attitude control =====================
void updateControl() {
  lastYawError = yawError;
  float currentYaw = normalizeAngle(angleYaw - yawOffset);
  yawError = normalizeAngle(currentYaw - setpointYaw);
  yawErrorDerivative = (yawError - lastYawError) / DT;
  
  yawOutput = sSurfaceControl(yawError, yawErrorDerivative, &yawDeltaU, YAW_ZETA1, YAW_ZETA2);
  
  if((currentMode == MODE_YAW_PITCH) || (currentMode == MODE_ALL)) {
    lastPitchError = pitchError;
    float currentPitch = normalizeAngle(anglePitch - pitchOffset);
    pitchError = normalizeAngle(currentPitch - setpointPitch);
    pitchErrorDerivative = (pitchError - lastPitchError) / DT;
    pitchOutput = sSurfaceControl(pitchError, pitchErrorDerivative, &pitchDeltaU, PITCH_ZETA1, PITCH_ZETA2);
    rollOutput = 0; 
  }
  if((currentMode == MODE_YAW_ROLL) || (currentMode == MODE_ALL)){
    lastRollError = rollError;
    float currentRoll = normalizeAngle(angleRoll - rollOffset);
    rollError = normalizeAngle(currentRoll - setpointRoll);
    rollErrorDerivative = (rollError - lastRollError) / DT;
    rollOutput = sSurfaceControl(rollError, rollErrorDerivative, &rollDeltaU, ROLL_ZETA1, ROLL_ZETA2);
    pitchOutput = 0;
  }
  
  for(int i=0; i<8; i++){
    int pulse = NEUTRAL_PULSE + 
               yawOutput * OUTPUT_SCALE * thrustMatrix[i][0] +
               pitchOutput * OUTPUT_SCALE * thrustMatrix[i][1] +
               rollOutput * OUTPUT_SCALE * thrustMatrix[i][2];
    setThruster(i, pulse);
  }
}

// ===================== constain to (-180, 180) =====================
float normalizeAngle(float angle) {
  angle = fmod(angle + 180, 360);
  if (angle < 0) angle += 360;
  return angle - 180;
}

// ===================== Serial data handle =====================
void handleSerialInput() {
  if(Serial.available()){
    char cmd = Serial.read();
    switch(cmd){
      case 'a': // Enable control system
        systemArmed = true;
        yawOffset = angleYaw;
        pitchOffset = anglePitch;
        rollOffset = angleRoll;
        yawDeltaU = 0.0;
        pitchDeltaU = 0.0;
        rollDeltaU = 0.0;
        Serial.println("Control system had been activated.");
        break;
        
      case 'e': // emergent stop
        systemArmed = false;
        for(int i=0; i<8; i++) setThruster(i, 1500);
        Serial.println("Emergent Stop.");
        break;
        
      case 'y': // yaw target
        setpointYaw = Serial.parseFloat();
        break;
        
      case 'p': // pitch target
        setpointPitch = Serial.parseFloat();
        break;
        
      case 'r': // roll target
        setpointRoll = Serial.parseFloat();
        break;

      // controller parameter adjustment
      case 'z': 
        YAW_ZETA1 *= Serial.parseFloat();
        break;
        
      case 'x': 
        YAW_ZETA2 *= Serial.parseFloat();
        break;

      case 'v':
        PITCH_ZETA1 *= Serial.parseFloat();
        break;
        
      case 'b':
        PITCH_ZETA2 *= Serial.parseFloat();
        break;

      case 'n':
        ROLL_ZETA1 *= Serial.parseFloat();
        break;
        
      case 'm':
        ROLL_ZETA2 *= Serial.parseFloat();
        break;

      case 'K':
        currentMode = MODE_YAW_PITCH;
        Serial.println("Now Mode: YAW+PITCH");
        break;
        
      case 'L':
        currentMode = MODE_YAW_ROLL;
        Serial.println("Now Mode: YAW+ROLL");
        break;

      case 'M':
        currentMode = MODE_ALL;
        Serial.println("Now Mode: YAW+ROLL+PITCH");
        break;
    }
  }
}
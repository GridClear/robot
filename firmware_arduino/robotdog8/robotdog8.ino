// ============================================================================
// robotdog8 — 8-servo quadruped bring-up + scripted trot gait (ESP32 + PCA9685)
//
// 4 legs x 2 joints (hip-abduction + knee). PCA9685 channels 0..7:
//   ch0 FL_abd  ch1 FL_knee   ch2 FR_abd  ch3 FR_knee
//   ch4 RL_abd  ch5 RL_knee   ch6 RR_abd  ch7 RR_knee
//
// 180-deg servos, calibrated in PCA9685 ticks (SERVOMIN/MAX), 50 Hz — matching
// the user's validated single-servo test sketch.
//
// Control: line-based serial @ 115200. Everything is tunable LIVE (no reflash):
//   c                 center all servos (safe pose), stop gait
//   s                 stand pose (= centers), stop gait
//   r                 relax  (cut PWM signal to all servos)
//   x                 stop gait, hold current pose
//   w                 start walking (scripted trot gait)
//   t <ch> <deg>      test: drive one channel to a raw angle (0..180)
//   j <ch> <deg>      alias of t
//   amp  <deg>        abduction swing amplitude
//   kamp <deg>        knee swing amplitude
//   freq <hz>         gait frequency
//   phase <deg>       abduction->knee phase offset
//   dir <ch> <+1|-1>  flip a servo's direction
//   ctr <ch> <deg>    set a servo's center angle
//   lim <ch> <lo> <hi> set a servo's clamp range (deg)
//   slew <deg/s>      max slew rate
//   dump              print calibration + gait params + targets
//   save              record CURRENT commanded angles as the stand pose -> NVS (persists)
//   loadcal           reload the saved stand pose from NVS
//   rstcal            reset centers to 90 deg (not saved until 'save')
// ============================================================================
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Preferences.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Preferences prefs;                 // NVS storage for the saved stand pose
#define CAL_NS  "robotdog8"        // NVS namespace
#define CAL_KEY "centers"         // blob of NSERVO floats (per-channel center deg)

#define SERVOMIN   125     // PCA9685 tick at 0 deg   (user-validated)
#define SERVOMAX   575     // PCA9685 tick at 180 deg (user-validated)
#define SERVO_FREQ 50
#define SDA_PIN    21
#define SCL_PIN    22
#define NSERVO     8
#define CONTROL_HZ 50

// per-servo calibration (degrees)
struct Cal { float center; float lo; float hi; int dir; };
Cal cal[NSERVO];

const char* JNAME[NSERVO] = {
  "FL_abd","FL_knee","FR_abd","FR_knee","RL_abd","RL_knee","RR_abd","RR_knee"
};

float target[NSERVO];    // commanded servo angle (deg, 0..180)
float current[NSERVO];   // slewed actual angle (deg)
bool  walking = false;
bool  relaxed = false;

// gait params (live-tunable)
float gAmpAbd  = 18.0f;   // abduction swing (deg)
float gAmpKnee = 25.0f;   // knee swing (deg)
float gFreq    = 1.2f;    // Hz
float gPhase   = 90.0f;   // abduction->knee phase (deg)
float gSlew    = 240.0f;  // deg/s
float gaitClock = 0.0f;

// trot: diagonal pairs FL+RR and FR+RL in antiphase
const float legPhaseDeg[4] = { 0.0f, 180.0f, 180.0f, 0.0f };  // FL,FR,RL,RR

static inline float clampf(float v, float a, float b){ return v<a?a:(v>b?b:v); }
static inline float d2r(float d){ return d*0.0174532925f; }

void writeServo(int ch, float deg) {
  deg = clampf(deg, cal[ch].lo, cal[ch].hi);
  int pulse = (int)(SERVOMIN + (SERVOMAX - SERVOMIN) * (deg / 180.0f));
  pwm.setPWM(ch, 0, pulse);
}

// Load saved stand-pose centers from NVS into cal[].center (no-op if none saved).
bool loadCal() {
  prefs.begin(CAL_NS, true);                 // read-only
  size_t n = prefs.getBytesLength(CAL_KEY);
  bool ok = (n == sizeof(float)*NSERVO);
  if (ok) {
    float buf[NSERVO];
    prefs.getBytes(CAL_KEY, buf, sizeof(buf));
    for (int i=0;i<NSERVO;i++) cal[i].center = buf[i];
  }
  prefs.end();
  return ok;
}

// Capture the CURRENT commanded angles as the new stand-pose centers + persist.
// NOTE: open-loop servos -> this records what was last COMMANDED (e.g. via `t`),
// not where a relaxed leg was physically moved by hand. Jog with `t` first.
void saveCal() {
  for (int i=0;i<NSERVO;i++) cal[i].center = current[i];
  float buf[NSERVO];
  for (int i=0;i<NSERVO;i++) buf[i] = cal[i].center;
  prefs.begin(CAL_NS, false);                // read-write
  prefs.putBytes(CAL_KEY, buf, sizeof(buf));
  prefs.end();
}

void centerAll() {
  walking = false; relaxed = false;
  for (int i=0;i<NSERVO;i++) target[i] = cal[i].center;
}

void relaxAll() {
  walking = false; relaxed = true;
  for (int i=0;i<NSERVO;i++) pwm.setPWM(i, 0, 0);   // 0 = no pulse -> servo released
}

void gaitStep(float dt) {
  if (!walking) return;
  gaitClock += dt;
  for (int leg=0; leg<4; leg++) {
    float ph = d2r(legPhaseDeg[leg] + 360.0f*gFreq*gaitClock);
    float abdOff  = gAmpAbd  * sinf(ph);
    float kneeOff = gAmpKnee * sinf(ph + d2r(gPhase));
    int a = leg*2, k = leg*2+1;
    target[a] = clampf(cal[a].center + cal[a].dir*abdOff,  cal[a].lo, cal[a].hi);
    target[k] = clampf(cal[k].center + cal[k].dir*kneeOff, cal[k].lo, cal[k].hi);
  }
}

void update(float dt) {
  if (relaxed) return;
  float maxStep = gSlew * dt;
  for (int i=0;i<NSERVO;i++) {
    float e = target[i] - current[i];
    if (e >  maxStep) e =  maxStep;
    if (e < -maxStep) e = -maxStep;
    current[i] += e;
    writeServo(i, current[i]);
  }
}

void dump() {
  Serial.println(F("--- robotdog8 state ---"));
  for (int i=0;i<NSERVO;i++) {
    Serial.printf("ch%d %-8s ctr=%.0f lo=%.0f hi=%.0f dir=%+d  target=%.1f cur=%.1f\n",
      i, JNAME[i], cal[i].center, cal[i].lo, cal[i].hi, cal[i].dir, target[i], current[i]);
  }
  Serial.printf("gait: walking=%d amp=%.1f kamp=%.1f freq=%.2f phase=%.0f slew=%.0f relaxed=%d\n",
    walking, gAmpAbd, gAmpKnee, gFreq, gPhase, gSlew, relaxed);
}

String line;
void handleLine(String s) {
  s.trim(); if (s.length()==0) return;
  char cmd = s[0];
  // tokenize
  int sp1 = s.indexOf(' ');
  String rest = sp1<0 ? "" : s.substring(sp1+1); rest.trim();

  if (s=="c")      { centerAll(); Serial.println(F("centered")); }
  else if (s=="s") { centerAll(); Serial.println(F("stand")); }
  else if (s=="r") { relaxAll(); Serial.println(F("relaxed")); }
  else if (s=="x") { walking=false; for(int i=0;i<NSERVO;i++) target[i]=current[i]; Serial.println(F("stop/hold")); }
  else if (s=="w") { relaxed=false; walking=true; gaitClock=0; Serial.println(F("walking")); }
  else if (s=="dump") { dump(); }
  else if (s=="save") { saveCal(); Serial.print(F("saved stand pose to NVS:")); for(int i=0;i<NSERVO;i++) Serial.printf(" %s=%.0f", JNAME[i], cal[i].center); Serial.println(); }
  else if (s=="loadcal") { if (loadCal()){ centerAll(); Serial.println(F("loaded saved stand pose")); } else Serial.println(F("no saved pose in NVS")); }
  else if (s=="rstcal")  { for(int i=0;i<NSERVO;i++) cal[i].center=90.0f; Serial.println(F("centers reset to 90 (use 'save' to persist)")); }
  else if (s.startsWith("t ")||s.startsWith("j ")) {
    int ch,deg; if (sscanf(rest.c_str(), "%d %d", &ch,&deg)==2 && ch>=0&&ch<NSERVO){
      walking=false; relaxed=false; target[ch]=clampf(deg,cal[ch].lo,cal[ch].hi);
      Serial.printf("ch%d -> %d deg\n", ch, deg);
    } else Serial.println(F("usage: t <ch 0-7> <deg 0-180>"));
  }
  else if (s.startsWith("amp "))  { gAmpAbd=rest.toFloat();  Serial.printf("amp=%.1f\n",gAmpAbd); }
  else if (s.startsWith("kamp ")) { gAmpKnee=rest.toFloat(); Serial.printf("kamp=%.1f\n",gAmpKnee); }
  else if (s.startsWith("freq ")) { gFreq=rest.toFloat();    Serial.printf("freq=%.2f\n",gFreq); }
  else if (s.startsWith("phase ")){ gPhase=rest.toFloat();   Serial.printf("phase=%.0f\n",gPhase); }
  else if (s.startsWith("slew ")) { gSlew=rest.toFloat();    Serial.printf("slew=%.0f\n",gSlew); }
  else if (s.startsWith("dir ")) {
    int ch,d; if (sscanf(rest.c_str(),"%d %d",&ch,&d)==2 && ch>=0&&ch<NSERVO){ cal[ch].dir=(d<0)?-1:1; Serial.printf("dir ch%d=%+d\n",ch,cal[ch].dir);} }
  else if (s.startsWith("ctr ")) {
    int ch; float v; if (sscanf(rest.c_str(),"%d %f",&ch,&v)==2 && ch>=0&&ch<NSERVO){ cal[ch].center=v; Serial.printf("ctr ch%d=%.0f\n",ch,v);} }
  else if (s.startsWith("lim ")) {
    int ch; float lo,hi; if (sscanf(rest.c_str(),"%d %f %f",&ch,&lo,&hi)==3 && ch>=0&&ch<NSERVO){ cal[ch].lo=lo; cal[ch].hi=hi; Serial.printf("lim ch%d [%.0f,%.0f]\n",ch,lo,hi);} }
  else Serial.printf("unknown cmd: %s\n", s.c_str());
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  for (int i=0;i<NSERVO;i++){ cal[i]={90.0f,15.0f,165.0f,+1}; current[i]=90.0f; target[i]=90.0f; }
  // Right-side legs are MIRROR-MOUNTED (see robot/images): the FR & RR servos face
  // opposite the left side, so the same angle rotates them the other way. Flip their
  // gait direction so the trot is physically symmetric. (dir only affects gaitStep;
  // c/s/t still write absolute angles, so centering stays symmetric at 90 deg.)
  cal[2].dir = -1;  // FR_abd
  cal[3].dir = -1;  // FR_knee
  cal[6].dir = -1;  // RR_abd
  cal[7].dir = -1;  // RR_knee
  bool haveSaved = loadCal();   // override default 90 centers with saved stand pose
  delay(500);
  centerAll();
  Serial.println(haveSaved ? F("loaded saved stand pose from NVS")
                           : F("no saved stand pose - using 90 deg centers"));
  Serial.println();
  Serial.println(F("robotdog8 ready (8 servos, ch0-7 = FL/FR/RL/RR abd+knee)."));
  Serial.println(F("cmds: c center | t <ch> <deg> | s stand | w walk | x stop | r relax | dump | save | loadcal | rstcal"));
}

void loop() {
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch=='\n'||ch=='\r'){ if(line.length()) { handleLine(line); line=""; } }
    else line += ch;
  }

  static unsigned long last=0, lastLog=0;
  unsigned long now=millis();
  unsigned long period = 1000/CONTROL_HZ;
  if (now-last < period) return;
  float dt=(now-last)/1000.0f; last=now;

  gaitStep(dt);
  update(dt);

  if (walking && now-lastLog > 500) {   // ~2 Hz gait telemetry
    lastLog=now;
    Serial.printf("[t=%.1f] FL(%.0f,%.0f) FR(%.0f,%.0f) RL(%.0f,%.0f) RR(%.0f,%.0f)\n",
      gaitClock, current[0],current[1],current[2],current[3],
      current[4],current[5],current[6],current[7]);
  }
}

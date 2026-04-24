// ============================================================
//  TRI-FACTOR BIOMETRIC SYSTEM — Arduino Uno Q (Master)
//  Includes Hardware Watchdog for Master Fingerprint Sensor.
// ============================================================

#include <Adafruit_Fingerprint.h>
#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>

Adafruit_Fingerprint finger = Adafruit_Fingerprint(&Serial);
hd44780_I2Cexp lcd;

uint8_t templateBuffer[512];

int rowPins[4] = {2, 3, 4, 5};
int colPins[4] = {A2, A3, A4, A5};
char keys[4][4] = {{'1', '2', '3', 'A'},
                   {'4', '5', '6', 'B'},
                   {'7', '8', '9', 'C'},
                   {'*', '0', '#', 'D'}};

void enrollFlow();
void recognizeFlow();
void countFlow();
void deleteFlow();
bool enrollFinger(int id, int tries);
bool scanFinger(int expectedId, int tries);
bool awaitResult(const char *ok, const char *fail, unsigned long ms);
void mirrorPythonLcd();
String getIdFromKeypad(const char *title, const char *prefix,
                       bool checkR3 = false);
void lcdPrint(const char *l1, const char *l2);
void showMainMenu();
char readKeypad();
bool extractAndSendTemplate(int id);
bool verifyAdmin();
void enrollAdminFlow();
bool waitForR3Reconnect();


const int LED_GREEN = 8;
const int LED_RED   = 6;

enum LedState {
  LED_IDLE,           // Green heartbeat — waiting
  LED_SCANNING,       // Green slow blink — "place finger / look at camera"
  LED_FP_ENROLL,      // Both rapid alternate 80ms — "capturing fingerprint"
  LED_FACE_ACTIVE,    // Green fast + red rare blip — "camera is watching"
  LED_TRANSFER,       // Sequential fill: G→R→both — "sending data to R3"
  LED_FACE_PROCESS,   // Red fast + green slow — "AI computing face math"
  LED_PROCESSING,     // Green/red alternate 200ms — "generic working"
  LED_SUCCESS,        // Triple green flash then solid — "done!"
  LED_ERROR           // Double red pulse — "something went wrong"
};
LedState masterLedState = LED_IDLE;

void setMasterState(LedState state) {
  masterLedState = state;
  // Prime LEDs immediately on state entry — no visual lag
  if (state == LED_SUCCESS) {
    digitalWrite(LED_GREEN, HIGH); digitalWrite(LED_RED, LOW);
  } else if (state == LED_ERROR) {
    digitalWrite(LED_RED, HIGH);  digitalWrite(LED_GREEN, LOW);
  } else if (state == LED_IDLE) {
    digitalWrite(LED_GREEN, LOW); digitalWrite(LED_RED, LOW);
  } else if (state == LED_TRANSFER) {
    // Start with both off — animation fills them in
    digitalWrite(LED_GREEN, LOW); digitalWrite(LED_RED, LOW);
  }
}

void updateMasterLEDs() {
  unsigned long ms = millis();

  if (masterLedState == LED_IDLE) {
    // Gentle green heartbeat (1.5s): alive and waiting
    bool on = (ms % 1500) < 400;
    digitalWrite(LED_GREEN, on ? HIGH : LOW);
    digitalWrite(LED_RED, LOW);

  } else if (masterLedState == LED_SCANNING) {
    // Slow green blink (500ms) — "place finger / look at camera"
    bool on = (ms / 500) % 2 == 0;
    digitalWrite(LED_GREEN, on ? HIGH : LOW);
    digitalWrite(LED_RED, LOW);

  } else if (masterLedState == LED_FP_ENROLL) {
    // Both LEDs alternate rapidly (80ms) — "capturing fingerprint data"
    // Feels energetic and data-intensive, clearly different from scanning
    bool toggle = (ms / 80) % 2 == 0;
    digitalWrite(LED_GREEN, toggle ? HIGH : LOW);
    digitalWrite(LED_RED,   toggle ? LOW  : HIGH);

  } else if (masterLedState == LED_FACE_ACTIVE) {
    // Camera active: green fast blink (250ms) + red rare blip once per 2s
    // Two-speed pattern is unmistakable — "camera is watching"
    bool greenOn = (ms / 250) % 2 == 0;
    unsigned long phase = ms % 2000;
    bool redBlip = (phase < 80);   // brief red pulse every 2s
    digitalWrite(LED_GREEN, greenOn ? HIGH : LOW);
    digitalWrite(LED_RED,   redBlip ? HIGH : LOW);

  } else if (masterLedState == LED_TRANSFER) {
    // Template transfer progress bar: G on 0-1s, R on 1-2s, both 2-3s, off briefly, repeat
    // Gives a left→right fill feel matching R3's BLED_FP_SYNC
    unsigned long phase = ms % 3200;
    bool g = (phase >= 0    && phase < 2800);
    bool r = (phase >= 1000 && phase < 2800);
    digitalWrite(LED_GREEN, g ? HIGH : LOW);
    digitalWrite(LED_RED,   r ? HIGH : LOW);

  } else if (masterLedState == LED_FACE_PROCESS) {
    // AI face math: red fast blink (150ms) + green slow blink (800ms)
    // Distinct from LED_PROCESSING — more red-dominant = "intensive compute"
    bool redOn   = (ms / 150) % 2 == 0;
    bool greenOn = (ms / 800) % 2 == 0;
    digitalWrite(LED_RED,   redOn   ? HIGH : LOW);
    digitalWrite(LED_GREEN, greenOn ? HIGH : LOW);

  } else if (masterLedState == LED_PROCESSING) {
    // Generic working: alternating green/red (200ms)
    bool toggle = (ms / 200) % 2 == 0;
    digitalWrite(LED_GREEN, toggle ? HIGH : LOW);
    digitalWrite(LED_RED,   toggle ? LOW  : HIGH);

  } else if (masterLedState == LED_SUCCESS) {
    // Triple green flash (60ms) then solid green
    unsigned long phase = ms % 1200;
    bool flashOn = (phase < 60 || (phase > 120 && phase < 180) || (phase > 240 && phase < 300));
    bool solid   = (phase >= 300);
    digitalWrite(LED_GREEN, (flashOn || solid) ? HIGH : LOW);
    digitalWrite(LED_RED, LOW);

  } else if (masterLedState == LED_ERROR) {
    // Double-pulse red SOS: blink-blink ... long pause ...
    unsigned long phase = ms % 1200;
    bool on = (phase < 100) || (phase > 200 && phase < 300);
    digitalWrite(LED_RED,   on ? HIGH : LOW);
    digitalWrite(LED_GREEN, LOW);
  }
}


void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    updateMasterLEDs();
  }
}

void setup() {
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  setMasterState(LED_IDLE);

  Bridge.begin();
  lcd.begin(16, 2);
  for (int i = 0; i < 4; i++) {
    pinMode(colPins[i], INPUT_PULLUP);
    pinMode(rowPins[i], OUTPUT);
    digitalWrite(rowPins[i], HIGH);
  }

  lcdPrint("Starting Up...", "Please Wait");
  finger.begin(57600);
  smartDelay(1200);
  if (!finger.verifyPassword()) {
    lcdPrint("Hardware Error", "Check FP Sensor");
    while (true)
      ;
  }

  String bootStatus = "";
  while (bootStatus != "System Ready") {
    Bridge.call("get_boot_status").result(bootStatus);
    if (bootStatus == "System Ready")
      break;
    if (bootStatus != "") {
      lcdPrint("Starting Up...", "Please Wait");
    }
    smartDelay(400);
  }
  lcdPrint("Ready!", "Boot Complete.");
  smartDelay(1000);

  while (true) {
    String fExists = "";
    Bridge.call("check_user_exists", "1").result(fExists);

    if (finger.loadModel(1) == FINGERPRINT_OK && fExists == "YES") {
      break;
    }

    lcdPrint("Officer Missing", "System Reset");
    finger.deleteModel(1);
    smartDelay(1000);

    enrollAdminFlow();
  }

  lcdPrint("System Ready", "");
  smartDelay(1000);
  showMainMenu();
}

void loop() {
  char key = readKeypad();
  if (!key) {
    smartDelay(20);
    return;
  }

  static String inputBuffer = "";
  if (key == '*' || key == '#' || (key >= '0' && key <= '9')) {
    if (inputBuffer.length() > 15)
      inputBuffer = ""; // Prevent RAM fragmentation
    inputBuffer += key;
  } else {
    inputBuffer = "";
  }

  if (inputBuffer.indexOf("*611#") >= 0) {
    inputBuffer = "";
    if (verifyAdmin()) {
      // Wait until camera is ready before proceeding
      while (true) {
        String camResult = "";
        Bridge.call("begin_cam_mode").result(camResult);
        if (camResult != "ERROR")
          break;
        lcdPrint("Camera missing!", "Plug in & wait");
        smartDelay(2000);
      }
      lcdPrint("Clearing Data...", "Please Wait");
      finger.emptyDatabase();
      lcdPrint("Clearing Data...", "Please Wait");
      Bridge.call("wipe_all_auth_data");
      lcdPrint("WIPED!", "Setup Required");
      smartDelay(2000);

      while (true) {
        String fExists = "";
        Bridge.call("check_user_exists", "1").result(fExists);
        if (finger.loadModel(1) == FINGERPRINT_OK && fExists == "YES") {
          break;
        }
        enrollAdminFlow();
      }

      Bridge.call("end_cam_mode");
      showMainMenu();
    } else {
      showMainMenu();
    }
    return;
  }

  // ── DEV-ONLY: *999# — Unauthenticated factory reset ───────────────────
  // Intentional: no officer auth required. For developer use only.
  // Do NOT remove this comment — this shortcut is kept by design.
  if (inputBuffer.indexOf("*999#") >= 0) {
    inputBuffer = "";
    lcdPrint("Factory Reset", "Clearing All...");
    finger.emptyDatabase();
    Bridge.call("wipe_all_auth_data");
    lcdPrint("RESET DONE", "Re-enroll now");
    smartDelay(2000);

    while (true) {
      String fExists = "";
      Bridge.call("check_user_exists", "1").result(fExists);
      if (finger.loadModel(1) == FINGERPRINT_OK && fExists == "YES") {
        break;
      }
      enrollAdminFlow();
    }

    showMainMenu();
    return;
  }

  switch (key) {
  case 'A': {
    String bootStatus = "";
    Bridge.call("get_boot_status").result(bootStatus);
    if (bootStatus != "System Ready") {
      lcdPrint("AI Bridge Down", "Restart bridge");
      smartDelay(2000);
    } else if (verifyAdmin()) {
      enrollFlow();
    }
  }
    showMainMenu();
    break;
  case 'B': {
    String bootStatus = "";
    Bridge.call("get_boot_status").result(bootStatus);
    if (bootStatus != "System Ready") {
      lcdPrint("System Error", "Check AI Bridge");
      smartDelay(2000);
    } else if (verifyAdmin()) {
      recognizeFlow();
    }
  }
    showMainMenu();
    break;
  case 'C':
    countFlow();
    showMainMenu();
    break;
  case 'D':
    if (verifyAdmin())
      deleteFlow();
    showMainMenu();
    break;
  default:
    break;
  }
}

bool verifyAdmin() {
  // Fetch stored finger name so admin knows which finger to place
  String adminFin = "";
  Bridge.call("get_admin_finger").result(adminFin);
  if (adminFin.length() == 0)
    adminFin = "Enrolled finger";

  setMasterState(LED_SCANNING);
  lcdPrint("Officer Login", adminFin.c_str());
  smartDelay(1200);
  if (!scanFinger(1, 3)) {
    setMasterState(LED_ERROR);
    // Python/scanFinger already shows failures; keeping this simple
    smartDelay(2000);
    return false;
  }
  setMasterState(LED_PROCESSING);
  lcdPrint("Finger OK", "Checking Face..");
  setMasterState(LED_FACE_ACTIVE);    // Camera now running — "watching you"
  Bridge.call("verify_admin_face");
  if (!awaitResult("FACE_GRANTED", "FACE_DENIED", 46000UL)) {
    setMasterState(LED_ERROR);
    smartDelay(2000);
    return false;
  }
  setMasterState(LED_SUCCESS);
  smartDelay(1200);
  return true;
}

// Prompts admin/voter to choose which hand and finger to enroll.
// Returns "Right Index", "Left Thumb" etc.  Returns "" only if user presses *.
String askWhichFinger() {
  Bridge.call("log_bridge", "ASK_FINGER_START");
  // ── Step 1: Hand ────────────────────────────────────────────
  lcdPrint("Which hand?", "A=Right  B=Left");
  String hand = "";
  while (hand == "") {
    char k = readKeypad();
    if (k == 'A')
      hand = "Right";
    else if (k == 'B')
      hand = "Left";
    else if (k == '*')
      return "";
  }

  // ── Step 2: Finger (both LCD lines show all 5 options) ───────
  lcdPrint("1Th 2Idx 3Mid", "4Ring 5Ltle *Bk");
  String fn = "";
  while (fn == "") {
    char k = readKeypad();
    if (k == '1')
      fn = "Thumb";
    else if (k == '2')
      fn = "Index";
    else if (k == '3')
      fn = "Middle";
    else if (k == '4')
      fn = "Ring";
    else if (k == '5')
      fn = "Little";
    else if (k == '*')
      return "";
  }

  String result = hand + " " + fn;
  lcdPrint("Finger selected:", result.c_str());
  smartDelay(1500);
  return result;
}

void enrollAdminFlow() {
  // Wait until camera is ready (mirrors *611# pattern)
  while (true) {
    String camResult = "";
    Bridge.call("begin_cam_mode").result(camResult);
    if (camResult != "ERROR")
      break;
    lcdPrint("Camera missing!", "Plug in & wait");
    smartDelay(2000);
  }
  lcdPrint("NO LOGIN FOUND", "Enroll ID: 1");
  smartDelay(2000);

  // Wipe any stale face data for ID 1 before re-enrolling.
  lcdPrint("Clearing old", "Officer data...");
  Bridge.call("delete_admin_face");
  smartDelay(800);

  lcdPrint("Starting face", "scan...");
  Bridge.call("start_enrollment", "1");
  smartDelay(500);

  lcdPrint("Face capture...", "Please wait");
  if (!awaitResult("CAPTURE_DONE", "FACE_ERROR", 180000UL)) {
    lcdPrint("Face capture", "FAILED");
    smartDelay(2500);
    Bridge.call("end_cam_mode");
    return;
  }

  // ── Ask which finger (mandatory for admin — loop until chosen) ──
  String fingerName = "";
  while (fingerName == "") {
    fingerName = askWhichFinger();
    if (fingerName == "") {
      lcdPrint("Must select", "a finger!");
      smartDelay(1500);
    }
  }

  lcdPrint("Scan Finger Now", fingerName.c_str());
  smartDelay(800);

  Bridge.call("set_admin_finger", fingerName.c_str());

  bool fpOk = enrollFinger(1, 3);

  if (fpOk) {
    Bridge.call("fp_success", "1");
    lcdPrint("FP done!", "Saving data...");
    if (!awaitResult("FACE_DONE", "FACE_ERROR", 150000UL)) {
      lcdPrint("Face math", "FAILED");
      finger.deleteModel(1);
      smartDelay(2500);
      Bridge.call("end_cam_mode");
      return;
    }
    lcdPrint("* ADMIN SAVED *", fingerName.c_str());
    smartDelay(3000);
  } else {
    Bridge.call("fp_failed", "1");
    lcdPrint("FP FAILED", "Rolled back");
    smartDelay(2500);
  }
  Bridge.call("end_cam_mode");
}

void enrollFlow() {
  // Wait until camera is ready (mirrors *611# pattern)
  while (true) {
    String camResult = "";
    Bridge.call("begin_cam_mode").result(camResult);
    if (camResult != "ERROR")
      break;
    lcdPrint("Camera missing!", "Plug in & wait");
    smartDelay(2000);
  }
  while (true) {
    String uid = getIdFromKeypad("Enrollment mode", "Voter ID:");
    if (uid == "") {
      Bridge.call("end_cam_mode");
      return;
    }

    int fpId = uid.toInt();
    if (fpId < 2 || fpId > 1000) {
      lcdPrint("ID 2-1000 only", "Admin is ID 1");
      smartDelay(2200);
      continue;
    }

    lcdPrint("Searching...", uid.c_str());
    String exists = "NO";
    Bridge.call("check_user_exists", uid.c_str()).result(exists);
    if (exists == "YES") {
      lcdPrint("Identity Found", "Delete to Re-use");
      smartDelay(2500);
      continue;
    }

    String voted = "NO";
    Bridge.call("check_has_voted", uid.c_str()).result(voted);
    if (voted == "YES") {
      lcdPrint("Already Voted", "ID is Blocked");
      smartDelay(2500);
      continue;
    }
    if (voted == "ERROR") {
      lcdPrint("DB Error", "Try again later");
      smartDelay(2500);
      continue;
    }

    lcdPrint("Starting...", "Identity Scan");
    setMasterState(LED_FACE_ACTIVE);    // Face enrollment camera active
    Bridge.call("start_enrollment", uid.c_str());
    smartDelay(500);

    if (!awaitResult("CAPTURE_DONE", "FACE_ERROR", 180000UL)) {
      setMasterState(LED_ERROR);
      smartDelay(2500);
      continue;
    }
    setMasterState(LED_FACE_PROCESS);   // Face math / embedding computing

    String fingerName = askWhichFinger();
    if (fingerName == "") {
      lcdPrint("Cancelled", "Back to ID entry");
      Bridge.call("cancel_operation");
      smartDelay(1500);
      continue;
    }

    String combined = uid + "|" + fingerName;
    Bridge.call("set_voter_finger", combined.c_str());

    lcdPrint("Scan Finger Now", fingerName.c_str());
    smartDelay(800);
    setMasterState(LED_FP_ENROLL);      // FP enrollment — rapid alternating LEDs
    bool fpOk = enrollFinger(fpId, 3);

    if (fpOk) {
      Bridge.call("fp_success", uid.c_str());
      lcdPrint("FP done!", "Saving data...");
      setMasterState(LED_FACE_PROCESS); // Face math finalising
      if (!awaitResult("FACE_DONE", "FACE_ERROR", 150000UL)) {
        lcdPrint("Face math", "FAILED");
        finger.deleteModel(fpId);
        smartDelay(2500);
        continue;
      }
      String doneMsg = "ID:" + uid + " Done!";
      lcdPrint("* ENROLLED *", doneMsg.c_str());
      smartDelay(1500);
      lcdPrint(fingerName.c_str(), "Remember this!");
      smartDelay(2500);
    } else {
      Bridge.call("fp_failed", uid.c_str());
      lcdPrint("FP FAILED", "Rolled back");
      smartDelay(2500);
    }
  }
}

void recognizeFlow() {
  // ── Tell Python to start socat ── ballot mode only ─────────────
  Bridge.call("begin_ballot_mode");
  // Wait until camera is ready (mirrors *611# pattern)
  while (true) {
    String camResult = "";
    Bridge.call("begin_cam_mode").result(camResult);
    if (camResult != "ERROR")
      break;
    lcdPrint("Camera missing!", "Plug in & wait");
    smartDelay(2000);
  }
  smartDelay(300);

  // ── Wait for R3 to come online ──────────────────────────────────
  {
    String r3Status = "";
    Bridge.call("get_r3_status").result(r3Status);
    bool msgToggle = false;
    unsigned long lastToggle = millis();

    while (r3Status != "ONLINE") {
      if (millis() - lastToggle > 800) {
        lastToggle = millis();
        msgToggle = !msgToggle;
        if (msgToggle)
          lcdPrint("Unit Offline", "Plug in unit");
        else
          lcdPrint("Ballot halted", "Plug in R3 now");
      }
      smartDelay(500);
      char key = readKeypad();
      if (key == '*') {
        lcdPrint("Ballot Cancelled", "");
        Bridge.call("end_cam_mode");
        Bridge.call("end_ballot_mode");
        smartDelay(1500);
        return;
      }
      Bridge.call("get_r3_status").result(r3Status);
    }
    lcdPrint("Unit Connected!", "Starting...");
    smartDelay(800);
  }

  // ── Check unvoted count ─────────────────────────────────────────
  String unvoted = "0";
  Bridge.call("get_unvoted_count").result(unvoted);
  if (unvoted.toInt() == 0) {
    lcdPrint("All Votes Cast", "Poll Closed");
    smartDelay(2500);
    Bridge.call("end_cam_mode");
    Bridge.call("end_ballot_mode");
    return;
  }

  // ── Sync Admin Template with R3 ─────────────────────────────────
  while (true) {
    lcdPrint("Connecting...", "Unit Prep");
    setMasterState(LED_TRANSFER);       // Admin template sync in progress

    String adminExtract = "";
    Bridge.call("extract_template", "1").result(adminExtract);

    if (adminExtract == "ERROR" || adminExtract == "TIMEOUT") {
      if (waitForR3Reconnect())
        continue;
      Bridge.call("end_cam_mode");
      Bridge.call("end_ballot_mode");
      return;
    }
    if (adminExtract != "EXTRACT") {
      lcdPrint("Extract ERROR", "Try again");
      smartDelay(2000);
      Bridge.call("end_cam_mode");
      Bridge.call("end_ballot_mode");
      return;
    }
    if (!extractAndSendTemplate(1)) {
      if (waitForR3Reconnect())
        continue;
      Bridge.call("end_cam_mode");
      Bridge.call("end_ballot_mode");
      return;
    }

    String adminFin = "";
    Bridge.call("finish_template_transfer", "1").result(adminFin);

    if (adminFin == "ERROR" || adminFin == "TIMEOUT") {
      if (waitForR3Reconnect())
        continue;
      Bridge.call("end_cam_mode");
      Bridge.call("end_ballot_mode");
      return;
    }
    if (adminFin != "OK") {
      lcdPrint("Finish FAILED", "Try again");
      smartDelay(2000);
      Bridge.call("end_cam_mode");
      Bridge.call("end_ballot_mode");
      return;
    }
    break;
  }

  // ── Main Ballot Loop ─────────────────────────────────────────────
  while (true) {
    String uid = getIdFromKeypad("Ballot Mode", "Voter ID:", true);
    if (uid == "") {
      Bridge.call("end_cam_mode");
      Bridge.call("end_ballot_mode");
      return;
    }

    String vStatus = "NO";
    Bridge.call("check_has_voted", uid.c_str()).result(vStatus);
    if (vStatus == "YES") {
      lcdPrint("Already Voted!", "Access Denied");
      smartDelay(2000);
      continue;
    }
    if (vStatus == "ERROR") {
      lcdPrint("DB Error", "Ballot halted");
      smartDelay(2500);
      continue;
    }

    String exists = "NO";
    Bridge.call("check_user_exists", uid.c_str()).result(exists);
    if (exists != "YES") {
      lcdPrint("ID Not Found", uid.c_str());
      smartDelay(2000);
      continue;
    }

    String voterFin = "";
    Bridge.call("get_voter_finger", uid.c_str()).result(voterFin);
    if (voterFin.length() == 0)
      voterFin = "Enrolled finger";

    lcdPrint("FP Scan (Master)", voterFin.c_str());
    smartDelay(1200);
    setMasterState(LED_SCANNING);       // FP recognition
    bool fpOk = scanFinger(uid.toInt(), 3);
    if (!fpOk) {
      lcdPrint("FP Denied", "Identity fail");
      Bridge.call("notify_access", "DENIED");
      smartDelay(3000);
      continue;
    }

    lcdPrint("FP OK", "Preparing unit..");
    smartDelay(800);

    String r3Voted = "";
    Bridge.call("check_voted_r3", uid.c_str()).result(r3Voted);
    if (r3Voted == "VOTED") {
      lcdPrint("Already Voted", "R3 Check: VOTED");
      smartDelay(2500);
      continue;
    }

    // Transfer voter template to R3
    bool transferSuccess = false;
    while (true) {
      lcdPrint("Syncing Voter..", uid.c_str());
      setMasterState(LED_TRANSFER);     // Template transfer to R3

      String extract = "";
      Bridge.call("extract_template", uid.c_str()).result(extract);
      if (extract == "ERROR" || extract == "TIMEOUT") {
        if (waitForR3Reconnect())
          continue;
        Bridge.call("end_cam_mode");
        Bridge.call("end_ballot_mode");
        return;
      }
      if (!extractAndSendTemplate(uid.toInt())) {
        if (waitForR3Reconnect())
          continue;
        Bridge.call("end_cam_mode");
        Bridge.call("end_ballot_mode");
        return;
      }

      String fin = "";
      Bridge.call("finish_template_transfer", uid.c_str()).result(fin);
      if (fin == "ERROR" || fin == "TIMEOUT") {
        if (waitForR3Reconnect())
          continue;
        Bridge.call("end_cam_mode");
        Bridge.call("end_ballot_mode");
        return;
      }
      if (fin != "OK") {
        lcdPrint("Finish FAILED", "Try again");
        smartDelay(2000);
        break;
      }
      transferSuccess = true;
      break;
    }

    if (!transferSuccess)
      continue;

    lcdPrint("Sending to R3", uid.c_str());
    setMasterState(LED_FACE_ACTIVE);    // Ballot session: face recognition on R3
    Bridge.call("run_ballot_session", uid.c_str());
    smartDelay(500);

    if (!awaitResult("BALLOT_SUCCESS", "BALLOT_ABORT", 120000UL)) {
      smartDelay(3500);
      continue;
    }
    smartDelay(3500);
  }
}

// ══════════════════════════════════════════════════════════
// R3 Watchdog Function - Waits for R3 reconnection
// Returns: true if reconnected, false if user cancelled
// ══════════════════════════════════════════════════════════
bool waitForR3Reconnect() {
  Bridge.call("lock_r3");
  smartDelay(500);

  bool msgToggle = false;
  unsigned long lastToggle = millis();

  while (true) {
    smartDelay(500);

    String r3Status = "";
    Bridge.call("get_r3_status").result(r3Status);

    if (r3Status == "ONLINE") {
      lcdPrint("R3 RECONNECTED!", "Resuming...");
      smartDelay(1200);
      return true;
    }

    if (millis() - lastToggle > 800) {
      lastToggle = millis();
      msgToggle = !msgToggle;
      if (msgToggle)
        lcdPrint("Unit Offline", "Plug in unit");
      else
        lcdPrint("Ballot halted", "Plug in unit");
    }

    char key = readKeypad();
    if (key == '*') {
      lcdPrint("Ballot Cancelled", "Unit unplugged");
      smartDelay(1500);
      return false;
    }
  }
}

void countFlow() {
  lcdPrint("Counting...", "Please wait");
  finger.getTemplateCount();
  int fp = finger.templateCount;
  String fc = "0";
  Bridge.call("get_face_count").result(fc);
  String line1 = "FP:" + String(max(0, fp - 1)) + " Face:" + fc;
  lcdPrint(line1.c_str(), "Total voters");
  smartDelay(4500);
}

void deleteFlow() {
  Bridge.call("begin_cam_mode");
  finger.getTemplateCount();
  if (finger.templateCount <= 1) {
    lcdPrint("No Voters", "To Delete");
    smartDelay(2500);
    Bridge.call("end_cam_mode");
    return;
  }

  while (true) {
    String uid = getIdFromKeypad("Delete Mode", "Voter ID:");
    if (uid == "") {
      Bridge.call("end_cam_mode");
      return;
    }

    int id = uid.toInt();
    if (id == 1) {
      lcdPrint("Admin Cannot", "Be Deleted");
      smartDelay(2000);
      continue;
    }

    if (id == 0) {
      lcdPrint("Wipe non-admins?", "A=YES *=NO");
      while (true) {
        char k = readKeypad();
        if (k == 'A') {
          lcdPrint("Deleting FPs...", "Please wait");
          finger.emptyDatabase(); // Instant hardware wipe
          lcdPrint("Deleting faces..", "Please wait");
          Bridge.call("delete_non_admins");
          lcdPrint("Users Wiped!", "");
          smartDelay(2000);
          break;
        }
        if (k == '*')
          break;
      }
    } else {
      lcdPrint("Deleting FP...", "Please wait");
      finger.deleteModel(id);
      Bridge.call("delete_single_user", uid.c_str());
      lcdPrint("Deleted ID:", uid.c_str());
      smartDelay(2000);
    }
  }
}

bool enrollFinger(int id, int tries) {
  for (int t = 1; t <= tries; t++) {
    String hdr = "FP Enroll " + String(t) + "/" + String(tries);
    setMasterState(LED_SCANNING);
    lcdPrint(hdr.c_str(), "Place finger...");
    unsigned long ts = millis();

    while (true) {
      if (readKeypad() == '*')
        return false;
      uint8_t imgStatus = finger.getImage();

      if (imgStatus == FINGERPRINT_PACKETRECIEVEERR) {
        lcdPrint("Sensor Lost!", "Reconnect it");
        while (!finger.verifyPassword()) {
          if (readKeypad() == '*')
            return false;
          smartDelay(1000);
        }
        lcdPrint("Sensor Back!", "Continuing...");
        smartDelay(1000);
        lcdPrint(hdr.c_str(), "Place finger...");
        ts = millis();
        continue;
      }

      if (imgStatus == FINGERPRINT_OK)
        break;
      if (millis() - ts > 12000UL) {
        setMasterState(LED_ERROR);
        lcdPrint("No Finger!", "Time is up");
        smartDelay(1200);
        goto next_enroll_try;
      }
      smartDelay(50);
    }

    setMasterState(LED_PROCESSING);
    if (finger.image2Tz(1) != FINGERPRINT_OK) {
      setMasterState(LED_ERROR);
      lcdPrint("Bad image", "");
      smartDelay(1000);
      goto next_enroll_try;
    }

    lcdPrint("Remove finger", "");
    smartDelay(1500);
    while (finger.getImage() != FINGERPRINT_NOFINGER)
      smartDelay(100);

    setMasterState(LED_SCANNING);
    lcdPrint("Same angle", "Scan again...");
    ts = millis();
    while (true) {
      if (readKeypad() == '*')
        return false;
      uint8_t imgStatus = finger.getImage();

      if (imgStatus == FINGERPRINT_PACKETRECIEVEERR) {
        lcdPrint("Sensor Lost!", "Reconnect it");
        while (!finger.verifyPassword()) {
          if (readKeypad() == '*')
            return false;
          smartDelay(1000);
        }
        lcdPrint("Sensor Back!", "Continuing...");
        smartDelay(1000);
        lcdPrint("Same angle", "Scan again...");
        ts = millis();
        continue;
      }

      if (imgStatus == FINGERPRINT_OK)
        break;
      if (millis() - ts > 12000UL) {
        setMasterState(LED_ERROR);
        lcdPrint("No Finger!", "Time is up");
        smartDelay(1200);
        goto next_enroll_try;
      }
      smartDelay(50);
    }

    setMasterState(LED_PROCESSING);
    if (finger.image2Tz(2) != FINGERPRINT_OK) {
      setMasterState(LED_ERROR);
      lcdPrint("Bad image 2", "");
      smartDelay(1000);
      goto next_enroll_try;
    }

    if (finger.createModel() != FINGERPRINT_OK) {
      setMasterState(LED_ERROR);
      lcdPrint("Prints differ", "Try again");
      smartDelay(1500);
      goto next_enroll_try;
    }

    // FRAUD CHECK: fingerprint already exists in DB?
    if (finger.fingerFastSearch() == FINGERPRINT_OK) {
      setMasterState(LED_ERROR);
      lcdPrint("Duplicate FP!",
               (String("Matches ID:") + String(finger.fingerID)).c_str());
      smartDelay(3000);
      return false;
    }

    if (finger.storeModel(id) == FINGERPRINT_OK) {
      setMasterState(LED_SUCCESS);
      String msg = "FP ID:" + String(id);
      lcdPrint("FP Saved OK!", msg.c_str());
      smartDelay(1200);
      return true;
    }
    setMasterState(LED_ERROR);
    lcdPrint("Store failed", "");
    smartDelay(1200);

  next_enroll_try:;
  }
  return false;
}

#define FP_MIN_CONFIDENCE 60

bool scanFinger(int expectedId, int tries) {
  for (int t = 1; t <= tries; t++) {
    String hdr = "FP try " + String(t) + "/" + String(tries);
    setMasterState(LED_SCANNING);
    lcdPrint(hdr.c_str(), "Place finger...");
    unsigned long ts = millis();

    bool gotImage = false;
    while (millis() - ts < 10000UL) {
      if (readKeypad() == '*')
        return false;

      uint8_t imgStatus = finger.getImage();

      if (imgStatus == FINGERPRINT_PACKETRECIEVEERR) {
        lcdPrint("Sensor Lost!", "Reconnect it");
        while (!finger.verifyPassword()) {
          if (readKeypad() == '*')
            return false;
          smartDelay(1000);
        }
        lcdPrint("Sensor Back!", "Continuing...");
        smartDelay(1000);
        lcdPrint(hdr.c_str(), "Place finger...");
        ts = millis();
        continue;
      }

      if (imgStatus == FINGERPRINT_OK) {
        gotImage = true;
        break;
      }
      smartDelay(50);
    }

    if (!gotImage) {
      setMasterState(LED_ERROR);
      lcdPrint("No Finger!", "Time is up");
      smartDelay(800);
      continue;
    }

    setMasterState(LED_PROCESSING);
    if (finger.image2Tz() != FINGERPRINT_OK) {
      setMasterState(LED_ERROR);
      lcdPrint("Bad FP image", "");
      smartDelay(800);
      continue;
    }
    if (finger.fingerFastSearch() != FINGERPRINT_OK) {
      setMasterState(LED_ERROR);
      lcdPrint("FP no match", "");
      smartDelay(900);
      continue;
    }

    int matchedId = finger.fingerID;
    int confidence = finger.confidence;
    bool idOk = (matchedId == expectedId);
    bool confOk = (confidence >= FP_MIN_CONFIDENCE);

    String dbg = "ID:" + String(matchedId) + " C:" + String(confidence);
    if (idOk && confOk) {
      setMasterState(LED_SUCCESS);
      lcdPrint("FP Match!", dbg.c_str());
      smartDelay(800);
      return true;
    }

    setMasterState(LED_ERROR);
    // Developer info: shows matched ID and confidence score
    lcdPrint("FP Denied", dbg.c_str());

    if (!idOk) {
      lcdPrint(
          "Wrong finger!",
          ("Exp:" + String(expectedId) + " Got:" + String(matchedId)).c_str());
      smartDelay(2000);
      return false;
    }

    lcdPrint("Weak match",
             ("Conf:" + String(confidence) + "<" + String(FP_MIN_CONFIDENCE))
                 .c_str());
    smartDelay(1200);
  }
  return false;
}

bool extractAndSendTemplate(int id) {
  memset(templateBuffer, 0, sizeof(templateBuffer));
  while (Serial.available())
    Serial.read();

  if (finger.loadModel(id) != FINGERPRINT_OK) {
    lcdPrint("Load failed", "");
    smartDelay(1000);
    return false;
  }

  uint8_t upChar[] = {0xEF, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x01,
                      0x00, 0x04, 0x08, 0x01, 0x00, 0x0E};
  Serial.write(upChar, 13);

  long t = millis();
  while (Serial.available() < 12 && millis() - t < 1000)
    ;
  for (int i = 0; i < 12; i++)
    Serial.read();

  int idx = 0;
  for (int p = 0; p < 4; p++) {
    unsigned long chunkStart = millis();
    while (Serial.available() < 9) {
      if (millis() - chunkStart > 2000) {
        lcdPrint("Extract timeout", "");
        smartDelay(1000);
        return false;
      }
    }
    for (int i = 0; i < 9; i++)
      Serial.read();

    for (int i = 0; i < 128; i++) {
      chunkStart = millis();
      while (!Serial.available()) {
        if (millis() - chunkStart > 5000) {
          lcdPrint("Data timeout", "");
          smartDelay(1000);
          return false;
        }
      }
      templateBuffer[idx++] = Serial.read();
    }

    chunkStart = millis();
    while (Serial.available() < 2) {
      if (millis() - chunkStart > 5000) {
        lcdPrint("Checksum timeout", "");
        smartDelay(1000);
        return false;
      }
    }
    Serial.read();
    Serial.read();
    lcdPrint("Extract", String("Chunk " + String(p + 1) + "/4").c_str());
  }

  for (int chunk = 0; chunk < 4; chunk++) {
    String hex = "";
    hex.reserve(260);
    for (int i = 0; i < 128; i++) {
      uint8_t b = templateBuffer[chunk * 128 + i];
      if (b < 0x10)
        hex += "0";
      hex += String(b, HEX);
    }
    hex.toUpperCase();
    lcdPrint("Sending to R3",
             String("Chunk " + String(chunk + 1) + "/4").c_str());

    String result = "";
    Bridge.call("send_template_chunk", hex.c_str()).result(result);
    if (result != "OK") {
      lcdPrint("Send failed", "");
      smartDelay(1000);
      Bridge.call("abort_template_transfer");
      return false;
    }
  }

  lcdPrint("Transfer OK", "Template sent");
  smartDelay(800);
  return true;
}

bool awaitResult(const char *ok, const char *fail, unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    if (readKeypad() == '*') {
      Bridge.call("cancel_operation");
      lcdPrint("Cancelled", "");
      smartDelay(1500);
      return false;
    }
    String res = "";
    Bridge.call("get_result").result(res);
    if (res == ok)
      return true;
    if (res == fail) {
      mirrorPythonLcd();
      smartDelay(2500);
      return false;
    }
    mirrorPythonLcd();
    smartDelay(600);
  }
  Bridge.call("cancel_operation");
  lcdPrint("Timed out", "");
  smartDelay(1500);
  return false;
}

void mirrorPythonLcd() {
  String l1 = "", l2 = "";
  Bridge.call("get_lcd1").result(l1);
  Bridge.call("get_lcd2").result(l2);
  if (l1.length() > 0)
    lcdPrint(l1.c_str(), l2.c_str());
}

String getIdFromKeypad(const char *title, const char *prefix, bool checkR3) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(title);
  lcd.setCursor(0, 1);
  String p = String(prefix);
  lcd.print(p + " _");
  String id = "";
  unsigned long lastR3Check = millis();

  while (true) {
    if (checkR3 && millis() - lastR3Check > 2000UL) {
      lastR3Check = millis();
      String r3Status = "";
      Bridge.call("get_r3_status").result(r3Status);
      if (r3Status == "OFFLINE" || r3Status == "ERROR") {
        if (!waitForR3Reconnect()) {
          return "";
        }
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(title);
        lcd.setCursor(0, 1);
        String disp = p + " " + id + "_";
        lcd.print(disp.c_str());
      }
    }

    char k = readKeypad();
    if (!k) {
      smartDelay(20);
      continue;
    }
    if (k >= '0' && k <= '9' && id.length() < 10)
      id += k;
    else if (k == '#') {
      if (id.length() > 0)
        id.remove(id.length() - 1);
    } else if (k == 'A' || k == 'B') {
      if (id.length() > 0)
        return id;
    } else if (k == '*')
      return "";
    lcd.setCursor(0, 1);
    lcd.print("                ");
    lcd.setCursor(0, 1);
    String disp = p + " " + id + "_";
    lcd.print(disp.c_str());
  }
}

void lcdPrint(const char *l1, const char *l2) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(l1);
  lcd.setCursor(0, 1);
  lcd.print(l2);
}

void showMainMenu() { 
  setMasterState(LED_IDLE);
  lcdPrint("A:Enrol B:Ballot", "C:Count (*Exit)"); 
}

char readKeypad() {
  for (int r = 0; r < 4; r++) {
    digitalWrite(rowPins[r], LOW);
    for (int c = 0; c < 4; c++) {
      if (digitalRead(colPins[c]) == LOW) {
        smartDelay(50);
        while (digitalRead(colPins[c]) == LOW)
          ;
        digitalWrite(rowPins[r], HIGH);
        return keys[r][c];
      }
    }
    digitalWrite(rowPins[r], HIGH);
  }
  return 0;
}

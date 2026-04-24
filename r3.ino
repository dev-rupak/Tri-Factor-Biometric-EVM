#include <Adafruit_Fingerprint.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
#include <ctype.h>

// ── LED state enum — must be before any function that uses BallotLedState ──
enum BallotLedState {
  BLED_LOCKED,      // Powered, idle: all 3 solid ON
  BLED_READY,       // Ballot mode connected: one-by-one chase
  BLED_SELECTED_A,  // Party A chosen: A fast-blink, others off
  BLED_SELECTED_B,  // Party B chosen
  BLED_SELECTED_C,  // Party C chosen
  BLED_SEAL_A,      // Sealed A: solid A only, scan finger
  BLED_SEAL_B,
  BLED_SEAL_C,
  BLED_FP_SYNC,     // Fingerprint scanning: fill-bar A→AB→ABC
  BLED_SUCCESS      // Vote cast: fanfare then all solid
};
BallotLedState bLedState = BLED_LOCKED;

#define ADDR_VOTES_A 0
#define ADDR_VOTES_B 4
#define ADDR_VOTES_C 8

SoftwareSerial fingerSerial(3, 2);
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&fingerSerial);
LiquidCrystal_I2C lcd(0x27, 16, 2);
uint8_t templateBuffer[512];
int byteIdx = 0;
int pendingTargetID = -1;

#define ADDR_VOTED_BITMAP 100 // Starting addr up to 128 bytes

bool hasVotedR3(int id) {
  if (id <= 0 || id > 1000)
    return false;
  int byteOffset = id / 8;
  int bitOffset = id % 8;
  uint8_t val = EEPROM.read(ADDR_VOTED_BITMAP + byteOffset);
  return (val & (1 << bitOffset)) != 0;
}

void markVotedR3(int id) {
  if (id <= 0 || id > 1000)
    return;
  int byteOffset = id / 8;
  int bitOffset = id % 8;
  uint8_t val = EEPROM.read(ADDR_VOTED_BITMAP + byteOffset);
  val |= (1 << bitOffset);
  EEPROM.update(ADDR_VOTED_BITMAP + byteOffset, val);
}

void unmarkVotedR3(int id) {
  if (id <= 0 || id > 1000)
    return;
  int byteOffset = id / 8;
  int bitOffset = id % 8;
  uint8_t val = EEPROM.read(ADDR_VOTED_BITMAP + byteOffset);
  val &= ~(1 << bitOffset);
  EEPROM.update(ADDR_VOTED_BITMAP + byteOffset, val);
}

void clearVotesR3() {
  for (int i = 0; i < 128; i++) {
    EEPROM.update(ADDR_VOTED_BITMAP + i, 0);
  }
}

#define PIN_ADMIN 4
#define PIN_PARTY_A 8
#define PIN_PARTY_B 12
#define PIN_PARTY_C 13
#define PIN_BUZZER 6
#define PIN_DELETE 5

uint8_t hexToNibble(char c) {
  c = toupper(c);
  if (c >= '0' && c <= '9')
    return c - '0';
  if (c >= 'A' && c <= 'F')
    return c - 'A' + 10;
  return 0;
}

uint8_t hexToByte(char h, char l) {
  return (hexToNibble(h) << 4) | hexToNibble(l);
}


#define LED_PARTY_A 9
#define LED_PARTY_B 10
#define LED_PARTY_C 11

// (enum BallotLedState defined at top of file)

void setBallotState(BallotLedState state) {
  bLedState = state;
}

void updateBallotLEDs() {
  unsigned long ms = millis();

  if (bLedState == BLED_LOCKED) {
    // All 3 solid ON — unit is powered and ready, clearly visible
    digitalWrite(LED_PARTY_A, HIGH);
    digitalWrite(LED_PARTY_B, HIGH);
    digitalWrite(LED_PARTY_C, HIGH);

  } else if (bLedState == BLED_READY) {
    // One-by-one chase A→B→C→A like a loading bar — ballot mode connected
    // Each step 400ms, full cycle = 1200ms
    unsigned long step = (ms % 1200) / 400;  // 0, 1, 2
    digitalWrite(LED_PARTY_A, (step == 0) ? HIGH : LOW);
    digitalWrite(LED_PARTY_B, (step == 1) ? HIGH : LOW);
    digitalWrite(LED_PARTY_C, (step == 2) ? HIGH : LOW);

  } else if (bLedState == BLED_SELECTED_A || bLedState == BLED_SELECTED_B || bLedState == BLED_SELECTED_C) {
    // Selected party: fast blink 200ms — "you picked this one"
    // Others: off — unambiguous
    bool on = (ms / 200) % 2 == 0;
    digitalWrite(LED_PARTY_A, (bLedState == BLED_SELECTED_A && on) ? HIGH : LOW);
    digitalWrite(LED_PARTY_B, (bLedState == BLED_SELECTED_B && on) ? HIGH : LOW);
    digitalWrite(LED_PARTY_C, (bLedState == BLED_SELECTED_C && on) ? HIGH : LOW);

  } else if (bLedState == BLED_SEAL_A || bLedState == BLED_SEAL_B || bLedState == BLED_SEAL_C) {
    // Chosen party solid ON, others off — "locked in, scan finger"
    digitalWrite(LED_PARTY_A, (bLedState == BLED_SEAL_A) ? HIGH : LOW);
    digitalWrite(LED_PARTY_B, (bLedState == BLED_SEAL_B) ? HIGH : LOW);
    digitalWrite(LED_PARTY_C, (bLedState == BLED_SEAL_C) ? HIGH : LOW);

  } else if (bLedState == BLED_FP_SYNC) {
    // Fingerprint sync bar: fills A→AB→ABC over 3s, then resets
    // Looks like a progress bar — user sees scan progressing
    unsigned long phase = ms % 3000;
    bool a = true;                  // A always ON once scanning starts
    bool b = (phase > 1000);        // B lights up after 1s
    bool c = (phase > 2000);        // C lights up after 2s
    digitalWrite(LED_PARTY_A, a ? HIGH : LOW);
    digitalWrite(LED_PARTY_B, b ? HIGH : LOW);
    digitalWrite(LED_PARTY_C, c ? HIGH : LOW);

  } else if (bLedState == BLED_SUCCESS) {
    // Fanfare: A→B→C→all chase (150ms each), then all 3 solid
    unsigned long elapsed = ms % 2400;
    if (elapsed < 150) {
      digitalWrite(LED_PARTY_A, HIGH); digitalWrite(LED_PARTY_B, LOW);  digitalWrite(LED_PARTY_C, LOW);
    } else if (elapsed < 300) {
      digitalWrite(LED_PARTY_A, LOW);  digitalWrite(LED_PARTY_B, HIGH); digitalWrite(LED_PARTY_C, LOW);
    } else if (elapsed < 450) {
      digitalWrite(LED_PARTY_A, LOW);  digitalWrite(LED_PARTY_B, LOW);  digitalWrite(LED_PARTY_C, HIGH);
    } else {
      // Hold all 3 solid: vote permanently cast
      digitalWrite(LED_PARTY_A, HIGH); digitalWrite(LED_PARTY_B, HIGH); digitalWrite(LED_PARTY_C, HIGH);
    }
  }
}

void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    updateBallotLEDs();
  }
}

void beepShort() {
  digitalWrite(PIN_BUZZER, HIGH);
  smartDelay(100);
  digitalWrite(PIN_BUZZER, LOW);
}

void beepDouble() {
  beepShort();
  smartDelay(100);
  beepShort();
}

void beepLong() {
  digitalWrite(PIN_BUZZER, HIGH);
  smartDelay(600);
  digitalWrite(PIN_BUZZER, LOW);
}

// ── Forward declarations — must appear before setup() ────────────────────
void lockUI();
void showResultsStandalone();
void wipeR3Standalone();
void processWakeFinger(int targetID);
void processUnlockAndVote(int targetID);
bool scanTargetFinger(int targetID, int attemptsMax, int timeoutSec);
bool waitForFingerClear();

void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  fingerSerial.begin(57600);

  // Button inputs (active LOW with internal pull-up)
  pinMode(PIN_ADMIN,   INPUT_PULLUP);
  pinMode(PIN_PARTY_A, INPUT_PULLUP);
  pinMode(PIN_PARTY_B, INPUT_PULLUP);
  pinMode(PIN_PARTY_C, INPUT_PULLUP);
  pinMode(PIN_DELETE,  INPUT_PULLUP);

  // LED outputs (PWM-capable: 9, 10, 11)
  pinMode(LED_PARTY_A, OUTPUT);
  pinMode(LED_PARTY_B, OUTPUT);
  pinMode(LED_PARTY_C, OUTPUT);

  // Buzzer output
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);

  // Start in locked state
  setBallotState(BLED_LOCKED);

  if (EEPROM.read(510) != 0xAA) {
    unsigned long zero = 0UL;
    EEPROM.put(ADDR_VOTES_A, zero);
    EEPROM.put(ADDR_VOTES_B, zero);
    EEPROM.put(ADDR_VOTES_C, zero);
    for (int i = 0; i < 128; i++) {
      EEPROM.update(ADDR_VOTED_BITMAP + i, 0);
    }
    EEPROM.write(510, 0xAA);
  }

  lockUI();
}


unsigned long lastPingTime = 0;
bool uiIsLocked = false;
bool isConnectedUI = false;

void loop() {
  updateBallotLEDs();
  bool currentConn = (millis() - lastPingTime <= 6000);
  if (uiIsLocked && currentConn != isConnectedUI) {
    isConnectedUI = currentConn;
    lcd.setCursor(0, 1);
    if (isConnectedUI) {
      lcd.print(F("System Ready   "));
    } else {
      lcd.print(F("Not Connected  "));
    }
  }

  if (Serial.available() > 0) {
    char peakC = Serial.peek();
    if (peakC == 'D') {
      Serial.read(); // consume D
      unsigned long t = millis();
      while (Serial.available() == 0 && (millis() - t < 500))
        ;
      if (Serial.read() == ':') {
        for (int i = 0; i < 128; i++) {
          t = millis();
          while (Serial.available() < 2 && (millis() - t < 1000))
            ;
          if (Serial.available() >= 2) {
            char h = Serial.read();
            char l = Serial.read();
            if (byteIdx < 512) {
              templateBuffer[byteIdx++] = hexToByte(h, l);
            }
          }
        }
        Serial.println(F("OK"));
      }
    } else {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd == "PING") {
        lastPingTime = millis();
      } else if (cmd.startsWith("FINISH:")) {
        unsigned long expectedCrc = strtoul(cmd.substring(7).c_str(), NULL, 16);
        unsigned long actualCrc = 0;
        for (int i = 0; i < 512; i++) {
          actualCrc += templateBuffer[i];
        }
        if (byteIdx == 512 && actualCrc == expectedCrc) {
          injectTemplate(pendingTargetID);
          setBallotState(BLED_LOCKED);  // Transfer done, revert to powered-idle
        } else {
          byteIdx = 0;
          pendingTargetID = -1;
          memset(templateBuffer, 0, 512);
          setBallotState(BLED_LOCKED);  // Error: revert
          Serial.println(F("ERROR_CRC"));
        }
      } else if (cmd == "ABORT") {
        byteIdx = 0;
        pendingTargetID = -1;
        memset(templateBuffer, 0, 512);
        setBallotState(BLED_LOCKED);    // Aborted: revert to powered-idle
        Serial.println(F("OK"));
      } else if (cmd.startsWith("CHECK:")) {
        int checkID = cmd.substring(6).toInt();
        if (finger.loadModel(checkID) == FINGERPRINT_OK) {
          Serial.println(F("EXISTS"));
        } else {
          Serial.println(F("MISSING"));
        }
      } else if (cmd.startsWith("START:")) {
        uiIsLocked = false;
        pendingTargetID = cmd.substring(6).toInt();
        byteIdx = 0;
        memset(templateBuffer, 0, sizeof(templateBuffer));
        setBallotState(BLED_FP_SYNC);   // Fill-bar: shows transfer in progress
        lcd.clear();
        lcd.print(F("Setting Up     "));
        lcd.setCursor(0, 1);
        lcd.print(F("Please Wait... "));
        Serial.println(F("OK"));
      } else if (cmd.startsWith("WAKE:")) {
        int voteID = cmd.substring(5).toInt();
        if (hasVotedR3(voteID)) {
          lcd.clear();
          lcd.print(F("Already Voted"));
          smartDelay(2000);
          lockUI();
          Serial.println(F("FAIL"));
        } else {
          processWakeFinger(voteID);
        }
      } else if (cmd.startsWith("UNLOCK:")) {
        int voteID = cmd.substring(7).toInt();
        if (hasVotedR3(voteID)) {
          Serial.println(F("FAIL"));
        } else {
          processUnlockAndVote(voteID);
        }
      } else if (cmd == "WIPE_VOTES") {
        clearVotesR3();
        Serial.println(F("OK"));
      } else if (cmd.startsWith("CLEAR_VOTE:")) {
        int voteID = cmd.substring(11).toInt();
        unmarkVotedR3(voteID);
        Serial.println(F("OK"));
      } else if (cmd.startsWith("CHECK_VOTED:")) {
        int checkID = cmd.substring(12).toInt();
        Serial.println(hasVotedR3(checkID) ? F("VOTED") : F("NOT_VOTED"));
      } else if (cmd == "LOCK") {
        lockUI();
        Serial.println(F("OK"));
      } else if (cmd.startsWith("BEEP:")) {
        String type = cmd.substring(5);
        if (type == "SHORT") beepShort();
        else if (type == "DOUBLE") beepDouble();
        else if (type == "LONG") beepLong();
        Serial.println(F("OK"));
      }
    }
  }

  if (millis() - lastPingTime > 6000) {
    if (digitalRead(PIN_ADMIN) == LOW) {
      showResultsStandalone();
    }
    if (digitalRead(PIN_DELETE) == LOW) {
      wipeR3Standalone();
    }
  }
}

void injectTemplate(int id) {
  uint8_t downChar[] = {0xEF, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x01,
                        0x00, 0x04, 0x09, 0x01, 0x00, 0x0F};
  fingerSerial.write(downChar, 13);
  unsigned long ackTimer = millis();
  while (fingerSerial.available() < 12 && millis() - ackTimer < 500) {
    smartDelay(5);
  }
  while (fingerSerial.available()) {
    fingerSerial.read();
  }

  for (int p = 0; p < 4; p++) {
    uint8_t pid = (p == 3) ? 0x08 : 0x02;
    uint16_t sum = pid + 0x00 + 0x82;
    uint8_t head[] = {0xEF, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, pid, 0x00, 0x82};
    fingerSerial.write(head, 9);
    for (int i = 0; i < 128; i++) {
      uint8_t b = templateBuffer[p * 128 + i];
      fingerSerial.write(b);
      sum += b;
    }
    fingerSerial.write((uint8_t)(sum >> 8));
    fingerSerial.write((uint8_t)(sum & 0xFF));
  }
  if (finger.storeModel(id) == FINGERPRINT_OK) {
    byteIdx = 0;
    lcd.clear();
    lcd.print(F("Ready!         "));
    smartDelay(500);
    Serial.println(F("SAVED"));
  } else {
    byteIdx = 0;
    lcd.clear();
    lcd.print(F("Setup Error    "));
    smartDelay(500);
    Serial.println(F("ERROR_STORE"));
  }
}

bool waitForFingerClear() {
  while (fingerSerial.available() > 0)
    fingerSerial.read();
  lcd.clear();
  lcd.print(F("Lift Finger    "));
  while (true) {
    uint8_t st = finger.getImage();
    if (st == FINGERPRINT_NOFINGER || st == FINGERPRINT_PACKETRECIEVEERR)
      break;
    smartDelay(200);
  }
  return true;
}

bool scanTargetFinger(int targetID, int attemptsMax, int timeoutSec) {
  for (int attempt = 1; attempt <= attemptsMax; attempt++) {
    lcd.clear();
    lcd.print(F("Place Finger   "));
    lcd.setCursor(0, 1);
    lcd.print(F("Try "));
    lcd.print(attempt);
    lcd.print(F("/"));
    lcd.print(attemptsMax);
    setBallotState(BLED_FP_SYNC);   // Fill-bar: A→AB→ABC while scanning

    unsigned long tryStart = millis();   // fresh clock for THIS try
    bool gotImage = false;

    while (millis() - tryStart < ((unsigned long)timeoutSec * 1000)) {
      updateBallotLEDs();
      if (Serial.available() > 0) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd == "LOCK") return false;
      }

      uint8_t imgStatus = finger.getImage();

      if (imgStatus == FINGERPRINT_PACKETRECIEVEERR) {
        lcd.clear();
        lcd.print(F("Sensor Lost    "));
        lcd.setCursor(0, 1);
        lcd.print(F("Reconnect FP..."));
        unsigned long recStart = millis();
        bool recovered = false;
        while (millis() - recStart < 30000UL) {
          updateBallotLEDs();
          if (finger.verifyPassword() == FINGERPRINT_OK) {
            recovered = true;
            break;
          }
          smartDelay(1000);
        }
        if (!recovered) return false;
        lcd.clear();
        lcd.print(F("Sensor Back!   "));
        smartDelay(1000);
        tryStart = millis();   // reset THIS try's clock after recovery
        continue;
      }

      if (imgStatus == FINGERPRINT_OK) {
        gotImage = true;
        break;
      }
    }

    if (!gotImage) {
      lcd.clear();
      lcd.print(F("No Finger!     "));
      lcd.setCursor(0, 1);
      lcd.print(F("Time is up"));
      beepLong();
      smartDelay(1000);
      continue;   // next attempt with a fresh timer
    }

    finger.image2Tz();
    if (finger.fingerFastSearch() == FINGERPRINT_OK) {
      if (finger.fingerID == targetID) {
        return true;                    // match — done
      } else {
        lcd.clear();
        lcd.print(F("Identity Error "));
        beepLong();
        smartDelay(2000);
        waitForFingerClear();
        // continue to next attempt
      }
    } else {
      lcd.clear();
      lcd.print(F("No Match Found "));
      beepLong();
      smartDelay(1500);
      waitForFingerClear();
      // continue to next attempt
    }
  }
  return false;
}


void processWakeFinger(int targetID) {
  uiIsLocked = false;
  waitForFingerClear();
  lcd.clear();
  lcd.print(F("Locked         "));
  lcd.setCursor(0, 1);
  lcd.print(F("Scan to Start"));

  if (scanTargetFinger(targetID, 3, 8)) {
    beepDouble();
    lcd.clear();
    lcd.print(F("Finger Match OK"));
    lcd.setCursor(0, 1);
    lcd.print(F("Look at Camera"));
    Serial.println(F("AWAKE_OK"));

    unsigned long faceDeadline = millis() + 50000UL;
    unsigned long lastLcdUpdate = 0;
    unsigned long lastCountdownUpdate = 0;
    while (millis() < faceDeadline) {
      if (Serial.available() > 0) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd == "PING") {
          Serial.println(F("PONG"));
          lastPingTime = millis();
        } else if (cmd == "LOCK") {
          lockUI();
          Serial.println(F("OK"));
          return;
        } else if (cmd.startsWith("UNLOCK:")) {
          processUnlockAndVote(cmd.substring(7).toInt());
          return;
        } else if (cmd.startsWith("BEEP:")) {
          String type = cmd.substring(5);
          if (type == "SHORT") beepShort();
          else if (type == "DOUBLE") beepDouble();
          else if (type == "LONG") beepLong();
        } else if (cmd.startsWith("LCD:")) {
          int delim = cmd.indexOf('|');
          lcd.clear();
          if (delim != -1) {
            lcd.print(cmd.substring(4, delim));
            lcd.setCursor(0, 1);
            lcd.print(cmd.substring(delim + 1));
          } else {
            lcd.print(cmd.substring(4));
          }
          lastLcdUpdate = millis();
        }
      }
      
      unsigned long now = millis();
      if ((now - lastLcdUpdate > 2500) && (now - lastCountdownUpdate > 1000)) {
        int secLeft = (int)((faceDeadline - now) / 1000UL);
        lcd.setCursor(0, 1);
        lcd.print(F("Face Check: "));
        lcd.print(secLeft);
        lcd.print(F("s  "));
        lastCountdownUpdate = now;
      }
      smartDelay(50);
    }
  } else {
    lcd.clear();
    lcd.print(F("FP Denied      "));
    lcd.setCursor(0, 1);
    lcd.print(F("No match found "));
    beepLong();
    smartDelay(2500);
    lockUI();
    Serial.println(F("TIMEOUT"));
    return;
  }
}

void processUnlockAndVote(int targetID) {
  lcd.clear();
  lcd.print(F("Session Unlocked"));
  lcd.setCursor(0, 1);
  lcd.print(F("Press a Button"));


  String chosenParty = "";
  bool lockSelection = false;
  unsigned long voteTimer = 0;
  bool voteTimerRunning = false;
  int changesLeft = 1;

  unsigned long globalSessionTimer = millis();

  setBallotState(BLED_READY);

  while (true) {
    updateBallotLEDs();
    if (millis() - globalSessionTimer > 70000UL) {
      lcd.clear();
      lcd.print(F("Time is Up!    "));
      smartDelay(2000);
      lockUI();
      Serial.println(F("TIMEOUT"));
      return;
    }

    if (Serial.available() > 0) {
      String cmd = Serial.readStringUntil('\n');
      cmd.trim();
      if (cmd == "LOCK") {
        lcd.clear();
        lcd.print(F("Security Alert "));
        smartDelay(2000);
        lockUI();
        Serial.println(F("FAIL"));
        return;
      }
    }

    String currentPress = "";
    if (digitalRead(PIN_PARTY_A) == LOW)
      currentPress = "A";
    else if (digitalRead(PIN_PARTY_B) == LOW)
      currentPress = "B";
    else if (digitalRead(PIN_PARTY_C) == LOW)
      currentPress = "C";

    if (currentPress != "") {
      if (chosenParty == "") {
        // First selection
        beepShort();
        chosenParty = currentPress;
        if (chosenParty == "A") setBallotState(BLED_SELECTED_A);
        else if (chosenParty == "B") setBallotState(BLED_SELECTED_B);
        else if (chosenParty == "C") setBallotState(BLED_SELECTED_C);

        voteTimerRunning = true;
        voteTimer = millis();
        lcd.clear();
        lcd.print(F("Selected: "));
        lcd.print(chosenParty);
        lcd.setCursor(0, 1);
        lcd.print(F("5s to change..."));
        smartDelay(800);
      } else if (chosenParty != currentPress && changesLeft == 1) {
        // Vote changed — immediately lock in, no second countdown
        beepShort();
        chosenParty = currentPress;
        if (chosenParty == "A") setBallotState(BLED_SELECTED_A);
        else if (chosenParty == "B") setBallotState(BLED_SELECTED_B);
        else if (chosenParty == "C") setBallotState(BLED_SELECTED_C);

        changesLeft = 0;
        lockSelection = true;
        lcd.clear();
        lcd.print(F("CHANGED! Party "));
        lcd.print(chosenParty);
        lcd.setCursor(0, 1);
        lcd.print(F("Saving Now...  "));
        smartDelay(1500);
        break;
      }
    }

    if (voteTimerRunning) {
      unsigned long elapsed = millis() - voteTimer;
      int remaining = 5 - (int)(elapsed / 1000);
      if (remaining <= 0) {
        lockSelection = true;
        break;
      } else {
        lcd.clear();
        lcd.print(F("You Picked: "));
        lcd.print(chosenParty);
        lcd.setCursor(0, 1);
        lcd.print(remaining);
        lcd.print(F("s to Change..."));
        smartDelay(250); // Refresh rate
      }
    }
  }

  if (chosenParty == "") {
    lcd.clear();
    lcd.print(F("Pick a Party!  "));
    smartDelay(2500);
    lockUI();
    Serial.println(F("FAIL"));
    return;
  }

  // ── Show FINAL confirmation before seal ──
  lcd.clear();
  lcd.print(F("Choice: Party "));
  lcd.print(chosenParty);
  lcd.setCursor(0, 1);
  lcd.print(F("Saving Vote... "));
  smartDelay(2000);


  // --- STEP 6: SEAL VOTE ---
  waitForFingerClear();
  if (chosenParty == "A") setBallotState(BLED_SEAL_A);
  else if (chosenParty == "B") setBallotState(BLED_SEAL_B);
  else if (chosenParty == "C") setBallotState(BLED_SEAL_C);

  lcd.clear();
  lcd.print(F("Vote Ready!    "));
  lcd.setCursor(0, 1);
  lcd.print(F("Place Finger"));

  if (!scanTargetFinger(targetID, 3, 8)) {
    lcd.clear();
    lcd.print(F("Error: Retry   "));
    beepLong();
    smartDelay(2500);
    lockUI();
    Serial.println(F("FAIL"));
    return;
  }

  beepDouble();
  lcd.clear();
  lcd.print(F("Vote OK!"));
  setBallotState(BLED_SUCCESS);
  lcd.setCursor(0, 1);
  lcd.print(F("Locked         "));

  // Flash Buzzer
  digitalWrite(PIN_BUZZER, HIGH);
  smartDelay(1500);
  digitalWrite(PIN_BUZZER, LOW);

  // Save vote to local R3 EEPROM
  int addr = -1;
  if (chosenParty == "A")
    addr = ADDR_VOTES_A;
  else if (chosenParty == "B")
    addr = ADDR_VOTES_B;
  else if (chosenParty == "C")
    addr = ADDR_VOTES_C;
  if (addr != -1) {
    unsigned long count = 0UL;
    EEPROM.get(addr, count);
    if (count > 4000000000UL)
      count = 0UL;
    count++;
    EEPROM.put(addr, count);
  }

  // Mark local tracking
  markVotedR3(targetID);

  // Erase Template (Self-Clean session - exclude permanent Admin template!)
  if (targetID != 1) {
    finger.deleteModel(targetID);
  }

  Serial.println(F("CAST:SUCCESS"));
  smartDelay(3000);
  lockUI();
}

void lockUI() {
  setBallotState(BLED_LOCKED);
  uiIsLocked = true;
  isConnectedUI = (millis() - lastPingTime <= 6000);
  lcd.clear();
  lcd.print(F("Ballot Unit"));
  lcd.setCursor(0, 1);
  if (isConnectedUI) {
    lcd.print(F("Unit Connected "));
  } else {
    lcd.print(F("Not Connected  "));
  }
}

void showResultsStandalone() {
  uiIsLocked = false;
  waitForFingerClear();
  lcd.clear();
  lcd.print(F("Officer Auth   "));
  lcd.setCursor(0, 1);
  lcd.print(F("Scan ID 1 Fin"));
  if (scanTargetFinger(1, 3, 10)) {
    unsigned long cA = 0, cB = 0, cC = 0;
    EEPROM.get(ADDR_VOTES_A, cA);
    EEPROM.get(ADDR_VOTES_B, cB);
    EEPROM.get(ADDR_VOTES_C, cC);
    if (cA > 4000000000UL)
      cA = 0;
    if (cB > 4000000000UL)
      cB = 0;
    if (cC > 4000000000UL)
      cC = 0;
    lcd.clear();
    lcd.print(F("A:"));
    lcd.print(cA);
    lcd.print(F(" B:"));
    lcd.print(cB);
    lcd.setCursor(0, 1);
    lcd.print(F("C:"));
    lcd.print(cC);
    smartDelay(6000);
  } else {
    lcd.clear();
    lcd.print(F("Access Denied  "));
    lcd.setCursor(0, 1);
    lcd.print(F("Wrong finger   "));
    smartDelay(2000);
  }
  lockUI();
}

void wipeR3Standalone() {
  uiIsLocked = false;
  waitForFingerClear();
  lcd.clear();
  lcd.print(F("Officer Auth   "));
  lcd.setCursor(0, 1);
  lcd.print(F("Scan ID 1 Finger"));
  if (scanTargetFinger(1, 3, 10)) {
    lcd.clear();
    lcd.print(F("Wiping R3 db..."));
    unsigned long zero = 0UL;
    EEPROM.put(ADDR_VOTES_A, zero);
    EEPROM.put(ADDR_VOTES_B, zero);
    EEPROM.put(ADDR_VOTES_C, zero);
    clearVotesR3();
    for (int i = 2; i <= 1000; i++)
      finger.deleteModel(i);
    lcd.clear();
    lcd.print(F("Memory Wiped!"));
    smartDelay(2000);
  } else {
    lcd.clear();
    lcd.print(F("Access Denied  "));
    lcd.setCursor(0, 1);
    lcd.print(F("Wipe cancelled "));
    smartDelay(2000);
  }
  lockUI();
}
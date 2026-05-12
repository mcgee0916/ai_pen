/*
 * PAA.ino 
 * Serial Monitor 會顯示：
 *   [S1] ... → Serial3 收到的資料
 *   AT 指令送出後若 PAA 回 "OK" 代表接線正確
 */

#define PAA_BAUD 115200

// ── UART 行緩衝 ───────────────────────────────────────────
char buf1[128]; int len1 = 0;
int  s1_count = 0;
bool done = false;
unsigned long lastDataTime = 0;

void sendAT(const char *cmd) {
    Serial.print("[AT] send: "); 
    Serial.println(cmd);
    Serial3.println(cmd);
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("=== PAA serial test start ===");

    Serial3.begin(PAA_BAUD);
    delay(500);

    // ── AT 指令序列 ──────────────────────────────────────
    sendAT("at0g+");      delay(400);
    sendAT("atrpt+50");  delay(200);
    sendAT("atx+");       delay(200);

    Serial.println("[READY] waiting for PAA data...");
    lastDataTime = millis();
}

void sendATSequence() {
    sendAT("at0g+");     delay(400);
    sendAT("atrpt+50"); delay(200);
    sendAT("atx+");      delay(200);
}

void loop() {
    if (done) return;

    // ── 讀 Serial3 ───────────────────────────────────────
    while (Serial3.available() && !done) {
        char c = (char)Serial3.read();
        if (c == '\n' || c == '\r') {
            if (len1 > 0) {
                buf1[len1] = '\0';
                s1_count++;
                Serial.print("[S1] "); Serial.println(buf1);
                len1 = 0;
                lastDataTime = millis();
                if (s1_count >= 3) { Serial.println("[S1] TEST OK"); done = true; }
            }
        } else if (len1 < (int)sizeof(buf1) - 1) {
            buf1[len1++] = c;
            lastDataTime = millis();
        }
    }

    // ── 5 秒無資料則重送 AT 指令序列 ─────────────────────
    if (millis() - lastDataTime >= 5000) {
        Serial.println("[RETRY] no data for 5s, resending AT sequence...");
        sendATSequence();
        lastDataTime = millis();
    }
}

/*
 * BTN.ino — 按鈕測試（已驗證 OK）
 *
 * 接線：
 *   BTN → pin 15 (INPUT_PULLUP，另一端接 GND)
 *
 * Serial Monitor 輸出：
 *   [BTN] LONG_START  → 長按開始（≥400ms）
 *   [BTN] LONG_END    → 長按放開
 *   [BTN] DOUBLE      → 快速連按兩下
 */

#define BTN             15
#define LONG_PRESS_MS   400
#define DOUBLE_CLICK_MS 350

// ── 按鈕狀態機 ────────────────────────────────────────────
enum BtnEvt { BTN_NONE, BTN_LONG_START, BTN_LONG_END, BTN_DOUBLE };
static bool     _prev      = HIGH;
static bool     _in_long   = false;
static bool     _wait2     = false;
static uint32_t _press_t   = 0;
static uint32_t _release_t = 0;

BtnEvt pollButton() {
    bool     cur = digitalRead(BTN);
    uint32_t now = millis();
    BtnEvt   evt = BTN_NONE;

    if (cur == LOW && _prev == HIGH) {
        if (_wait2 && (now - _release_t < DOUBLE_CLICK_MS)) {
            _wait2 = false; _prev = cur; return BTN_DOUBLE;
        }
        _press_t = now;
    } else if (cur == HIGH && _prev == LOW) {
        if (_in_long) { _in_long = false; evt = BTN_LONG_END; }
        else if (now - _press_t < LONG_PRESS_MS) { _release_t = now; _wait2 = true; }
    } else if (cur == LOW) {
        if (!_in_long && (now - _press_t >= LONG_PRESS_MS)) { _in_long = true; evt = BTN_LONG_START; }
    } else {
        if (_wait2 && (now - _release_t >= DOUBLE_CLICK_MS)) _wait2 = false;
    }
    _prev = cur;
    return evt;
}

// ── 測試計數 ──────────────────────────────────────────────
int long_count   = 0;
int double_count = 0;

void setup() {
    Serial.begin(115200);
    pinMode(BTN, INPUT_PULLUP);
    delay(500);
    Serial.println("=== BTN test start ===");
    Serial.println("long press: hold >=400ms then release");
    Serial.println("double click: press twice quickly");
}

void loop() {
    switch (pollButton()) {
        case BTN_LONG_START:
            Serial.println("[BTN] LONG_START");
            break;
        case BTN_LONG_END:
            long_count++;
            Serial.print("[BTN] LONG_END  (total="); Serial.print(long_count); Serial.println(")");
            if (long_count >= 2 && double_count >= 2) {
                Serial.println("[BTN] TEST OK");
            }
            break;
        case BTN_DOUBLE:
            double_count++;
            Serial.print("[BTN] DOUBLE  (total="); Serial.print(double_count); Serial.println(")");
            if (long_count >= 2 && double_count >= 2) {
                Serial.println("[BTN] TEST OK");
            }
            break;
        default: break;
    }
}

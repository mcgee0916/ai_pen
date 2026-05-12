/*
 * ALL.ino — 飛機射擊小遊戲
 * 同時驗證 OLED + PAA5163D + 按鈕 三項功能
 *
 * 接線：（不可呼叫 Wire.begin()，否則 Pin 0 衝突）
 *   OLED SDA/SCL → 預設 I2C
 *   BTN          → pin 15 (INPUT_PULLUP，另一端接 GND)
 *   PAA TX       → Serial3 RX
 *   PAA RX       → Serial3 TX
 *
 * 啟動：PAA 驗證（跑馬燈）→ 進遊戲
 *
 * 玩法：
 *   PAA 移動   → 飛機在畫面左側自由移動（X/Y 皆可）
 *   按鈕單按   → 發射飛彈（最多同時 3 顆）
 *   小魚從右往左游：
 *     飛彈打中 → 加分
 *     撞到飛機 → 扣一條命（共 3 命）
 *   3 命歸零 → GAME OVER，再按按鈕重新開始
 */

#include <Wire.h>
#include <Adafruit_OLED_libraries/Adafruit_GFX.h>
#include <Adafruit_OLED_libraries/Adafruit_SSD1306.h>

// ── OLED ──────────────────────────────────────────────────
#define SCREEN_W   128
#define SCREEN_H    32
#define OLED_RESET  -1
#define OLED_ADDR  0x3C
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, OLED_RESET);

// ── PAA ───────────────────────────────────────────────────
#define PAA_SERIAL   Serial3
#define PAA_BAUD     115200
#define PAA_SCALE    4      // 靈敏度：值越小越靈敏

// ── 按鈕 ──────────────────────────────────────────────────
#define BTN          15

// ── 飛機活動範圍 ──────────────────────────────────────────
// 右側留給小魚，飛機活動在左側 ~60px
#define PLANE_X_MIN   5
#define PLANE_X_MAX  55
#define PLANE_Y_MIN   4
#define PLANE_Y_MAX  27

// PAA 座標映射（ATX+ 輸出絕對 mm 座標）
// 飛機中心 = PAA 零點，移動 PAA_RANGE_MM mm 時抵達邊界
#define PLANE_CX       30    // 畫面中心 X（對應 PAA X=0）
#define PLANE_CY       16    // 畫面中心 Y（對應 PAA Y=0）
#define PAA_RANGE_MM   25.0f // 物理移動幾 mm 達到邊界（調整靈敏度）

// ── 遊戲參數 ──────────────────────────────────────────────
#define MAX_FISH      4
#define MAX_MISSILES  3
#define FISH_SPEED    2     // px/frame
#define MISSILE_SPEED 6     // px/frame
#define SPAWN_MS    1000
#define FRAME_MS      40    // 25 fps
#define MAX_LIVES      3
#define PAA_WATCHDOG  3000  // ms 無資料則重初始化

// ── 遊戲物件 ──────────────────────────────────────────────
struct Fish    { int x, y; bool alive; };
struct Missile { int x, y; bool alive; };

Fish     fish[MAX_FISH];
Missile  missiles[MAX_MISSILES];

int      planeX   = PLANE_CX;   // 由 PAA 絕對座標映射更新
int      planeY   = PLANE_CY;
int      lives    = MAX_LIVES;
int      score    = 0;
bool     gameOver = false;

uint32_t lastSpawn    = 0;
uint32_t lastFrame    = 0;
uint32_t paaLastData  = 0;   // 最後一次收到 PAA 資料的時間
bool     btnPrev      = HIGH;

// ── PAA 行緩衝 ────────────────────────────────────────────
char paaBuf[128];
int  paaLen = 0;

// ─────────────────────────────────────────────────────────
// PAA 初始化（參考 ai_pan_collect.ino 的 initPAA）
// ─────────────────────────────────────────────────────────

// 等待 PAA 回應 "OK"（case-insensitive）
static bool waitForPAAOK(uint32_t timeout_ms) {
    char rbuf[64]; int rlen = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < timeout_ms) {
        if (!PAA_SERIAL.available()) continue;
        char c = (char)PAA_SERIAL.read();
        if (c == '\n' || c == '\r') {
            if (rlen > 0) {
                rbuf[rlen] = '\0'; rlen = 0;
                for (int i = 0; rbuf[i] && rbuf[i+1]; i++) {
                    char a = (rbuf[i]   >= 'a') ? rbuf[i]   - 32 : rbuf[i];
                    char b = (rbuf[i+1] >= 'a') ? rbuf[i+1] - 32 : rbuf[i+1];
                    if (a == 'O' && b == 'K') return true;
                }
            }
        } else if (rlen < (int)sizeof(rbuf) - 1) {
            rbuf[rlen++] = c;
        }
    }
    return false;
}

bool paaInit() {
    while (PAA_SERIAL.available()) PAA_SERIAL.read();   // 清殘留
    PAA_SERIAL.println("at");                           // 喚醒，等 OK
    if (!waitForPAAOK(1000)) {
        Serial.println("[PAA] no OK");
        return false;
    }
    PAA_SERIAL.println("at0g+");    // 重置感測器並設定零點
    waitForPAAOK(500);              // 等待 "ok"（at0g+ 約需 200ms）
    PAA_SERIAL.println("atrpt+50"); delay(200);   // 50ms（手冊最小值）
    PAA_SERIAL.println("atx+");     delay(200);   // 啟動座標模式：輸出 X= Y=（絕對 mm）
    paaLastData = millis();
    Serial.println("[PAA] init OK, zero set");
    return true;
}

// ─────────────────────────────────────────────────────────
// PAA 驗證階段：跑馬燈 + 自動重試
// ─────────────────────────────────────────────────────────
void runPAACheck() {
    paaLen = 0;
    int confirmed = 0;
    int retries   = 0;

    const char* txt = "  HUB-PAA5163D  Checking...  ";
    int  txtPx      = strlen(txt) * 6;
    int  scrollX    = SCREEN_W;
    uint32_t lastScr  = millis();
    uint32_t lastData = millis();

    // 第一次嘗試初始化
    while (!paaInit()) {
        retries++;
        Serial.print("[PAA] init retry #"); Serial.println(retries);
        delay(500);
    }

    Serial.println("[PAA] checking data...");

    while (confirmed < 3) {

        // 跑馬燈（每 35ms 移 1px）
        if (millis() - lastScr >= 35) {
            lastScr = millis();
            if (--scrollX < -txtPx) scrollX = SCREEN_W;

            display.clearDisplay();
            display.setTextSize(1);
            display.setTextColor(WHITE);
            display.setCursor(scrollX, 4);
            display.print(txt);
            display.setCursor(24, 20);
            display.print("PAA: "); display.print(confirmed); display.print(" / 3");
            if (retries > 0) {
                display.setCursor(84, 20);
                display.print("R:"); display.print(retries);
            }
            display.display();
        }

        // 讀 PAA — 同時解析 X= 與 Y=（同 ai_pan_collect processLine）
        while (PAA_SERIAL.available()) {
            char c = (char)PAA_SERIAL.read();
            if (c == '\n' || c == '\r') {
                if (paaLen > 0) {
                    paaBuf[paaLen] = '\0'; paaLen = 0;
                    Serial.print("[PAA] raw: "); Serial.println(paaBuf);
                    if (strstr(paaBuf, "X=")) {
                        confirmed++;
                        lastData = millis();
                        Serial.print("[PAA] "); Serial.print(confirmed); Serial.println("/3");
                    }
                }
            } else if (paaLen < (int)sizeof(paaBuf) - 1) {
                paaBuf[paaLen++] = c;
                lastData = millis();
            }
        }

        // 5 秒無資料 → 重試
        if (millis() - lastData >= 5000) {
            retries++;
            Serial.print("[PAA] timeout, retry #"); Serial.println(retries);
            paaInit();
            lastData = millis();
            paaLen   = 0;
        }
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(28,  4); display.print("PAA OK!");
    display.setCursor(10, 20); display.print("Game starting...");
    display.display();
    Serial.println("[PAA] OK -> game start");
    delay(1200);
}

// ─────────────────────────────────────────────────────────
// 繪圖
// ─────────────────────────────────────────────────────────

// 小飛機（面朝右，中心 cx,cy，約 11×8px）
void drawPlane(int cx, int cy) {
    display.fillRect(cx-4, cy-1, 10, 3, WHITE);          // 機身
    display.fillRect(cx-2, cy-3,  5, 7, WHITE);          // 主翼
    display.drawLine(cx-4, cy-3, cx-4, cy-2, WHITE);     // 尾翼上
    display.drawLine(cx-4, cy+2, cx-4, cy+3, WHITE);     // 尾翼下
    display.drawPixel(cx+6, cy, WHITE);                  // 機頭尖端
    display.drawPixel(cx+3, cy-1, BLACK);                // 座艙
    display.drawPixel(cx+3, cy,   BLACK);
}

// 小魚（面朝左，左上角 x,y，約 8×4px）
void drawFish(int x, int y) {
    display.fillRect(x+1, y+1, 5, 2, WHITE);
    display.drawPixel(x,   y+1, WHITE);
    display.drawPixel(x,   y+2, WHITE);
    display.drawPixel(x+7, y,   WHITE);
    display.drawPixel(x+7, y+3, WHITE);
    display.drawPixel(x+6, y+1, WHITE);
    display.drawPixel(x+6, y+2, WHITE);
}

// 飛彈（4×2px）
void drawMissile(int x, int y) {
    display.fillRect(x, y-1, 4, 2, WHITE);
}

// 命數（左下角，小飛機圖示）
void drawLives(int n) {
    for (int i = 0; i < n; i++) {
        int ox = i * 10 + 1;
        display.fillRect(ox+1, 27, 5, 2, WHITE);   // 小機身
        display.fillRect(ox+2, 26, 3, 4, WHITE);   // 小機翼
    }
}

// ─────────────────────────────────────────────────────────
// PAA 輪詢 — ATX+ 輸出格式：A=.. X=<mm> Y=<mm> cA=..
//
// X= / Y= 是從 at0g+ 零點起算的「絕對累積座標」（單位 mm，浮點數）
// 不是每包的 delta！直接用 atof() 讀取並映射到畫面。
// ─────────────────────────────────────────────────────────
void pollPAA() {
    while (PAA_SERIAL.available()) {
        char c = (char)PAA_SERIAL.read();
        paaLastData = millis();

        if (c == '\n' || c == '\r') {
            if (paaLen > 0) {
                paaBuf[paaLen] = '\0'; paaLen = 0;

                char *xp = strstr(paaBuf, "X=");
                if (xp) {
                    float absX = atof(xp + 2);                          // mm，絕對值

                    char *yp = strstr(paaBuf, "Y=");
                    float absY = yp ? atof(yp + 2) : 0.0f;              // mm，絕對值

                    // 線性映射：零點 → 畫面中心；移動 PAA_RANGE_MM mm → 抵達邊界
                    float scaleX = (float)(PLANE_X_MAX - PLANE_X_MIN) / 2.0f / PAA_RANGE_MM;
                    float scaleY = (float)(PLANE_Y_MAX - PLANE_Y_MIN) / 2.0f / PAA_RANGE_MM;

                    planeX = PLANE_CX + (int)(absX * scaleX);
                    planeY = PLANE_CY - (int)(absY * scaleY);   // Y 反轉：往前 = 往上

                    planeX = constrain(planeX, PLANE_X_MIN, PLANE_X_MAX);
                    planeY = constrain(planeY, PLANE_Y_MIN, PLANE_Y_MAX);
                }
            }
        } else if (paaLen < (int)sizeof(paaBuf) - 1) {
            paaBuf[paaLen++] = c;
        }
    }
}

// ─────────────────────────────────────────────────────────

void resetGame() {
    planeX   = PLANE_CX;
    planeY   = PLANE_CY;
    lives    = MAX_LIVES;
    score    = 0;
    gameOver = false;
    for (int i = 0; i < MAX_FISH;     i++) fish[i].alive     = false;
    for (int i = 0; i < MAX_MISSILES; i++) missiles[i].alive = false;
    lastSpawn = millis();
    lastFrame = millis();
}

// ── setup ─────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    pinMode(BTN, INPUT_PULLUP);
    delay(300);

    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        Serial.println("[OLED] FAIL");
        for (;;);
    }
    Serial.println("[OLED] OK");

    PAA_SERIAL.begin(PAA_BAUD);
    delay(500);
    runPAACheck();

    resetGame();
    Serial.println("=== GAME START ===");
}

// ── loop ──────────────────────────────────────────────────
void loop() {
    uint32_t now = millis();

    // PAA 隨時讀，不節流
    pollPAA();

    // PAA watchdog：3 秒無資料自動重初始化（同 ai_pan_collect）
    if (now - paaLastData >= PAA_WATCHDOG) {
        Serial.println("[PAA] watchdog: no data, reinit...");
        paaLen = 0;
        if (paaInit()) {
            Serial.println("[PAA] reinit OK");
        } else {
            Serial.println("[PAA] reinit fail");
            paaLastData = now;   // 避免連續觸發
        }
    }

    // 按鈕偵測
    bool btnNow = digitalRead(BTN);
    if (btnNow == LOW && btnPrev == HIGH) {
        if (gameOver) {
            resetGame();
        } else {
            // 發射飛彈
            for (int i = 0; i < MAX_MISSILES; i++) {
                if (!missiles[i].alive) {
                    missiles[i].x     = planeX + 7;   // 從機頭射出
                    missiles[i].y     = planeY;
                    missiles[i].alive = true;
                    Serial.print("[FIRE] x="); Serial.print(planeX);
                    Serial.print(" y="); Serial.println(planeY);
                    break;
                }
            }
        }
    }
    btnPrev = btnNow;

    // GAME OVER 畫面
    if (gameOver) {
        if (now - lastFrame >= 40) {
            lastFrame = now;
            display.clearDisplay();
            display.setTextSize(1);
            display.setTextColor(WHITE);
            display.setCursor(22,  4); display.print("GAME  OVER");
            display.setCursor(10, 18); display.print("Score:"); display.print(score);
            display.setCursor(70, 18); display.print("BTN:retry");
            display.display();
        }
        return;
    }

    // 遊戲邏輯節流（25 fps）
    if (now - lastFrame < FRAME_MS) return;
    lastFrame = now;

    // 生成小魚
    if (now - lastSpawn >= SPAWN_MS) {
        lastSpawn = now;
        for (int i = 0; i < MAX_FISH; i++) {
            if (!fish[i].alive) {
                fish[i].x     = SCREEN_W - 8;
                fish[i].y     = random(2, SCREEN_H - 6);
                fish[i].alive = true;
                break;
            }
        }
    }

    // 移動飛彈 + 打中小魚（AABB）
    for (int i = 0; i < MAX_MISSILES; i++) {
        if (!missiles[i].alive) continue;
        missiles[i].x += MISSILE_SPEED;
        if (missiles[i].x >= SCREEN_W) { missiles[i].alive = false; continue; }

        for (int j = 0; j < MAX_FISH; j++) {
            if (!fish[j].alive) continue;
            bool xHit = (missiles[i].x + 4 >= fish[j].x) && (missiles[i].x <= fish[j].x + 7);
            bool yHit = (missiles[i].y + 1 >= fish[j].y) && (missiles[i].y - 1 <= fish[j].y + 3);
            if (xHit && yHit) {
                fish[j].alive     = false;
                missiles[i].alive = false;
                score++;
                Serial.print("[HIT] score="); Serial.println(score);
                break;
            }
        }
    }

    // 移動小魚 + 撞飛機（AABB）
    // 飛機包圍盒：[planeX-4, planeX+6] × [planeY-3, planeY+3]
    for (int i = 0; i < MAX_FISH; i++) {
        if (!fish[i].alive) continue;
        fish[i].x -= FISH_SPEED;
        if (fish[i].x < -8) { fish[i].alive = false; continue; }

        bool xHit = (fish[i].x + 7 >= planeX - 4) && (fish[i].x <= planeX + 6);
        bool yHit = (fish[i].y + 3 >= planeY - 3) && (fish[i].y <= planeY + 3);
        if (xHit && yHit) {
            fish[i].alive = false;
            lives--;
            Serial.print("[CRASH] lives="); Serial.println(lives);
            if (lives <= 0) { gameOver = true; return; }
        }
    }

    // ── 繪製 ──────────────────────────────────────────────
    display.clearDisplay();

    // 分數（右上角）
    {
        char buf[8];
        sprintf(buf, "%d", score);
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(SCREEN_W - (int)strlen(buf) * 6, 0);
        display.print(buf);
    }

    // 命數（左下角）
    drawLives(lives);

    // 飛機（2D 自由位置）
    drawPlane(planeX, planeY);

    // 飛彈
    for (int i = 0; i < MAX_MISSILES; i++)
        if (missiles[i].alive) drawMissile(missiles[i].x, missiles[i].y);

    // 小魚
    for (int i = 0; i < MAX_FISH; i++)
        if (fish[i].alive) drawFish(fish[i].x, fish[i].y);

    display.display();
}

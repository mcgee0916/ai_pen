/*
 * OLED_BTN.ino — "疊房子" Tower Stack
 * 接線：OLED I2C, BTN pin 15 (INPUT_PULLUP)
 *
 * 方塊上下滑動（32px 短邊），從右側往左疊
 * 每層寬度 = 32px 短邊；疊偏則縮短為重疊部分
 * 誤差 ≤ 2px 視為完美不縮短
 * 方塊中心超過畫面上半後，畫面平滑向上跟隨
 *
 * 操作：單擊 = 放置 / 開始 / 重試；長按 = 減速
 *
 * 畫面示意：
 *   x=0                           x=128
 *   |  [BASE][■][■][■]  [□]       |  ← 塔從左往右疊
 *   |                   ↑↓        |
 *   |          滑塊在右邊上下滑     |
 */

#include <Wire.h>
#include <Adafruit_OLED_libraries/Adafruit_GFX.h>
#include <Adafruit_OLED_libraries/Adafruit_SSD1306.h>

#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   32
#define OLED_RESET      -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ══════════════════════════════════════════════════════════
// 按鈕驅動
// ══════════════════════════════════════════════════════════
#define BTN             15
#define LONG_PRESS_MS   400
#define DOUBLE_CLICK_MS 300

enum BtnEvt { BTN_NONE, BTN_SINGLE, BTN_LONG_START, BTN_LONG_END, BTN_DOUBLE };
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
        if (_wait2 && (now - _release_t >= DOUBLE_CLICK_MS)) {
            _wait2 = false; evt = BTN_SINGLE;
        }
    }
    _prev = cur;
    return evt;
}

// ══════════════════════════════════════════════════════════
// 遊戲常數
// ══════════════════════════════════════════════════════════
#define COL_W       8     // 每塊厚度（水平方向 px）
#define INIT_H     20     // 初始方塊高度（垂直方向 px，短邊 32px 的起點）
#define INIT_SPD    0.8f  // 初始上下滑動速度
#define SPD_INC     0.12f // 每層加速
#define SLOW_R      0.35f // 長按減速比
#define TOLERANCE   2     // 容錯像素（兩邊誤差 ≤ 此值 → 完美不縮短）
#define WIN_LEVEL  12     // 疊到此層獲勝

// ══════════════════════════════════════════════════════════
// 遊戲資料
// ══════════════════════════════════════════════════════════
enum GameState { TITLE, PLAYING, WIN, DEAD };
GameState state = TITLE;

struct Block { int y, h; };          // y=頂部，h=高度（垂直方向）
Block  tower[WIN_LEVEL + 2];         // [0]=底座（右側）, [1..]=往左疊
int    towerCount;
int    curH;                         // 目前方塊高度
float  sliderY;                      // 方塊頂部 Y（上下滑動）
float  sliderVel;
bool   slowed;
float  viewOffY;                     // 畫面垂直偏移（跟隨方塊中心）

#define MAX_PARTS 14
struct Particle { float x, y, vx, vy; int life; };
static Particle parts[MAX_PARTS];
static int perfectFlash = 0;

// 第 i 塊的 X 座標（底座在最左，往右疊）
int colX(int i) { return i * COL_W; }

// 目前移動塊的 X（在塔的右邊）
int sliderColX() { return towerCount * COL_W; }

// 邏輯 Y → 螢幕 Y
int toScreen(float logY) { return (int)(logY - viewOffY); }

void initGame() {
    tower[0]   = {(SCREEN_HEIGHT - INIT_H) / 2, INIT_H}; // 底座：與初始方塊同高，垂直置中
    towerCount = 1;
    curH       = INIT_H;
    sliderY    = (SCREEN_HEIGHT - INIT_H) / 2.0f; // 垂直置中出發
    sliderVel  = INIT_SPD;
    slowed     = false;
    viewOffY   = 0;
    for (int i = 0; i < MAX_PARTS; i++) parts[i].life = 0;
    perfectFlash = 0;
}

// ══════════════════════════════════════════════════════════
// 遊戲更新
// ══════════════════════════════════════════════════════════
void updatePlaying(BtnEvt evt) {
    if (evt == BTN_LONG_START) slowed = true;
    if (evt == BTN_LONG_END)   slowed = false;

    // 粒子更新
    for (int i = 0; i < MAX_PARTS; i++) {
        if (parts[i].life > 0) { parts[i].x += parts[i].vx; parts[i].y += parts[i].vy; parts[i].life--; }
    }
    if (perfectFlash > 0) perfectFlash--;

    // 上下滑動
    float step = slowed ? sliderVel * SLOW_R : sliderVel;
    sliderY += step;
    if (sliderY <= 0)                        { sliderY = 0;                        sliderVel =  fabsf(sliderVel); }
    if (sliderY >= SCREEN_HEIGHT - curH)     { sliderY = SCREEN_HEIGHT - curH;     sliderVel = -fabsf(sliderVel); }

    // 單擊放置
    if (evt == BTN_SINGLE) {
        Block &top = tower[towerCount - 1];
        int sy = (int)sliderY;
        int ah = curH;

        // 容錯：兩邊誤差 ≤ TOLERANCE → 完美對齊，不縮短
        bool isPerfect = (abs(sy - top.y) <= TOLERANCE &&
                          abs((sy + ah) - (top.y + top.h)) <= TOLERANCE);
        if (isPerfect) { sy = top.y; ah = top.h; }

        int overlapT = max(sy,      top.y);
        int overlapB = min(sy + ah, top.y + top.h);
        int overlapH = overlapB - overlapT;

        if (overlapH <= 0) {
            state = DEAD;
            Serial.print("[MISS] Lv="); Serial.println(towerCount - 1);
            return;
        }

        tower[towerCount] = {overlapT, overlapH};
        towerCount++;
        curH = overlapH;
        Serial.print("[OK]  Lv="); Serial.print(towerCount - 1);
        Serial.print("  h="); Serial.println(overlapH);

        // 放置特效：粒子爆發 + 完美閃爍
        float pcx = colX(towerCount - 1) + COL_W / 2.0f;
        float pcy = tower[towerCount - 1].y + tower[towerCount - 1].h / 2.0f;
        for (int i = 0; i < MAX_PARTS; i++) {
            float a = i * (6.2832f / MAX_PARTS);
            float spd = 3.0f + (float)(i % 3) * 1.5f;  // 3 / 4.5 / 6 px/frame
            parts[i] = { pcx, pcy, cosf(a) * spd, sinf(a) * spd * 0.75f, 10 };
        }
        if (isPerfect) { perfectFlash = 12; Serial.println("[PERFECT]"); }

        if (towerCount - 1 >= WIN_LEVEL) {
            state = WIN;
            Serial.println("[WIN]");
            return;
        }

        float dir = (sliderVel > 0) ? 1.0f : -1.0f;
        sliderVel = dir * (INIT_SPD + (towerCount - 1) * SPD_INC);
        slowed = false;
    }

    // 畫面跟隨：方塊中心超過畫面上半後平滑向上移動
    float sliderCenter = sliderY + curH / 2.0f;
    float target = (sliderCenter < SCREEN_HEIGHT / 2.0f)
                   ? sliderCenter - SCREEN_HEIGHT / 2.0f
                   : 0.0f;
    viewOffY += (target - viewOffY) * 0.08f;
}

// ══════════════════════════════════════════════════════════
// 畫面
// ══════════════════════════════════════════════════════════
void drawPlaying() {
    display.clearDisplay();

    // 已疊好的塊（底座在最右，往左排）
    for (int i = 0; i < towerCount; i++) {
        int x  = colX(i);
        int sy = toScreen(tower[i].y);
        int ey = toScreen(tower[i].y + tower[i].h);
        if (ey < 0 || sy >= SCREEN_HEIGHT) continue;
        display.fillRect(x, sy, COL_W - 1, ey - sy, SSD1306_WHITE);
    }

    // 移動中方塊（空心）
    int sx  = sliderColX();
    int ssy = toScreen(sliderY);
    int sey = toScreen(sliderY + curH);
    if (sx >= 0 && ssy < SCREEN_HEIGHT && sey > 0)
        display.drawRect(sx, ssy, COL_W - 1, sey - ssy, SSD1306_WHITE);

    // 粒子（2×2 方塊）
    for (int i = 0; i < MAX_PARTS; i++) {
        if (parts[i].life > 0) {
            int ppx = (int)parts[i].x;
            int ppy = toScreen(parts[i].y);
            if (ppx >= 0 && ppx < SCREEN_WIDTH - 1 && ppy >= 0 && ppy < SCREEN_HEIGHT - 1)
                display.fillRect(ppx, ppy, 2, 2, SSD1306_WHITE);
        }
    }

    // 分數（右上角）
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(110, 0);
    display.print(towerCount - 1);

    // 完美特效：前 3 幀全螢幕白底 PERFECT!，之後大號 !
    if (perfectFlash > 9) {
        display.fillRect(0, 4, SCREEN_WIDTH, 24, SSD1306_WHITE);
        display.setTextSize(2);
        display.setTextColor(SSD1306_BLACK);
        display.setCursor(20, 8);
        display.print("PERFECT!");
        display.setTextColor(SSD1306_WHITE);
    } else if (perfectFlash > 0) {
        display.setTextSize(2);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(56, 8);
        display.print('!');
        display.setTextSize(1);
    }

    display.display();
}

void drawTitle() {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(28, 4);  display.print("TOWER STACK");
    display.setCursor(16, 20); display.print("BTN = Start");
    display.display();
}

void drawDead() {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(4, 0);   display.print("MISS!");
    display.setTextSize(1);
    display.setCursor(80, 6);  display.print("Lv:"); display.print(towerCount - 1);
    display.setCursor(4, 22);  display.print("BTN = Try Again");
    display.display();
}

void drawWin() {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(16, 0);  display.print("WIN!");
    display.setTextSize(1);
    display.setCursor(76, 6);  display.print("Lv:"); display.print(towerCount - 1);
    display.setCursor(4, 22);  display.print("BTN = Play Again");
    display.display();
}

// ══════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    pinMode(BTN, INPUT_PULLUP);
    randomSeed(analogRead(0));
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println("OLED FAIL"); for (;;);
    }
    Serial.println("=== OLED_BTN: Tower Stack ===");
    Serial.println("BTN=place  LONG=slow");
    drawTitle();
}

void loop() {
    BtnEvt evt = pollButton();
    switch (state) {
        case TITLE:
            drawTitle();
            if (evt == BTN_SINGLE) { initGame(); state = PLAYING; }
            break;
        case PLAYING:
            updatePlaying(evt);
            drawPlaying();
            break;
        case DEAD:
            drawDead();
            if (evt == BTN_SINGLE) { initGame(); state = PLAYING; }
            break;
        case WIN:
            drawWin();
            if (evt == BTN_SINGLE) { initGame(); state = PLAYING; }
            break;
    }
    delay(30);
}

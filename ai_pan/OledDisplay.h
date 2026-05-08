/*
 * OledDisplay.h
 * OLED SSD1306 (128×32) API for ai_pan project
 *
 * 佈局：4 × 32×32 sections
 *   [SEC0: 路徑預覽 / bitmap 縮圖 / 結果] [SEC1] [SEC2] [SEC3]
 *
 * 使用方式：
 *   OledDisplay oled;
 *   oled.begin();
 *   oled.drawSec0Preview(pts, stroke_start, pt_count);
 *   oled.dev.display();   // flush（需要時直接呼叫）
 */

#pragma once

#include <string.h>
#include <Wire.h>
#include <Adafruit_OLED_libraries/Adafruit_GFX.h>
#include <Adafruit_OLED_libraries/Adafruit_SSD1306.h>

// ── 顯示器常數 ───────────────────────────────────────────────
#define OLED_W       128
#define OLED_H        32
#define OLED_ADDR     0x3C
#define OLED_SECS      4      // section 數量
#define OLED_SEC_W    32      // 每個 section 寬（px）

// ── 專案共用常數（亦供 ai_pan.ino 使用）────────────────────
#define BMP_SIZE      192     // 推論 bitmap 解析度（192×192）
#define BMP_PAD        20     // bitmap 四周邊距（像素），實際書寫區 152×152
#define MAX_TOKEN_LEN   8     // token 字串最大長度
#define MAX_TOKENS      3     // 運算式最多 token 數

// ── 軌跡點型別（路徑預覽與推論共用）────────────────────────
struct Pt { float x, y; };

// ════════════════════════════════════════════════════════════
//  OledDisplay class
// ════════════════════════════════════════════════════════════
class OledDisplay {
public:
    Adafruit_SSD1306 dev;    // 直接存取底層物件以使用 printf/setCursor 等

    OledDisplay() : dev(OLED_W, OLED_H, &Wire, -1) {}

    // ── 初始化 / 特效 ─────────────────────────────────────
    bool begin();
    void showSplash();                                          // "AI METH PAN" 跑馬燈

    // ── Section 基本操作 ──────────────────────────────────
    void clearSec    (int sec);
    void secBigChar  (int sec, char c);                        // textSize=3，置中
    void secFitText  (int sec, const char *text);              // 自動縮放置中
    void secSmallText(int sec, const char *l1,
                               const char *l2 = nullptr);     // 最多 2 行
    void clearExprArea();
    void drawExprText(const char *text, uint32_t now_ms);
    void drawDividers();                                        // SEC0 與算式區分隔線

    // ── 內容渲染 ─────────────────────────────────────────
    // SEC0：寫字過程的即時路徑預覽（rolling auto-fit → 30×30）
    void drawSec0Preview(Pt *pts, bool *stroke_start, int pt_count);

    // SEC0：64→32 2:1 降採樣 bitmap 縮圖
    void drawSec0Bitmap(uint8_t bmp[][BMP_SIZE]);

    // SEC1~3：顯示目前 expression tokens
    void refreshExprSecs(char expr[][MAX_TOKEN_LEN], int expr_cnt);

    // Idle 畫面（等待寫字）
    void redrawIdle(char expr[][MAX_TOKEN_LEN], int expr_cnt);
};

// ════════════════════════════════════════════════════════════
//  Inline 實作
// ════════════════════════════════════════════════════════════

inline bool OledDisplay::begin() {
    return dev.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
}

inline void OledDisplay::clearSec(int sec) {
    dev.fillRect(sec * OLED_SEC_W, 0, OLED_SEC_W, OLED_H, SSD1306_BLACK);
}

inline void OledDisplay::secBigChar(int sec, char c) {
    clearSec(sec);
    dev.setTextSize(3);
    dev.setTextColor(SSD1306_WHITE);
    dev.setCursor(sec * OLED_SEC_W + (OLED_SEC_W - 18) / 2,
                  (OLED_H - 24) / 2);
    dev.print(c);
}

inline void OledDisplay::secFitText(int sec, const char *text) {
    clearSec(sec);
    if (!text || text[0] == '\0') return;

    int ox = sec * OLED_SEC_W;
    int len = (int)strlen(text);
    if (len == 1) {
        secBigChar(sec, text[0]);
        return;
    }

    dev.setTextColor(SSD1306_WHITE);
    if (len <= 2) {
        dev.setTextSize(2);
        int w = len * 12;
        dev.setCursor(ox + max(0, (OLED_SEC_W - w) / 2), 8);
    } else {
        dev.setTextSize(1);
        int w = len * 6;
        dev.setCursor(ox + max(0, (OLED_SEC_W - w) / 2), 12);
    }
    dev.print(text);
}

inline void OledDisplay::secSmallText(int sec, const char *l1, const char *l2) {
    clearSec(sec);
    int ox = sec * OLED_SEC_W;
    dev.setTextSize(1);
    dev.setTextColor(SSD1306_WHITE);
    dev.setCursor(ox + 2, 4);
    dev.print(l1);
    if (l2) { dev.setCursor(ox + 2, 18); dev.print(l2); }
}

inline void OledDisplay::clearExprArea() {
    dev.fillRect(OLED_SEC_W + 1, 0, OLED_W - OLED_SEC_W - 1, OLED_H, SSD1306_BLACK);
}

inline void OledDisplay::drawExprText(const char *text, uint32_t now_ms) {
    clearExprArea();
    if (!text || text[0] == '\0') return;

    const int area_x = OLED_SEC_W + 2;
    const int area_y = 12;
    const int area_w = OLED_W - OLED_SEC_W - 4;
    const int gap_px = 12;

    dev.setTextSize(1);
    dev.setTextColor(SSD1306_WHITE);

    int16_t x1, y1;
    uint16_t w, h;
    dev.getTextBounds((char *)text, 0, 0, &x1, &y1, &w, &h);

    if ((int)w <= area_w) {
        int start_x = area_x + max(0, (area_w - (int)w) / 2);
        dev.setCursor(start_x, area_y);
        dev.print(text);
        return;
    }

    int cycle_px = (int)w + gap_px;
    int scroll_px = (int)((now_ms / 120) % cycle_px);
    int start_x = area_x - scroll_px;

    dev.setCursor(start_x, area_y);
    dev.print(text);
    dev.setCursor(start_x + cycle_px, area_y);
    dev.print(text);
}

inline void OledDisplay::drawDividers() {
    dev.drawLine(OLED_SEC_W, 0, OLED_SEC_W, OLED_H - 1, SSD1306_WHITE);
}

inline void OledDisplay::showSplash() {
    dev.clearDisplay();
    dev.setTextSize(2);
    dev.setTextColor(SSD1306_WHITE);
    int tw = 11 * 12;  // "AI METH PAN" 11字 × 12px
    dev.setCursor(max(0, (OLED_W - tw) / 2), (OLED_H - 16) / 2);
    dev.print(F("AI METH PAN"));
    dev.display();
    dev.startscrollright(0x00, 0x03);
    delay(4000);
    dev.stopscroll();
    delay(200);
    dev.clearDisplay();
    dev.display();
}

inline void OledDisplay::drawSec0Preview(Pt *pts, bool *stroke_start,
                                          int pt_count) {
    clearSec(0);
    if (pt_count < 2) return;

    // 計算 bounding box
    float mn_x = pts[0].x, mx_x = pts[0].x;
    float mn_y = pts[0].y, mx_y = pts[0].y;
    for (int i = 1; i < pt_count; i++) {
        if (pts[i].x < mn_x) mn_x = pts[i].x;
        if (pts[i].x > mx_x) mx_x = pts[i].x;
        if (pts[i].y < mn_y) mn_y = pts[i].y;
        if (pts[i].y > mx_y) mx_y = pts[i].y;
    }
    float w = mx_x - mn_x, h = mx_y - mn_y;
    float sc = 1.0f;
    if      (w == 0 && h == 0) sc = 1.0f;
    else if (w == 0)           sc = 30.0f / h;
    else if (h == 0)           sc = 30.0f / w;
    else                       sc = (30.0f/w < 30.0f/h) ? 30.0f/w : 30.0f/h;
    float cx = (mn_x + mx_x) * 0.5f;
    float cy = (mn_y + mx_y) * 0.5f;

    for (int i = 1; i < pt_count; i++) {
        if (stroke_start[i]) continue;
        int x0 = (int)(16.0f + (pts[i-1].x - cx) * sc + 0.5f);
        int y0 = (int)(16.0f - (pts[i-1].y - cy) * sc + 0.5f);
        int x1 = (int)(16.0f + (pts[i  ].x - cx) * sc + 0.5f);
        int y1 = (int)(16.0f - (pts[i  ].y - cy) * sc + 0.5f);
        if (x0 < 0 || x0 >= OLED_SEC_W || y0 < 0 || y0 >= OLED_H) continue;
        if (x1 < 0 || x1 >= OLED_SEC_W || y1 < 0 || y1 >= OLED_H) continue;
        dev.drawLine(x0, y0, x1, y1, SSD1306_WHITE);
    }
}

inline void OledDisplay::drawSec0Bitmap(uint8_t bmp[][BMP_SIZE]) {
    clearSec(0);
    // 28×28 → 32×32 nearest-neighbor 放大
    for (int r = 0; r < OLED_SEC_W; r++) {
        for (int c = 0; c < OLED_SEC_W; c++) {
            int sr = r * BMP_SIZE / OLED_SEC_W;
            int sc = c * BMP_SIZE / OLED_SEC_W;
            if (bmp[sr][sc]) dev.drawPixel(c, r, SSD1306_WHITE);
        }
    }
}

inline void OledDisplay::refreshExprSecs(char expr[][MAX_TOKEN_LEN],
                                          int expr_cnt) {
    for (int s = 0; s < MAX_TOKENS; s++) {
        if (s < expr_cnt && strlen(expr[s]) > 0)
            secBigChar(s + 1, expr[s][0]);
        else
            clearSec(s + 1);
    }
}

inline void OledDisplay::redrawIdle(char expr[][MAX_TOKEN_LEN],
                                     int expr_cnt) {
    dev.clearDisplay();
    secSmallText(0, "Write", "& hold");
    clearExprArea();
    drawDividers();
    dev.display();
}

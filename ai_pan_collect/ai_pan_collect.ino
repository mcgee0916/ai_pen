/*
 * ai_pan_collect.ino
 * 訓練資料集收集（無推論、無重啟）
 *
 * OLED 佈局：
 *   [SEC0: 路徑預覽] | [SEC1: 目標字元] [SEC2-3: 目前數量]
 *
 * 操作：
 *   長按   → 開始畫字
 *   放開   → 停筆
 *   雙擊   → 暫存 1 張；累積 10 張後批次儲存 BMP
 *   三擊   → 清除筆跡（不儲存）
 *
 * Serial 指令：
 *   <class_idx>-<sample_idx>  例：3-79  從 class 3 第 79 張繼續
 *   class 0~13 對應 0 1 2 3 4 5 6 7 8 9 + - × ÷
 */

#include <Wire.h>
#include "OledDisplay.h"

// 覆蓋 OledDisplay.h 的預設值，收集程式使用較大解析度
#undef BMP_SIZE
#define BMP_SIZE 192
#undef BMP_PAD
#define BMP_PAD  20

#ifdef __cplusplus
extern "C" {
#endif
#include "vfs.h"
#ifdef __cplusplus
}
#endif

// ── 硬體定義 ─────────────────────────────────────────────────────
#define BTN              15
#define LONG_PRESS_MS  300
#define DOUBLE_CLICK_MS 350

#define PAA_SERIAL  Serial3
#define PAA_BAUD    115200

// ── 資料集設定 ────────────────────────────────────────────────────
#define SD_DATASET_DIR    "sd:/dataset"
#define SAMPLES_PER_CLASS 50
#define MAX_PTS           800
#define BATCH_SAVE_COUNT   10

static const char *LABELS[14] = {
    "0","1","2","3","4","5","6","7","8","9",
    "+","-","x","/"
};
static const char *SAFE_NAMES[14] = {
    "0","1","2","3","4","5","6","7","8","9",
    "add","sub","mul","div"
};
#define NUM_CLASSES 14

// ── 全域物件 ──────────────────────────────────────────────────────
OledDisplay oled;

Pt   raw_pts[MAX_PTS];
bool stroke_start[MAX_PTS];
int  pt_count   = 0;
int  stroke_cnt = 0;

uint8_t bmp[BMP_SIZE][BMP_SIZE];

struct FitCtx { int px[MAX_PTS]; int py[MAX_PTS]; float scale; };
static FitCtx  fit_ctx;
static float   fit_work_x[MAX_PTS];
static float   fit_work_y[MAX_PTS];
struct PendingSample {
    Pt pts[MAX_PTS];
    uint8_t stroke_start[MAX_PTS];
    uint16_t pt_count;
    uint16_t class_idx;
    uint16_t sample_idx;
};
static PendingSample pending_samples[BATCH_SAVE_COUNT];
static int pending_count = 0;

static bool     is_writing         = false;
static bool     first_pt_of_stroke = false;
static bool     is_batch_saving    = false;

static int cur_class  = 0;
static int cur_sample = 0;

enum BtnEvt { BTN_NONE, BTN_LONG_START, BTN_LONG_END, BTN_SINGLE, BTN_DOUBLE, BTN_TRIPLE };
static bool     _prev_btn     = HIGH;
static bool     _in_long      = false;
static uint32_t _press_t      = 0;
static uint32_t _last_click_t = 0;
static uint8_t  _click_count  = 0;

char uart_buf[128]; int uart_len = 0;
char cmd_buf[128];  int cmd_len  = 0;

// ── 顯示 ─────────────────────────────────────────────────────────
void drawCount() {
    char s[4];
    snprintf(s, sizeof(s), "%03d", cur_sample);
    oled.secBigChar(2, s[0]);          // 百位
    char s2[3] = { s[1], s[2], '\0' };
    oled.secFitText(3, s2);            // 十位+個位
}

void drawCountValue(int sample_idx) {
    char s[4];
    snprintf(s, sizeof(s), "%03d", sample_idx);
    oled.secBigChar(2, s[0]);
    char s2[3] = { s[1], s[2], '\0' };
    oled.secFitText(3, s2);
}

void redrawMain(bool show_preview) {
    oled.dev.clearDisplay();
    if (show_preview && pt_count > 0)
        oled.drawSec0Preview(raw_pts, stroke_start, pt_count);
    else
        oled.clearSec(0);
    oled.secBigChar(1, LABELS[cur_class][0]);
    drawCount();
    oled.drawDividers();
    oled.dev.display();
}

void redrawSavingStatus(int class_idx, int sample_idx, int done, int total) {
    char progress[8];
    snprintf(progress, sizeof(progress), "%d/%d", done, total);

    oled.dev.clearDisplay();
    oled.secSmallText(0, "Saving", progress);
    oled.secBigChar(1, LABELS[class_idx][0]);
    drawCountValue(sample_idx);
    oled.drawDividers();
    oled.dev.display();
}

// ── 按鈕 ─────────────────────────────────────────────────────────
BtnEvt pollButton() {
    bool cur = digitalRead(BTN);
    uint32_t now = millis();
    BtnEvt evt = BTN_NONE;

    if (cur == LOW && _prev_btn == HIGH) {
        _press_t = now;
    } else if (cur == HIGH && _prev_btn == LOW) {
        if (_in_long) {
            _in_long = false;
            evt = BTN_LONG_END;
        } else if (now - _press_t < LONG_PRESS_MS) {
            if (_click_count < 3) _click_count++;
            _last_click_t = now;
        }
    } else if (cur == LOW) {
        if (!_in_long && (now - _press_t >= LONG_PRESS_MS)) {
            _in_long = true;
            _click_count = 0;
            evt = BTN_LONG_START;
        }
    } else if (_click_count > 0 && (now - _last_click_t >= DOUBLE_CLICK_MS)) {
        if      (_click_count == 1) evt = BTN_SINGLE;
        else if (_click_count == 2) evt = BTN_DOUBLE;
        else if (_click_count >= 3) evt = BTN_TRIPLE;
        _click_count = 0;
    }

    _prev_btn = cur;
    return evt;
}

// ── 筆跡 / BMP ───────────────────────────────────────────────────
static int cmpFloat(const void *a, const void *b) {
    float fa = *(const float *)a, fb = *(const float *)b;
    return (fa < fb) ? -1 : (fa > fb) ? 1 : 0;
}

bool autoFit(const Pt *pts, int count, FitCtx &out) {
    if (count < 2) return false;
    float rmx = pts[0].x, rMx = pts[0].x;
    float rmy = pts[0].y, rMy = pts[0].y;
    for (int i = 0; i < count; i++) {
        fit_work_x[i] = pts[i].x;
        fit_work_y[i] = pts[i].y;
        if (pts[i].x < rmx) rmx = pts[i].x;
        if (pts[i].x > rMx) rMx = pts[i].x;
        if (pts[i].y < rmy) rmy = pts[i].y;
        if (pts[i].y > rMy) rMy = pts[i].y;
    }
    qsort(fit_work_x, count, sizeof(float), cmpFloat);
    qsort(fit_work_y, count, sizeof(float), cmpFloat);
    int trim = (count >= 20) ? count / 20 : 0;
    if (trim * 2 >= count) trim = 0;
    float mn_x = fit_work_x[trim],        mx_x = fit_work_x[count-1-trim];
    float mn_y = fit_work_y[trim],        mx_y = fit_work_y[count-1-trim];
    if (mx_x <= mn_x) { mn_x = rmx; mx_x = rMx; }
    if (mx_y <= mn_y) { mn_y = rmy; mx_y = rMy; }
    float w = mx_x - mn_x, h = mx_y - mn_y;
    float fit = (float)(BMP_SIZE - BMP_PAD * 2);
    float sc = (w == 0 && h == 0) ? 1.0f
             : (w == 0) ? fit / h
             : (h == 0) ? fit / w
             : (fit/w < fit/h) ? fit/w : fit/h;
    float cx = (mn_x+mx_x)*0.5f, cy = (mn_y+mx_y)*0.5f, c = BMP_SIZE*0.5f;
    for (int i = 0; i < count; i++) {
        out.px[i] = (int)(c + (pts[i].x - cx) * sc + 0.5f);
        out.py[i] = (int)(c - (pts[i].y - cy) * sc + 0.5f);
    }
    out.scale = sc;
    return true;
}

void bmpDot(int x, int y) {
    // 3×3 方塊筆畫（stroke width ≈ 3px）
    for (int dr = -1; dr <= 1; dr++)
        for (int dc = -1; dc <= 1; dc++) {
            int px = x + dc, py = y + dr;
            if (px >= 0 && px < BMP_SIZE && py >= 0 && py < BMP_SIZE)
                bmp[py][px] = 1;
        }
}

void bmpLine(int x0, int y0, int x1, int y1) {
    int dx=abs(x1-x0), dy=abs(y1-y0);
    int sx=x0<x1?1:-1, sy=y0<y1?1:-1, err=dx-dy;
    while (true) {
        bmpDot(x0, y0);
        if (x0==x1&&y0==y1) break;
        int e2=2*err;
        if (e2>-dy){err-=dy;x0+=sx;}
        if (e2< dx){err+=dx;y0+=sy;}
    }
}

void renderToBmp(const FitCtx &fc, const uint8_t *starts, int count) {
    memset(bmp, 0, sizeof(bmp));
    for (int i = 1; i < count; i++) {
        if (starts[i]) continue;
        bmpLine(fc.px[i-1], fc.py[i-1], fc.px[i], fc.py[i]);
    }
}

void clearStrokes() {
    pt_count = stroke_cnt = 0;
    memset(bmp, 0, sizeof(bmp));
}

bool writeBmp(const char *path) {
    uint8_t hdr[54] = {0};
    uint32_t px_size = BMP_SIZE * BMP_SIZE * 3;
    uint32_t f_size  = 54 + px_size;
    hdr[0]='B'; hdr[1]='M';
    hdr[2]=(uint8_t)f_size;       hdr[3]=(uint8_t)(f_size>>8);
    hdr[4]=(uint8_t)(f_size>>16); hdr[5]=(uint8_t)(f_size>>24);
    hdr[10]=54; hdr[14]=40;
    hdr[18]=(uint8_t)BMP_SIZE; hdr[22]=(uint8_t)BMP_SIZE;
    hdr[26]=1;  hdr[28]=24;
    hdr[34]=(uint8_t)px_size;       hdr[35]=(uint8_t)(px_size>>8);
    hdr[36]=(uint8_t)(px_size>>16); hdr[37]=(uint8_t)(px_size>>24);
    FILE *fp = fopen(path, "wb");
    if (!fp) { Serial.print("[SD] open fail: "); Serial.println(path); return false; }
    fwrite(hdr, 1, 54, fp);
    uint8_t row[BMP_SIZE*3];
    for (int r = BMP_SIZE-1; r >= 0; r--) {
        for (int c = 0; c < BMP_SIZE; c++) {
            uint8_t v = bmp[r][c] ? 255 : 0;
            row[c*3]=row[c*3+1]=row[c*3+2]=v;
        }
        fwrite(row, 1, BMP_SIZE*3, fp);
    }
    fclose(fp);
    return true;
}

// ── 儲存樣本 ─────────────────────────────────────────────────────
void advanceSampleCursor() {
    cur_sample++;

    if (cur_sample >= SAMPLES_PER_CLASS) {
        cur_class++;
        cur_sample = 0;
        if (cur_class >= NUM_CLASSES) {
            cur_class = NUM_CLASSES - 1;
            Serial.println("[COLLECT] ALL DONE!");
        } else {
            Serial.print("[COLLECT] -> class ");
            Serial.println(LABELS[cur_class]);
        }
    }
}

bool flushPendingSamples() {
    if (pending_count == 0) return true;

    is_batch_saving = true;
    for (int i = 0; i < pending_count; i++) {
        PendingSample &ps = pending_samples[i];
        if (!autoFit(ps.pts, ps.pt_count, fit_ctx)) {
            Serial.print("[SAVE] autoFit failed at batch #");
            Serial.println(i);
            is_batch_saving = false;
            redrawMain(false);
            return false;
        }

        renderToBmp(fit_ctx, ps.stroke_start, ps.pt_count);
        redrawSavingStatus(ps.class_idx, ps.sample_idx, i + 1, pending_count);

        char path[80];
        snprintf(path, sizeof(path), "%s/%s/img_%05d.bmp",
                 SD_DATASET_DIR, SAFE_NAMES[ps.class_idx], ps.sample_idx);

        if (!writeBmp(path)) {
            is_batch_saving = false;
            redrawMain(false);
            return false;
        }

        Serial.print("[SAVE] "); Serial.print(LABELS[ps.class_idx]);
        Serial.print(" #"); Serial.print(ps.sample_idx);
        Serial.print(" -> "); Serial.println(path);
    }

    pending_count = 0;
    is_batch_saving = false;
    redrawMain(false);
    return true;
}

void onSave() {
    if (pt_count < 2) { Serial.println("[SAVE] too few points"); return; }
    if (pending_count >= BATCH_SAVE_COUNT) {
        Serial.println("[SAVE] buffer full");
        return;
    }

    PendingSample &ps = pending_samples[pending_count];
    ps.pt_count = (uint16_t)pt_count;
    ps.class_idx = (uint16_t)cur_class;
    ps.sample_idx = (uint16_t)cur_sample;
    for (int i = 0; i < pt_count; i++) {
        ps.pts[i] = raw_pts[i];
        ps.stroke_start[i] = stroke_start[i] ? 1 : 0;
    }

    pending_count++;
    Serial.print("[CACHE] "); Serial.print(LABELS[ps.class_idx]);
    Serial.print(" #"); Serial.print(ps.sample_idx);
    Serial.print(" buffered ");
    Serial.print(pending_count);
    Serial.print("/"); Serial.println(BATCH_SAVE_COUNT);

    advanceSampleCursor();
    clearStrokes();
    redrawMain(false);

    if (pending_count >= BATCH_SAVE_COUNT && !flushPendingSamples())
        Serial.println("[SAVE] batch flush failed");
}

// ── Serial 指令：<class_idx>-<sample_idx> ────────────────────────
void parseCmd(const char *line) {
    const char *dash = strchr(line, '-');
    if (!dash) return;
    int cls = atoi(line);
    int smp = atoi(dash + 1);
    if (cls < 0 || cls >= NUM_CLASSES)      { Serial.println("[CMD] class out of range");  return; }
    if (smp < 0 || smp >= SAMPLES_PER_CLASS){ Serial.println("[CMD] sample out of range"); return; }
    if (!flushPendingSamples())             { Serial.println("[CMD] flush pending failed"); return; }
    cur_class  = cls;
    cur_sample = smp;
    clearStrokes();
    redrawMain(false);
    Serial.print("[CMD] class="); Serial.print(LABELS[cur_class]);
    Serial.print(" sample="); Serial.println(cur_sample);
}

// ── PAA ──────────────────────────────────────────────────────────
static bool containsOK(const char *s) {
    for (int i = 0; s[i] && s[i+1]; i++) {
        char a = (s[i]  >='a'&&s[i]  <='z') ? s[i]  -32 : s[i];
        char b = (s[i+1]>='a'&&s[i+1]<='z') ? s[i+1]-32 : s[i+1];
        if (a=='O' && b=='K') return true;
    }
    return false;
}

static bool waitForPAAOK(uint32_t timeout_ms) {
    char rbuf[64]; int rlen = 0;
    uint32_t t0 = millis();
    while (millis()-t0 < timeout_ms) {
        if (!PAA_SERIAL.available()) continue;
        char c = (char)PAA_SERIAL.read();
        if (c=='\n'||c=='\r') {
            if (rlen > 0) { rbuf[rlen]='\0'; if (containsOK(rbuf)) return true; rlen=0; }
        } else if (rlen < (int)sizeof(rbuf)-1) rbuf[rlen++] = c;
    }
    return false;
}

bool initPAA() {
    PAA_SERIAL.begin(PAA_BAUD);
    delay(500);
    while (PAA_SERIAL.available()) PAA_SERIAL.read();
    PAA_SERIAL.println("at");
    if (!waitForPAAOK(1000)) { Serial.println("[PAA] no OK"); return false; }
    PAA_SERIAL.println("atrpt+50"); delay(200);
    PAA_SERIAL.println("atx+");     delay(200);
    Serial.println("[PAA] ready");
    return true;
}

void processLine(const char *line) {
    if (!strstr(line, "X=")) return;
    auto pf = [](const char *l, const char *key) -> float {
        const char *p = strstr(l, key);
        return p ? (float)atof(p+strlen(key)) : 0.0f;
    };
    float x = pf(line, "X="), y = pf(line, "Y=");

    if (is_writing && pt_count < MAX_PTS) {
        raw_pts[pt_count]     = {x, y};
        stroke_start[pt_count] = first_pt_of_stroke;
        pt_count++;
        first_pt_of_stroke = false;
    }

    static uint32_t _t = 0;
    if (millis()-_t < 150) return;
    _t = millis();

    if (is_batch_saving) return;

    oled.dev.clearDisplay();
    oled.drawSec0Preview(raw_pts, stroke_start, pt_count);

    // 停筆後顯示閃爍位置點（與原版 ai_pan 相同邏輯）
    if (!is_writing && pt_count > 0) {
        static bool     _blink_on = true;
        static uint32_t _blink_t  = 0;
        if (millis() - _blink_t >= 300) { _blink_on = !_blink_on; _blink_t = millis(); }

        if (_blink_on) {
            float mn_x = raw_pts[0].x, mx_x = raw_pts[0].x;
            float mn_y = raw_pts[0].y, mx_y = raw_pts[0].y;
            for (int i = 1; i < pt_count; i++) {
                if (raw_pts[i].x < mn_x) mn_x = raw_pts[i].x;
                if (raw_pts[i].x > mx_x) mx_x = raw_pts[i].x;
                if (raw_pts[i].y < mn_y) mn_y = raw_pts[i].y;
                if (raw_pts[i].y > mx_y) mx_y = raw_pts[i].y;
            }
            float rw = mx_x - mn_x, rh = mx_y - mn_y;
            float sc = (rw >= rh) ? (rw > 0 ? 30.0f/rw : 1.0f)
                                  : (rh > 0 ? 30.0f/rh : 1.0f);
            float cx = (mn_x+mx_x)*0.5f, cy = (mn_y+mx_y)*0.5f;
            int px = constrain((int)(16.0f + (x - cx)*sc + 0.5f), 1, 30);
            int py = constrain((int)(16.0f - (y - cy)*sc + 0.5f), 1, 30);
            oled.dev.fillRect(px-1, py-1, 3, 3, SSD1306_WHITE);
        }
    }

    oled.secBigChar(1, LABELS[cur_class][0]);
    drawCount();
    oled.drawDividers();
    oled.dev.display();
}

void handleButtonEvent(BtnEvt evt) {
    switch (evt) {
        case BTN_LONG_START:
            is_writing = first_pt_of_stroke = true;
            stroke_cnt++;
            Serial.println("[BTN] LONG_START");
            break;
        case BTN_LONG_END:
            is_writing = false;
            Serial.print("[BTN] LONG_END strokes="); Serial.println(stroke_cnt);
            break;
        case BTN_SINGLE:
            clearStrokes();
            redrawMain(false);
            Serial.println("[BTN] clear bmp");
            break;
        case BTN_DOUBLE:
            is_writing = first_pt_of_stroke = false;
            onSave();
            break;
        case BTN_TRIPLE:
            if (is_writing) break;
            showStatusScreen("PAA PROCESSING", "REINIT...");
            if (!initPAA()) {
                showStatusScreen("PAA ERROR", "CHECK SENSOR");
                Serial.println("[PAA] manual reinit failed");
            } else {
                redrawMain(false);
            }
            break;
        default: break;
    }
}
// ── setup / loop ──────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    pinMode(BTN, INPUT_PULLUP);

    if (!oled.begin()) { Serial.println("[OLED] fail"); for(;;); }
    oled.showSplash();

    vfs_init(NULL);
    if (vfs_user_register("sd", VFS_FATFS, VFS_INF_SD) == 0)
        Serial.println("[VFS] SD mounted");
    else
        Serial.println("[VFS] SD fail");

    for (int i = 0; i < NUM_CLASSES; i++) {
        char dir[64];
        snprintf(dir, sizeof(dir), "%s/%s", SD_DATASET_DIR, SAFE_NAMES[i]);
        if (access(dir, F_OK) != 0) mkdir(dir, 0);
    }

    if (!initPAA()) {
        showStatusScreen("PAA ERROR", "CHECK SENSOR");
        Serial.println("[PAA] init failed");
    }
    clearStrokes();
    redrawMain(false);
    Serial.println("[SETUP] done  cmd: <class>-<sample>  e.g. 3-79");
}

void loop() {
    handleButtonEvent(pollButton());

    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c=='\n'||c=='\r') {
            if (cmd_len > 0) { cmd_buf[cmd_len]='\0'; parseCmd(cmd_buf); cmd_len=0; }
        } else if (cmd_len < (int)sizeof(cmd_buf)-1) cmd_buf[cmd_len++] = c;
    }

    static uint32_t _paa_last = 0;
    static bool _paa_init = false;
    if (!_paa_init) { _paa_last = millis(); _paa_init = true; }

    while (PAA_SERIAL.available()) {
        handleButtonEvent(pollButton());
        _paa_last = millis();
        char c = (char)PAA_SERIAL.read();
        if (c=='\n'||c=='\r') {
            if (uart_len > 0) { uart_buf[uart_len]='\0'; processLine(uart_buf); uart_len=0; }
        } else if (uart_len < (int)sizeof(uart_buf)-1) uart_buf[uart_len++] = c;
    }

    if (millis()-_paa_last > 3000) {
        showStatusScreen("PAA PROCESSING", "RETRY...");
        Serial.println("[PAA] no data, retry init");
        while (PAA_SERIAL.available()) PAA_SERIAL.read();
        uart_len = 0;
        if (initPAA()) {
            redrawMain(false);
        } else {
            showStatusScreen("PAA ERROR", "CHECK SENSOR");
            Serial.println("[PAA] retry failed");
        }
        _paa_last = millis();
    }

    static uint32_t _hb = 0;
    if (millis()-_hb > 5000) { _hb = millis(); Serial.println("[LOOP] alive"); }
}

static void showStatusScreen(const char *line1, const char *line2) {
    oled.dev.clearDisplay();
    oled.dev.setTextSize(1);
    oled.dev.setTextColor(SSD1306_WHITE);
    oled.dev.setCursor(0, 0);
    if (line1 && line1[0] != '\0') oled.dev.println(line1);
    if (line2 && line2[0] != '\0') oled.dev.println(line2);
    oled.dev.display();
}
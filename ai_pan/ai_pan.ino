/*
 * ai_pan.ino
 * 單次辨識流程：
 * 1. 使用官方 ObjectDetectionImage 的 SD 圖片辨識機制。
 * 2. 每次只辨識一個 token。
 * 3. 辨識完成後把目前算式字串存進 SD。
 * 4. OLED 先顯示辨識圖，再直接 sys_reset 重啟。
 */

#include <Wire.h>
#include "OledDisplay.h"
#include <JPEGENC.h>
#include "NNObjectDetectionImage.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "vfs.h"
#include "cJSON.h"
#include "sys_api.h"
#ifdef __cplusplus
}
#endif

#define BTN              15
#define LONG_PRESS_MS  300
#define DOUBLE_CLICK_MS 350

#define PAA_SERIAL  Serial3

#define PAA_BAUD    115200

#define SD_JPG_PATH    "sd:/char-0001.jpg"
#define SD_JSON_PATH   "sd:/char-0001.json"
#define SD_LIST_PATH   "sd:/char_list.txt"
#define SD_STATE_PATH  "sd:/calc_state.txt"
#define SD_RECOG_DIR   "sd:/recognized"

#define MAX_PTS         800
#define MAX_EXPR_CHARS   31

static const char *CALC_LABELS[14] = {
    "0","1","2","3","4","5","6","7","8","9",
    "add","sub","mul","div"
};
#define CALC_NUM_LABELS 14

OledDisplay oled;
NNObjectDetectionImage objDetImg;
static char filelist_name[] = "char_list.txt";

Pt   raw_pts[MAX_PTS];
bool stroke_start[MAX_PTS];
int  pt_count   = 0;
int  stroke_cnt = 0;

uint8_t bmp[BMP_SIZE][BMP_SIZE];
uint8_t recog_bmp[BMP_SIZE][BMP_SIZE];

struct FitCtx {
    int   px[MAX_PTS];
    int   py[MAX_PTS];
    float scale;
};
static FitCtx fit_ctx;
static float fit_work_x[MAX_PTS];
static float fit_work_y[MAX_PTS];

static bool is_writing         = false;
static bool first_pt_of_stroke = false;
static char calc_state[MAX_EXPR_CHARS + 1] = "";
enum BtnEvt { BTN_NONE, BTN_LONG_START, BTN_LONG_END, BTN_SINGLE, BTN_DOUBLE, BTN_TRIPLE, BTN_QUAD };
static bool     _prev_btn    = HIGH;
static bool     _in_long     = false;
static uint32_t _press_t     = 0;
static uint32_t _last_click_t = 0;
static uint8_t  _click_count  = 0;

enum RunState { RUN_IDLE, RUN_CAPTURING, RUN_RECOGNIZING };
static RunState run_state = RUN_IDLE;
static bool recognize_pending = false;

char uart_buf[128];
int  uart_len = 0;
char cmd_buf[128];
int  cmd_len = 0;
static uint32_t debug_run_seq = 0;
static uint16_t debug_pts_dropped = 0;
static int debug_last_json_count = -1;
static int debug_last_best_area_cls = -1;
static float debug_last_best_area = -1.0f;
static int debug_last_best_score_cls = -1;
static float debug_last_best_score = -1.0f;
static uint32_t debug_last_json_checksum = 0;
static int debug_last_json_len = -1;

static bool isDigitChar(char c) {
    return c >= '0' && c <= '9';
}

static bool isOperatorChar(char c) {
    return c == '+' || c == '-' || c == '*' || c == '/';
}

// 將 CALC_LABELS 的運算子字串轉為內部單字元（'+' '-' '*' '/'）
static char labelToOpChar(const char *label) {
    if (strcmp(label, "add") == 0) return '+';
    if (strcmp(label, "sub") == 0) return '-';
    if (strcmp(label, "mul") == 0) return '*';
    if (strcmp(label, "div") == 0) return '/';
    return '\0';
}

// 取得 OLED 顯示用字元（數字直接取首字元，運算子映射回可見 ASCII）
static char labelToDisplayChar(const char *label) {
    if (!label || label[0] == '\0') return '?';
    if (label[1] == '\0') return label[0];
    return labelToOpChar(label);
}

static char toUpperAscii(char c) {
    return (c >= 'a' && c <= 'z') ? (char)(c - 'a' + 'A') : c;
}

static bool containsOK(const char *s) {
    if (!s) return false;
    for (int i = 0; s[i] != '\0' && s[i + 1] != '\0'; i++) {
        if (toUpperAscii(s[i]) == 'O' && toUpperAscii(s[i + 1]) == 'K') {
            return true;
        }
    }
    return false;
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

static int compareFloatAsc(const void *a, const void *b) {
    float fa = *(const float *)a;
    float fb = *(const float *)b;
    if (fa < fb) return -1;
    if (fa > fb) return 1;
    return 0;
}

static bool waitForPAAOK(uint32_t timeout_ms, const char *log_prefix) {
    char rbuf[64];
    int rlen = 0;
    uint32_t t0 = millis();

    while (millis() - t0 < timeout_ms) {
        if (PAA_SERIAL.available()) {
            char c = (char)PAA_SERIAL.read();
            if (c == '\n' || c == '\r') {
                if (rlen > 0) {
                    rbuf[rlen] = '\0';
                    if (log_prefix && log_prefix[0] != '\0') {
                        Serial.print(log_prefix); Serial.println(rbuf);
                    }
                    if (containsOK(rbuf)) return true;
                    rlen = 0;
                }
            } else if (rlen < (int)sizeof(rbuf) - 1) {
                rbuf[rlen++] = c;
            }
        }
    }

    if (rlen > 0) {
        rbuf[rlen] = '\0';
        if (log_prefix && log_prefix[0] != '\0') {
            Serial.print(log_prefix); Serial.println(rbuf);
        }
        if (containsOK(rbuf)) return true;
    }
    return false;
}

static int findEqualIndex(const char *s) {
    for (int i = 0; s[i] != '\0'; i++) {
        if (s[i] == '=') return i;
    }
    return -1;
}

static int findOperatorIndex(const char *s) {
    for (int i = 0; s[i] != '\0'; i++) {
        if (isOperatorChar(s[i])) return i;
    }
    return -1;
}

static void copySpan(char *dst, size_t dstlen, const char *src, int count) {
    if (dstlen == 0) return;
    if (count < 0) count = 0;
    int n = count;
    if (n > (int)dstlen - 1) n = (int)dstlen - 1;
    memcpy(dst, src, n);
    dst[n] = '\0';
}

void renderExprSections() {
    char display_text[MAX_EXPR_CHARS * 2 + 4] = "";

    if (calc_state[0] != '\0') {
        int eq_idx = findEqualIndex(calc_state);
        int op_idx = findOperatorIndex(calc_state);

        if (eq_idx >= 0) {
            strncpy(display_text, calc_state, sizeof(display_text) - 1);
            display_text[sizeof(display_text) - 1] = '\0';
        } else if (op_idx < 0) {
            snprintf(display_text, sizeof(display_text), "%s=%s", calc_state, calc_state);
        } else {
            strncpy(display_text, calc_state, sizeof(display_text) - 1);
            display_text[sizeof(display_text) - 1] = '\0';
        }
    }

    oled.drawExprText(display_text, millis());
}

void redrawStateScreen(bool show_bitmap) {
    oled.dev.clearDisplay();
    if (show_bitmap) {
        oled.drawSec0Bitmap(bmp);
    } else {
        oled.clearSec(0);
    }
    renderExprSections();
    oled.drawDividers();
    oled.dev.display();
}

void showNoRecognitionScreen() {
    oled.dev.clearDisplay();
    oled.drawSec0Bitmap(bmp);
    oled.drawExprText("NO RESULT", millis());
    oled.drawDividers();
    oled.dev.display();
}

void saveCalcState() {
    FILE *fp = fopen(SD_STATE_PATH, "w");
    if (!fp) {
        Serial.println("[SD] cannot write calc_state.txt");
        return;
    }
    fputs(calc_state, fp);
    fclose(fp);
}

void clearCalcState() {
    calc_state[0] = '\0';
    saveCalcState();
    clearStrokes();
    recognize_pending = false;
    is_writing = false;
    first_pt_of_stroke = false;
    run_state = RUN_IDLE;
    redrawStateScreen(false);
}

void loadCalcState() {
    calc_state[0] = '\0';
    FILE *fp = fopen(SD_STATE_PATH, "r");
    if (!fp) return;

    int len = (int)fread(calc_state, 1, MAX_EXPR_CHARS, fp);
    fclose(fp);
    if (len < 0) len = 0;
    calc_state[len] = '\0';

    while (len > 0 &&
           (calc_state[len - 1] == '\r' || calc_state[len - 1] == '\n' || calc_state[len - 1] == ' ')) {
        calc_state[--len] = '\0';
    }
}

bool evaluateExpression(const char *expr_text, char *result, size_t result_len) {
    if (!expr_text || !result || result_len == 0) return false;

    int op_idx = findOperatorIndex(expr_text);
    if (op_idx <= 0) return false;

    char lhs[16] = "";
    char rhs[16] = "";
    copySpan(lhs, sizeof(lhs), expr_text, op_idx);
    strncpy(rhs, expr_text + op_idx + 1, sizeof(rhs) - 1);
    rhs[sizeof(rhs) - 1] = '\0';

    if (lhs[0] == '\0' || rhs[0] == '\0') return false;

    long a = atol(lhs);
    long b = atol(rhs);
    char op = expr_text[op_idx];
    long value = 0;

    switch (op) {
        case '+': value = a + b; break;
        case '-': value = a - b; break;
        case '*': value = a * b; break;
        case '/':
            if (b == 0) {
                strncpy(result, "ERR", result_len - 1);
                result[result_len - 1] = '\0';
                return true;
            }
            value = a / b;
            break;
        default:
            return false;
    }

    snprintf(result, result_len, "%ld", value);
    return true;
}

void finalizeExpressionIfReady() {
    if (findEqualIndex(calc_state) >= 0) return;

    int op_idx = findOperatorIndex(calc_state);
    if (op_idx <= 0) return;
    if (calc_state[op_idx + 1] == '\0') return;
    if (!isDigitChar(calc_state[strlen(calc_state) - 1])) return;

    char result[16] = "";
    if (!evaluateExpression(calc_state, result, sizeof(result))) return;

    char expr_copy[MAX_EXPR_CHARS + 1];
    strncpy(expr_copy, calc_state, sizeof(expr_copy) - 1);
    expr_copy[sizeof(expr_copy) - 1] = '\0';

    size_t expr_len = strlen(expr_copy);
    size_t result_len = strlen(result);
    if (expr_len + 1 + result_len > MAX_EXPR_CHARS) return;

    memcpy(calc_state, expr_copy, expr_len);
    calc_state[expr_len] = '=';
    memcpy(calc_state + expr_len + 1, result, result_len);
    calc_state[expr_len + 1 + result_len] = '\0';
}

bool appendRecognizedToken(const char *token) {
    if (!token || token[0] == '\0') return false;

    char ch;
    if (token[1] == '\0') {
        ch = token[0];                 // 數字 "0"–"9"
    } else {
        ch = labelToOpChar(token);     // "add"→'+' "sub"→'-' "mul"→'*' "div"→'/'
        if (ch == '\0') return false;
    }
    int eq_idx = findEqualIndex(calc_state);

    if (eq_idx >= 0) {
        char result_part[MAX_EXPR_CHARS + 1] = "";
        strncpy(result_part, calc_state + eq_idx + 1, sizeof(result_part) - 1);
        result_part[sizeof(result_part) - 1] = '\0';

        if (isOperatorChar(ch) && strcmp(result_part, "ERR") != 0 && result_part[0] != '\0') {
            size_t result_len = strlen(result_part);
            if (result_len + 1 > MAX_EXPR_CHARS) return false;
            memcpy(calc_state, result_part, result_len);
            calc_state[result_len] = ch;
            calc_state[result_len + 1] = '\0';
            return true;
        } else if (isDigitChar(ch)) {
            calc_state[0] = ch;
            calc_state[1] = '\0';
            return true;
        } else {
            return false;
        }
    }

    int len = (int)strlen(calc_state);
    int op_idx = findOperatorIndex(calc_state);

    if (isDigitChar(ch)) {
        if (op_idx < 0 && len == 1 && calc_state[0] == '0') {
            calc_state[0] = ch;
            calc_state[1] = '\0';
            finalizeExpressionIfReady();
            return true;
        }
        if (op_idx >= 0 && calc_state[op_idx + 1] == '0' && calc_state[op_idx + 2] == '\0') {
            calc_state[op_idx + 1] = ch;
            calc_state[op_idx + 2] = '\0';
            finalizeExpressionIfReady();
            return true;
        }
        if (len >= MAX_EXPR_CHARS) return false;
        calc_state[len] = ch;
        calc_state[len + 1] = '\0';
        finalizeExpressionIfReady();
        return true;
    }

    if (!isOperatorChar(ch)) return false;
    if (len == 0) return false;

    if (op_idx < 0) {
        if (len >= MAX_EXPR_CHARS) return false;
        calc_state[len] = ch;
        calc_state[len + 1] = '\0';
        return true;
    }

    if (isOperatorChar(calc_state[len - 1])) {
        calc_state[len - 1] = ch;
        return true;
    }

    return false;
}

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
            if (_click_count < 4) _click_count++;
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
        else if (_click_count == 3) evt = BTN_TRIPLE;
        else if (_click_count >= 4) evt = BTN_QUAD;
        _click_count = 0;
    }

    _prev_btn = cur;
    return evt;
}

void handleButtonEvent(BtnEvt evt) {
    switch (evt) {
        case BTN_LONG_START:
            if (run_state == RUN_RECOGNIZING || recognize_pending) {
                break;
            }
            is_writing = true;
            first_pt_of_stroke = true;
            stroke_cnt++;
            run_state = RUN_CAPTURING;
            break;

        case BTN_LONG_END:
            if (!is_writing) break;
            is_writing = false;
            if (run_state != RUN_RECOGNIZING) run_state = RUN_IDLE;
            break;

        case BTN_SINGLE:
            if (run_state == RUN_RECOGNIZING) break;
            clearStrokes();
            Serial.println("[BTN] clear bmp");
            break;

        case BTN_DOUBLE:
            if (run_state == RUN_RECOGNIZING || recognize_pending) {
                break;
            }
            recognize_pending = true;
            if (is_writing) {
                is_writing = false;
                first_pt_of_stroke = false;
            }
            break;

        case BTN_TRIPLE:
            if (run_state == RUN_RECOGNIZING) {
                break;
            }
            clearCalcState();
            break;

        case BTN_QUAD:
            if (run_state == RUN_RECOGNIZING || is_writing) {
                break;
            }
            showStatusScreen("PAA PROCESSING", "REINIT...");
            if (!initPAA()) {
                showStatusScreen("PAA ERROR", "CHECK SENSOR");
                Serial.println("[PAA] manual reinit failed");
            } else {
                redrawStateScreen(false);
            }
            break;

        default:
            break;
    }
}

void compassCalibrate() {
    // 停止感測器輸出
    while (PAA_SERIAL.available()) PAA_SERIAL.read();
    PAA_SERIAL.println("at-");
    delay(300);
    while (PAA_SERIAL.available()) PAA_SERIAL.read();

    // 發送校正指令（20 秒橢圓擬合）
    PAA_SERIAL.println("at0c+8");

    // OLED 倒數顯示，同時收集回應
    uint32_t start = millis();
    bool got_ok = false;
    char cal_resp[128] = "";
    int  cal_resp_len  = 0;

    while (millis() - start < 25000) {
        int remain = 20 - (int)((millis() - start) / 1000);
        if (remain < 0) remain = 0;

        oled.dev.clearDisplay();
        oled.dev.setTextSize(1);
        oled.dev.setTextColor(SSD1306_WHITE);
        oled.dev.setCursor(0, 0);
        oled.dev.println("COMPASS CAL");
        oled.dev.println("ROTATE SENSOR");
        oled.dev.print("Time: "); oled.dev.print(remain); oled.dev.print("s");
        oled.dev.display();

        while (PAA_SERIAL.available() && cal_resp_len < 127) {
            char c = PAA_SERIAL.read();
            cal_resp[cal_resp_len++] = c;
            cal_resp[cal_resp_len]   = '\0';
        }
        if (strstr(cal_resp, "ok ") || strstr(cal_resp, "\nok")) {
            got_ok = true;
            break;
        }
        delay(500);
    }

    // 顯示結果
    oled.dev.clearDisplay();
    oled.dev.setTextSize(2);
    oled.dev.setCursor(0, 8);
    oled.dev.println(got_ok ? "CAL OK" : "CAL FAIL");
    oled.dev.display();
    delay(2000);

    // 重啟資料輸出
    while (PAA_SERIAL.available()) PAA_SERIAL.read();
    uart_len = 0;
    PAA_SERIAL.println("atx+");
    delay(200);

    redrawStateScreen(false);
}

bool autoFit(int sz, int pad, FitCtx &out) {
    if (pt_count < 2) return false;

    float raw_mn_x = raw_pts[0].x, raw_mx_x = raw_pts[0].x;
    float raw_mn_y = raw_pts[0].y, raw_mx_y = raw_pts[0].y;
    for (int i = 0; i < pt_count; i++) {
        fit_work_x[i] = raw_pts[i].x;
        fit_work_y[i] = raw_pts[i].y;
        if (raw_pts[i].x < raw_mn_x) raw_mn_x = raw_pts[i].x;
        if (raw_pts[i].x > raw_mx_x) raw_mx_x = raw_pts[i].x;
        if (raw_pts[i].y < raw_mn_y) raw_mn_y = raw_pts[i].y;
        if (raw_pts[i].y > raw_mx_y) raw_mx_y = raw_pts[i].y;
    }

    qsort(fit_work_x, pt_count, sizeof(float), compareFloatAsc);
    qsort(fit_work_y, pt_count, sizeof(float), compareFloatAsc);

    int trim = (pt_count >= 20) ? (pt_count / 20) : 0;  // trim 5% extreme points
    if (trim * 2 >= pt_count) trim = 0;

    float mn_x = fit_work_x[trim];
    float mx_x = fit_work_x[pt_count - 1 - trim];
    float mn_y = fit_work_y[trim];
    float mx_y = fit_work_y[pt_count - 1 - trim];

    if (mx_x <= mn_x) { mn_x = raw_mn_x; mx_x = raw_mx_x; }
    if (mx_y <= mn_y) { mn_y = raw_mn_y; mx_y = raw_mx_y; }

    float w = mx_x - mn_x;
    float h = mx_y - mn_y;
    float fit = (float)(sz - pad * 2);
    float sc;
    if      (w == 0 && h == 0) sc = 1.0f;
    else if (w == 0)           sc = fit / h;
    else if (h == 0)           sc = fit / w;
    else                       sc = (fit / w < fit / h) ? fit / w : fit / h;

    float cx = (mn_x + mx_x) * 0.5f;
    float cy = (mn_y + mx_y) * 0.5f;
    float c  = (float)sz * 0.5f;
    for (int i = 0; i < pt_count; i++) {
        out.px[i] = (int)(c + (raw_pts[i].x - cx) * sc + 0.5f);
        out.py[i] = (int)(c - (raw_pts[i].y - cy) * sc + 0.5f);
    }
    out.scale = sc;
    return true;
}

static inline void plotBmp3px(int x, int y) {
    for (int dy = -1; dy <= 1; dy++) {
        int py = y + dy;
        if (py < 0 || py >= BMP_SIZE) continue;
        for (int dx = -1; dx <= 1; dx++) {
            int px = x + dx;
            if (px < 0 || px >= BMP_SIZE) continue;
            bmp[py][px] = 1;
        }
    }
}

void bmpLine(int x0, int y0, int x1, int y1) {
    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;

    while (true) {
        if (x0 >= 0 && x0 < BMP_SIZE && y0 >= 0 && y0 < BMP_SIZE) plotBmp3px(x0, y0);
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

void renderToBmp(const FitCtx &fc) {
    memset(bmp, 0, sizeof(bmp));
    for (int i = 1; i < pt_count; i++) {
        if (stroke_start[i]) continue;
        bmpLine(fc.px[i - 1], fc.py[i - 1], fc.px[i], fc.py[i]);
    }
}

void clearStrokes() {
    pt_count = 0;
    stroke_cnt = 0;
    debug_pts_dropped = 0;
    memset(raw_pts, 0, sizeof(raw_pts));
    memset(stroke_start, 0, sizeof(stroke_start));
    memset(bmp, 0, sizeof(bmp));
    memset(recog_bmp, 0, sizeof(recog_bmp));
    memset(&fit_ctx, 0, sizeof(fit_ctx));
    memset(fit_work_x, 0, sizeof(fit_work_x));
    memset(fit_work_y, 0, sizeof(fit_work_y));
}


static void *jpegOpen(const char *path) {
    return fopen(path, "w+b");
}

static void jpegClose(JPEGE_FILE *pFile) {
    FILE *fp = (FILE *)pFile->fHandle;
    if (fp) fclose(fp);
}

static int32_t jpegRead(JPEGE_FILE *pFile, uint8_t *buffer, int32_t length) {
    FILE *fp = (FILE *)pFile->fHandle;
    if (!fp) return 0;
    return (int32_t)fread(buffer, 1, (size_t)length, fp);
}

static int32_t jpegWrite(JPEGE_FILE *pFile, uint8_t *buffer, int32_t length) {
    FILE *fp = (FILE *)pFile->fHandle;
    if (!fp) return 0;
    return (int32_t)fwrite(buffer, 1, (size_t)length, fp);
}

static int32_t jpegSeek(JPEGE_FILE *pFile, int32_t position) {
    FILE *fp = (FILE *)pFile->fHandle;
    if (!fp) return -1;
    if (fseek(fp, position, SEEK_SET) != 0) return -1;
    return (int32_t)ftell(fp);
}

bool writeRasterBufferToSD(const char *path, uint8_t src[][BMP_SIZE]) {
    if (!path || !src) return false;

    uint8_t hdr[54] = {0};
    uint32_t px_size = BMP_SIZE * BMP_SIZE * 3;
    uint32_t f_size = 54 + px_size;

    hdr[0] = 'B'; hdr[1] = 'M';
    hdr[2] = (uint8_t)(f_size);       hdr[3] = (uint8_t)(f_size >> 8);
    hdr[4] = (uint8_t)(f_size >> 16); hdr[5] = (uint8_t)(f_size >> 24);
    hdr[10] = 54;
    hdr[14] = 40;
    hdr[18] = (uint8_t)(BMP_SIZE);
    hdr[22] = (uint8_t)(BMP_SIZE);
    hdr[26] = 1;
    hdr[28] = 24;
    hdr[34] = (uint8_t)(px_size);       hdr[35] = (uint8_t)(px_size >> 8);
    hdr[36] = (uint8_t)(px_size >> 16); hdr[37] = (uint8_t)(px_size >> 24);

    FILE *fp = fopen(path, "wb");
    if (!fp) {
        Serial.print("[SD] open fail: "); Serial.println(path);
        return false;
    }

    fwrite(hdr, 1, 54, fp);
    uint8_t row[BMP_SIZE * 3];
    for (int r = BMP_SIZE - 1; r >= 0; r--) {
        for (int c = 0; c < BMP_SIZE; c++) {
            uint8_t v = src[r][c] ? 255 : 0;
            row[c * 3 + 0] = v;
            row[c * 3 + 1] = v;
            row[c * 3 + 2] = v;
        }
        fwrite(row, 1, BMP_SIZE * 3, fp);
    }
    fclose(fp);
    return true;
}

bool writeRasterImageToSD(const char *path) {
    return writeRasterBufferToSD(path, bmp);
}

bool writeJpegBufferToSD(const char *path, uint8_t src[][BMP_SIZE]) {
    if (!path || !src) return false;

    JPEGENC jpg;
    JPEGENCODE enc;
    uint8_t mcu[16 * 16 * 3];

    int rc = jpg.open(path, jpegOpen, jpegClose, jpegRead, jpegWrite, jpegSeek);
    if (rc != JPEGE_SUCCESS) {
        return false;
    }

    rc = jpg.encodeBegin(&enc, BMP_SIZE, BMP_SIZE, JPEGE_PIXEL_RGB888, JPEGE_SUBSAMPLE_420, JPEGE_Q_HIGH);
    if (rc != JPEGE_SUCCESS) {
        jpg.close();
        return false;
    }

    for (int y = 0; y < BMP_SIZE; y += enc.cy) {
        for (int x = 0; x < BMP_SIZE; x += enc.cx) {
            for (int yy = 0; yy < enc.cy; yy++) {
                for (int xx = 0; xx < enc.cx; xx++) {
                    uint8_t v = src[y + yy][x + xx] ? 255 : 0;
                    int idx = (yy * enc.cx + xx) * 3;
                    mcu[idx + 0] = v;
                    mcu[idx + 1] = v;
                    mcu[idx + 2] = v;
                }
            }
            rc = jpg.addMCU(&enc, mcu, enc.cx * 3);
            if (rc != JPEGE_SUCCESS) {
                jpg.close();
                return false;
            }
        }
    }

    return jpg.close() > 0;
}

bool writeJpegImageToSD(const char *path) {
    return writeJpegBufferToSD(path, bmp);
}

static inline void drawRectOnBuffer(uint8_t dst[][BMP_SIZE], int x, int y, int w, int h) {
    if (!dst || w <= 0 || h <= 0) return;

    int x0 = constrain(x, 0, BMP_SIZE - 1);
    int y0 = constrain(y, 0, BMP_SIZE - 1);
    int x1 = constrain(x + w - 1, 0, BMP_SIZE - 1);
    int y1 = constrain(y + h - 1, 0, BMP_SIZE - 1);

    for (int xx = x0; xx <= x1; xx++) {
        dst[y0][xx] = 1;
        dst[y1][xx] = 1;
    }
    for (int yy = y0; yy <= y1; yy++) {
        dst[yy][x0] = 1;
        dst[yy][x1] = 1;
    }
}

const char *recognizedLabelTag(int class_id) {
    if (class_id < 0 || class_id >= CALC_NUM_LABELS) return "na";
    return CALC_LABELS[class_id];
}

void saveRecognizedBmp(int class_id) {
    char image_name[40];
    char image_path[80];

    int sample_id = 0;
    bool found_slot = false;
    for (; sample_id < 100000; sample_id++) {
        snprintf(image_name, sizeof(image_name), "rec_%05d_%s.bmp", sample_id, recognizedLabelTag(class_id));
        snprintf(image_path, sizeof(image_path), "%s/%s", SD_RECOG_DIR, image_name);
        if (access(image_path, F_OK) != 0) {
            found_slot = true;
            break;
        }
    }

    if (!found_slot) {
        Serial.println("[RECOG] no free image slot");
        return;
    }

    if (!writeRasterBufferToSD(image_path, recog_bmp)) {
        Serial.println("[RECOG] image save fail");
        return;
    }
}

void setupSDFiles() {
    FILE *fp = fopen(SD_LIST_PATH, "w");
    if (fp) {
        fprintf(fp, "char-0001.jpg\n");
        fclose(fp);
    }

    if (access(SD_JPG_PATH, F_OK) != 0) {
        fp = fopen(SD_JPG_PATH, "wb");
        if (fp) fclose(fp);
    }

    if (access(SD_STATE_PATH, F_OK) != 0) {
        fp = fopen(SD_STATE_PATH, "w");
        if (fp) {
            fputs("", fp);
            fclose(fp);
        }
    }

    if (access(SD_RECOG_DIR, F_OK) != 0) {
        if (mkdir(SD_RECOG_DIR, 0) == 0) {
        }
    }
}

bool initPAA() {
    static bool serial_begun = false;
    if (!serial_begun) {
        PAA_SERIAL.begin(PAA_BAUD);
        serial_begun = true;
        delay(500);
    }
    PAA_SERIAL.println("at-");
    delay(200);
    while (PAA_SERIAL.available()) PAA_SERIAL.read();
    PAA_SERIAL.println("at");
    if (!waitForPAAOK(1000, NULL)) {
        return false;
    }
    PAA_SERIAL.println("at0g+");
    waitForPAAOK(1000, NULL);
    PAA_SERIAL.println("atrpt+50");
    delay(200);
    PAA_SERIAL.println("atx+");
    delay(200);
    return true;
}

bool writeJpegToSD() {
    if (!writeJpegImageToSD(SD_JPG_PATH)) {
        Serial.println("[SD] open char.jpg fail");
        return false;
    }
    return true;
}

int waitForInference(uint32_t timeout_ms) {
    uint32_t t0 = millis();
    int last_len = -1;
    int stable_reads = 0;
    bool touched_json = false;

    // Let the SDK finish loading/decoding before we touch SD again.
    delay(700);

    while (millis() - t0 < timeout_ms) {
        if (access(SD_JSON_PATH, F_OK) != 0) {
            delay(250);
            continue;
        }

        FILE *fp = fopen(SD_JSON_PATH, "r");
        if (!fp) {
            delay(250);
            continue;
        }

        char buf[768];
        int len = (int)fread(buf, 1, sizeof(buf) - 1, fp);
        fclose(fp);
        touched_json = true;

        if (len <= 0) {
            last_len = -1;
            stable_reads = 0;
            delay(250);
            continue;
        }

        buf[len] = '\0';
        if (len == last_len) {
            stable_reads++;
        } else {
            last_len = len;
            stable_reads = 0;
            delay(250);
            continue;
        }

        if (stable_reads < 1) {
            delay(250);
            continue;
        }

        cJSON *root = cJSON_Parse(buf);
        if (!root) {
            delay(250);
            continue;
        }

        memcpy(recog_bmp, bmp, sizeof(bmp));

        int best_area_cls = -1;
        float best_area = -1.0f;
        int best_score_cls = -1;
        float best_score = -1.0f;
        int high_score_area_cls = -1;
        float high_score_area = -1.0f;
        int high_score_count = 0;
        int parsed_count = cJSON_GetArraySize(root);
        for (int i = 0; i < cJSON_GetArraySize(root); i++) {
            cJSON *item = cJSON_GetArrayItem(root, i);
            cJSON *cat  = cJSON_GetObjectItem(item, "category_id");
            cJSON *bbox = cJSON_GetObjectItem(item, "bbox");
            cJSON *score = cJSON_GetObjectItem(item, "score");
            // bbox 格式：[x, y, w, h]
            if (cat && bbox && cJSON_GetArraySize(bbox) >= 4) {
                int bx = (int)(cJSON_GetArrayItem(bbox, 0)->valuedouble + 0.5f);
                int by = (int)(cJSON_GetArrayItem(bbox, 1)->valuedouble + 0.5f);
                float dx   = (float)cJSON_GetArrayItem(bbox, 2)->valuedouble;
                float dy   = (float)cJSON_GetArrayItem(bbox, 3)->valuedouble;
                drawRectOnBuffer(recog_bmp, bx, by, (int)(dx + 0.5f), (int)(dy + 0.5f));
                float area = dx * dy;
                if (area > best_area) {
                    best_area = area;
                    best_area_cls = cat->valueint;
                }
                if (score && (float)score->valuedouble > best_score) {
                    best_score = (float)score->valuedouble;
                    best_score_cls = cat->valueint;
                }
                if (score && (float)score->valuedouble > 0.8f) {
                    high_score_count++;
                    if (area > high_score_area) {
                        high_score_area = area;
                        high_score_area_cls = cat->valueint;
                    }
                }
            }
        }
        cJSON_Delete(root);

        // 若有 2 個以上偵測框 score > 0.8，取其中 bbox 面積最大者
        int selected_cls;
        if (high_score_count > 1) {
            selected_cls = high_score_area_cls;
        } else {
            selected_cls = (best_score_cls >= 0) ? best_score_cls : best_area_cls;
        }

        debug_last_json_count = parsed_count;
        debug_last_best_area_cls = best_area_cls;
        debug_last_best_area = best_area;
        debug_last_best_score_cls = best_score_cls;
        debug_last_best_score = best_score;
        debug_last_json_checksum = 0;
        debug_last_json_len = len;

        return selected_cls;
    }

    if (!touched_json) {
        Serial.println("[NN] json not ready");
    }
    Serial.println("[NN] timeout");
    return -1;
}

void resetAfterRecognize() {
    delay(1000);
    sys_reset();
}

void onRecognize() {
    if (run_state == RUN_RECOGNIZING) return;

    debug_run_seq++;
    recognize_pending = false;
    run_state = RUN_RECOGNIZING;
    is_writing = false;
    first_pt_of_stroke = false;
    debug_last_json_count = -1;
    debug_last_best_area_cls = -1;
    debug_last_best_area = -1.0f;
    debug_last_best_score_cls = -1;
    debug_last_best_score = -1.0f;
    debug_last_json_checksum = 0;
    debug_last_json_len = -1;

    Serial.print("[REC] triggered, pt_count="); Serial.println(pt_count);
    if (pt_count < 2) {
        Serial.println("[REC] too few points");
        run_state = RUN_IDLE;
        return;
    }
    if (!autoFit(BMP_SIZE, BMP_PAD, fit_ctx)) {
        Serial.println("[REC] autoFit failed");
        run_state = RUN_IDLE;
        return;
    }

    renderToBmp(fit_ctx);
    memcpy(recog_bmp, bmp, sizeof(bmp));  // snapshot before inference, so rec BMP is never blank
    redrawStateScreen(true);

    if (!writeJpegToSD()) {
        resetAfterRecognize();
        return;
    }

    if (access(SD_JSON_PATH, F_OK) == 0) {
        remove(SD_JSON_PATH);
    }

    objDetImg.begin(filelist_name);
    int class_id = waitForInference(5000);
    if (class_id >= 0 && class_id < CALC_NUM_LABELS) {
        if (appendRecognizedToken(CALC_LABELS[class_id])) {
            saveCalcState();
        }
        // sec0 顯示辨識結果字元，其餘區塊顯示算式
        oled.dev.clearDisplay();
        oled.secBigChar(0, labelToDisplayChar(CALC_LABELS[class_id]));
        renderExprSections();
        oled.drawDividers();
        oled.dev.display();
    } else {
        Serial.println("[REC] no valid result");
        showNoRecognitionScreen();
        delay(1200);
        resetAfterRecognize();
        return;
    }

    resetAfterRecognize();
}

void processLine(const char *line) {
    if (!strstr(line, "X=")) return;

    auto parseField = [](const char *l, const char *key) -> float {
        const char *p = strstr(l, key);
        return p ? (float)atof(p + strlen(key)) : 0.0f;
    };

    float x = parseField(line, "X=");
    float y = parseField(line, "Y=");

    if (is_writing && pt_count < MAX_PTS) {
        raw_pts[pt_count] = {x, y};
        stroke_start[pt_count] = first_pt_of_stroke;
        pt_count++;
        first_pt_of_stroke = false;
    } else if (is_writing && debug_pts_dropped < 65535) {
        debug_pts_dropped++;
    }

    static uint32_t _oled_t = 0;
    if (millis() - _oled_t < 150) return;
    _oled_t = millis();

    oled.dev.clearDisplay();
    oled.drawSec0Preview(raw_pts, stroke_start, pt_count);

    if (!is_writing && pt_count > 0) {
        static bool _blink_on = true;
        static uint32_t _blink_t = 0;
        if (millis() - _blink_t >= 300) {
            _blink_on = !_blink_on;
            _blink_t = millis();
        }

        if (_blink_on) {
            float mn_x = raw_pts[0].x, mx_x = raw_pts[0].x;
            float mn_y = raw_pts[0].y, mx_y = raw_pts[0].y;
            for (int i = 1; i < pt_count; i++) {
                if (raw_pts[i].x < mn_x) mn_x = raw_pts[i].x;
                if (raw_pts[i].x > mx_x) mx_x = raw_pts[i].x;
                if (raw_pts[i].y < mn_y) mn_y = raw_pts[i].y;
                if (raw_pts[i].y > mx_y) mx_y = raw_pts[i].y;
            }
            float rw = mx_x - mn_x;
            float rh = mx_y - mn_y;
            float sc = (rw >= rh) ? (rw > 0 ? 30.0f / rw : 1.0f)
                                  : (rh > 0 ? 30.0f / rh : 1.0f);
            float cx = (mn_x + mx_x) * 0.5f;
            float cy = (mn_y + mx_y) * 0.5f;
            int px = (int)(16.0f + (x - cx) * sc + 0.5f);
            int py = (int)(16.0f - (y - cy) * sc + 0.5f);
            px = constrain(px, 1, 30);
            py = constrain(py, 1, 30);
            oled.dev.fillRect(px - 1, py - 1, 3, 3, SSD1306_WHITE);
        }
    }

    renderExprSections();
    oled.drawDividers();
    oled.dev.display();
}

void setup() {
    Serial.begin(115200);
    pinMode(BTN, INPUT_PULLUP);

    if (!oled.begin()) {
        Serial.println("[OLED] init fail");
        for (;;);
    }

    vfs_init(NULL);
    if (vfs_user_register("sd", VFS_FATFS, VFS_INF_SD) == 0) {
    } else {
        Serial.println("[VFS] SD mount fail");
    }

    setupSDFiles();
    loadCalcState();
    objDetImg.modelSelect(OBJECT_DETECTION, CUSTOMIZED_YOLOV4TINY, NA_MODEL, NA_MODEL); /*DEFAULT_YOLOV4TINY CUSTOMIZED_YOLOV4TINY SD_YOLOV4TINY*/

    bool paa_ready = initPAA();
    if (!paa_ready) {
        showStatusScreen("PAA ERROR", "CHECK SENSOR");
        Serial.println("[PAA] init failed");
    }
    clearStrokes();
    if (paa_ready) {
        redrawStateScreen(false);
    }
}

void loop() {
    handleButtonEvent(pollButton());

    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (cmd_len > 0) {
                cmd_buf[cmd_len] = '\0';
                PAA_SERIAL.println(cmd_buf);
                cmd_len = 0;
            }
        } else if (cmd_len < (int)sizeof(cmd_buf) - 1) {
            cmd_buf[cmd_len++] = c;
        }
    }

    static uint32_t _hb = 0;
    if (millis() - _hb > 5000) {
        _hb = millis();
    }

    static uint32_t _paa_last = 0;
    static bool _paa_warned = false;
    static bool _paa_init = false;
    if (!_paa_init) {
        _paa_last = millis();
        _paa_init = true;
    }

    while (PAA_SERIAL.available()) {
        handleButtonEvent(pollButton());
        _paa_last = millis();
        _paa_warned = false;
        char c = (char)PAA_SERIAL.read();
        if (c == '\n' || c == '\r') {
            if (uart_len > 0) {
                uart_buf[uart_len] = '\0';
                processLine(uart_buf);
                uart_len = 0;
            }
        } else if (uart_len < (int)sizeof(uart_buf) - 1) {
            uart_buf[uart_len++] = c;
        }
    }

    if (millis() - _paa_last > 3000) {
        if (!_paa_warned) {
            showStatusScreen("PAA PROCESSING", "RETRY...");
            Serial.println("[PAA] no data, retry init");
            _paa_warned = true;
        }
        while (PAA_SERIAL.available()) PAA_SERIAL.read();
        uart_len = 0;
        if (initPAA()) {
            redrawStateScreen(false);
            _paa_warned = false;
        } else {
            Serial.println("[PAA] retry failed");
        }

        _paa_last = millis();
    }

    if (recognize_pending && run_state != RUN_RECOGNIZING) {
        onRecognize();
    }
}

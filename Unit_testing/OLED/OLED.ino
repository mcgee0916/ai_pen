/*
 * OLED.ino — 三段動畫循環：星際超空間 → 3D線框 → 火焰模擬
 * 接線：AmebaPro2 預設 I2C 腳位
 */

#include <Wire.h>
#include <Adafruit_OLED_libraries/Adafruit_GFX.h>
#include <Adafruit_OLED_libraries/Adafruit_SSD1306.h>

#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   32
#define OLED_RESET      -1
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ═══════════════════════════════════════════════════════════════════
// A: 星際
// ═══════════════════════════════════════════════════════════════════
#define NUM_STARS 35

struct { float x, y, z, pz; } stars[NUM_STARS];

void spawnStar(int i) {
    stars[i].x  = (float)random(-64, 65);
    stars[i].y  = (float)random(-16, 17);
    stars[i].z  = (float)random(8, 129);
    stars[i].pz = stars[i].z;
}

void animStarfield(unsigned long ms) {
    for (int i = 0; i < NUM_STARS; i++) spawnStar(i);
    unsigned long t0 = millis();

    while (millis() - t0 < ms) {
        float progress = (float)(millis() - t0) / ms;
        float speed    = 2.0f + progress * 7.0f;   // 2 → 9，漸加速

        display.clearDisplay();
        for (int i = 0; i < NUM_STARS; i++) {
            stars[i].pz = stars[i].z;
            stars[i].z -= speed;
            if (stars[i].z <= 1.0f) {
                spawnStar(i);
                stars[i].z = stars[i].pz = 128.0f;
                continue;
            }
            int sx = (int)(stars[i].x / stars[i].z  * 64) + 64;
            int sy = (int)(stars[i].y / stars[i].z  * 64) + 16;
            int ox = (int)(stars[i].x / stars[i].pz * 64) + 64;
            int oy = (int)(stars[i].y / stars[i].pz * 64) + 16;
            if (sx >= 0 && sx < SCREEN_WIDTH && sy >= 0 && sy < SCREEN_HEIGHT)
                display.drawLine(ox, oy, sx, sy, SSD1306_WHITE);
        }
        display.display();
        delay(30);
    }
}

// ═══════════════════════════════════════════════════════════════════
// B: 3D 旋轉線框正方體 (Wireframe Cube)
// ═══════════════════════════════════════════════════════════════════
void animWireframe(unsigned long ms) {
    static const float base[8][3] = {
        {-1,-1,-1}, { 1,-1,-1}, { 1, 1,-1}, {-1, 1,-1},
        {-1,-1, 1}, { 1,-1, 1}, { 1, 1, 1}, {-1, 1, 1}
    };
    static const uint8_t edg[12][2] = {
        {0,1},{1,2},{2,3},{3,0},
        {4,5},{5,6},{6,7},{7,4},
        {0,4},{1,5},{2,6},{3,7}
    };

    float rx = 0.0f, ry = 0.0f, rz = 0.0f;
    unsigned long t0 = millis();

    while (millis() - t0 < ms) {
        float cx = cos(rx), sx_ = sin(rx);
        float cy = cos(ry), sy_ = sin(ry);
        float cz = cos(rz), sz_ = sin(rz);

        float v[8][3];
        for (int i = 0; i < 8; i++) {
            float x = base[i][0], y = base[i][1], z = base[i][2];
            // Rotate X
            float y1 = y*cx - z*sx_,  z1 = y*sx_ + z*cx;
            y = y1; z = z1;
            // Rotate Y
            float x2 = x*cy + z*sy_,  z2 = -x*sy_ + z*cy;
            x = x2; z = z2;
            // Rotate Z
            float x3 = x*cz - y*sz_,  y3 = x*sz_ + y*cz;
            v[i][0] = x3; v[i][1] = y3; v[i][2] = z;
        }

        int px[8], py[8];
        for (int i = 0; i < 8; i++) {
            float s = 35.0f / (v[i][2] + 3.5f);
            px[i] = (int)(v[i][0] * s + 64.5f);
            py[i] = (int)(v[i][1] * s + 16.5f);
        }

        display.clearDisplay();
        for (int i = 0; i < 12; i++)
            display.drawLine(px[edg[i][0]], py[edg[i][0]],
                             px[edg[i][1]], py[edg[i][1]], SSD1306_WHITE);
        display.display();

        rx += 0.03f; ry += 0.05f; rz += 0.02f;
        delay(25);
    }
}

// ═══════════════════════════════════════════════════════════════════
// C: 火焰模擬 (Fire Simulation + Bayer Dithering)
// ═══════════════════════════════════════════════════════════════════
#define FIRE_W SCREEN_WIDTH
#define FIRE_H (SCREEN_HEIGHT + 2)   // 底 2 列為螢幕外火種

uint8_t fireGrid[FIRE_H][FIRE_W];

// 4×4 Bayer 矩陣：讓黑白螢幕模擬火焰濃淡灰階
static const uint8_t bayer4[4][4] = {
    {  0, 128,  32, 160 },
    {192,  64, 224,  96 },
    { 48, 176,  16, 144 },
    {240, 112, 208,  80 }
};

void animFire(unsigned long ms) {
    memset(fireGrid, 0, sizeof(fireGrid));
    unsigned long t0 = millis();

    while (millis() - t0 < ms) {
        // 點燃底部兩列（螢幕外）
        for (int x = 0; x < FIRE_W; x++) {
            fireGrid[FIRE_H-1][x] = (uint8_t)random(180, 256);
            fireGrid[FIRE_H-2][x] = (uint8_t)random(120, 256);
        }
        // 向上傳播：取下方三鄰值平均後隨機冷卻
        for (int y = 0; y < FIRE_H - 2; y++) {
            for (int x = 0; x < FIRE_W; x++) {
                int sum = (int)fireGrid[y+1][(x-1+FIRE_W) % FIRE_W]
                        + (int)fireGrid[y+1][x]
                        + (int)fireGrid[y+1][(x+1) % FIRE_W]
                        + (int)fireGrid[y+2][x];
                int val = (sum >> 2) - random(3, 9);
                fireGrid[y][x] = (uint8_t)(val < 0 ? 0 : val);
            }
        }
        // Bayer 抖動渲染（上 32 列輸出到螢幕）
        display.clearDisplay();
        for (int y = 0; y < SCREEN_HEIGHT; y++)
            for (int x = 0; x < FIRE_W; x++)
                if (fireGrid[y][x] > bayer4[y & 3][x & 3])
                    display.drawPixel(x, y, SSD1306_WHITE);
        display.display();
        delay(30);
    }
}

// ═══════════════════════════════════════════════════════════════════
// D: 彈跳方塊
// ═══════════════════════════════════════════════════════════════════
void animBounce(unsigned long ms) {
    int bx = 0, by = 0, vx = 2, vy = 1;
    const int BS = 8;
    unsigned long t0 = millis();

    while (millis() - t0 < ms) {
        display.clearDisplay();
        bx += vx; by += vy;
        if (bx <= 0 || bx >= SCREEN_WIDTH  - BS) vx = -vx;
        if (by <= 0 || by >= SCREEN_HEIGHT - BS) vy = -vy;
        display.fillRect(bx, by, BS, BS, SSD1306_WHITE);
        display.setTextSize(1);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.print("OLED OK!");
        display.display();
        delay(30);
    }
}

// ═══════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    randomSeed(analogRead(0));

    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println("OLED FAIL");
        for (;;);
    }
    Serial.println("OLED OK");
    display.clearDisplay();
    display.display();
}

void loop() {
    animStarfield(5000);
    animWireframe(5000);
    animFire(5000);
    animBounce(5000);
}

# AI PEN — 智慧軌跡筆
<img width="896" height="1195" alt="Gemini_Generated_Image_a84ghia84ghia84g" src="https://github.com/user-attachments/assets/d8fe6427-a152-48f7-a4f6-c781f322b7df" />

以 YOLOv4-Tiny 為推論核心的手寫數學式辨識裝置。使用者以實體筆在空中或紙上書寫數字與運算符號，裝置即時辨識並在 OLED 顯示計算結果。

\---

## 硬體規格

|元件|說明|
|-|-|
|MCU|HUB-8735Ultra|
|軌跡感測器|PAA5163D 光學感測器（Serial3，115200 baud）|
|顯示器|SSD1306 OLED（I2C）|
|按鈕|Pin 15，INPUT\_PULLUP|
|儲存|SD 卡（VFS FAT）|
|神經網路|NNObjectDetectionImage（板載 NPU）|

3D 列印外殼檔案：`ai\\\_pan\\\_frame.stl`

\---

## 辨識類別（14 類）

|類別|標籤|
|-|-|
|數字|`0` `1` `2` `3` `4` `5` `6` `7` `8` `9`|
|運算子|`add` (+)、`sub` (-)、`mul` (×)、`div` (÷)|

\---

## 目錄結構

```
.
├── ai\\\_pan/                  ← 主韌體（辨識 + 計算機）
│   ├── ai\\\_pan.ino
│   └── OledDisplay.h
├── ai\\\_pan\\\_collect/          ← 資料收集韌體（建立訓練集用）
│   ├── ai\\\_pan\\\_collect.ino
│   └── OledDisplay.h
├── NN\\\_MDL/
│   └── yolov4\\\_tiny.nb       ← 編譯完成的模型權重（燒錄至 SD 卡）
├── Unit\\\_testing/            ← 元件單元測試草稿
│   ├── BTN/                 ← 按鈕測試
│   ├── OLED/                ← OLED 顯示測試
│   ├── OLED\\\_BTN/            ← OLED + 按鈕整合測試
│   ├── PAA5163D/            ← 光學感測器測試
│   ├── ALL/                 ← 全元件整合測試
│   ├── HUB-PAA5163D\\\_Tracker.exe           ← Windows 軌跡可視化工具
│   └── HUB-PAA5163D\\\_路徑測試程式使用說明.pdf
├── YOLOv4Tiny\\\_AutoTrain.ipynb  ← Google Colab 訓練 Notebook
├── ai\\\_pan\\\_frame.stl            ← 外殼 3D 列印檔
└── AI-PEN.pptx                 ← 專案簡報
```

\---

## 環境建置

### 步驟 1：安裝 Arduino IDE

前往 [arduino.cc/en/software](https://www.arduino.cc/en/software) 下載並安裝 **Arduino IDE 1.8.19 或 2.x**。

### 步驟 2：新增開發板來源 URL

開啟 Arduino IDE → **檔案 → 偏好設定**，在「額外的開發板管理員網址」貼上：

```
https://github.com/ideashatch/HUB-8735/raw/main/amebapro2\_arduino/Arduino\_package/ideasHatch.json
```

點選 OK 儲存。

### 步驟 3：安裝開發板套件

**工具 → 開發板 → 開發板管理員**，搜尋 `ideasHatch` 或 `8735`，安裝 **HUB8735ULTRA**。
出現 `Platform ideasHatch:AmebaPro2@4.1.1-Release installed` 即完成。

### 步驟 4：安裝函式庫

**工具 → 管理函式庫**，分別搜尋並安裝：

|函式庫|用途|
|-|-|
|JPEGENC|AI辨識影像處理|

> `cJSON` 與 `NNObjectDetectionImage` 已包含在 ideasHatch SDK 中，無需額外安裝。

### 步驟 5：連接裝置與設定 COM Port

1. 用 USB Type-C 線連接 HUB 8735 Ultra 與電腦
2. **工具 → 開發板** 選擇 `HUB-8735\_ultra`（AmebaPro2 ARM 32-bits Boards - ideasHatch）
3. **工具 → 序列埠** 選擇對應 COM Port（Windows 裝置管理員確認 USB-SERIAL 編號）

\---

## 快速開始

### 1\. 準備 SD 卡

1. 將 SD 卡格式化為 **FAT32**
2. 將 `NN\_MDL\\yolov4\_tiny.nb` 連同資料夾複製到 SD 卡**根目錄**（路徑：`sd:NN\_MDL\\yolov4\_tiny.nb`）
3. 插入 HUB 8735 Ultra 的 SD 卡槽

### 2\. 硬體接線

|元件|元件接腳|HUB 8735 Ultra 接腳|
|-|-|-|
|HUB-PAA5163D|UART TX|Pin 5 (SERIAL3\_RX)|
|HUB-PAA5163D|UART RX|Pin 6 (SERIAL3\_TX)|
|HUB-PAA5163D|5V|5V|
|HUB-PAA5163D|GND|GND|
|SSD1306 OLED|SDA|Pin 0 (I2C1 SDA)|
|SSD1306 OLED|SCL|Pin 1 (I2C1 SCL)|
|SSD1306 OLED|VCC|5V|
|SSD1306 OLED|GND|GND|
|按鈕|S|Pin 15|
|按鈕|VCC|中間腳|
|按鈕|GND|GND|

### 3\. 燒錄韌體

開啟 `ai\\\_pan/ai\\\_pan.ino`，先讓裝置進入 Bootloader 模式再上傳：

**方法 A（按鍵操作）：**

1. USB Type-C 連接電腦
2. 長按「功能鍵」（不放開）
3. 按一下「Reset 鍵」後放開
4. 放開「功能鍵」→ 裝置進入 Bootloader 模式

**方法 B（短路 BOOT 引腳）：**

1. 短接 `BOOT\_MODE`（Pin 14）與 `BOOT\_V3P3`（3.3V）
2. 按下 Reset 後放開，再移除跳線

進入 Bootloader 後，點選 **上傳（Ctrl+U）**，輸出視窗出現 `Start Upload Flash → End Upload Flash` 即成功。上傳完成後按 Reset 重啟。

### 4\. 按鈕操作

|手勢|功能|
|-|-|
|長按（≥300 ms）按住|開始書寫軌跡|
|長按放開|結束書寫|
|單擊|清除目前筆畫|
|雙擊|觸發辨識|
|三擊|清除整條算式|
|四擊|重新初始化 PAA5163D|

### 4\. 書寫流程

1. **長按** → 書寫一個字元 → **放開**
2. 確認 OLED 預覽正確後 **雙擊** 觸發辨識
3. 辨識結果自動附加至算式；算式完整（數字＋運算子＋數字）時立即計算並顯示結果
4. **三擊**清除算式，重新開始

\---

## 訓練自訂模型

使用 `YOLOv4Tiny\\\_AutoTrain.ipynb`（需 Google Colab GPU）：

1. 在 Google Drive 建立 `yolov4\\\_tiny\\\_training/dataset/` 資料夾
2. 上傳圖片與對應 YOLO 格式 `.txt` 標註（`class\\\_id cx cy w h`，相對比例）
3. 在 Notebook 第一個 Cell 設定類別名稱與超參數
4. 依序執行所有 Cell；最佳權重輸出至 `yolov4\\\_tiny\\\_training/backup/`
5. 將 `.weights` 轉為 `.nb` 格式後複製至 SD 卡

\---

## 元件單元測試

開發或除錯時可先燒錄對應的單元測試草稿，確認硬體功能正常：

|草稿|測試目標|
|-|-|
|`Unit\\\_testing/BTN/BTN.ino`|按鈕長按 / 雙擊事件|
|`Unit\\\_testing/OLED/OLED.ino`|OLED 顯示輸出|
|`Unit\\\_testing/OLED\\\_BTN/OLED\\\_BTN.ino`|OLED + 按鈕整合|
|`Unit\\\_testing/PAA5163D/PAA5163D.ino`|PAA5163D 軌跡串流|
|`Unit\\\_testing/ALL/ALL.ino`|全元件整合|

Windows 上可用 `HUB-PAA5163D\\\_Tracker.exe` 將 PAA5163D 的 X/Y 串流可視化，確認感測器路徑是否正確。

\---

## SD 卡檔案說明

裝置啟動時會自動建立以下檔案：

|路徑|用途|
|-|-|
|`sd:/char-0001.jpg`|最近一次書寫的 JPEG 圖（推論輸入）|
|`sd:/char-0001.json`|NPU 輸出的偵測結果 JSON|
|`sd:/char\\\_list.txt`|推論圖片清單|
|`sd:/calc\\\_state.txt`|算式狀態（斷電保留）|
|`sd:/recognized/`|每次辨識後的 BMP 快照（用於資料收集）|

\---

## 開發環境

* Arduino IDE（含 Ameba RTL8720 BSP）
* &#x20;Google Colab（模型訓練）
* 相依函式庫：`JPEGENC`、`NNObjectDetectionImage`、`cJSON`、`SSD1306`


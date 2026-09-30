# Unit CamS3-5MP Companion Computer for StampFly

StampFly (ArduPilot FC) 用のコンパニオンコンピュータ（CC）ファームウェアです。
M5Stack Unit CamS3-5MP 上で動作し、**リアルタイム FPV 映像ストリーミング** と **MAVLink テレメトリ双方向ブリッジ** を提供します。

---

## 📐 システム構成

```text
[ Game Controller ]
       │ (Bluetooth)
       ▼
 [ SmartPhone ] ──(Wi-Fi: UDP 14550)──▶ [ Unit CamS3-5MP (CC) ] ──(UART: G43/G44)──▶ [ StampFly (FC) ]
(Mission Planner) ◀──(Wi-Fi: HTTP /stream)─┘   (PY260 Camera)
```

- **Wi-Fi SoftAP**: SSID `StampFly-Cam` (IP: `192.168.4.1`)
- **MAVLink Bridge**: UART (G43/G44 @ 115200bps) ⇆ Wi-Fi (UDP 14550) [Core 1]
- **Video Stream**: 5MP センサー (PY260) による MJPEG ストリーミング [Core 0]
  - URL: `http://192.168.4.1/stream` (Mission Planner / QGC / ブラウザ対応)
  - Webプレビュー: `http://192.168.4.1/`
  - 静止画キャプチャ: `http://192.168.4.1/capture`
- **コンソール / 書き込み**: Grove ポート経由の USB-Serial-JTAG (GPIO 19/20)

---

## 🔌 配線（Pinout）

### 1. CamS3 ⇆ StampFly (FC) 配線
ピンヘッダ（右上）の `G43`, `G44`, `GND` を StampFly の Grove Black (SERIAL3) に接続します。

| Unit CamS3-5MP | 信号 | StampFly (Grove Black / SERIAL3) |
| :--- | :--- | :--- |
| **G44** | RXD | **TX (GPIO 2)** |
| **G43** | TXD | **RX (GPIO 1)** |
| **G** | GND | **GND** |
| *(5V)* | *(5V)* | *※フライト時のみ給電（PC接続時は未結線推奨）* |

### 2. PC 接続（プログラム書き込み・デバッグ）
CamS3 本体の **Grove ポート** に `Grove to USB-C` ケーブルを挿して PC と接続します。
- ESP32-S3 のネイティブ USB-Serial-JTAG として認識されます。
- G43/G44 の配線は外す必要はありません（常時挿したままで OK）。

---

## 🛠️ ビルド＆書き込み方法

※ESP-IDF v5.1 以降を推奨します。

```bash
cd /Volumes/work2/unitcams3-5mp

# 1. ターゲットを esp32s3 に設定（初回のみ）
idf.py set-target esp32s3

# 2. ビルド
idf.py build

# 3. 書き込み & シリアルモニタ
idf.py -p /dev/cu.usbmodem* flash monitor
```

---

## 📱 接続手順（フライト時）

1. **CamS3 の電源を投入**
   - 青色 LED がゆっくり点滅（クライアント接続待ち）。
2. **スマホから Wi-Fi 接続**
   - SSID: `StampFly-Cam`（パスワードなし）に接続。
   - 接続が完了すると、CamS3 の青色 LED が点灯状態に変わります。
3. **ブラウザで確認**
   - スマホのブラウザで `http://192.168.4.1/` を開くと、リアルタイムカメラ映像が表示されます。
4. **Mission Planner の設定**
   - **テレメトリ接続**:
     - 通信形式: `UDP`
     - ポート: `14550`
     - 「Connect」を押すと、StampFly の姿勢・高度・バッテリー・フライトモードが同期されます。
   - **カメラ映像（HUD背景）**:
     - 設定 > ビデオソース: `MJPEG Stream`
     - URL: `http://192.168.4.1/stream`
   - ゲームコントローラーで StampFly を快適に FPV 操縦できます！

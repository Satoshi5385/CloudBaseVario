# VARIO HW仕様書 — ESP32-S3 GPIO／周辺インターフェース

## 1. 適用範囲

ESP32-S3-WROOM-1-N16R8 を搭載したバリオメーターに適応する

適応機種
- Aohazuku(Rev.0)

---

## 2. GPIO割り当て一覧

### 2.1 使用GPIO

| GPIO | 回路図信号名 | SW定義名 | 用途 | 方向 | アクティブレベル／備考 |
|---:|---|---|---|---|---|
| 0 | `BOOTMODE` | `PIN_BOOTMODE` | ダウンロードモード選択 | 入力 | Lowでダウンロードモード。通常動作中の用途には使用しない |
| 1 | `BAT_ADC` | `PIN_BAT_ADC` | バッテリー電圧測定 | アナログ入力 | ADC入力 |
| 2 | `SW_2` | `PIN_SW_2` | 操作スイッチ2 | 入力 | 押下時Low、外付けプルアップ |
| 4 | `I2C_SDA` | `PIN_I2C_SDA` | I2Cデータ | 双方向 | 外付け2.2kΩプルアップ |
| 5 | `I2C_SCL` | `PIN_I2C_SCL` | I2Cクロック | 双方向 | 外付け2.2kΩプルアップ |
| 6 | `DP_SCLK` | `PIN_LCD_SCLK` | Sharp Memory LCD クロック | 出力 | SPI相当 |
| 7 | `DP_SI` | `PIN_LCD_SI` | Sharp Memory LCD データ | 出力 | SPI相当 |
| 8 | `DP_SCS` | `PIN_LCD_SCS` | Sharp Memory LCD チップセレクト | 出力 | LCDドライバ仕様に従う |
| 9 | `SW_3` | `PIN_SW_3` | 操作スイッチ3 | 入力 | 押下時Low、外付けプルアップ |
| 10 | `SD_CS` | `PIN_SD_CS` | microSD チップセレクト | 出力 | Lowアクティブ |
| 11 | `SD_MOSI` | `PIN_SD_MOSI` | microSD CMD／MOSI | 出力 | SPI |
| 12 | `SD_CLK` | `PIN_SD_CLK` | microSD クロック | 出力 | SPI |
| 13 | `SD_MISO` | `PIN_SD_MISO` | microSD DAT0／MISO | 入力 | SPI |
| 14 | `INT_ICM` | `PIN_INT_ICM` | ICM-42688 INT1 | 入力 | 割り込み極性はセンサ設定に依存 |
| 15 | `BUZZER_MODE1` | `PIN_BUZZER_MODE1` | PAM8904E EN1 | 出力 | Highアクティブ、外付け100kΩプルダウン |
| 16 | `LED_2` | `PIN_LED_2` | 黄LED | 出力 | Lowで点灯、Highで消灯 |
| 17 | `GPS_RX` | `PIN_GPS_UART_TX` | GPS受信端子へのUART送信 | 出力 | ESP32側UART1 TX。L96-M33 RXD1へ接続 |
| 18 | `GPS_TX` | `PIN_GPS_UART_RX` | GPS送信端子からのUART受信 | 入力 | ESP32側UART1 RX。L96-M33 TXD1から入力 |
| 19 | `USB_DN` | `PIN_USB_DN` | USB D− | USB | 通常GPIOとして使用しない |
| 20 | `USB_DP` | `PIN_USB_DP` | USB D+ | USB | 通常GPIOとして使用しない |
| 21 | `INT_BMP` | `PIN_INT_BMP` | BMP581 INT | 入力 | 割り込み極性はセンサ設定に依存 |
| 38 | `DP_DISP` | `PIN_LCD_DISP` | Sharp Memory LCD 表示有効 | 出力 | DISP制御 |
| 39 | `GPS_IO` | `PIN_GPS_PPS` | GPS TIMEPULSE／PPS | 入力 | 時刻同期、測位タイミング取得 |
| 40 | `BUZZER_PWM` | `PIN_BUZZER_PWM` | PAM8904E DIN | 出力 | ブザー駆動波形を出力。停止時はLow |
| 41 | `SD_DET` | `PIN_SD_DET` | microSD挿入検出 | 入力 | カード挿入時Low、外付けプルアップ |
| 42 | `PWR_EXT` | `PIN_PWR_EXT` | USB外部電源検出 | 入力 | USB VBUSありでHigh |
| 43 | `LED_1` | `PIN_LED_1` | 緑LED | 出力 | Lowで点灯、Highで消灯。既定機能はU0TXD |
| 44 | `BUZZER_MODE2` | `PIN_BUZZER_MODE2` | PAM8904E EN2 | 出力 | Highアクティブ、外付け100kΩプルダウン。既定機能はU0RXD |
| 47 | `PWR_HOLD` | `PIN_PWR_HOLD` | 電源自己保持 | 出力 | Highで電源保持 |
| 48 | `SW_1` | `PIN_SW_1` | 電源／操作スイッチ状態 | 入力 | 電源ラッチ回路を介したスイッチ入力 |

### 2.2 未使用または使用禁止GPIO

| GPIO | 扱い | 理由／注意事項 |
|---:|---|---|
| 3 | 未使用 | ストラップ端子のため、現行基板では使用しない |
| 35 | 使用禁止 | N16R8のOctal PSRAMで使用 |
| 36 | 使用禁止 | N16R8のOctal PSRAMで使用 |
| 37 | 使用禁止 | N16R8のOctal PSRAMで使用 |
| 45 | 未使用 | ストラップ端子のため、現行基板では使用しない |
| 46 | 未使用 | ストラップ端子のため、現行基板では使用しない |

---

## 3. ソフトウェア端子定義名

以下の名称をソフトウェア内のGPIO定義名として使用する。

### 3.1 電源・スイッチ

| SW定義名 | GPIO | 用途 |
|---|---:|---|
| `PIN_BOOTMODE` | 0 | ブートモード選択 |
| `PIN_BAT_ADC` | 1 | バッテリー電圧ADC |
| `PIN_SW_2` | 2 | 操作スイッチ2 |
| `PIN_SW_3` | 9 | 操作スイッチ3 |
| `PIN_SD_DET` | 41 | microSD挿入検出 |
| `PIN_PWR_EXT` | 42 | USB外部電源検出 |
| `PIN_PWR_HOLD` | 47 | 電源自己保持 |
| `PIN_SW_1` | 48 | 電源／操作スイッチ |

### 3.2 LED・ブザー

| SW定義名 | GPIO | 用途 |
|---|---:|---|
| `PIN_LED_1` | 43 | 緑LED |
| `PIN_LED_2` | 16 | 黄LED |
| `PIN_BUZZER_PWM` | 40 | PAM8904E DIN |
| `PIN_BUZZER_MODE1` | 15 | PAM8904E EN1 |
| `PIN_BUZZER_MODE2` | 44 | PAM8904E EN2 |

### 3.3 I2C・センサ

| SW定義名 | GPIO | 用途 |
|---|---:|---|
| `PIN_I2C_SDA` | 4 | I2C SDA |
| `PIN_I2C_SCL` | 5 | I2C SCL |
| `PIN_INT_ICM` | 14 | ICM-42688 INT1 |
| `PIN_INT_BMP` | 21 | BMP581 INT |

### 3.4 GPS

| SW定義名 | GPIO | 用途 |
|---|---:|---|
| `PIN_GPS_UART_TX` | 17 | ESP32 TX → GPS RXD1 |
| `PIN_GPS_UART_RX` | 18 | GPS TXD1 → ESP32 RX |
| `PIN_GPS_PPS` | 39 | GPS TIMEPULSE／PPS |

### 3.5 microSD

| SW定義名 | GPIO | 用途 |
|---|---:|---|
| `PIN_SD_CS` | 10 | SPI CS |
| `PIN_SD_MOSI` | 11 | SPI MOSI／CMD |
| `PIN_SD_CLK` | 12 | SPI CLK |
| `PIN_SD_MISO` | 13 | SPI MISO／DAT0 |
| `PIN_SD_DET` | 41 | カード挿入検出 |

### 3.6 Sharp Memory LCD

| SW定義名 | GPIO | 用途 |
|---|---:|---|
| `PIN_LCD_SCLK` | 6 | シリアルクロック |
| `PIN_LCD_SI` | 7 | シリアルデータ |
| `PIN_LCD_SCS` | 8 | チップセレクト |
| `PIN_LCD_DISP` | 38 | 表示有効制御 |

### 3.7 USB

| SW定義名 | GPIO | 用途 |
|---|---:|---|
| `PIN_USB_DN` | 19 | USB D− |
| `PIN_USB_DP` | 20 | USB D+ |

---

## 4. ブザー制御仕様

### 4.1 接続

| PAM8904E端子 | 回路図信号名 | SW定義名 | GPIO | 用途 |
|---|---|---|---:|---|
| DIN | `BUZZER_PWM` | `PIN_BUZZER_PWM` | 40 | ブザー駆動波形 |
| EN1 | `BUZZER_MODE1` | `PIN_BUZZER_MODE1` | 15 | チャージポンプモード選択 |
| EN2 | `BUZZER_MODE2` | `PIN_BUZZER_MODE2` | 44 | チャージポンプモード選択 |

### 4.2 動作モード

| DIN | EN1 | EN2 | PAM8904E動作 |
|---:|---:|---:|---|
| 0 | X | X | シャットダウン |
| 1 | 0 | 0 | シャットダウン |
| 1 | 0 | 1 | 1倍モード |
| 1 | 1 | 0 | 2倍モード |
| 1 | 1 | 1 | 3倍モード |

---

## 5. LED制御仕様

### 5.1 接続

| 信号名 | SW定義名 | 色 | GPIO | 点灯論理 |
|---|---|---|---:|---|
| `LED_1` | `PIN_LED_1` | 緑 | 43 | Lowで点灯、Highで消灯 |
| `LED_2` | `PIN_LED_2` | 黄 | 16 | Lowで点灯、Highで消灯 |

LEDは3.3V側から電流制限抵抗およびLEDを通してGPIOへ接続されており、GPIOが電流を吸い込む構成である。

### 5.2 GPIO43使用時の注意

GPIO43の既定機能はUART0 TXである。ROM起動ログまたはUART0ログ出力により、起動時に緑LEDが一時的に点滅する場合がある。

この一時点滅は許容動作とする。

アプリケーション初期化後は、GPIO43を通常のGPIO出力へ設定し、初期値をHighとしてLEDを消灯すること。

アプリケーション稼働中のUART0使用は禁止する。

---

## 6. 推奨GPIO初期値

| SW定義名 | GPIO | 初期出力値 | 初期状態 |
|---|---:|---:|---|
| `PIN_LED_1` | 43 | High | 消灯 |
| `PIN_LED_2` | 16 | High | 消灯 |
| `PIN_BUZZER_PWM` | 40 | Low | 発音停止 |
| `PIN_BUZZER_MODE1` | 15 | Low | シャットダウン設定 |
| `PIN_BUZZER_MODE2` | 44 | Low | シャットダウン設定 |
| `PIN_PWR_HOLD` | 47 | High | 電源保持 |

`PIN_PWR_HOLD`は起動後できるだけ早い段階でHighに設定すること。

電源OFF処理を行う場合は、必要なデータ保存および周辺停止処理を完了した後にLowへ変更する。

---

## 7. ソフトウェア設計上の制約

1. GPIO43をUART0の通常ログ出力として使用しない。
1. GPIO44をUART0 RXとして使用しない。
1. デバッグ、書き込み、コンソール出力には原則としてUSB Serial/JTAGまたはUSB CDCを使用する。
1. ブザー初期化前にGPIO40から駆動波形を出力しない。
1. GPIO35、GPIO36、GPIO37をソフトウェアから初期化しない。
1. LED制御はLowアクティブとして実装する。
1. GPSの回路図信号名はGPS側を基準としているため、ソフトウェアでは以下の方向として扱う。
   - `GPS_RX`：ESP32 TX → GPS RXD1
   - `GPS_TX`：GPS TXD1 → ESP32 RX

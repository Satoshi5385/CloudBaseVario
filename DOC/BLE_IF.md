# BLE搭載バリオメーター インターフェース仕様書 (XCTrack連携用)

本書は、自作バリオメーター（以下、デバイス）からAndroid用フライトアプリケーション「XCTrack」へ、Bluetooth Low Energy (BLE) を用いてデータを送信するためのインターフェース仕様を定義します。

---

## 1. 通信レイヤー（BLE仕様）

通信には、シリアル通信をエミュレートする **Nordic UART Service (NUS)** を使用し、デバイスからXCTrackに対してデータをプッシュ（Notify）します。

### 1.1. BLEプロファイル構成

* **デバイスロール (Role):** BLEペリフェラル (Peripheral)
* **ATT MTU:** 23バイト以上。MTU negotiationの成功を前提とせず、1センテンスが `ATT_MTU - 3` を超える場合はNUSのbyte streamとして複数Notifyへ分割します。

### 1.2. サービス・キャラクタリスティック定義

| 項目 | 識別名 | UUID | プロパティ | 備考 |
| --- | --- | --- | --- | --- |
| **サービス** | Nordic UART Service | `6e400001-b5a3-f393-e0a9-e50e24dcca9e` | - | 基盤となるシリアル通信サービス |
| **特性 (Char)** | TX Characteristic | `6e400003-b5a3-f393-e0a9-e50e24dcca9e` | **Notify** | デバイスからXCTrackへデータを送信 |
| **特性 (Char)** | RX Characteristic | `6e400002-b5a3-f393-e0a9-e50e24dcca9e` | Write / Write Without Response | NUS互換のため公開。受信byte列は解釈せず破棄する |
| **サービス** | Battery Service | `0000180f-0000-1000-8000-00805f9b34fb` | - | デバイスのバッテリー残量と電源状態 |
| **特性 (Char)** | Battery Level | `00002a19-0000-1000-8000-00805f9b34fb` | Read / Notify | 3.0～4.1 Vを0～100 %へ線形換算した残量 |
| **特性 (Char)** | Battery Level Status | `00002bed-0000-1000-8000-00805f9b34fb` | Read / Notify | USB外部電源と充電状態をGSS形式で公開 |

### 1.3. Battery Service

Battery Levelは、100 ms周期で得る5点中央値のうち有効な値だけを30秒間収集し、その区間の最低電圧で表示値を30秒ごとに更新します。最初の有効値は即時に表示し、ADC値が一時的に無効な場合は前回表示値を保持します。起動後に有効値を一度も取得していない場合は0 %とします。表示電圧の3.0 Vを0 %、4.1 Vを100 %として線形換算し、最も近い整数へ丸めて0～100 %に制限するため、4.1 V以上は100 %です。Readは常に最新の表示値を返し、有効な換算値が変化した場合だけNotifyします。

Battery Level StatusはFlags 1 byteとPower State 2 byteをlittle-endianで送信し、Identifier、Battery LevelおよびAdditional Statusの各optional fieldは含めません。内蔵バッテリーは常にPresent、Wireless External Power SourceはNo、Battery Charge LevelとCharging TypeはUnknown、Charging Fault Reasonはなしとします。

| `PIN_PWR_EXT` | 解釈 | Power State | Battery Level Status値 |
| ---: | --- | --- | --- |
| Low | USB外部電源なし、放電中 | Battery Present + Discharging: Active | `00 41 00` |
| High | USB外部電源あり、充電中 | Battery Present + Wired External Power Source Connected + Charging | `00 23 00` |

Battery Level StatusはReadに常に最新値を返し、Power Stateが変化した場合だけNotifyします。満充電や充電電流は検出せず、`PIN_PWR_EXT`がHighである間はChargingとして公開します。Battery Serviceの更新はNUS TXの購読状態およびLK8EX1の送信可否に依存しません。

---

## 2. アプリケーションレイヤー（データプロトコル）

データフォーマットには、XCSoarやLK8000等のフライト互換フォーマットである **`$LK8EX1`** センテンスを採用します。

### 2.1. センテンス基本フォーマット

データはASCIIテキストとして生成し、末尾は `\r\n` (CRLF) で区切ります。

```text
$LK8EX1,raw_pressure,altitude,vario,temperature,battery,*checksum\r\n

```

### 2.2. フィールド詳細仕様

| 項番 | フィールド名 | データ型 | 単位 | 必須値 / 無効値 | 説明 |
| --- | --- | --- | --- | --- | --- |
| 1 | `raw_pressure` | 整数 | Pa | `999999` | センサーから取得した気圧をPa単位で送信する。例：1013.25 hPaは `101325`。気圧が無効な場合は `999999`。 |
| 2 | `altitude` | 整数 | m | `99999` | 気圧高度。XCTrack側で気圧から高度を自動計算させる場合は、無効値 `99999` で固定。 |
| 3 | `vario` | 整数 | **cm/s** | `9999` | **昇降速度。** XCTrackの昇降率表示に使用する。<br>

<br>・上昇 1.5m/s $\rightarrow$ `150`<br>

<br>・下降 0.8m/s $\rightarrow$ `-80`<br>

<br>・無効時は `9999` |
| 4 | `temperature` | 整数 | °C | `99` | 常に `99` で固定。BMP581の測定温度は送信しない。 |
| 5 | `battery` | 小数または整数 | Vまたは% | `999` | Battery Serviceと共通の30秒最低表示値を使用する。`bluetooth_battery_mode`が`VOLTAGE`ならV単位・小数点以下2桁で送信する。`PERCENT`なら3.0～4.1 Vの換算値へLK8EX1規定の1000を加え、0 %を`1000`、100 %を`1100`として送信する。最初の有効値を取得する前は `999`。 |
| 6 | `*checksum` | 16進数 | - | 必須 | `*`に続く2桁の16進数（大文字）。データの整合性検証用。 |

LK8EX1には充電状態を示す標準フィールドがないため、独自フィールドは設けません。充電状態はBattery Level Statusだけで公開し、LK8EX1のbatteryフィールドの表記だけを`bluetooth_battery_mode`で選択します。

### 2.3. チェックサム（Checksum）の計算規則

* 計算対象文字列：`$` と `*` の間にあるすべての文字（カンマを含む）。
* 計算方法：対象文字列の各文字のASCIIコードに対して、順次 **XOR（排他的論理和）** を行います。
* 出力形式：計算結果を2桁の16進数テキスト（大文字、必要に応じてゼロ埋め）に変換します。

---

## 3. 運用・パフォーマンス要件

* **データ送信頻度 (Update Rate):** `setting.json`の共有設定`bluetooth_notify_rate_hz`で1～50 Hzに設定します。既定値は10 Hzです。
※BLEがbusyの場合はその周期のセンテンスを破棄し、再送や追いつき連送は行いません。このため、実際の成功Notify数は設定値を下回ることがあります。
* **データ送信時の注意点:**
1つの `$LK8EX1` センテンスが `ATT_MTU - 3` を超える場合は、CRLFまでのbyte列を複数Notifyへ順序どおり分割します。別センテンスのfragmentを途中へ割り込ませてはなりません。

XCTrackでは、`vario`フィールドを昇降率表示に使用し、`raw_pressure`フィールドをバリオ音の生成に使用します。このため、有効な両フィールドを同じセンテンスで送信します。一方だけが無効な場合はそのフィールド固有の無効値を送信し、気圧と昇降率の両方が無効な場合はセンテンスを送信しません。

---

## 4. データサンプル

### サンプル1：正常フライト中（上昇、気圧データあり、高度計・温度・バッテリーをパスする場合）

* 条件：気圧 1011.30hPa (101130Pa)、高度計不使用、上昇 2.3m/s (230cm/s)、温度不使用、バッテリー不使用

```text
$LK8EX1,101130,99999,230,99,999,*20\r\n

```

### サンプル2：バッテリー電圧あり

* 条件：気圧 1008.15hPa (100815Pa)、高度計不使用、下降 1.2m/s (-120cm/s)、温度フィールド固定、バッテリー電圧 3.95V

```text
$LK8EX1,100815,99999,-120,99,3.95,*28\r\n

```

### サンプル3：昇降率だけが無効

* 条件：気圧 1013.25hPa (101325Pa)、高度計不使用、昇降率無効、温度フィールド固定、バッテリー不使用

```text
$LK8EX1,101325,99999,9999,99,999,*17\r\n

```

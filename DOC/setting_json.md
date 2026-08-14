# `setting.json` 設定ファイル説明書

## 1. 概要

`setting.json` は、CloudBaseVario の動作設定を、共通パラメータ9項目と番号1～5のバリオ音パラメータセットとして保存するファイルです。各セットは音の感度、判定、音程、テンポ、予測音および出力波形に関する22項目を保持します。基板実装に依存する IMU 軸変換、およびSW1／SW2で操作する音量・シンク音設定は保持しません。

| 項目 | 内容 |
| --- | --- |
| 配置場所 | USB MSC で公開される config FAT のルート直下 |
| ファイル名 | `setting.json` |
| 文字コード | UTF-8。UTF-8 BOM 付きも読込み可能 |
| JSON 形式 | top-level に `format_version`、`mc_parameters`、`vario_parameter_sets` を持つ object |
| 現行 version | `1` |
| 最大ファイルサイズ | 32 KiB |
| 反映時点 | 起動時の読込み、または CDC の `PARAM SET` / `PARAM RESET` による RAM 変更時 |

設定値の型、範囲、項目間の関係をファームウェアが検証します。ファイル全体が妥当な場合だけ、RAM 上の動作設定を一括で置き換えます。一部の項目だけを先に反映することはありません。

`mc_data.json` は IMU 個体校正専用の別ファイルです。詳細は [`mc_data_json.md`](mc_data_json.md) を参照してください。

## 2. 基本的な使い方

### 2.1 推奨する変更方法

動作中の設定変更には TinyUSB CDC のコマンド、または `tools/cloudbasevario_gui.py` を使用します。

```text
PARAM LIST
PARAM GET <name>
PARAM SET <name> <value>
PARAM RESET <name|ALL>
PARAM SAVE
```

- `PARAM LIST` は共通値と選択中セットを合成した RAM 上の全項目を表示します。
- `PARAM GET` は指定項目の RAM 値を表示します。
- `PARAM SET` は型、範囲、全項目の関係を検証してから RAM 値を変更します。
- `PARAM RESET` は指定項目、または `ALL` で共通値と選択中セットの全項目を組込み既定値へ戻します。`ALL`でも非選択セットは変更しません。
- `PARAM SET` と `PARAM RESET` は共通項目なら単一の共通値、音関連項目なら選択中セットだけを変更します。セットを切り替えても各セットのRAM値は保持されます。
- `PARAM SAVE` が成功したときだけ、RAM上の共通値と全セットが `setting.json` へ永続化されます。
- MSC を PC が所有している間の `PARAM SAVE` は、ファイルを変更せず `ERR SAVE BUSY` を返します。OS で安全な取り外しを行うか USB を切断し、ESP32 が FAT を再マウントしてから保存してください。

例:

```text
PARAM SET sea_level_pressure_pa 101800
PARAM SET predictive_buzzer_enabled true
PARAM SAVE
```

コマンドのパラメータ名と `true` / `false` / `AUTO` / `BARO_ONLY` / `VOLTAGE` / `PERCENT` は大文字・小文字を区別しません。JSON ファイル内の key と enum 文字列は、表記どおりの大文字・小文字で記述してください。

### 2.2 PC で直接編集する場合

1. config FAT が USB ドライブとして公開された状態で `setting.json` をバックアップします。
2. UTF-8 のテキストエディタで編集し、JSON 構文を維持して保存します。
3. OS の「安全な取り外し」を実行します。
4. 本体を再起動します。PC でのファイル編集は動作中の RAM 値へ即時反映されません。
5. CDC で `PARAM LIST` または `PARAM GET <name>` を実行し、読込み後の値を確認します。

ファイルが不正な場合は、その中の一部だけを採用せず、全項目を組込み既定値として起動します。不正なファイルは自動上書きされません。

### 2.3 初期化

- `setting.json` が存在しない場合、組込み既定値で起動し、MSC 公開前に既定値ファイルを自動生成します。
- SW2 と SW3 を同時に押したまま電源を入れると、起動時判定後に config FAT をフォーマットし、既定の `setting.json` を生成するとともに、NVSのスイッチ設定だけを消去します。BLEなどが使用するNVS領域は消去しません。
- `idf.py -p <PORT> config-flash` でも config FAT を初期化できます。

フォーマットと `config-flash` は、`setting.json`、`mc_data.json`、更新関連ファイルを含む config FAT の保存内容を消去します。必要な値を退避してから実行してください。

## 3. JSON 構造

次は、組込み既定値と等価な番号1～3を持つversion 1の例です。読みやすさのため小数表記を簡略化しています。ファームウェアが保存したファイルでは、単精度浮動小数点数の再読込みを保証するため、小数末尾の桁が増えることがあります。

```json
{
  "format_version": 1,
  "mc_parameters": {
    "sea_level_pressure_pa": 101325.0,
    "auto_power_off_minutes": 60,
    "filter_mode": "AUTO",
    "bluetooth_battery_mode": "VOLTAGE",
    "bluetooth_notify_rate_hz": 10,
    "i2c_reinit_error_count": 10,
    "imu_gyro_calibration_samples": 200,
    "imu_mahony_kp": 5.0,
    "imu_mahony_ki": 0.05
  },
  "vario_parameter_sets": [
    {
      "parameter_number": 1,
      "parameters": {
        "predictive_buzzer_enabled": false,
        "audio_climb_rate_average_s": 1.0,
        "lift_start_mps": 0.1,
        "lift_end_mps": 0.08,
        "sink_start_mps": -1.8,
        "sink_end_mps": -1.7,
        "audio_state_hold_ms": 200,
        "audio_stale_ms": 500,
        "lift_freq_base_hz": 1047,
        "lift_freq_rate_hz_per_mps": 100.0,
        "lift_freq_max_hz": 2600,
        "lift_time_ms_at_0p2": 400,
        "lift_time_ms_at_1p0": 400,
        "lift_time_ms_at_2p5": 300,
        "lift_time_ms_at_5p0": 100,
        "sink_freq_start_hz": 523,
        "sink_freq_rate_hz_per_mps": 40.0,
        "sink_freq_min_hz": 240,
        "audio_duty_percent": 50,
        "predictive_interval_ms": 1000,
        "predictive_duration_ms": 150,
        "predictive_min_mps": 0.01
      }
    },
    {
      "parameter_number": 2,
      "parameters": {
        "predictive_buzzer_enabled": false,
        "audio_climb_rate_average_s": 1.0,
        "lift_start_mps": 0.2,
        "lift_end_mps": 0.18,
        "sink_start_mps": -2.0,
        "sink_end_mps": -1.9,
        "audio_state_hold_ms": 200,
        "audio_stale_ms": 500,
        "lift_freq_base_hz": 1047,
        "lift_freq_rate_hz_per_mps": 100.0,
        "lift_freq_max_hz": 2600,
        "lift_time_ms_at_0p2": 400,
        "lift_time_ms_at_1p0": 400,
        "lift_time_ms_at_2p5": 300,
        "lift_time_ms_at_5p0": 100,
        "sink_freq_start_hz": 523,
        "sink_freq_rate_hz_per_mps": 40.0,
        "sink_freq_min_hz": 240,
        "audio_duty_percent": 50,
        "predictive_interval_ms": 1000,
        "predictive_duration_ms": 150,
        "predictive_min_mps": 0.01
      }
    },
    {
      "parameter_number": 3,
      "parameters": {
        "predictive_buzzer_enabled": false,
        "audio_climb_rate_average_s": 1.0,
        "lift_start_mps": 0.3,
        "lift_end_mps": 0.29,
        "sink_start_mps": -2.2,
        "sink_end_mps": -2.1,
        "audio_state_hold_ms": 200,
        "audio_stale_ms": 500,
        "lift_freq_base_hz": 1047,
        "lift_freq_rate_hz_per_mps": 100.0,
        "lift_freq_max_hz": 2600,
        "lift_time_ms_at_0p2": 400,
        "lift_time_ms_at_1p0": 400,
        "lift_time_ms_at_2p5": 300,
        "lift_time_ms_at_5p0": 100,
        "sink_freq_start_hz": 523,
        "sink_freq_rate_hz_per_mps": 40.0,
        "sink_freq_min_hz": 240,
        "audio_duty_percent": 50,
        "predictive_interval_ms": 1000,
        "predictive_duration_ms": 150,
        "predictive_min_mps": 0.01
      }
    }
  ]
}
```

### 3.1 構造上の規則

- top-level では `format_version`、`mc_parameters`、`vario_parameter_sets` だけが使用できます。
- `format_version` は整数で、現行形式では `1` です。それ以外のversionは読み込みません。
- top-level `mc_parameters` は共通9項目すべてを持つobjectです。
- `vario_parameter_sets` は1～5件の配列です。各要素は `parameter_number` と `parameters` だけを持ちます。
- `parameter_number` は1～5の整数で重複できません。配列順は任意ですが、保存時は番号順に整列します。
- `mc_parameters` と各セットの `parameters` は JSON object です。1セットでも不正ならファイル全体を無効とします。
- parameter key は大文字・小文字を区別します。
- 未知の top-level key、未知の parameter、同じ key の重複はエラーです。
- 各セットの`parameters`には音関連22項目すべてが必要です。共通項目をセット内へ置く、または音関連項目をtop-level `mc_parameters`へ置くこともエラーです。
- boolean は引用符なしの `true` または `false`、整数は小数部なし、float は有限の JSON number として記述します。
- `filter_mode` は文字列 `"AUTO"` または `"BARO_ONLY"` です。
- `bluetooth_battery_mode` は文字列 `"VOLTAGE"` または `"PERCENT"` です。
- `NaN`、`Infinity`、非有限値、範囲外の値は使用できません。

## 4. パラメータ詳細

範囲の両端は、特記がない限り使用できます。
4.1と4.2の9項目はtop-level `mc_parameters`に1組だけ保存し、4.4～4.8の22項目は各`vario_parameter_sets[].parameters`に保存します。

### 4.1 電源・気圧・推定・I2C

| パラメータ | 型 | 既定値 | 設定範囲 | 詳細 |
| --- | --- | ---: | --- | --- |
| `sea_level_pressure_pa` | float | 101325 | 80000～110000 Pa | 気圧から高度を求めるときの海面更正気圧です。値を大きくすると算出高度は高くなります。動作中に値が変わると高度・昇降率推定器をリセットします。 |
| `auto_power_off_minutes` | uint32 | 60 | 0～1440 min | 外部給電がなく、有効な実センサー高度の期間内変動幅が10 m以下の状態が継続したときに自動電源OFFする時間です。`0`で無効にします。変動幅が10 mを超えた場合は現在高度から計時をやり直します。外部給電中、高度が無効・stale・非有限の場合、および設定変更時は計時状態をリセットします。デバッグ高度は判定に使用しません。 |
| `filter_mode` | enum | `AUTO` | `AUTO`, `BARO_ONLY` | `AUTO` は、有効な姿勢補正済み IMU 鉛直加速度がある間、気圧と IMU を融合します。IMU が無効・停止・stale の場合は自動的に気圧単独へ戻ります。`BARO_ONLY` は常に気圧単独で昇降率を推定します。IMU の取得や診断そのものを無効にする設定ではありません。 |
| `bluetooth_battery_mode` | enum | `VOLTAGE` | `VOLTAGE`, `PERCENT` | LK8EX1のbatteryフィールドには、5点中央値から求めた30秒区間の最低表示値を使用します。`VOLTAGE`ではV単位の小数2桁、`PERCENT`ではBattery Serviceと同じ3.0～4.1 V換算値へLK8EX1規定の1000を加えた整数1000～1100で送信します。最初の有効値を取得する前は`999`とし、一時的なADC無効時は前回表示値を保持します。 |
| `bluetooth_notify_rate_hz` | uint32 | 10 | 1～50 Hz | LK8EX1センテンスのNotify試行頻度です。BLEがbusyの場合はその周期のセンテンスを破棄して再送しないため、成功Notify数は設定値を下回ることがあります。Battery Serviceの更新周期には影響しません。 |
| `i2c_reinit_error_count` | uint32 | 10 | 1～100 回 | BMP581 または ICM-42688P-HXY の連続 I2C エラーがこの回数に達したとき、センサを offline として共有 I2C bus の復旧・再初期化を試みます。小さすぎる値は一過性エラーで復旧処理を頻発させ、大きすぎる値は故障検出を遅らせます。 |

### 4.2 IMU 姿勢推定

| パラメータ | 型 | 既定値 | 設定範囲 | 詳細 |
| --- | --- | ---: | --- | --- |
| `imu_gyro_calibration_samples` | uint32 | 200 | 50～2000 samples | 起動後のジャイロ bias と初期姿勢を求める連続静止サンプル数です。公称 400 Hz では既定値 200 が約 0.5 秒に相当します。加速度 norm が 0.9～1.1 g 外、またはいずれかのジャイロ軸が ±3 dps を超えると蓄積をやり直します。 |
| `imu_mahony_kp` | float | 5.0 | 0～20 | Mahony 姿勢フィルタの比例 gain です。実効値は IMU confidence を掛けた `imu_kp_effective` です。大きくすると加速度方向への追従が速くなりますが、振動・並進加速度の影響を受けやすくなります。0 は比例補正を無効にします。 |
| `imu_mahony_ki` | float | 0.05 | 0～5 | Mahony 姿勢フィルタの積分 gain です。静止に近い状態が 0.5 秒継続した場合だけ有効になり、実効値には IMU confidence が掛かります。条件を外れた場合や gain 変更時は積分値をクリアします。0 は積分補正を無効にします。 |

`imu_mahony_kp` と `imu_mahony_ki` の実効状態は、`DIAG STATUS` の `confidence`、`kp_effective`、`ki_effective`、`ki_active`、または `BARO` 行の対応フィールドで確認できます。

### 4.3 ボード固定の IMU 軸変換

加速度とジャイロは、それぞれ次の式でセンサ座標から基板座標へ変換します。

```text
board_axis_value = sensor[source] * sign
```

`source` の `0`、`1`、`2` は、センサの X、Y、Z 軸を表します。X/Y/Z の三つの `source` は重複できず、それぞれ 0、1、2 を一度ずつ使い、`sign` は正確に `-1` または `1` とします。

Aohazuku Rev.0 は加速度・ジャイロとも `source={0, 1, 2}`、`sign={+1, +1, +1}` に固定されています。これらは `setting.json`、`PARAM LIST`、`PARAM GET`、`PARAM SET`、`PARAM RESET` の対象ではありません。軸変換は `SRC/platform/board.c` のボード定義を使用します。

version 1では、軸keyを含む未知のparameterはファイル全体のエラーになります。`mc_data.json` のoffsetは、軸変換前のセンサ座標で保持されます。

### 4.4 音状態制御

| パラメータ | 型 | 既定値 | 設定範囲 | 詳細 |
| --- | --- | ---: | --- | --- |
| `predictive_buzzer_enabled` | bool | `false` | `true`, `false` | ゼロ付近の指定範囲で鳴らす予測ブザーを有効にします。通常の無音域を補助します。 |
| `audio_climb_rate_average_s` | float | 1.0 | 0～10 s | 音状態、音程、テンポに使う上昇率の単純移動平均窓です。起動直後は取得済みサンプルだけを平均し、`0`では平均せず最新値を使います。表示、BLE、ログの上昇率は変えません。 |
| `audio_state_hold_ms` | uint32 | 200 | 0～1000 ms | 通常の音状態切替を抑制する最小保持時間です。しきい値付近の細かな往復によるチャタリングを減らします。入力無効、stale、SW1消音による強制無音は待ちません。 |
| `audio_stale_ms` | uint32 | 500 | 100～500 ms | 最新の有効な昇降率がこの時間より古い場合に強制無音とする期限です。 |

SW1の音量（消音・小・中・大）、SW2のシンク音ON/OFF、およびSW3で選択するパラメータ番号は、`setting.json`ではなくNVSのnamespace `switch_pref`、key `state`を正本とします。NVSが未保存または不正な場合は小音量・シンク音ON・番号1（番号1がなければ最小番号）で起動します。操作はRAMへ即時反映し、起動時の保存値と異なる場合だけdirtyになります。通常の電源OFFシーケンスでdirtyな最終値を一括保存し、操作時にはFlashへ書きません。変更後に保存値へ戻した場合は保存を省略します。

Watchdog reset、外部reset、突然の電源断では未保存の変更を失います。保存失敗または電源OFFの15秒期限到達時も、設定保存より電源OFFを優先します。起動音、終了音およびSW1～SW3の通知音には現在のSW1音量を使用し、消音時は鳴りません。SW2はONを低音→高音、OFFを高音→低音で通知します。低音は700 Hz・180 ms、高音は1200 Hz・120 msとし、2音の間に80 msの無音を置きます。これらの設定は`PARAM LIST`、`PARAM GET`、`PARAM SET`、`PARAM RESET`、`PARAM SAVE`の対象外です。

### 4.5 リフト・シンク判定

| パラメータ | 型 | 既定値 | 設定範囲 | 詳細 |
| --- | --- | ---: | --- | --- |
| `lift_start_mps` | float | 0.10 / 0.20 / 0.30 | -1～5 m/s | セット1 / 2 / 3の、無音または予測音からリフト断続音へ入る平均上昇率しきい値です。値を上回ると開始します。 |
| `lift_end_mps` | float | 0.08 / 0.18 / 0.29 | -1～5 m/s | セット1 / 2 / 3の、リフト状態を終了するしきい値です。昇降率がこの値を下回ると終了要求になります。`lift_start_mps` との差がヒステリシスです。 |
| `sink_start_mps` | float | -1.80 / -2.00 / -2.20 | -10～0 m/s | セット1 / 2 / 3の、無音状態からシンク連続音へ入る平均上昇率しきい値です。値を下回ると開始します。 |
| `sink_end_mps` | float | -1.70 / -1.90 / -2.10 | -10～0 m/s | セット1 / 2 / 3の、シンク状態を終了するしきい値です。昇降率がこの値を上回ると終了します。`sink_start_mps` との差がヒステリシスです。 |

上昇率の平均履歴には、異なるタイムスタンプを持つ有効サンプルだけを追加します。欠測区間は補間せず、同じサンプルを音声タスクが再評価しても重複加算しません。無効・stale入力、デバッグ入力源の切替、設定変更、音響リセットでは履歴を消去します。

リフト終了時に断続音の ON phase 中であれば、現在の ON phase を完了してから状態を切り替えます。

### 4.6 リフト音

| パラメータ | 型 | 既定値 | 設定範囲 | 詳細 |
| --- | --- | ---: | --- | --- |
| `lift_freq_base_hz` | uint32 | 1047 | 200～5000 Hz | 上昇率 0 m/s におけるリフト音の基準周波数です。 |
| `lift_freq_rate_hz_per_mps` | float | 100 | 0～1000 Hz/(m/s) | 上昇率 1 m/s 増加ごとの周波数増加量です。 |
| `lift_freq_max_hz` | uint32 | 2600 | 200～5000 Hz | リフト音周波数の上限です。 |
| `lift_time_ms_at_0p2` | uint32 | 400 | 20～2000 ms | 上昇率 0.2 m/s 以下で使う ON 時間および OFF 時間です。 |
| `lift_time_ms_at_1p0` | uint32 | 400 | 20～2000 ms | 上昇率 1.0 m/s の ON/OFF 時間制御点です。 |
| `lift_time_ms_at_2p5` | uint32 | 300 | 20～2000 ms | 上昇率 2.5 m/s の ON/OFF 時間制御点です。 |
| `lift_time_ms_at_5p0` | uint32 | 100 | 70～2000 ms | 上昇率 5.0 m/s 以上で使う ON/OFF 時間です。 |

周波数は次式で求めます。

```text
lift_hz = min(lift_freq_base_hz
              + lift_freq_rate_hz_per_mps * max(audio_rate_mps, 0),
              lift_freq_max_hz)
```

ON 時間と OFF 時間には同じ値を使います。0.2、1.0、2.5、5.0 m/s の制御点間を線形補間し、0.2 m/s 以下と 5.0 m/s 以上では端の値を使います。

### 4.7 シンク音

| パラメータ | 型 | 既定値 | 設定範囲 | 詳細 |
| --- | --- | ---: | --- | --- |
| `sink_freq_start_hz` | uint32 | 523 | 130～2000 Hz | シンク音の基準周波数です。-1.0 m/s まではこの周波数を使用します。 |
| `sink_freq_rate_hz_per_mps` | float | 40 | 0～500 Hz/(m/s) | -1.0 m/s より沈下が 1 m/s 強くなるごとの周波数低下量です。 |
| `sink_freq_min_hz` | uint32 | 240 | 130～2000 Hz | シンク音周波数の下限です。 |

周波数は次式で求めます。周波数式の基準は `sink_start_mps` ではなく、固定の -1.0 m/s です。

```text
sink_hz = max(sink_freq_start_hz
              - sink_freq_rate_hz_per_mps * max(-audio_rate_mps - 1, 0),
              sink_freq_min_hz)
```

### 4.8 出力と予測ブザー

| パラメータ | 型 | 既定値 | 設定範囲 | 詳細 |
| --- | --- | ---: | --- | --- |
| `audio_duty_percent` | uint32 | 50 | 10～90 % | 音声 PWM の duty 比です。知覚音量だけを表す値ではなく、出力波形の設定です。 |
| `predictive_interval_ms` | uint32 | 1000 | 20～2000 ms | 予測ブザーの鳴り始めから次の鳴り始めまでの固定周期です。 |
| `predictive_duration_ms` | uint32 | 150 | 10～1000 ms | 各周期で音を出す固定時間です。周期以下でなければなりません。 |
| `predictive_min_mps` | float | 0.01 | -2～1 m/s | 予測ブザー対象範囲の下限です。境界値を含みます。 |

予測音の上限は`lift_start_mps`から導出し、同値を予測音側に含めます。周波数は通常リフト音と同じ式を使います。平均上昇率が`lift_start_mps`を超えると直ちに通常リフト音へ、通常リフト中に`lift_end_mps`未満かつ予測範囲内になると直ちに予測音へ移り、`audio_state_hold_ms`やリフトON区間の完了を待ちません。

## 5. 項目間の検証規則

各項目の範囲に加え、次の関係をすべて満たす必要があります。

```text
sink_start_mps <= sink_end_mps < lift_end_mps <= lift_start_mps
lift_freq_base_hz <= lift_freq_max_hz
sink_freq_min_hz <= sink_freq_start_hz
lift_time_ms_at_0p2 >= lift_time_ms_at_1p0
lift_time_ms_at_1p0 >= lift_time_ms_at_2p5
lift_time_ms_at_2p5 >= lift_time_ms_at_5p0
predictive_min_mps <= lift_start_mps
predictive_duration_ms <= predictive_interval_ms
```

`PARAM SET` は変更後の全体を検証するため、最終的には妥当な組合せでも、変更順によって途中状態が拒否されることがあります。その場合は、関係を崩さない順序で変更するか、PC で JSON 全体を編集して再起動時に一括適用してください。

## 6. 起動時の読込みとエラー動作

| 状態 | 動作 |
| --- | --- |
| 正常な version 1 | 共通9項目と音関連22項目を持つ全セットを読込み |
| version 1以外 | 非対応versionとしてファイル全体を無効化 |
| ファイルなし | 全項目を組込み既定値とし、既定ファイルを自動生成 |
| JSON 構文、型、範囲、関係が不正 | ファイルの値を一切適用せず、全項目を組込み既定値として継続。不正ファイルは自動上書きしない |
| 読込み I/O error | 全項目を組込み既定値として継続し、診断へ error を記録 |
| `setting.json` がなく、有効な `setting.bak` がある | backup を `setting.json` へ復元して読込み |
| `setting.json` 自体が不正 | `setting.bak` へ自動 fallback せず、組込み既定値を使用 |

`DIAG STATUS` の USB 行で、設定 source、validation reason、format version、key、storage owner、storage error を確認できます。SYSTEM行では現在音量、シンク状態、選択番号、セット数、dirtyを確認できます。

## 7. 保存の安全性

`PARAM SAVE` は次の全量置換手順を使用します。

1. RAM 上の全設定を再検証します。
2. `setting.tmp` へ version 1 の共通値と全パラメータセットを書きます。
3. flush と media sync を行います。
4. 一時ファイルを再読込みし、保存前の RAM 値と一致することを確認します。
5. 既存の `setting.json` を `setting.bak` へ移動します。
6. `setting.tmp` を `setting.json` へ rename します。
7. 成功後に backup を削除します。

電源断などで正本がなく backup だけが残った場合は、次回起動時に検証済み backup を復元します。一時ファイルは起動設定として読み込みません。

## 8. versionの扱い

ファームウェアはversion 1の構造だけを読み込みます。top-level、共通9項目、各セットの音関連22項目について、未知のkey、誤った階層、欠落、重複、型違いまたは値域違反があるファイルは全体を無効とし、自動変換しません。

## 9. 実装上の正本

本書の既定値、型、範囲、項目間関係は `SRC/domain/app_config.c`、JSON の検証・保存動作は `SRC/platform/config_storage.c`、CDC コマンドは `SRC/app/app_tasks.c` の現行実装に基づきます。

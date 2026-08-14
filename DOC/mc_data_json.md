# `mc_data.json` IMU 個体校正ファイル説明書

## 1. 概要

`mc_data.json` は、CloudBaseVario に搭載した ICM-42688P-HXY の加速度個体差を補正する校正データです。通常の動作パラメータではなく、端末ごと、センサ個体ごとにファームウェアが生成します。

| 項目 | 内容 |
| --- | --- |
| 配置場所 | USB MSC で公開される config FAT のルート直下 |
| ファイル名 | `mc_data.json` |
| 文字コード | UTF-8。UTF-8 BOM 付きも読込み可能 |
| 現行 version | `2` |
| 最大ファイルサイズ | 2 KiB |
| 対象センサ | `ICM-42688P-HXY`、WHO_AM_I `0x6A` |
| 校正方法 | 基板上面を上にした水平静止、800 連続サンプル |
| 保存値 | センサ座標 X/Y/Z の加速度 offset、単位 m/s² |

このファイルは手作業で調整する設定ファイルではありません。通常はファームウェアの初回水平校正に生成させてください。動作設定については [`setting_json.md`](setting_json.md) を参照してください。

## 2. 役割

ICM-42688P-HXY が出力する生の加速度には、センサ個体ごとのゼロ点誤差が含まれます。ファームウェアは、水平・上面上向きで静止しているときの平均値と、期待値 `[0, 0, +1 g]` との差から三軸 offset を求めます。

通常動作では、次の順序で補正します。

```text
1. センサ生加速度を取得
2. センサ座標の各軸から offset_mps2 を減算
3. ボード定義の固定軸マップで基板座標へ変換
4. 姿勢推定と鉛直加速度推定へ使用
```

したがって `offset_mps2` は基板座標ではなく、軸変換前のセンサ座標です。

`mc_data.json` は加速度 offset だけを保存します。起動ごとのジャイロ bias と初期姿勢は、`setting.json` の `imu_gyro_calibration_samples` で指定した連続静止サンプルから別途求め、ファイルには保存しません。

## 3. JSON 構造

次は構造例です。実機で生成される `offset_mps2` は、端末固有の値になります。

```json
{
  "format_version": 2,
  "imu_accel_calibration": {
    "model": "ICM-42688P-HXY",
    "offset_mps2": [0.0123, -0.0456, 0.0789]
  }
}
```

検証は厳格です。top-level と `imu_accel_calibration` object は、次表の key をそれぞれ一度ずつ、過不足なく含む必要があります。未知、欠落、重複 key は受理しません。

## 4. フィールド詳細

### 4.1 top-level

| フィールド | 型 | 必須値 | 詳細 |
| --- | --- | --- | --- |
| `format_version` | number | 整数値 `2` | ファイル形式の version です。version 2 以外は受理しません。 |
| `imu_accel_calibration` | object | 下記二項目を保持 | IMU の加速度個体校正 section です。 |

### 4.2 `imu_accel_calibration`

| フィールド | 型 | 必須値・範囲 | 詳細 |
| --- | --- | --- | --- |
| `model` | string | `"ICM-42688P-HXY"` | 校正対象のセンサ model です。文字列は完全一致が必要です。 |
| `offset_mps2` | array of number | 有限値三個 | センサ X、Y、Z 軸の順の加速度 offset です。各値は ±0.20 g、すなわち約 ±1.96133 m/s² 以内でなければなりません。 |

`who_am_i`、座標系、校正方法、校正サンプル数は JSON には保存しません。これらは `model` ごとのファームウェア固定定義です。現在の `ICM-42688P-HXY` では順に `0x6A`、`SENSOR`、`LEVEL_Z_UP`、800 サンプルです。別 model を追加する場合は、model 定義を追加して一括で切り替えます。

`offset_mps2` の適用式は次のとおりです。

```text
corrected_sensor_accel[axis]
    = raw_sensor_accel[axis] - offset_mps2[axis]
```

値の符号を逆にして加算しないでください。

## 5. 初回水平校正

### 5.1 準備

1. 本体を振動の少ない水平で安定した台に置きます。
2. 基板上面を上向きにします。
3. 本体と机に触れず、校正完了まで静止させます。
4. 必要に応じて TinyUSB CDC へ接続し、`BARO` または `DIAG STATUS` で進捗を確認します。

初回水平校正では `setting.json` の加速度軸 mapping と sign を使って姿勢条件を判定します。ハードウェア実装に合わせた正しい値を先に設定してください。

### 5.2 サンプル採用条件

次の全条件を満たすサンプルだけを連続して蓄積します。

| 条件 | 採用範囲 |
| --- | --- |
| 基板 X 加速度 | ±0.10 g 以内 |
| 基板 Y 加速度 | ±0.10 g 以内 |
| 基板 Z 加速度 | +0.75～+1.25 g |
| 各軸ジャイロ絶対値 | 3 dps 以下 |
| 加速度 norm の振動 RMS | 0.02 g 以下 |
| 連続サンプル数 | 800 |

姿勢、静止、振動のいずれかの条件を外れると、それまでのサンプル蓄積を破棄して 0 からやり直します。

800 サンプルの平均から算出した各センサ軸 offset が ±0.20 g を超える場合も校正失敗となり、ファイルを保存しません。

### 5.3 保存と起動動作

- 有効な `mc_data.json` がない起動では、ファームウェアが水平校正を要求します。
- 校正データを原子的に保存できるまで config FAT は `APP_OWNED` のままとなり、通常 build の MSC media は PC へ公開されません。
- 待機中も TinyUSB CDC 診断、気圧単独の昇降率、音、BLE は継続します。IMU の姿勢融合は、有効な個体校正値が保存されるまで使用しません。
- 校正候補の保存に失敗した場合は、2 秒間隔で再試行します。
- 保存成功後に offset を有効化し、起動シーケンスの残りを進めます。

### 5.4 一時スキップ

校正待機中に SW3 を 3 秒長押しすると、その起動に限って初回水平校正をスキップできます。

- 未保存の校正候補を破棄します。
- その起動中は IMU を停止し、気圧単独で動作します。
- 通常 build では MSC media を公開します。
- スキップ状態を `mc_data.json`、`setting.json`、NVS のいずれにも保存しません。
- 次回起動時に有効な `mc_data.json` がなければ、再び水平校正を要求します。
- 3 秒未満で SW3 を離した場合は、通常のパラメータセット切替として扱います。

スキップは校正を完了したことにはならず、恒久的に IMU を無効化する設定でもありません。

## 6. 再校正

次の場合は再校正してください。

- ICM-42688P-HXY または基板を交換した。
- IMU の取付状態が変わった。
- 水平静止時の補正後加速度や姿勢に明らかな偏りがある。
- `DIAG STATUS` で `mc_data=INVALID` が報告される。

手順:

1. MSC ドライブ上の既存 `mc_data.json` を退避します。
2. `mc_data.json` を削除します。
3. OS の「安全な取り外し」を実行します。
4. 本体を再起動します。
5. 基板上面を上向きにして水平静止させ、800 連続サンプルの校正と保存を完了させます。
6. `DIAG STATUS` で新しい offset と保存状態を確認します。

ファイルを削除しただけでは動作中の offset は変わりません。次回起動時にファイルがないことを検出して再校正へ移ります。

## 7. 読込み検証と異常時動作

次のいずれかに該当するファイルは無効です。

- 空ファイル、2 KiB 超過、JSON 構文不正、JSON の後ろに別データがある。
- 必須 key の欠落、未知 key、重複 key がある。
- version または model が必須値と一致しない。
- version 1 形式の `who_am_i`、`coordinate`、`method`、`sample_count` など、現行形式にない key がある。
- `offset_mps2` が三要素の array ではない。
- offset が number ではない、非有限値である、または ±0.20 g を超える。

| 状態 | 診断値 | 動作 |
| --- | --- | --- |
| 正常な正本 | `VALID` | offset を読み込み、IMU 個体校正済みとして起動 |
| 正本がなく、有効な `mc_data.bak` がある | `RECOVERED` | backup を `mc_data.json` へ復元し、offset を使用 |
| 正本と backup がない | `MISSING` | 初回水平校正を要求 |
| 正本の内容が不正 | `INVALID` | backup へ自動 fallback せず、初回水平校正を要求 |
| ファイル操作に失敗 | `IO_ERROR` | offset を使用せず、error を診断へ記録 |

正本が存在するが不正な場合は、古い backup を誤って使わないよう backup 復元を行いません。

## 8. 保存の安全性

ファームウェアは次の全量置換手順で保存します。

1. model 固定の sample count、有限値、offset 範囲を検証します。
2. `mc_data.tmp` へ全フィールドを書きます。
3. flush と media sync を行います。
4. 一時ファイルを再読込みし、三軸 offset が保存前と一致することを確認します。
5. 既存の `mc_data.json` を `mc_data.bak` へ移動します。
6. `mc_data.tmp` を `mc_data.json` へ rename します。
7. 成功後に backup を削除します。

電源断などで正本がなく backup だけが残った場合は、次回起動で有効な backup を復元します。`mc_data.tmp` は起動時に読み込みません。

## 9. 診断方法

TinyUSB CDC から次を実行します。

```text
DIAG STATUS
```

IMU 行の主な確認項目は次のとおりです。

| 診断項目 | 意味 |
| --- | --- |
| `accel_calibrated` | 有効な加速度個体校正を現在使用しているか |
| `accel_persisted` | 校正値を `mc_data.json` へ保存済みか |
| `accel_save_pending` | 有効な候補があり、保存を待っているか |
| `accel_skipped` | この起動で校正を一時スキップしたか |
| `accel_calibration` | 初回水平校正で現在蓄積済みのサンプル数 |
| `accel_offset_mps2` | 読込み済み、または保存済みの X/Y/Z offset |
| `mc_data` | `VALID`、`RECOVERED`、`MISSING`、`INVALID`、`IO_ERROR` |
| `mc_error` | storage I/O error code。正常時は 0 |

10 Hz の `BARO` 行でも、`imu_accel_calibrated`、`imu_accel_cal_persisted`、`imu_cal_samples`、`imu_cal_save_pending`、`imu_cal_storage`、`imu_cal_storage_error` を確認できます。

## 10. `setting.json` との関係

| `setting.json` の項目 | `mc_data.json` への影響 |
| --- | --- |
| ボード固定のIMU軸マップ | 水平校正時の基板姿勢判定と、期待する +1 g のセンサ軸を決めます。offset 自体はセンサ座標で保存されます。 |
| `imu_gyro_*_source`, `imu_gyro_*_sign` | 水平校正中の静止判定に使う基板座標ジャイロを決めます。 |
| `imu_gyro_calibration_samples` | `mc_data.json` 保存後の、起動ごとのジャイロ bias・初期姿勢校正時間を決めます。model 固定の加速度校正サンプル数 800 には影響しません。 |
| `imu_mahony_kp`, `imu_mahony_ki` | 保存済み offset の内容には影響せず、その後の姿勢フィルタ gain を決めます。 |
| `filter_mode` | 保存済み offset の内容には影響せず、校正後の気圧・IMU 融合を使うかを決めます。 |

## 11. 実装上の正本

本書の JSON 形式と保存動作は `SRC/platform/imu_calibration_storage.c`、校正条件、offset 算出、補正順序は `SRC/domain/imu_fusion.c`、起動・スキップ・保存再試行は `SRC/app/app_tasks.c` と `SRC/app/main.c` の現行実装に基づきます。

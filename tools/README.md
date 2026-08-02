# CloudBaseVario Monitor

`cloudbasevario_gui.py`は、CloudBaseVarioのTinyUSB CDCへ接続するWindows向けホストGUIです。

## 機能

- `BARO key=value ...`テレメトリーの10 Hz受信
- 気圧、高度、昇降率、鉛直加速度、温度の飛行値表示
- 気圧、昇降率、鉛直加速度の直近60秒グラフ
- IMUのroll／pitch人工水平儀、yaw、クォータニオン表示
- BARO、推定、IMU、較正、姿勢、融合、BLE Notify、シリアルストリームの状態表示
- `Diagnostics`タブでBMP581／Kalman品質、IMU信頼度・振動・実効Mahonyゲイン・加速度校正、LK8EX1/BLEとストリーム状態を表示
- 全テレメトリーフィールドの一覧表示
- `PARAM LIST`によるパラメーター一覧取得
- `PARAM SET`、`PARAM RESET`、`PARAM SAVE`操作
- 起動ログ、コマンド、応答のシリアルログ表示

## インストール

Python 3.10以降をインストールし、リポジトリのルートで実行します。標準のWindows版PythonにはTkinterが含まれています。

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r tools\requirements-gui.txt
```

## 起動

ESP-IDF monitorや他のターミナルが対象COMポートを使用している場合は、先に終了してください。

```powershell
.\.venv\Scripts\python.exe tools\cloudbasevario_gui.py
```

GUI上部でCloudBaseVarioのTinyUSB CDC COMポートを選び、`Connect`を押します。接続後、テレメトリー受信と`PARAM LIST`取得が自動的に始まります。USB CDCではbaud値は実質的な通信速度を決めませんが、pyserialのポート設定として既定値115200を使用します。

## パラメーター操作

1. `Parameters`タブで項目を選択します。
2. `New RAM value`へ値を入力し、`Set in RAM`を押します。
3. 必要な項目をすべて変更した後、永続化する場合だけ`PARAM SAVE`を押します。

`PARAM SET`と`PARAM RESET`はRAMだけを変更します。GUIは`parameters.json`を直接編集せず、型、値域、項目間関係の最終検証はファームウェアに任せます。設定FATがmountできない状態では`PARAM SAVE`がエラーになります。

## 表示上の注意

- yawは磁気センサーによる絶対方位ではなく、6DoF姿勢推定開始時を基準とした相対角です。
- `pressure_valid`、`estimate_valid`、`climb_valid`、`vertical_accel_valid`が偽の値は、数値カードとグラフで無効表示になります。
- `Diagnostics`のKalman innovationは対応する`*_innovation_valid`が真のときだけ値を表示します。I²C error、overrun、missed IMU sample、stream dropは0以外を警告色で表示します。
- IMU加速度校正は`READY`、`CALIBRATING`、`SAVING`、`SKIPPED`、`SAVE ERROR`として区別します。`SKIPPED`は圧力のみモードであり、校正済みを意味しません。
- BLE欄は実際のLK8EX1送信と同じ整形済み値です。LK8EX1の無効sentinel（気圧`999999`、高度`99999`、vario`9999`、温度`99`、battery`999`）は`--`として表示します。`ble_notify=0`では接続先へ実際のNotifyは行われていません。
- `All fields`タブは、GUIがまだ専用表示を持たない将来の`key=value`フィールドもそのまま表示します。
- `Serial log`タブは、既定では10 Hzテレメトリーを省略してコマンド応答を読みやすくしています。

# Vario Sound Simulator

`vario_sound_simulator.py`は、CloudBaseVarioのバリオ音をPC上で調整する独立GUIです。デバイスやCOMポートへ接続せず、現行firmwareと同じしきい値、ヒステリシス、確認距離、音程、テンポおよび予測ブザーの状態をシミュレーションします。

## インストールと起動

標準のWindows版Python 3.10以降とTkinterを使用します。音声出力用の依存関係はmonitor GUIと分けてあります。

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r tools\requirements-vario-simulator.txt
.\.venv\Scripts\python.exe tools\vario_sound_simulator.py
```

音声は48 kHz、モノラルでWindowsの既定出力デバイスへ送ります。起動時に依存関係または出力デバイスを使用できない場合は、`Start`を押したときにエラーを表示します。

## 操作

1. 分類別タブでバリオ音パラメータを編集します。有効な組合せになるとプレビューへ自動反映されます。不正な値の間は直前の有効値で動作し、JSON保存は無効になります。
2. `Climb rate`へ数値を入力するか、-15～+15 m/sのスライダーを動かします。
3. `Start`で仮想高度の更新と発音を開始します。`Stop`は直ちに無音化し、`Reset state`は仮想高度、符号反転基準および音状態を初期化します。
4. `PC volume`でPC再生音量を調整します。この値は`parameters.json`へ保存しません。

仮想高度は、設定した昇降率と実経過時間を10 ms周期で積分します。このため、`lift_confirm_distance_m`または`sink_confirm_distance_m`が0より大きい場合、符号反転後に必要な高度差を超えるまで開始音が保留されます。右側には現在の状態、発音位相、周波数、仮想高度、基準高度および確認距離を表示します。

## JSONの読込みと保存

- `New`は現行の全34項目を組込み既定値で作成します。
- `Open...`はfirmwareと同じ構造、型、範囲および項目間関係でversion 1または2を検証します。
- `Save`は確認後に現在のファイルを上書きし、`Save As...`は任意のJSONファイルへ保存します。
- 保存結果は常にUTF-8の完全な`format_version: 2`です。読み込んだ音声以外の値を維持し、音声項目だけをGUIの値で置き換えます。
- version 1専用項目と旧ボード軸項目は現行firmwareと同様に読込み時だけ無視し、保存結果には出力しません。
- 一時ファイルを書いて再読込み検証してから置換するため、書込みまたは検証に失敗した場合は既存ファイルを変更しません。

## 音の再現範囲

出力はESP32 LEDCに近い矩形波とし、`audio_duty_percent`を波形のduty比、`audio_amp_mode`をPAM8904Eの1x／2x／3xに対応する相対振幅として反映します。開始・停止時はPCスピーカー固有のクリックを抑えるため3 msのgain rampを加えます。圧電素子の周波数応答、電源電圧、基板実装および筐体共振は再現しないため、実機と同じ音圧や音色を保証するものではありません。

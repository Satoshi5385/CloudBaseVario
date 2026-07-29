# CloudBaseVario Monitor

`cloudbasevario_gui.py`は、CloudBaseVarioのTinyUSB CDCへ接続するWindows向けホストGUIです。

## 機能

- `BARO key=value ...`テレメトリーの10 Hz受信
- 気圧、高度、昇降率、鉛直加速度、BLE batteryの数値表示
- 気圧、昇降率、鉛直加速度の直近60秒グラフ
- IMUのroll／pitch人工水平儀、yaw、クォータニオン表示
- BARO、IMU、較正、姿勢、融合、BLE Notifyの状態表示
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
- validフラグが偽の値は数値カードとグラフで無効表示になります。
- BLE欄はLK8EX1へ整形される値です。`ble_notify=0`では接続先へ実際のNotifyは行われていません。
- `Serial log`タブは、既定では10 Hzテレメトリーを省略してコマンド応答を読みやすくしています。

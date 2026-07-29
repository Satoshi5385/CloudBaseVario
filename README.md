# CloudBaseVario

CloudBaseVarioは、ハンググライダーおよびパラグライダー向けの自作バリオメーターを開発するオープンソースプロジェクトです。


## 特徴
- 高分解能気圧センサーとIMUを融合した昇降率推定
- ピエゾドライバと大型スピーカーを使った大きなバリオ音
- Bluetooth(BLE)経由でのXCTrack連携
- (オプション)GPS、microSDカード、LCD


> [!WARNING]
> 本プロジェクトは開発中の実験用機器です。動作・精度・安全性を保証しません。本機器に依存せずにフライトできる環境で使用してください。


## 開発状況

現在は初期実装段階です。次の機能が実装されています。

- メインボードのGPIO定義と安全な初期状態
- 電源自己保持、スイッチ、LED、バッテリーADC
- BMP581とのI2C通信、データ取得・復旧と気圧単独の高度・昇降率推定
- ICM-42688P-HXYとのI2C通信、WHO_AM_I検証、設定read-back、データ取得・異常時の自動復旧
- GPIO14 Data Readyによるaccel/gyro取得、静止ジャイロ較正、6DoF姿勢推定
- 気圧高度と姿勢補正済み鉛直加速度の融合、および気圧単独への自動縮退
- LEDC/PAM8904Eによるバリオ音
- NimBLE NUSによるXCTrack向けLK8EX1送信
- TinyUSB CDC + MSC複合デバイスによる10 Hzモニター、JSONパラメータ共有およびコンソールコマンド
- MSCへコピーした `UPDATE.BIN`によるdual-OTA更新とbootloader rollback
- FreeRTOSタスク、共有リソース、省電力管理
- 16 MB Flash、8 MB Octal PSRAMの既定設定


## 対象環境

- 基板: Aohazuku Rev.0
- MCU: ESP32-S3-WROOM-1-N16R8
- SDK: ESP-IDF v6.0.2
- 言語: C

## ビルド

ESP-IDF v6.0.2をインストールし、ESP-IDFの環境を有効にしたシェルで実行します。

```console
idf.py set-target esp32s3
idf.py build
```

消去済みまたは新品のFlashへ初めて書き込む場合は、設定用FAT領域を一度だけ初期化してからアプリを書き込みます。

```console
idf.py -p <PORT> config-flash
idf.py -p <PORT> flash
```

代わりに、アプリを書き込んだ後、SW2とSW3を同時に押したまま電源ONしても設定FATをformatできます。両スイッチは電源投入から1秒程度押し続けてください。format後は組込み既定値の `parameters.json` が自動生成されます。この操作と `config-flash` は保存済み設定を消去するため、初期化が必要な場合だけ使用してください。

以後の通常のファームウェア更新では `flash` だけを実行します。通常起動またはFAT mount失敗時に自動formatは行いません。FATをmountできない場合もバリオ、BLE、音声およびTinyUSB CDCは起動を継続し、MSCは「メディアなし」として安全に応答します。この状態は `DIAG STATUS` の `msc_driver=1 msc_media=0` と `storage_error` で確認できます。FATを復旧するには、SW2とSW3による起動時初期化または `config-flash` を実行してください。

FATが正常な場合は、同じUSB接続がTinyUSB CDCのCOMポートとMSCドライブとして認識されます。MSCをhostが所有している間、`PARAM SAVE`は `ERR SAVE BUSY`を返します。安全な取り外しまたはUSB切断後にESP32側へremountします。

wear levelling Performance modeを使用していた旧ファームウェアから更新した実機では、Safety modeへの変更により既存FATをmountできない場合があります。その場合は必要な設定値を事前に控え、SW2とSW3による起動時初期化または `config-flash` を実行してください。

旧partition tableから本構成へ初めて移行するときは、MSC更新だけではpartition tableを変更できません。設定値を控えたうえで、GPIO0 + resetのROM download modeから `idf.py flash`でfactory imageと新partition tableを書き、その後 `config-flash`またはSW2 + SW3起動で新しい4 MiB FATを初期化してください。以後はMSCの `UPDATE.BIN`でapplicationだけを更新できます。

TinyUSB CDCのCOMポートを指定してモニターを開始します。

```console
idf.py -p <PORT> monitor
```

接続中は `BARO` で始まる `key=value` 形式の1行が10 Hzで連続出力されます。BMP581の生値・変換値、LK8EX1へ渡す5フィールド、IMUのクォータニオン／roll／pitch／yaw、姿勢補正済み鉛直加速度および各valid状態を同じ時点のsnapshotから確認できます。6DoF推定のyawは磁気方位ではなく、起動後の相対角でドリフトを含みます。

### MSCファームウェア更新

`idf.py build`は通常の `build/CloudBaseVario.bin`に加えて、同一内容で3.5 MiB上限を確認した `build/UPDATE.BIN`を生成します。

1. USBのMSCドライブ直下へ `UPDATE.BIN`をコピーします。
2. OSの「安全な取り外し」を実行します。
3. USB外部給電を接続した状態で本体を再起動します。

MSCの各WRITE(10)はwear levelling領域への実書込みが完了してからhostへ成功応答し、SCSI SYNCHRONIZE CACHEにも応答します。このため、安全な取り外しの完了時点では端末側に未完了の遅延書込みを残しません。

次回起動時、通常task開始前にESP32-S3用application imageとproject名を検証し、inactive OTA slotへ書き込みます。成功後は更新firmwareで再起動し、必須taskが生成されて10秒動作すると確定します。初回bootの確認中はTinyUSB CDC + MSCを開始しません。確定後に `UPDATE.TXT`を `CONFIRMED`へ更新して `UPDATE.PND`を削除し、その完了後にだけUSBを公開します。確定前のresetまたはcrashでは以前のfirmwareへrollbackします。

同じドライブに `parameters.json`と更新ファイルを共存できます。`UPDATE.PND`は確認待ち、`UPDATE.BAD`は拒否・rollbackされたimage、`UPDATE.TXT`はASCIIの状態表示です。USB DFU classは使用せず、MSC更新が使えない場合の最終復旧手段はGPIO0 + resetのROM download modeです。同一versionとdowngradeは許可し、secure boot／署名検証は現時点では行いません。

### Python GUI

Windows上でテレメトリー、グラフ、姿勢、BLE送信値を表示し、`PARAM LIST/SET/RESET/SAVE`を操作するTkinter GUIを同梱しています。

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r tools\requirements-gui.txt
.\.venv\Scripts\python.exe tools\cloudbasevario_gui.py
```

使用方法とパラメーター保存時の注意は [tools/README.md](tools/README.md) を参照してください。GUIとESP-IDF monitorは同じCOMポートを同時に開けないため、GUI接続前にmonitorを終了します。

プロジェクトの再現可能な既定値は `sdkconfig.defaults` と `partitions.csv` に保存しています。ローカルで生成される `sdkconfig` はバージョン管理しません。

VS CodeではESP-IDF拡張機能を使用してください。共有設定では、通常のCMake ToolsがESP-IDF環境外で自動構成を始めないようにしています。

## リポジトリ構成

- `SRC/`: ESP-IDFコンポーネントとファームウェア実装
- `components/esp_tinyusb/`: MSC書込み完了保証の修正を含むローカル`esp_tinyusb` component
- `DOC/SW_spec.md`: ソフトウェア要件と簡易設計
- `DOC/hw_spec.md`: ESP32-S3 GPIO・周辺インターフェース仕様
- `DOC/ble.md`: XCTrack連携用BLEインターフェース仕様
- `DOC/vario_sound_spec.md`: バリオ音仕様
- `DOC/CODING_RULES.md`: C言語コーディングルール


## ライセンス

現在このリポジトリで公開しているソフトウェアと文書は、[Apache License 2.0](LICENSE)の下で提供します。

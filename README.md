# CloudBaseVario

CloudBaseVarioは、ハンググライダーおよびパラグライダー向けの自作バリオメーターを開発するオープンソースプロジェクトです。


## 特徴
- 高分解能気圧センサーとIMUを融合した昇降率推定
- ピエゾドライバと大型スピーカーを使った大きなバリオ音
- Bluetooth(BLE)経由でのXCTrack連携
- USB CDC/MSCによるモニター、設定、およびファームウェア更新


> [!WARNING]
> 本プロジェクトは開発中の実験用機器です。動作・精度・安全性を保証しません。本機器に依存せずにフライトできる環境で使用してください。


## 実装機能

ファームウェアは次の機能を提供します。

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

ファームウェアのリリース版番号は `SRC/firmware_version.h` の
`CBV_FIRMWARE_VERSION`を手動で更新します。ビルド時にはこの版番号とHEADの
7桁Git hashを組み合わせてimageへ埋め込みます。未コミット変更の有無はhashへ
含めないため、実際のバイナリ内容を厳密に識別するときはSHA-256も併用します。

SW2とSW3を同時に押したまま電源ONすると設定FATをformatできます。
両スイッチは電源投入から1秒程度押し続けてください。format後は組込み既定値の
`setting.json`が自動生成されます。

以後の通常のファームウェア更新では `flash` だけを実行します。通常起動またはFAT mount失敗時に自動formatは行いません。FATをmountできない場合もバリオ、BLE、音声およびTinyUSB CDCは起動を継続し、MSCは「メディアなし」として安全に応答します。この状態は `DIAG STATUS` の `msc_driver=1 msc_media=0` と `storage_error` で確認できます。FATを復旧するには、SW2とSW3による起動時初期化または `config-flash` を実行してください。

FATが正常な場合、TinyUSB CDCのCOMポートは起動処理の早い段階から利用できます。初回IMU校正、OTA確認および起動時ファイル処理が完了するまでは、config FATをESP32側の`APP_OWNED`に維持し、MSC媒体はhostへ公開しません。全ゲート完了後、同じUSB接続のMSCドライブを有効化して`HOST_OWNED`へ切り替えます。MSCをhostが所有している間、`PARAM SAVE`は `ERR SAVE BUSY`を返します。安全な取り外しまたはUSB切断後にESP32側へremountします。

SW1による明示的な電源OFFでは、USB給電が残っていてもMSC書込み完了とworker停止を確認した後にアプリケーション側TinyUSBを停止します。この時点でCDC COMポートとMSCドライブは切断され、SAFE_STOPのLight-sleep待機へ移ります。SW1を一度解放してから2秒長押しして再起動すると再び公開されます。ROM download用USB Serial/JTAGはアプリケーション側TinyUSBとは別のため、この停止対象には含まれません。

電池駆動で起動したとき、有効な電池電圧が3.2 V以下なら通常起動せず電源保持を解除します。動作中に3.1 V以下を検出した場合は安全終了を要求し、MSC書込み中なら書込み完了後にシャットダウンします。USB外部給電中はこれらの低電圧停止を適用しません。Battery ServiceとLK8EX1の残量%は、3.2 Vを0 %、4.1 Vを100 %とする簡単なリチウムイオン放電曲線近似で表示します。

`mc_data.json`がない初回水平校正中にSW3を3秒長押しすると、その起動に限って校正をスキップします。未保存の校正候補は破棄され、IMUを停止して気圧単独で動作し、正式ビルドではMSC媒体を公開します。`mc_data.json`や設定ファイルへスキップ状態を保存しないため、次回起動では初回水平校正を再び要求します。校正中の3秒未満のSW3操作は、ボタンを離した時点で次のパラメータセットへ切り替えます。

wear levelling Performance modeを使用していた旧ファームウェアから更新した実機では、Safety modeへの変更により既存FATをmountできない場合があります。その場合は必要な設定値を事前に控え、SW2とSW3による起動時初期化または `config-flash` を実行してください。

partition tableの完全復旧には、GPIO0 + resetのROM download modeと
製造ツールを使用します。MSCの`UPDATE.BIN`はapplicationだけを更新します。

TinyUSB CDCのCOMポートを指定してモニターを開始します。

```console
idf.py -p <PORT> monitor
```

接続中は `BARO` で始まる `key=value` 形式の1行が10 Hzで連続出力されます。BMP581の生値・変換値、LK8EX1へ渡す5フィールド、IMUのクォータニオン／roll／pitch／yaw、姿勢補正済み鉛直加速度、カルマンフィルタの加速度バイアス・innovation・実効観測分散、および各valid状態を同じ時点のsnapshotから確認できます。6DoF推定のyawは磁気方位ではなく、起動後の相対角でドリフトを含みます。

### MSCファームウェア更新

`idf.py build`は通常の `build/CloudBaseVario-Aohazuku.bin`に加えて、同一内容で3.5 MiB上限を確認した `build/UPDATE.BIN`を生成します。両方のESP-IDF project nameはAohazukuシリーズ共通の機種ID `CloudBaseVario-Aohazuku`です。

1. USBのMSCドライブ直下へ `UPDATE.BIN`をコピーします。
2. OSの「安全な取り外し」を実行します。
3. USB外部給電を接続した状態、または有効な電池電圧が3.4 Vを超える状態で本体を再起動します。

更新中はMSCドライブが一時的に表示されなくなります。新しいファームウェアで10秒間正常に起動すると更新完了となり、再表示されたドライブの `UPDATE.TXT`で `state=CONFIRMED`、手動管理の`version`、7桁Git `hash`を確認できます。`setting.json`は更新時もそのまま保持されます。

更新できなかった場合も、次のように元のファームウェアで動作を継続または復旧します。詳しい理由は `UPDATE.TXT`で確認できます。

- USB外部給電がなく、電池電圧が3.4 V以下または取得できない場合: 更新を延期し、`UPDATE.BIN`を残します。USBを接続するか十分に充電して再起動してください。
- 別機種向け、旧project名 `CloudBaseVario`、または破損したファイルの場合: 更新を拒否し、ファイルを `UPDATE.BAD`へ移します。
- 新しいファームウェアが確認中にresetまたはcrashした場合: 自動的に以前のファームウェアへ戻します。

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

- `SRC/app/`: 薄い入口、段階化した起動処理、task生成・起動状態、worker実行管理
- `SRC/domain/`: SDK非依存の推定・音・system・USB所有権・LK8EX1処理
- `SRC/platform/`: ESP-IDF、NimBLE、TinyUSBおよびハードウェアアクセス
- `tools/`: Monitor GUI、Sound Simulator、共通version 1 parameter model
- `manufacturing_tools/`: カメラQR認識、製造書込み、CSV serial台帳
- `tests/`: Python回帰テストとSDK非依存Cテスト
- `components/esp_tinyusb/`: MSC書込み完了保証の修正を含むローカル`esp_tinyusb` component
- `DOC/SW_spec.md`: ソフトウェア要件と簡易設計
- `DOC/HW_spec.md`: ESP32-S3 GPIO・周辺インターフェース仕様
- `DOC/BLE_IF.md`: XCTrack連携用BLEインターフェース仕様
- `DOC/setting_json.md`: version 1設定ファイル仕様
- `DOC/vario_sound_spec.md`: バリオ音仕様
- `DOC/CODING_RULES.md`: C言語コーディングルール


## ライセンス

現在このリポジトリで公開しているソフトウェアと文書は、[Apache License 2.0](LICENSE)の下で提供します。

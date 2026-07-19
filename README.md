# CloudBaseVario

CloudBaseVarioは、ハンググライダーおよびパラグライダー向けの自作バリオメーターを開発するオープンソースプロジェクトです。

高分解能気圧センサーとIMUを組み合わせた高応答の昇降率推定、バリオ音、Bluetooth経由でのXCTrack連携を目標としています。GPS、microSDカード、LCDは将来のオプション機能として検討しています。

> [!WARNING]
> 本プロジェクトは開発中の実験用機器です。認証済みの航空計器ではなく、動作・精度・安全性を保証しません。飛行判断や安全確保を本機器だけに依存しないでください。

## 開発状況

現在はESP32-S3向け初期実装の基盤段階です。次の機能が実装されています。

- Aohazuku Rev.0のGPIO定義と安全な初期状態
- 電源自己保持、スイッチ、LED、バッテリーADC
- 共有I2Cバス、LEDC音声出力、NimBLE NUSの基盤
- FreeRTOSタスク、共有リソース、省電力管理
- 16 MB Flash、8 MB Octal PSRAM、USB Serial/JTAGの既定設定

BMP581・ICM-42688のセンサードライバ、姿勢・高度・昇降率推定、バリオ音判定、LK8EX1送信、コンソールコマンドは未完成です。

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

実機へ書き込み、シリアルモニターを開始する場合は次を実行します。

```console
idf.py -p <PORT> flash monitor
```

プロジェクトの再現可能な既定値は `sdkconfig.defaults` と `partitions.csv` に保存しています。ローカルで生成される `sdkconfig` はバージョン管理しません。

VS CodeではESP-IDF拡張機能を使用してください。共有設定では、通常のCMake ToolsがESP-IDF環境外で自動構成を始めないようにしています。

## リポジトリ構成

- `SRC/`: ESP-IDFコンポーネントとファームウェア実装
- `DOC/SW_spec.md`: ソフトウェア要件と簡易設計
- `DOC/hw_spec.md`: ESP32-S3 GPIO・周辺インターフェース仕様
- `DOC/ble.md`: XCTrack連携用BLEインターフェース仕様
- `DOC/vario_sound_spec.md`: バリオ音仕様
- `DOC/vario_audio_requirements.adoc`: 操縦者視点のバリオ音要求
- `DOC/CODING_RULES.md`: C言語コーディングルール

## ロードマップ

- ESP32-S3初期版のセンサー取得、推定、バリオ音、BLE送信、コンソール機能を完成させる
- BLEで電池残量を通知する
- GPSとmicroSDカードによる位置・ログ記録を追加する
- OTA更新を検討する
- LCD表示をオプション機能として追加する

## ライセンス

現在このリポジトリで公開しているソフトウェアと文書は、[Apache License 2.0](LICENSE)の下で提供します。

将来公開する回路図、PCBデータなどのハードウェア設計には、公開時に適切なハードウェアライセンスを別途明示します。

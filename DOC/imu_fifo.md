# IMU FIFOの100 Hz一括取得

## 目的

ICM-42688P-HXYの加速度・ジャイロODRは400 Hzを維持し、4サンプルFIFO watermark（WTM）で`sensor_task`を約100 Hz起床させる。正常時の1周期は、次の順で処理する。

1. FIFOを取得し、完全な12 byteフレームを確定する。
2. FIFOをBY-PASSからFIFO modeへ再開する。
3. 直後にBMP581を取得する。
4. FIFO内のIMUサンプルを古い時刻から姿勢推定・加速度融合へ渡す。
5. より新しいBMP timestampで気圧更新を行う。
6. 結果と診断を最大100 Hzで公開する。

BMPサンプルを次周期へ保留しない。GPIO14通知からFIFO取得とBMP581取得の間には、設定コピーや推定演算を挟まない。

## FIFO設定とデータ形式

- `COM_CFG=0x40`としてAddr_Autoを無効にする。
- `FIFO_CFG0=0x06`、`FIFO_CFG1=0x10`、`FIFO_CFG2=0x19`、`FIFO_DOWNS=0x88`を使用する。
- `INT_CFG1=0x09`でWTMをactive HighのINT1へ出力する。
- Sensor_Time 2 wordと4サンプル24 wordが揃う26 wordでWTMを成立させるため、FTHは25 wordとする。
- `FIFO_STAT0/1`のcount単位は2 byteである。high/low/highの順に個別読出しし、high部の一致を最大3回確認する。
- `FIFO_DATA=0x21`は単一アドレス連続読出しとする。先頭4 byteがSensor_Time、その後は各12 byteのgyro XYZ、accel XYZで、各値はbig-endian signed 16 bitである。

FIFOの初回転送後に残量を再確認する。転送中に到着した完全フレームは、バッチ全体が最大16サンプルに収まる間は追加取得する。部分フレーム、16サンプルを超える残量、overflow、count不整合、通信失敗はバッチ全体を推定へ渡さず、破棄数とエラーを診断へ加算する。

空FIFO、またはSensor_Timeと最初の完全フレームがまだ揃っていない状態は`ESP_ERR_NOT_FINISHED`とし、データを消去しない。非空FIFOを処理した後は、成功・失敗にかかわらず`FIFO_CFG1=0x00`から`0x10`へ切り替える。再開に失敗した場合は、再開成功を確認するまで次のFIFOデータを読まない。GPIO14 ISRは通知時に割り込みをmaskし、FIFO再開成功後だけ再有効化する。

## 周期と異常時動作

純粋Cスケジューラは`IMU_WTM`、`IMU_INIT`、`BMP_TIMER`を管理する。WTM timeoutとBMP絶対期限は別々に保持する。

- `IMU_WTM`: 完全な4サンプルを優先し、I2C転送・FIFO再開時間による小さな周期変動を許容する。WTMから15 ms経過しても通知がなければtimeoutとする。
- `BMP_TIMER`: IMU不使用、最初のIMU取得エラー、4件未満、FIFO異常、またはWTM timeout後に使用する。BMP581を絶対10 ms期限で取得し、処理時間を次周期へ累積しない。
- `IMU_INIT`: BMP_TIMERの100 Hz取得を継続しながらIMUを段階初期化する。reset、電源起動、sensor起動待ちにtask delayを使わない。BMP期限まで5 ms未満なら新しい初期化I2C処理を開始しない。

IMU異常を検出した周期でもBMP581の取得を先に試行してからIMUを無効化し、2秒後に再初期化を始める。BMPエラーだけならWTM処理を継続する。共有I2C timeoutはその周期の取得を完了した後にbus recoveryを行い、BMP581を復旧してからIMU初期化を始める。

80 MHz CPU lockはFIFO/BMPの連続取得と、その直後の姿勢・融合演算だけを囲む。初期化待ち、再試行待ち、較正保存、公開待ちでは保持しない。

## 時刻

バッチ内のIMU timestampは公称2500 us間隔とし、FIFO読出し完了後のhost monotonic timeを最新サンプルへ割り当てて逆算する。Sensor_Timeはraw診断値として保持するが、個々のサンプルとの対応を仮定してhost timeへ変換しない。前バッチ末尾以下となるtimestamp、または整合しないバッチは破棄する。

## 診断と確認

`DIAG STATUS`の`IMU_FIFO`行は、`reads`、`last_samples`、`overflows`、`errors`、`discarded_samples`に加え、次を表示する。

| キー | 意味 |
| --- | --- |
| `cadence` | `WTM`、`BMP_TIMER`、`IMU_INIT`の現在状態 |
| `wtm_cycles` | 正常に完了したWTM周期数 |
| `bmp_timer_cycles` | BMP絶対タイマで完了した周期数 |
| `last_cycle_us` | 直近の周期間隔 |
| `max_cycle_us` | 起動後の最大周期間隔 |

`missed_interrupt_count`はWTM timeoutによって`BMP_TIMER`へ縮退した回数だけを表す。FIFO error、overflow、破棄サンプル、BMP周期超過、I2C errorは別カウンタとする。

実機正常時は30秒以上観測し、WTM timeout、overflow、FIFO error、破棄件数が増えず、各バッチが4件以上、IMU処理量が380～420 sample/s、BMP回数がWTM周期数と一致することを確認する。IMU不使用・異常時は`cadence=BMP_TIMER`、BMP 95～105 Hz、融合無効、気圧単独推定継続、初期化待ち中のBMP周期超過増加なしを確認する。

## 根拠

C46550687データシートのFIFO章に従い、FIFO読出し後はBY-PASSからFIFO modeへ戻す。FIFO countはデータ読出しに応じて減算され、WTMはcountがthresholdを超えたときに成立する。
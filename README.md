# STEM Rocket Project

このリポジトリには複数のArduinoスケッチが含まれています。

## 離陸検知

`rocket_release.ino` では以下の条件で離陸を検知します。

- フライトピン入力が HIGH になったとき
- 加速度合計 `totalAccel` が `LAUNCH_ACCEL_THRESHOLD` を超えたとき

加速度しきい値は 3.0g に設定されています。
`LAUNCH_ACCEL_THRESHOLD` 定数を変更することで、この閾値を調整できます。

離陸が検知されると `startFlight()` が呼び出され、`flightStartTime` を記録
して前段記録のバッファをフラッシュし、その後通常のログ記録と動画撮影を開始します。


前段記録データは `before_flight.csv` に保存されます。

パラシュートは実際には展開せず、展開が必要になった場合はログに記録するだけです。

## デバッグ出力

`rocket_release.ino` では以下のブール変数でシリアルモニターへの
デバッグ出力を制御できます。

- `debugPrintSensors` : センサーデータ全体を出力するか
- `debugPrintBME280`  : 温度・湿度・気圧の出力
- `debugPrintMPU6050` : 加速度・ジャイロの出力
- `debugPrintGNSS`    : GNSS の位置情報出力
- `debugPrintEvents`  : `event()` 関数がログを表示するか

デフォルトではすべて `true` になっており、必要に応じて `false` に設定することで
出力を抑制できます。

## 機能の有効・無効

各センサーやカメラを使用するかどうかは以下の変数で制御できます。

- `useBME280`  : BME280 センサーを使用するか
- `useMPU6050` : MPU6050 センサーを使用するか
- `useGNSS`    : GNSS を使用するか
- `useCamera`  : カメラによる動画記録を行うか

いずれもデフォルトは `true` です。

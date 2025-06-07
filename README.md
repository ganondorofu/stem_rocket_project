# STEM Rocket Project

このリポジトリには複数のArduinoスケッチが含まれています。

## 離陸検知

`rocket_release.ino` では以下の条件で離陸を検知します。

- フライトピン入力が HIGH になったとき
- 加速度合計 `totalAccel` が `LAUNCH_ACCEL_THRESHOLD` を超えたとき

加速度しきい値は 3.0g に設定されています。

前段記録データは `before_flight.csv` に保存されます。

パラシュートは実際には展開せず、展開が必要になった場合はログに記録するだけです。

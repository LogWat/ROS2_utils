## 概要
ROS2でlocalizationとかSLAMを利用する際に使うもの

すべてComponent形式で実装

まだ動作未確認

## 内容物
- livox_cloud_merger: 2つのLivox(MID360想定)を1つの仮想LivoxのTopicにまとめる 主にLIO-SAM用 ring fieldの復元も行う (ロボット座標系から見て2つのlivoxが同じx, y座標にあることが前提の設計)
- livox_imu_collector: IMUの剛体変換+加速度成分をG->m/s^2にする
- vel2odom: **ホイールオドメトリがとれない困ったロボットに対して使う** snd\_vel + IMUで疑似オドメトリを算出
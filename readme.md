## 概要
ROS2でlocalizationとかSLAMを利用する際に使うもの

すべてComponent形式で実装

## 内容物
- livox_cloud_merger: 2つのLivox(MID360想定)を1つの仮想LivoxのTopicにまとめる 主にLIO-SAM用にring fieldの復元も行う
- livox_imu_collector: IMUの剛体変換+加速度成分をG->m/s^2にする
- vel2odom: **ホイールオドメトリがとれない困ったロボットに対して使う** snd\_vel + IMUで疑似オドメトリを算出
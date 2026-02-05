# /odom トピック ガイド

このドキュメントでは、go2_ros2_sdk における `/odom` トピックについて説明します。

## `/odom` トピックとは

`/odom` トピックは、Unitree Go2 ロボットの**LiDARベースのオドメトリ情報**を配信するトピックです。

| 項目 | 内容 |
|------|------|
| トピック名 | `/odom`（シングルモード）<br>`/robot{i}/odom`（マルチロボットモード） |
| メッセージ型 | `nav_msgs/msg/Odometry` |
| データソース | ロボット内蔵 LiDAR オドメトリ (`rt/utlidar/robot_pose`) |
| 通信方式 | WebRTC DataChannel |
| 配信周期 | 10Hz（0.1秒間隔） |

## 配信されるデータ内容

```
nav_msgs/msg/Odometry
├── header
│   ├── stamp          # タイムスタンプ
│   └── frame_id       # "odom"
├── child_frame_id     # "base_link" または "robot{i}/base_link"
└── pose.pose
    ├── position
    │   ├── x          # X座標 [m]
    │   ├── y          # Y座標 [m]
    │   └── z          # Z座標 + 0.07m オフセット [m]
    └── orientation
        ├── x          # クォータニオン X
        ├── y          # クォータニオン Y
        ├── z          # クォータニオン Z
        └── w          # クォータニオン W
```

> **注意:** `twist`（速度情報）は含まれていません。

## ソースコード

**ファイル:** `go2_robot_sdk/go2_robot_sdk/go2_driver_node.py`

### Publisher 作成（129-130行目）

```python
self.go2_odometry_pub.append(
    self.create_publisher(Odometry, 'odom', qos_profile))
```

### Publish 処理（415-442行目）

```python
def publish_odom_topic_webrtc(self):
    for i in range(len(self.robot_odom)):
        if self.robot_odom[str(i)]:
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = "base_link"

            odom_msg.pose.pose.position.x = self.robot_odom[str(i)]['data']['pose']['position']['x']
            odom_msg.pose.pose.position.y = self.robot_odom[str(i)]['data']['pose']['position']['y']
            odom_msg.pose.pose.position.z = self.robot_odom[str(i)]['data']['pose']['position']['z'] + 0.07
            odom_msg.pose.pose.orientation.x = self.robot_odom[str(i)]['data']['pose']['orientation']['x']
            odom_msg.pose.pose.orientation.y = self.robot_odom[str(i)]['data']['pose']['orientation']['y']
            odom_msg.pose.pose.orientation.z = self.robot_odom[str(i)]['data']['pose']['orientation']['z']
            odom_msg.pose.pose.orientation.w = self.robot_odom[str(i)]['data']['pose']['orientation']['w']

            self.go2_odometry_pub[i].publish(odom_msg)
```

## 起動方法

### 1. 環境変数の設定

```bash
export ROBOT_IP="192.168.xxx.xxx"   # ロボットのIPアドレス
export ROBOT_TOKEN="your_token"     # 認証トークン（必要な場合）
export CONN_TYPE="webrtc"           # 接続タイプ（デフォルト: webrtc）
```

### 2. SDK の起動

```bash
ros2 launch go2_robot_sdk robot.launch.py
```

### 3. トピックの確認

```bash
# トピック一覧を確認
ros2 topic list | grep odom

# /odom トピックの内容を表示
ros2 topic echo /odom
```

## トラブルシューティング

### `/odom` トピックが見つからない場合

1. **接続タイプを確認**
   - `CONN_TYPE=webrtc` が設定されているか確認
   - CycloneDDS モード（EDU版）では `/odom` は直接 publish されません

2. **ロボットとの接続を確認**
   ```bash
   # ノードが起動しているか確認
   ros2 node list | grep go2_driver_node
   ```

3. **他のトピックが配信されているか確認**
   ```bash
   ros2 topic list
   ```
   他のトピック（`/joint_states` など）が見えていれば接続は成功しています。

4. **WebRTC 接続の確認**
   - ロボットの IP アドレスが正しいか確認
   - ロボットが起動しているか確認
   - ネットワーク接続を確認

### データが更新されない場合

- LiDAR が正常に動作しているか確認してください
- ロボットの `rt/utlidar/robot_pose` トピックがロボット側で配信されている必要があります

## 関連ファイル

| ファイル | 説明 |
|----------|------|
| `go2_robot_sdk/go2_driver_node.py` | メインドライバーノード |
| `scripts/go2_constants.py` | RTC トピック定義 |
| `launch/robot.launch.py` | 起動用 launch ファイル |

# Go2 ROS2 SDK ナビゲーション解析レポート

## 概要

本プロジェクト（go2_ros2_sdk）は Unitree Go2 四足歩行ロボット用の ROS2 ドライバであり、ナビゲーションには **ロボット内蔵のオドメトリ** と **ROS2 Navigation2 スタック** を組み合わせて使用している。

---

## 1. オドメトリ手法

### 1.1 オドメトリの生成源：ロボット内蔵 LiDAR-IMU オドメトリ

本プロジェクトでは、**SDK側でオドメトリを独自に計算していない**。オドメトリはすべて **Go2 ロボット本体の内部処理** によって生成され、WebRTC または CycloneDDS 経由で受信している。

| 項目 | 内容 |
|------|------|
| **データソース** | ロボット内蔵 UTLiDAR モジュール |
| **WebRTC トピック** | `rt/utlidar/robot_pose` (`go2_constants.py:84`) |
| **CycloneDDS トピック** | `/utlidar/robot_pose` (PoseStamped) |
| **ROS2 パブリッシュ先** | `/odom` (nav_msgs/Odometry) + TF (`odom` → `base_link`) |
| **更新周波数** | 10 Hz（0.1秒タイマー） |

#### データの流れ

```
[Go2 ロボット内蔵処理]
  UTLiDAR + IMU → LiDAR-IMU オドメトリ計算（ロボット内部）
       ↓
  rt/utlidar/robot_pose (6-DOF: position xyz + orientation quaternion)
       ↓
[WebRTC / CycloneDDS]
       ↓
[go2_driver_node.py]
  on_data_channel_message() → self.robot_odom に保存
       ↓
  publish_odom_webrtc()       → TF ブロードキャスト (odom → base_link)
  publish_odom_topic_webrtc() → /odom トピック (nav_msgs/Odometry)
```

#### コード上の根拠

**`go2_driver_node.py:378-379`** - WebRTC メッセージからオドメトリデータを受信:
```python
if msg.get('topic') == RTC_TOPIC['ROBOTODOM']:
    self.robot_odom[robot_num] = msg
```

**`go2_driver_node.py:387-413`** - TF ブロードキャスト:
```python
def publish_odom_webrtc(self):
    odom_trans.header.frame_id = 'odom'
    odom_trans.child_frame_id = "base_link"
    odom_trans.transform.translation.x = self.robot_odom[...]['data']['pose']['position']['x']
    # ... (position + orientation をそのまま転送)
```

**`go2_driver_node.py:415-442`** - Odometry メッセージのパブリッシュ:
```python
def publish_odom_topic_webrtc(self):
    odom_msg = Odometry()
    odom_msg.pose.pose.position.x = self.robot_odom[...]['data']['pose']['position']['x']
    # ... (ロボットから受信した値をそのまま使用)
```

### 1.2 ロボット内部のオドメトリ推定手法

`go2_constants.py:104` に以下の定義がある:

```python
"SLAM_ODOMETRY": "rt/lio_sam_ros2/mapping/odometry",
```

このトピック名から、Go2 ロボット内部では **LIO-SAM (Tightly-coupled Lidar Inertial Odometry via Smoothing and Mapping)** をベースとした手法でオドメトリを生成していると推定される。LIO-SAM は LiDAR と IMU を密結合したグラフベースの SLAM/オドメトリ手法であり、以下の特徴を持つ:

- **LiDAR-IMU 密結合（Tightly-coupled）**: IMU プリインテグレーションと LiDAR スキャンマッチングの統合
- **因子グラフ最適化**: GTSAM ライブラリによるスムージング
- **リアルタイム処理**: ロボット搭載の計算リソースで実行可能

ただし、SDK が直接使用しているオドメトリトピックは `rt/utlidar/robot_pose` であり、これは UTLiDAR モジュールが出力する最終的なロボット姿勢である。

### 1.3 補助的なセンサデータ

オドメトリに直接使用されていないが、以下のセンサデータも ROS2 トピックとしてパブリッシュされている:

| センサ | トピック | メッセージ型 | 用途 |
|--------|----------|-------------|------|
| **IMU** | `/imu` | go2_interfaces/IMU | quaternion, gyroscope, accelerometer, rpy, temperature |
| **関節角度** | `/joint_states` | sensor_msgs/JointState | 12関節（各脚3自由度）の角度、IKで計算 |
| **足力センサ** | `/go2_states` | go2_interfaces/Go2State | foot_force[4], foot_position_body[12] |

**重要**: このSDKでは **EKF (拡張カルマンフィルタ)** や **UKF (無香カルマンフィルタ)** によるセンサフュージョンは実装されていない。ロボット本体が内部でセンサフュージョン済みのオドメトリを提供している。

---

## 2. 自己位置推定手法

本プロジェクトでは、自己位置推定に **2段階のアプローチ** を採用している。

### 2.1 第1段階：SLAM Toolbox (オンライン非同期SLAM)

地図生成と同時位置推定を行う。

| 項目 | 内容 |
|------|------|
| **パッケージ** | `slam_toolbox` (online_async モード) |
| **アルゴリズム** | グラフベース SLAM + スキャンマッチング |
| **ソルバ** | Ceres Solver (SPARSE_NORMAL_CHOLESKY) |
| **信頼領域戦略** | Levenberg-Marquardt |
| **入力** | `/scan` (LaserScan, PointCloud2 から変換) |
| **出力** | `/map` (OccupancyGrid) + TF (`map` → `odom`) |
| **設定ファイル** | `config/mapper_params_online_async.yaml` |

#### 主要パラメータ

```yaml
solver_plugin: solver_plugins::CeresSolver
ceres_linear_solver: SPARSE_NORMAL_CHOLESKY
ceres_trust_strategy: LEVENBERG_MARQUARDT
mode: mapping
resolution: 0.05              # 5cm グリッド解像度
use_scan_matching: true       # スキャンマッチング有効
do_loop_closing: true         # ループ閉合有効
loop_search_maximum_distance: 3.0  # ループ検出の最大距離
minimum_travel_distance: 0.5  # スキャン処理に必要な最小移動距離
minimum_travel_heading: 0.5   # スキャン処理に必要な最小回転角(rad)
map_update_interval: 5.0      # 地図更新間隔(秒)
transform_publish_period: 0.02 # TF パブリッシュ周期(50Hz)
```

#### データパイプライン

```
[Go2 UTLiDAR]
  rt/utlidar/voxel_map_compressed (WebRTC 経由)
       ↓
[go2_lidar_decoder.py]
  libvoxel.wasm で Voxel デコード → PointCloud2
       ↓
[pointcloud_to_laserscan_node]
  /point_cloud2 (3D) → /scan (2D LaserScan)
  target_frame: base_link, max_height: 0.5m
       ↓
[slam_toolbox]
  /scan + /tf(odom→base_link) → スキャンマッチング → /map + TF(map→odom)
```

### 2.2 第2段階：AMCL (Adaptive Monte Carlo Localization)

既知の地図上でのパーティクルフィルタベースの位置推定。

| 項目 | 内容 |
|------|------|
| **パッケージ** | `nav2_amcl` |
| **アルゴリズム** | 適応的モンテカルロ位置推定（パーティクルフィルタ） |
| **運動モデル** | `nav2_amcl::OmniMotionModel`（全方向移動モデル） |
| **観測モデル** | `likelihood_field`（尤度場モデル） |
| **入力** | `/scan` (LaserScan) + `/map` (OccupancyGrid) |
| **出力** | TF (`map` → `odom`) |
| **設定ファイル** | `config/nav2_params.yaml` |

#### 主要パラメータ

```yaml
# パーティクルフィルタ
min_particles: 500            # 最小パーティクル数
max_particles: 2000           # 最大パーティクル数
pf_err: 0.05                  # パーティクルフィルタ誤差閾値
pf_z: 0.99                    # パーティクルフィルタ信頼区間

# 運動モデル（オドメトリノイズ）
alpha1: 0.2                   # 回転→回転ノイズ
alpha2: 0.2                   # 並進→回転ノイズ
alpha3: 0.2                   # 並進→並進ノイズ
alpha4: 0.2                   # 回転→並進ノイズ
alpha5: 0.2                   # 並進ノイズ（全方向）

# 観測モデル
laser_model_type: "likelihood_field"
laser_likelihood_max_dist: 2.0
max_beams: 60
laser_max_range: 100.0
z_hit: 0.5                    # ヒット重み
z_rand: 0.1                   # ランダム重み

# 更新トリガ
update_min_d: 0.25            # 最小並進距離(m)
update_min_a: 0.2             # 最小回転角(rad)

# フレーム
base_frame_id: "base_link"
odom_frame_id: "odom"
global_frame_id: "odom"
```

### 2.3 TFフレーム構成

```
map (SLAM Toolbox が管理)
 └── odom (オドメトリ原点)
      └── base_link (ロボット中心)
           ├── base_footprint (接地面)
           ├── front_camera (カメラ)
           ├── radar (LiDAR)
           └── FL_hip → FL_thigh → FL_calf (脚関節、各4脚)
```

- **`map` → `odom`**: SLAM Toolbox がパブリッシュ（スキャンマッチングによる補正）
- **`odom` → `base_link`**: go2_driver_node がパブリッシュ（ロボット内蔵オドメトリ）
- **関節TF**: robot_state_publisher が URDF + JointState からパブリッシュ

---

## 3. Navigation2 スタック構成

自律ナビゲーションには Nav2 を使用:

| コンポーネント | プラグイン | 説明 |
|--------------|-----------|------|
| **グローバルプランナ** | `NavfnPlanner` (Dijkstra) | 大域的経路計画 (A*は無効) |
| **ローカルプランナ** | `DWBLocalPlanner` | Dynamic Window Based 局所経路追従 |
| **コストマップ** | StaticLayer + VoxelLayer + InflationLayer | 障害物検出・膨張処理 |
| **行動制御** | Spin, Backup, DriveOnHeading, Wait | リカバリ行動 |
| **速度スムーサ** | VelocitySmoother (20Hz, OPEN_LOOP) | 速度指令の平滑化 |

---

## 4. 全体のデータフローまとめ

```
┌─────────────────────────────────────────────────────┐
│                Go2 ロボット本体                       │
│  ┌──────────────┐  ┌─────────┐  ┌────────────────┐  │
│  │  UTLiDAR     │  │  IMU    │  │ 脚制御/状態    │  │
│  │  (3D LiDAR)  │  │         │  │                │  │
│  └──────┬───────┘  └────┬────┘  └───────┬────────┘  │
│         │               │               │            │
│     ┌───▼───────────────▼───┐           │            │
│     │ LIO-SAM ベースの      │           │            │
│     │ LiDAR-IMU             │           │            │
│     │ オドメトリ推定        │           │            │
│     └───────────┬───────────┘           │            │
│                 │                       │            │
│    robot_pose   │  voxel_map_compressed │ sportmode  │
└────────┬────────┼───────────────────────┼────────────┘
         │        │        WebRTC         │
─────────┼────────┼───────────────────────┼────────────
         │        │     ROS2 SDK 側       │
         ▼        ▼                       ▼
    ┌────────┐ ┌──────────┐      ┌──────────────┐
    │ /odom  │ │ libvoxel │      │ /go2_states  │
    │ /tf    │ │ .wasm    │      │ /imu         │
    │        │ │ デコード  │      │ /joint_states│
    └───┬────┘ └────┬─────┘      └──────────────┘
        │           ▼
        │    ┌──────────────┐
        │    │ /point_cloud2│
        │    └──────┬───────┘
        │           ▼
        │    ┌──────────────────────┐
        │    │pointcloud_to_laserscan│
        │    └──────┬───────────────┘
        │           ▼
        │    ┌──────────┐
        │    │  /scan   │
        │    └─────┬────┘
        │          │
        ▼          ▼
  ┌─────────────────────┐      ┌──────────────────┐
  │   SLAM Toolbox      │      │  Nav2 (AMCL +    │
  │ (グラフベースSLAM)  │─────▶│   DWB + Navfn)   │
  │ → /map + TF(map→odom)│      │ → /cmd_vel       │
  └─────────────────────┘      └────────┬─────────┘
                                        │
                                        ▼
                                  ┌────────────┐
                                  │ twist_mux  │
                                  │ (優先制御)  │
                                  └─────┬──────┘
                                        ▼
                                  ┌────────────┐
                                  │ /cmd_vel_out│
                                  │ → ロボットへ │
                                  └────────────┘
```

---

## 5. 結論

| 要素 | 使用手法 | 実行場所 |
|------|---------|---------|
| **オドメトリ** | LIO-SAM ベースの LiDAR-IMU 密結合オドメトリ | ロボット本体（内蔵処理） |
| **地図生成 (SLAM)** | SLAM Toolbox (Ceres ソルバ + スキャンマッチング + ループ閉合) | ROS2 SDK 側 |
| **自己位置推定** | AMCL (適応的モンテカルロ位置推定、パーティクルフィルタ) | ROS2 SDK 側 |
| **経路計画** | NavfnPlanner (Dijkstra) + DWBLocalPlanner | ROS2 SDK 側 |
| **センサフュージョン** | ロボット内蔵（SDK側にEKF/UKFなし） | ロボット本体 |

SDK 側では独自のオドメトリ計算やセンサフュージョンを行わず、ロボット本体から受信したオドメトリをそのまま信頼して使用している。自己位置推定は SLAM Toolbox による地図生成と AMCL によるパーティクルフィルタ位置推定の2段構成となっている。

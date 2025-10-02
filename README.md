# STM32 MAVLink ROS2 システム

MAVLinkベースのSTM32デバイス通信のための包括的なROS2 Jazzyワークスペース。シリアル(UART)とネットワーク(UDP)の両方のインターフェースを提供し、プロフェッショナルなGUI管理ツールを備えています。

## 概要

本システムは、MAVLinkプロトコルを使用してROS2とSTM32マイクロコントローラー間の双方向通信を実現します。サーボ、エンコーダ、RoboMasterモーター(CANベース)、高度な制御機能を持つDCモーターなど、複数のデバイスタイプをサポートしています。

## アーキテクチャ

```
┌─────────────────────────────────────────────────────────────────────┐
│                         アプリケーション層                           │
│  ┌──────────────────┐  ┌──────────────┐  ┌────────────────────┐   │
│  │ mavlink_wizard   │  │   seiretu    │  │  カスタムアプリ     │   │
│  │ (GUI管理ツール)  │  │ (ロボット制御)│  │  (ユーザーコード)   │   │
│  └──────────────────┘  └──────────────┘  └────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
                              ▲ ▼
                    ROS2 トピック & サービス
                              ▲ ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    通信インターフェース                              │
│  ┌──────────────────────────────┐  ┌──────────────────────────┐   │
│  │  stm32_mavlink_uart          │  │  stm32_mavlink_udp       │   │
│  │  (シリアル/UARTインターフェース)│  │  (ネットワーク/UDP      │   │
│  │  - /dev/ttyUSB*, ttyACM*     │  │   インターフェース)      │   │
│  │  - デフォルト115200 baud      │  │  - WiFi/Ethernet         │   │
│  │                              │  │  - デフォルトポート14550  │   │
│  └──────────────────────────────┘  └──────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
                              ▲ ▼
                         MAVLink プロトコル
                              ▲ ▼
┌─────────────────────────────────────────────────────────────────────┐
│                         ハードウェア層                               │
│  ┌─────────────┐  ┌──────────────┐  ┌──────────────┐              │
│  │   サーボ    │  │  DCモーター  │  │  エンコーダ  │              │
│  │  (最大16)   │  │  (位置制御,  │  │  (最大16)    │              │
│  │             │  │   速度制御,  │  │              │              │
│  │             │  │   電流制御)  │  │              │              │
│  └─────────────┘  └──────────────┘  └──────────────┘              │
│                                                                     │
│  ┌─────────────────────────────────────────────────┐              │
│  │         RoboMaster モーター (CANバス)           │              │
│  │  GM6020, M3508等 (最大8モーター)               │              │
│  └─────────────────────────────────────────────────┘              │
│                                                                     │
│                      STM32F446RE                                    │
└─────────────────────────────────────────────────────────────────────┘
```

## パッケージ構成

### コア通信パッケージ

#### 1. **stm32_mavlink_uart** - シリアル/UARTインターフェース
- UART経由でSTM32デバイスと直接シリアル通信
- デフォルト設定: 115200ボー、8N1
- USB-to-Serial (ttyUSB*) およびUSB CDC (ttyACM*) デバイスをサポート
- 10Hzでリアルタイムテレメトリ
- 詳細は [stm32_mavlink_uart/README.md](stm32_mavlink_uart/README.md) を参照

#### 2. **stm32_mavlink_udp** - ネットワーク/UDPインターフェース
- WiFi/Ethernet経由でのワイヤレス通信
- サーバーモード(リモート自動検出)およびクライアントモード(固定リモート)
- ESP32、Raspberry Pi、ネットワーク対応マイクロコントローラーと互換性あり
- シリアルインターフェースの代替として利用可能 - 同じトピック/サービス
- デフォルトポート: 14550
- 詳細は [stm32_mavlink_udp/README.md](stm32_mavlink_udp/README.md) を参照

#### 3. **mavlink_SDK** - 共有MAVLinkヘッダー
- 統一されたMAVLink C ライブラリv2ヘッダー
- 標準メッセージを含む(common、minimal、standard)
- カスタムメッセージサポート(robomaster、robomaster_motor)
- UARTとUDP両方のインターフェースで共有
- **ROS2ではビルドされない** (COLCON_IGNORE を含む)

### アプリケーション層

#### 4. **mavlink_wizard** - デバイス管理GUI
- Dynamixel Wizardに類似したPyQt5ベースのGUI
- 機能:
  - 自動デバイス検出
  - リアルタイムモニタリングとプロット
  - バリデーション付きパラメーター設定
  - サーボとエンコーダのキャリブレーションウィザード
  - ファームウェア管理
- 詳細は [mavlink_wizard/README.md](mavlink_wizard/README.md) を参照

## サポートデバイス

### サーボモーター
- **数量**: 最大16個のサーボ
- **制御**: 角度ベースの位置決め(-180° ～ +180°)
- **機能**: 設定可能な制限、速度制御、有効/無効化
- **トピック**: `/servo/command`, `/servo/states`

### DCモーター
- **モーターID**: 10 (専用)
- **制御モード**:
  - 位置制御 (rad)
  - 速度制御 (rad/s)
  - 電流制御 (A)
  - デューティ位置制御 (デューティサイクル + 目標位置)
- **高度な機能**:
  - カスケードPID制御(速度と位置の独立したループ)
  - 設定可能な制限と安全監視
  - リアルタイム温度・電流監視
- **トピック**: `/dcmotor/command`, `/dcmotor/state`

### RoboMaster モーター (CAN)
- **数量**: 最大8モーター
- **サポートモデル**: GM6020、M3508等
- **制御モード**: 電流、速度、位置
- **通信**: STM32経由のCANバス
- **カスタムMAVLink**: メッセージID 180-183
- **トピック**: `/robomaster/motor_command`, `/robomaster/motor_state`

### エンコーダ
- **数量**: 最大16個のエンコーダ
- **データ**: 位置 (rad)、速度 (rad/s)
- **機能**: 位置リセット、設定管理
- **トピック**: `/encoder/states`

## クイックスタート

### インストール

```bash
# ワークスペースをクローン(まだの場合)
cd ~/ros2_jazzy
source /opt/ros/jazzy/setup.bash

# 依存関係をインストール
sudo apt update
sudo apt install ros-jazzy-diagnostic-msgs ros-jazzy-sensor-msgs \
                 ros-jazzy-geometry-msgs python3-pyqt5

# 全パッケージをビルド
colcon build --packages-select stm32_mavlink_uart stm32_mavlink_udp mavlink_wizard
source install/setup.bash

# シリアルポートアクセス用にユーザーをdialoutグループに追加
sudo usermod -a -G dialout $USER
# グループ変更を反映するため、ログアウトして再ログイン
```

### 基本的な使い方

#### シリアル (UART) 通信

```bash
# 自動検出されたシリアルポートでUARTインターフェースを起動
ros2 launch stm32_mavlink_uart stm32_interface.launch.py

# 特定のシリアルポートで起動
ros2 launch stm32_mavlink_uart stm32_interface.launch.py \
    serial_port:=/dev/ttyUSB0 baud_rate:=115200

# GUI管理ツールと共に起動
ros2 launch mavlink_wizard mavlink_wizard.launch.py
```

#### ネットワーク (UDP) 通信

```bash
# サーバーモード(リモート自動検出)でUDPインターフェースを起動
ros2 launch stm32_mavlink_udp stm32_udp.launch.py

# ESP32 WiFiモジュールに接続
ros2 launch stm32_mavlink_udp stm32_udp.launch.py \
    remote_host:=192.168.4.1 remote_port:=14550 is_server_mode:=false

# GUI管理ツールと共に起動 (UDPバックエンド)
ros2 launch mavlink_wizard mavlink_wizard.launch.py
```

### コマンド例

#### サーボ制御
```bash
# サーボ1を45度に移動
ros2 topic pub /servo/command stm32_mavlink_uart/msg/ServoCommand \
    "{servo_id: 1, angle_deg: 45.0, enabled: true}" --once

# サーボ状態を監視
ros2 topic echo /servo/states
```

#### DCモーター制御
```bash
# 位置制御: 1.57ラジアン(90度)に移動
ros2 topic pub /dcmotor/command stm32_mavlink_uart/msg/DCMotorCommand \
    "{motor_id: 10, control_mode: 0, target_value: 1.57, enabled: true}" --once

# 速度制御: 5 rad/s
ros2 topic pub /dcmotor/command stm32_mavlink_uart/msg/DCMotorCommand \
    "{motor_id: 10, control_mode: 1, target_value: 5.0, enabled: true}" --once

# デューティ位置制御: 80%デューティで180度到達
ros2 topic pub /dcmotor/command stm32_mavlink_uart/msg/DCMotorCommand \
    "{motor_id: 10, control_mode: 3, target_value: 0.8, target_position_rad: 3.14, enabled: true}" --once
```

#### RoboMaster モーター制御
```bash
# 速度制御: 10 RPS
ros2 topic pub /robomaster/motor_command stm32_mavlink_uart/msg/RobomasterMotorCommand \
    "{motor_id: 1, control_mode: 1, target_velocity_rps: 10.0, enabled: true}" --once

# モーター状態を監視
ros2 topic echo /robomaster/motor_state
```

## ROS2 トピック & サービス

### 共通トピック (UART と UDP 両方)

**パブリッシュ:**
- `/servo/states` - サーボ位置と状態
- `/encoder/states` - エンコーダ位置と速度
- `/robomaster/motor_state` - RoboMaster モーターフィードバック
- `/dcmotor/state` - DCモーター状態とテレメトリ
- `/diagnostics` - システム診断

**サブスクライブ:**
- `/servo/command` - サーボ制御コマンド
- `/robomaster/motor_command` - RoboMaster モーターコマンド
- `/dcmotor/command` - DCモーターコマンド

### サービス

**サーボ:**
- `/servo/set_config` - サーボパラメーター設定

**エンコーダ:**
- `/encoder/set_config` - エンコーダパラメーター設定
- `/encoder/reset_position` - エンコーダ位置をゼロにリセット

**DCモーター:**
- `/set_dcmotor_config` - PIDパラメーターと制限値を設定
- `/get_dcmotor_config` - 現在の設定を取得

**RoboMaster:**
- `/set_robomaster_motor_config` - モーターパラメーター設定
- `/get_robomaster_motor_config` - 現在の設定を取得

## 設定ファイル

### UARTインターフェース
- `stm32_mavlink_uart/config/serial_config.yaml`
  - シリアルポート、ボーレート、タイムアウト設定
  - MAVLinkシステム/コンポーネントID

### UDPインターフェース
- `stm32_mavlink_udp/config/udp_config.yaml`
  - ネットワークアドレスとポート
  - サーバー/クライアントモード設定
  - MAVLinkシステム/コンポーネントID

### GUI管理ツール
- `mavlink_wizard/config/wizard_config.yaml`
  - GUIレイアウトと外観
  - デバイスパラメーター定義
  - プロットと可視化設定

## 開発

### 個別パッケージのビルド

```bash
# UARTインターフェースのみビルド
colcon build --packages-select stm32_mavlink_uart

# UDPインターフェースのみビルド
colcon build --packages-select stm32_mavlink_udp

# GUI管理ツールのみビルド
colcon build --packages-select mavlink_wizard

# 依存関係を含めて全てビルド
colcon build --packages-up-to mavlink_wizard
```

### カスタムアプリケーションの追加

カスタムアプリケーションは `stm32_mavlink_uart` または `stm32_mavlink_udp` に依存する必要があります:

```xml
<!-- package.xml -->
<depend>stm32_mavlink_uart</depend>  <!-- シリアル通信用 -->
<!-- または -->
<depend>stm32_mavlink_udp</depend>   <!-- ネットワーク通信用 -->
```

```python
# Python 例
from stm32_mavlink_uart.msg import ServoCommand, DCMotorCommand

# パブリッシャーを作成
servo_pub = self.create_publisher(ServoCommand, '/servo/command', 10)
motor_pub = self.create_publisher(DCMotorCommand, '/dcmotor/command', 10)
```

## ハードウェアセットアップ

### STM32 ファームウェア
- 互換性のあるSTM32F446REファームウェアの場所: `/home/imanoob/ilias2026_ws/hal_ws/f446re`
- ファームウェアが実装すべき機能:
  - MAVLink v2プロトコルパーサー
  - デバイス固有のメッセージハンドラー
  - RoboMasterモーター用CANバス通信
  - タイマーベースのテレメトリ (100ms / 10Hz)

### 配線要件
- **UART**: TX/RXをUSB-to-SerialアダプターまたはSTM32 USB CDCに接続
- **CAN**: CAN_H/CAN_LをRoboMasterモーターバスに接続
- **サーボ**: PWM出力をサーボ信号ピンに接続
- **DCモーター**: モータードライバーHブリッジをSTM32 PWM/GPIOに接続
- **エンコーダ**: クアドラチャーエンコーダ信号をSTM32タイマー入力に接続

### ネットワークセットアップ (UDP)
- ESP32 WiFiブリッジまたは直接Ethernet接続
- ESP32をUART ↔ UDPブリッジとして設定 (MAVLink透過モード)
- デフォルトポート: 14550 (MAVLink標準)

## トラブルシューティング

### シリアルポートの問題

```bash
# 利用可能なシリアルポートを確認
ls /dev/tty* | grep -E "USB|ACM"

# パーミッションを確認
ls -la /dev/ttyUSB0

# ユーザーをdialoutグループに追加 (ログアウトが必要)
sudo usermod -a -G dialout $USER

# シリアルポートをテスト
screen /dev/ttyUSB0 115200
```

### ネットワークの問題

```bash
# UDP接続をテスト
nc -u 192.168.4.1 14550

# ファイアウォールを確認
sudo ufw status
sudo ufw allow 14550/udp

# ネットワークトラフィックを監視
sudo tcpdump -i any port 14550
```

### デバイスが見つからない

1. **ハードウェア接続を確認**: LEDインジケーター、電源供給
2. **ファームウェアを確認**: STM32が互換性のあるファームウェアでフラッシュされていることを確認
3. **ROS2トピックを確認**: `ros2 topic list | grep servo`
4. **診断を監視**: `ros2 topic echo /diagnostics`
5. **MAVLink通信を確認**: 起動ファイルでデバッグログを有効化

### ビルドエラー

```bash
# クリーンビルド
rm -rf build/ install/ log/
colcon build

# 詳細出力でビルド
colcon build --event-handlers console_direct+

# 依存関係を確認
rosdep check --from-paths src --ignore-src
```

## パフォーマンスノート

- **テレメトリレート**: 全デバイスで10Hz (100ms)
- **サーボ更新レート**: テレメトリと同じ10Hz
- **DCモーターPID**: 設定可能なループレート (通常STM32で100Hz)
- **ネットワークレイテンシ**: WiFiで10-50ms、Ethernetで1-5ms追加
- **CANバス**: RoboMasterモーター用1Mbps

## ライセンス

MIT License

## コントリビューション

本プロジェクトへの貢献時の注意点:
1. ROS2命名規則に従う
2. `mavlink_SDK/` ディレクトリのMAVLink SDKを使用
3. 新機能追加時はUARTとUDP両方のインターフェースを更新
4. シリアルとネットワーク通信の両方でテスト
5. 関連するREADMEファイルを更新

## サポート & ドキュメント

- **ROS2 ドキュメント**: https://docs.ros.org/en/jazzy/
- **MAVLink プロトコル**: https://mavlink.io/en/
- **STM32 HAL**: https://www.st.com/en/embedded-software/stm32cube-mcu-mpu-packages.html
- **RoboMaster SDK**: https://github.com/RoboMaster

## 関連パッケージ

- **seiretu**: 本MAVLinkシステムを使用したロボット制御アプリケーション
- **dynamixel_controller**: Dynamixelモーター用代替サーボシステム
- **rogilink_flex**: RogiLinkセンサー通信システム

---

**バージョン**: 1.0.0
**ROS2 ディストリビューション**: Jazzy
**最終更新**: 2025-10-03

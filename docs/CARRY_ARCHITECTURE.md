# 拡張可能な構成

このブランチには`isaac_carry/`としてIsaac Labバックエンドを追加しています。[対応範囲と未検証事項](ISAAC_LAB.md)を参照してください。以下は共通制御とMuJoCo版の設計です。

## 責務の分離

|層|ファイル|役割と拡張点|
|---|---|---|
|実験設定|`config.py`、`configs/*.toml`|物理・人間・荷物・制御パラメータを保存し検証|
|人間の意図|`humans.py`|`HumanDriver.target(time)`が目標変位と速度を返す|
|上位制御|`controllers.py`|`Controller.command(obs, dt)`が機体座標の速度指令を返す|
|全身方策|`policy.py`|公式98次元観測から29関節目標を生成し、PD制御する|
|シーン|`scene.py`|G1・荷物・人間のMJCFを組み立てる|
|物理実行|`environment.py`|反力、サブステップ、状態取得、終了判定|
|表示・記録|`cli.py`|GUIなし・対話表示・動画・CSV/JSON。制御には依存させない|
|外部資産|`assets.py`、`asset_manifest.json`|固定版取得とハッシュ検証。実行と分離|

## データの契約

世界座標はZが上。上位指令は機体座標の`[vx, vy, yaw_rate]`で、単位はm/sとrad/s。
`robot_force_world`は荷物からロボットに加わる力で、下向き支持反力は負のZです。
ログの`robot_vertical_force`は逆符号の「ロボットが荷物を支える力」を表します。
人間の水平操作力は左右2把持点の合力です。左右が打ち消す内力の評価は現版に含みません。

`Observation`は時刻、胴体位置・回転、手先合力のみ。荷物真値は`metrics`へ分離しています。
胴体位置も現版はシミュレーション真値で、実機では推定器の追加が必要です。
教師用の荷物状態や人間目標を生徒方策に無意識に混入させない構成です。

物理周期は2 ms、上位制御・公式ONNXは20 ms。公式方策の周期変更はエラーにします。
関節順序、98次元観測、29次元出力、重みと設定のSHA-256も起動時に検証します。

## 上位制御の追加例

```python
import numpy as np
from cooperative_carry.config import load_config
from cooperative_carry.environment import CarryEnvironment
from cooperative_carry.interfaces import register_controller

class MyController:
    def __init__(self, config):
        self.config = config
    def reset(self):
        pass
    def command(self, obs, dt):
        return np.zeros(3)

register_controller("my_controller", MyController)
cfg = load_config("configs/carry_g1.toml")
cfg.controller.kind = "my_controller"
env = CarryEnvironment(cfg)
while not env.done:
    result = env.step()
```

既存の登録名への上書きは拒否します。外部学習器から使う場合は`controller=`で状態を持つオブジェクトを渡し、各step前に行動を設定する方法もあります。
この版はGymnasium互換APIやPPO学習器そのものを実装していません。学習時はこの環境を評価基準とし、別の学習環境との物理・観測契約を検証してください。

## 人間モデルの追加

`reset()`と`target(time)`を実装し、`register_human()`で登録します。目標は初期手先からの世界座標変位と速度です。
`human=`でオブジェクトを直接注入することもできます。
物体状態に基づいて人間自身の意図も変える場合は、外側の実験ループから人間モデルへ観測を渡します。
既存の力モデルは毎物理ステップで現在の物体位置・速度と目標との差を使うため、力学的には閉ループです。

## シーンと低レベルモデルの変更

`CarryEnvironment(..., scene_builder=callable, policy_factory=callable)`を使用します。
現在のバックエンドが要求する名前は`pelvis`、`human_avatar`、`robot_left/right`、`payload`、`payload_free`、`payload_robot_left/right`、`payload_human_left/right`です。
任意のロボット形状を無変更で扱えるという意味ではなく、別のロボットでは対応するシーンと方策アダプターが必要です。

方策アダプターは次の契約を実装します。

- コンストラクター: `(mujoco_model, policy_directory, control_dt)`。
- `q`と`default`: reset時に使う関節位置アドレスと初期姿勢。
- `reset()`、`update(model, data, command)`、`torques(model, data)`。
- `torques`の結果はMuJoCoのactuator順で、有限値・駆動上限を守ること。

COLAなどの別方策は、専用の観測履歴・関節順序・接触モデル・学習時の物理条件をこのアダプター内で明示します。
ファイル名だけ変えて公式ONNX用アダプターへ読み込ませることはしません。

## 今後の実装順

1. yawモーメントに応じた旋回、停止時の速度・力振動の低減。
2. 荷重・摩擦・人間剛性・遅延のランダム化と条件別の評価。
3. 搬送向け全身方策と小型残差方策。mjlab学習環境は別依存セットで構築。
4. 荷物の種類をシーンビルダーで追加し、重心移動・柔軟物を検証。
5. 実機センサー相当の観測、力覚推定、ROS 2入出力。

コードに隠れた手動調整を増やさず、新条件は設定・新モデルはアダプター・新形状はシーンで追加します。

# G1-dev-master-thesis

Unitree G1を用いた、人間追従と人間との協調搬送の研究用シミュレーションです。**この `main` ブランチでは人間追従を扱います。**

## ブランチを選ぶ

| ブランチ | 用途 | 状態 |
|---|---|---|
| [main](https://github.com/okome88/G1-dev-master-thesis/tree/main) | 人間追従・駅構内シミュレーション | 実装済み |
| [cooperative-carry/mujoco](https://github.com/okome88/G1-dev-master-thesis/tree/cooperative-carry/mujoco) | 人間との協調搬送 | MuJoCoで動作確認済み |
| [cooperative-carry/isaaclab](https://github.com/okome88/G1-dev-master-thesis/tree/cooperative-carry/isaaclab) | 協調搬送のIsaac Lab対応 | 実装済み・GPU実行未検証 |
| [feat/simulation-human-follow](https://github.com/okome88/G1-dev-master-thesis/tree/feat/simulation-human-follow) | 元の追従開発ブランチ | 既存コード・履歴を保持 |

依存環境が異なるため、用途ごとに別フォルダへcloneしてください。[ブランチ運用の詳細](BRANCHES.md)。

## mainでできること

- **自動追従**：仮想人物の位置・速度を使い、公式29関節ONNXでG1を歩行させる。ゲームパッド不要。
- **シナリオ評価**：直進・旋回・停止・対象喪失・通行人の横断。
- **駅構内シミュレーション**：改札・柱・ベンチと、最大50人の群衆。
- **Follow-Bench評価**：公式2D環境へ追従方式を接続し、結果を記録。

画像による人物検出・再識別、実機通信は未実装です。実行はシミュレーション内で完結します。

## クイックスタート — 人間追従

Ubuntu / WSL2 Ubuntuで、Python 3.11とGitがある環境を想定しています。GUIにはOpenGL・WSLg等が必要です。

```bash
git clone --branch main https://github.com/okome88/G1-dev-master-thesis.git g1-follow
cd g1-follow
python3.11 -m venv .venv-follow
source .venv-follow/bin/activate
python -m pip install -r requirements-sim.txt

# 形状を読み込む公式ランタイムを、固定版で用意
git clone https://github.com/unitreerobotics/unitree_rl_mjlab.git ../official-runtime
git -C ../official-runtime checkout 1425b15f73bd4095f0df53709d7c389c3eb9e790

python -m human_follow.sim --runtime ../official-runtime --scenario straight --seconds 30 --viewer
```

このPython経路では、公式C++シミュレーターや `g1_ctrl` を別に起動する必要はありません。画面なしで実行するときは `--viewer` を外します。

### 駅構内で追従させる

上記の準備後、同じフォルダ・仮想環境で実行します。

```bash
python -m pip install -r requirements-station.txt
python -m human_follow.station --runtime ../official-runtime --pedestrians 20 --seconds 60 --viewer --camera follow --output results/station-01
```

出力先には新しいフォルダ名を指定してください。条件の詳細は[駅構内の実行手順](STATION.md)を参照してください。

## ドキュメント

| 資料 | 内容 |
|---|---|
| [自動追従](AUTO_FOLLOW.md) | シナリオ、速度、距離、通行人、結果の見方 |
| [駅構内](STATION.md) | 群衆・到着客・カメラ・記録 |
| [Follow-Bench](FOLLOW_BENCH.md) | 2Dベンチマークとの接続・反復評価 |
| [地形歩行の学習・評価](docs/LOCOMOTION.md) | 既存のmjlab学習実験 |
| [公式C++モデルの起動](HOW_TO_USE.md) | ゲームパッドを使う別の実行経路 |
| [モデルと出典](models/README.md) | 公式ONNX・設定・ハッシュ・ライセンス |
| [追従の初期計画](HUMAN_FOLLOW_PLAN.md) | 計画時点の目標。現在の実装状況は上記手順を参照 |

詳細資料には以前のフォルダ名や `feat/simulation-human-follow` の起動例が残っています。新規セットアップはこのREADMEの `main` の手順を基準にしてください。

## 主な構成

| パス | 役割 |
|---|---|
| `human_follow/` | 追従制御、シミュレーション、評価 |
| `configs/` | 通行人などの設定 |
| `tests/` | 自動テスト |
| `docs/validation/` | 保存済みの検証結果 |
| `models/` | 公式モデルと出典情報 |
| `g1_locomotion.py` | mjlabによる地形歩行実験 |

## 検証と制限

```bash
python -m unittest discover -s tests -v
```

任意依存や外部環境がない場合は、一部テストがスキップされます。記録済み結果は `docs/validation/` を参照してください。

公式モデルの前進指令上限は1.0 m/s（3.6 km/h）です。時速4 kmでの動作や、実機での追従性能を保証するものではありません。モデル・依存ライブラリの利用条件は各配布元のライセンスに従います。

# G1 協調搬送 — Isaac Lab

人間とUnitree G1の協調搬送を、Isaac Lab上で評価するための開発ブランチです。

**ブランチ：`cooperative-carry/isaaclab` ／ 状態：対応コード実装済み・GPU実行未検証**

[人間追従](https://github.com/okome88/G1-dev-master-thesis/tree/main) · [動作確認済みのMuJoCo版](https://github.com/okome88/G1-dev-master-thesis/tree/cooperative-carry/mujoco) · [ブランチ一覧](BRANCHES.md)

## 対応状況

| 項目 | 状態 |
|---|---|
| AppLauncher・InteractiveSceneによる起動経路 | 実装済み、Isaac上で未実行 |
| 固定版G1のMJCF → USD変換 | 接続済み、変換実行は未検証 |
| 公式ONNXの観測・関節対応 | CPUで既存アダプターとの推論一致を確認 |
| 力・反力・重心周りのモーメント | 実装済み、計算を単体テスト |
| PhysX上の歩行・搬送成功 | **未検証** |
| 並列学習・PPO・キーボード操作 | 未実装 |

通常CIはCPUテストとMuJoCo回帰試験です。**CIの成功はIsaac Labでの動作成功を意味しません。**

## 必要な環境

| 項目 | 対象 |
|---|---|
| Python | 3.11 |
| Isaac Sim | 5.1.0 |
| Isaac Lab | 2.3.0（固定コミットを使用） |
| 構築手順 | Ubuntu向け |
| GPU / RAM | 公式要件：VRAM 16 GB以上 / RAM 32 GB以上 |

**まず[Isaac Lab環境の構築手順](docs/ISAAC_LAB.md)を実施してください。** MuJoCo用の `.venv` や `uv.lock` をIsaac環境へ適用しないでください。

## コードの取得

```bash
git clone --branch cooperative-carry/isaaclab https://github.com/okome88/G1-dev-master-thesis.git g1-carry-isaaclab
cd g1-carry-isaaclab
```

## 起動 — 専用環境の準備後

Isaac Lab用のPython 3.11環境を有効にし、リポジトリ直下で実行します。

```bash
# ソフトウェア構成を確認。物理実行の検証ではありません
python scripts/run_isaac_carry.py --check

# 固定版のG1資産を取得
python -c "from cooperative_carry.assets import fetch_assets; fetch_assets('.cache/robots/unitree_g1')"

# 先に荷物なしで安定性を確認
python scripts/run_isaac_carry.py --config configs/walk_g1.toml --duration 5 --require-stable --device cuda:0

# 搬送試験
python scripts/run_isaac_carry.py --config configs/carry_g1.toml --device cuda:0 --require-goal
```

GUIなしの実行は `--headless` を追加します。初回はUSD変換と拡張の読み込みが発生します。

## 構成と拡張

| パス | 役割 |
|---|---|
| `isaac_carry/scene.py` | USD変換とシーン構築 |
| `isaac_carry/backend.py` | PhysXの状態取得・力の適用・終了判定 |
| `isaac_carry/policy.py` | 公式ONNXの観測・行動変換 |
| `isaac_carry/preflight.py` | 実行環境の確認 |
| `cooperative_carry/controllers.py` / `humans.py` | MuJoCo版と共用する上位制御・人間モデル |
| `scripts/run_isaac_carry.py` | 起動とログ出力 |

## ドキュメントと検証

- [環境構築・起動・GPU検証手順](docs/ISAAC_LAB.md)
- [共通部分とMuJoCo版の拡張設計](docs/CARRY_ARCHITECTURE.md)
- [MuJoCo版の検証記録](docs/CARRY_VALIDATION.md) — Isaac版の結果ではありません
- [公式モデルと出典](models/README.md)

GPU試験には、対応環境を用意したself-hosted runnerで手動ワークフローを実行する経路があります。GPU runnerの準備・実行は未実施です。

初版は1環境の評価用です。MuJoCoとPhysXの物理特性は同一ではなく、方策の追加調整・学習が必要になる可能性があります。

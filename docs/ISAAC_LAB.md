# Isaac Lab対応（開発版・GPUでの実行は未検証）

更新: 2026-11-15。ブランチ: `cooperative-carry/isaaclab`。

**Isaac Lab用のコードは追加済みですが、シミュレーター起動・USD変換・PhysXでの搬送成功はまだ検証していません。**
手元の環境はPython 3.12、RTX 2080 SUPER 8 GBで、Isaac Sim/Isaac Labは未導入です。
通常CIが成功しても、それはCPUテストとMuJoCo回帰試験の成功であり、Isaacで動いた証拠ではありません。

## 実装範囲

|項目|状態|
|---|---|
|Isaac LabのAppLauncherとInteractiveScene|起動経路を実装、GPU起動未検証|
|G1 MJCFからUSDへの変換|公式MjcfConverterを接続、変換実行未検証|
|関節名対応、98次元観測、ONNX推論|実装。既存MuJoCoアダプターとの推論一致をCPUで検証|
|荷物・手先の力と反力|PhysX外力APIへ接続。世界座標・重心周りのモーメント計算を単体検証|
|人間役|有限力の手先モデル＋頭・胴・手の表示。全身歩行の人体モデルではない|
|直進・横方向軌道・停止|既存の人間モデルと制御器を共用|
|記録|設定・実行環境・方策ハッシュ・CSV・終了理由・例外を出力|
|並列環境・PPO学習・キーボード操作|Isaac版では未実装。初版は1環境での評価用|

## 固定する環境

- Ubuntu 22.04/24.04 x86_64で構築する想定。公式推奨のGPUドライバーが必要。
- Python **3.11**、Isaac Sim **5.1.0**、Isaac Lab **2.3.0**。
- Isaac Lab固定コミット: `3c6e67bb5c7ada942a6d1884ab69338f57596f77`。
- 公式要件はGPUメモリ16 GB以上、RAM32 GB以上。手元8 GBでの動作は保証しません。
- MuJoCo用`.venv`、`uv.lock`はIsaac用環境へ適用しないでください。PyTorchとNumPyはIsaac側の環境を使います。

## Ubuntuでの構築手順

以下は適合GPUを持つPC・サーバー向けです。既存のPython環境と分離します。
Python 3.11とGitを用意した後、空いている作業ディレクトリで実行してください。

```bash
git clone --branch cooperative-carry/isaaclab https://github.com/okome88/unitree-g1-human-follow.git g1-carry-isaaclab
git clone --branch v2.3.0 https://github.com/isaac-sim/IsaacLab.git IsaacLab-carry
git -C IsaacLab-carry checkout 3c6e67bb5c7ada942a6d1884ab69338f57596f77
python3.11 -m venv env-isaac-carry
source env-isaac-carry/bin/activate
python -m pip install --upgrade pip
python -m pip install 'isaacsim[all,extscache]==5.1.0' --extra-index-url https://pypi.nvidia.com
python -m pip install torch==2.7.0 torchvision==0.22.0 --index-url https://download.pytorch.org/whl/cu128
cd IsaacLab-carry
./isaaclab.sh --install none
cd ../g1-carry-isaaclab
python -m pip install -r requirements-isaac.txt
python -m pip check
python scripts/run_isaac_carry.py --check
```

Isaac Simの初回起動では利用規約の確認と追加拡張の取得が発生します。規約への同意はセットアップ利用者が確認してください。
`--check`はソフトウェアの版とGPU情報を調べるだけです。結果がreadyでも物理実行の検証にはなりません。
別のシェルでは必ず`env-isaac-carry`を有効にしてください。

## G1資産の準備と無負荷試験

```bash
python -c "from cooperative_carry.assets import fetch_assets; fetch_assets('.cache/robots/unitree_g1')"
python scripts/run_isaac_carry.py --config configs/walk_g1.toml --duration 5 --require-stable --device cuda:0
```

初回は固定版G1の`g1.xml`をIsaac LabのMJCFインポーターでUSDへ変換します。
`.cache/robots/unitree_g1/isaaclab-2.3/`へ保存します。手指付きの別G1 USDに置き換えると関節数・名前チェックで停止します。
同じ29関節構成で変換済みのUSDがあれば、`--g1-usd /absolute/path/g1.usd`で指定できます。

**まず無負荷試験で起動・浮遊基部・姿勢・歩行を確認し、転倒した状態で搬送成功を主張しないでください。**
MuJoCo用方策のPhysXへの転送なので、摩擦、接触、慣性、初期配置の差によって追加調整や学習が必要になる可能性があります。

## 搬送の実行

```bash
# ウィンドウ付き
python scripts/run_isaac_carry.py --config configs/carry_g1.toml --device cuda:0

# サーバー・合格条件付き（到達しなければ終了コード1）
python scripts/run_isaac_carry.py --config configs/carry_g1.toml --headless --device cuda:0 --require-goal

# 横方向変化・停止応答
python scripts/run_isaac_carry.py --config configs/carry_curve.toml --headless --device cuda:0
python scripts/run_isaac_carry.py --config configs/carry_stop.toml --headless --device cuda:0 --require-stable
```

設定ファイルはMuJoCo版と共通です。既定の資産・モデルパスはリポジトリ直下から解決します。
人間・荷物の把持点には有限のばね力を適用し、G1に逆向きの力と重心からのモーメントを与えます。
荷物の位置を書き換えるのは初期化時だけで、実行中は外力で動かします。ロボット基部は固定しません。

Isaac版の人体・ハンドルの表示はMuJoCo版と異なります。接触把持は再現せず、把持点による理想化を共通にしています。
関節摩擦はMuJoCoのfrictionlossとPhysXの無次元摩擦係数を同一視せず、トルク側で近似しています。これは厳密な同一物理ではありません。

## 検証の方法と状態

- CPU: 四元数の順序、回転、把持点速度、力・モーメント、ONNX観測と推論一致をテスト。
- 通常GitHub Actions: CPUテスト＋MuJoCoの既存試験。GPU検証は行いません。
- `Isaac Lab GPU smoke (manual)`: 適合GPU付きself-hosted runnerを準備し、手動実行するワークフローを追加。
- runnerには`linux`、`x64`、`isaaclab-2.3-gpu`ラベルと、上記環境を有効にした`python`が必要。
- 現時点ではGPU runnerの登録・実行はしていません。大量並列学習用の環境ではありません。

GPU smokeは無負荷5秒の安定性と、その後の搬送目標到達をそれぞれ必須にします。
出力された`metadata.json`、`summary.json`、`trajectory.csv`、失敗時の`error.txt`を比較してください。

## 拡張点

`isaac_carry/backend.py`がIsaac依存部分です。`cooperative_carry/controllers.py`と`humans.py`は共用します。
低レベルモデルは`isaac_carry/policy.py`、シーンと変換は`isaac_carry/scene.py`で独立しています。
別のAIモデルは専用の観測・関節対応を実装して差し替えます。COLAやPPO学習器は現版に組み込んでいません。

## 公式参照

- [Isaac Lab 2.3の環境要件](https://isaac-sim.github.io/IsaacLab/v2.3.0/source/setup/installation/index.html)
- [Isaac Sim 5.1＋Isaac Labの導入手順](https://isaac-sim.github.io/IsaacLab/v2.3.0/source/setup/installation/pip_installation.html)
- [参照した固定版ソース](https://github.com/isaac-sim/IsaacLab/tree/3c6e67bb5c7ada942a6d1884ab69338f57596f77)

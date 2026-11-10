# ゲームパッド不要の自動追従（シミュレーション専用）

2Dの公式ベンチマークとの接続・反復評価は [FOLLOW_BENCH.md](FOLLOW_BENCH.md) を参照してください。

公式29関節ONNXをそのまま使い、MuJoCo内の仮想人物を追従します。
人物は緑、通行人は赤の簡略形状です。画像認識ではなくシミュレーター内の位置・速度を観測します。
実機SDK、DDS、ネットワーク送信は使いません。`g1_ctrl` と公式の `simulate` を起動する必要もありません。
既存C++ランタイムへのパッチではなく、同じモデルと設定を使うPython実行経路です。

## WSL2 Ubuntu 22.04で起動

以下はUbuntu側で実行します。Python 3.11を推奨します。
既存の `unitree_rl_mjlab` Conda環境を使える場合の例です。

```bash
cd ~/g1-official-demo/unitree-g1-human-follow
git fetch origin
git switch feat/simulation-human-follow
conda activate unitree_rl_mjlab
python -m venv .venv-follow
source .venv-follow/bin/activate
python -m pip install -r requirements-sim.txt
python -m unittest discover -s tests -v
python -m human_follow.validate
python -m human_follow.sim --scenario straight --seconds 30 --output results/straight.json
python -m human_follow.sim --scenario turn --seconds 30 --viewer
```

初回のみ環境を作成します。以後は `source .venv-follow/bin/activate` で再利用します。
GUIにはWSLg/OpenGLが必要です。画面なしの実行には `--viewer` を付けません。
ビューアを閉じるかCtrl+Cで終了できます。実機に停止指令を送る処理はありません。

`--runtime` の既定値は `~/g1-official-demo/official-runtime`。
別の場所なら `--runtime /path/to/official-runtime` を指定します。
必要な公式ソースは次の固定版です（既存環境が同じ版なら再取得不要）。

```bash
git clone https://github.com/unitreerobotics/unitree_rl_mjlab.git official-runtime
git -C official-runtime checkout 1425b15f73bd4095f0df53709d7c389c3eb9e790
```

`simulate/config.yaml` の `use_joystick` やネットワーク設定は読みません。
公式C++実行経路で `Waiting for connection rt/lowstate` が出るのはシミュレーターから状態を受け取れていない状態ですが、この新経路は通信待ち自体がありません。

## 条件と評価

### コードを編集しない条件設定

```bash
# 追従距離2m、緩い左旋回、横断する通行人3人
python -m human_follow.sim --scenario turn --distance 2.0 --turn-rate 0.08 --pedestrians 3 --pedestrian-path crossing --pedestrian-speed 0.25 --seconds 30 --viewer
# 通行人なしの右旋回
python -m human_follow.sim --scenario turn --turn-rate -0.1 --pedestrians 0 --viewer
# 任意の初期位置と直線経路をJSONで指定
python -m human_follow.sim --scenario straight --pedestrian-routes configs/pedestrians.example.json --viewer
```

|設定|引数|範囲・意味|
|---|---|---|
|目標追従距離|`--distance 1.5`|1.2～3.0 m。人物初期位置も距離+0.5 mに設定|
|人物の旋回角速度|`--turn-rate 0.1`|−0.3～0.3 rad/s。正=左、負=右、0=直進。`turn` 時のみ有効|
|通行人数|`--pedestrians 3`|0～20。省略時 `crossing` は1人、他は0人|
|通行人の経路|`--pedestrian-path crossing`|`crossing` 横断／`parallel` 並行／`oncoming` 対向／`stationary` 静止|
|通行人の速度|`--pedestrian-speed 0.25`|0～1.5 m/s。静止経路では無効|
|各通行人の個別経路|`--pedestrian-routes ファイル.json`|人数・経路プリセットを置換。`--pedestrians` と併用不可|

JSONは `[[x, y, vx, vy, radius], ...]`。単位はmとm/s、座標は世界座標、半径は0.1～1 mです。
各人は開始2秒後から指定速度で直進します。折れ線や周回の任意経路、設定GUIは未実装です。
プリセットは2m間隔で配置し、横断はy=-2から、並行・静止はy=1.5～2、対向はy=1.5を通ります。
設定値は結果JSONにも保存します。指定可能範囲全体での歩行成功を保証するものではありません。

**追従方法は現在1種類**：候補位置選択＋等速予測＋サンプリングMPC（`adaptive_sampling_mpc`）。
`straight` など5種類は試験シナリオで、追従アルゴリズムの種類ではありません。
固定後方追従、横並び追従、PPO、論文どおりのMPPIを切り替える機能はまだありません。

設定追加後は18件のテストと既存5条件の物理検証が通過。
距離2 m・旋回0.08 rad/s・横断3人・人物0.4 m/s・seed 42の30秒試験も転倒なしで完了しました。
平均対人距離2.026 m、最終1.851 m、通行人との最小近似クリアランス0.291 m。
結果は [configurable-turn.json](docs/validation/configurable-turn.json) に保存しています。

- `--scenario straight|turn|stop|loss|crossing`：直進、緩い旋回、停止、3秒の対象喪失、通行人横断。
- `--speed 0.4`：仮想人物の速度。指定範囲は0～0.8 m/s。4 km/hは対象外です。
- `--seed 42`：探索の乱数を固定。
- `--seconds 30`、`--output results/run.json`：時間と測定結果。
- 距離の目標は1.5 m。接近・古い観測・対象喪失・予測経路なしでは速度指令をゼロにします。
- 追従計画10 Hz、ONNX 50 Hz、物理500 Hz。速度指令の制限は前進0～0.8、横±0.25 m/s、旋回±0.7 rad/sで、公式範囲内です。

人物・通行人の形状は描画専用で、身体衝突の物理はありません。通行人は半径付き平面円として計画に入ります。
クリアランスはロボット半径0.35 mの近似値です。接触安全性や群衆内の成功を保証しません。
停止は歩行モデルにゼロ速度を入力するため、実際の身体には慣性・姿勢調整が残ります。
転倒検出時はエラー終了します。自動で立ち上がらせたり、失敗をリセットして隠したりしません。

## 論文の評価と適用範囲（2026-11-10）

元のリポジトリ `32f53ef` は歩行学習・公式モデル・追従計画のみで、追従実装はありませんでした。
外部には[Go1向け追従事例](https://exotech-labs.com/follow)や
[Go2向け実装](https://github.com/BeBecpp/DimOs_Windows)がありますが、このG1公式ONNX・MuJoCo経路への直接対応は確認できませんでした。
これは「人間追従の既存ソフトが世界に存在しない」という結論ではありません。

|論文|適用評価|今回の採用／未採用|
|---|---|---|
|[2609.25031](https://arxiv.org/html/2609.25031v1)|楕円状の対人空間からPPOで位置を選び、MPPIとCBFで制御。上位追従と下位制御の分離はG1にも使える。車輪モデルのCBF保証は二足歩行には移せない|距離・障害物・滑らかさを考慮する構成と追跡品質低下時の停止を参考にした。PPO学習・磁気トラッカー・CBFは実装していない|
|[2510.11308 Adap-RPF](https://arxiv.org/html/2510.11308v1)|等速予測、半環状候補、視認性・近接・距離・移動量・前候補との連続性の評価は再学習不要で導入しやすい。論文の差動二輪モデルはG1用に置換が必要|後方半環の格子候補・等速予測・簡略視線遮蔽・連続性評価を採用。Sobol、画像投影IoU、MPPI重み付き更新は未実装。制御は衝突チェック済み候補を選ぶサンプリングMPC|

[Adap-RPF公式ページ](https://adap-rpf.github.io/)は調査時点でコード公開予定と記載。
2609.25031の本文からG1向けにそのまま使える学習済み方策は確認できませんでした。
この実装は論文を参考にした小規模ベースラインであり、論文の再現実装・性能再現ではありません。
知覚、再識別、LiDAR地図、混雑環境ベンチマーク、学習済み同行位置方策は今後の拡張です。

## 公式ONNXとの整合

収録ONNX・YAML・ライセンスのSHA-256を起動時に照合。モデルファイルは変更しません。
固定版シーンXMLも照合し、センサーとアクチュエーターの関節順序を検査します。
観測は公式YAML順で98要素（角速度3、重力3、速度指令3、位相2、相対関節位置29、速度29、前回行動29）。
公式C++の位相更新と停止時ゼロ位相、行動尺度・オフセット・PDゲイン・関節トルク制限に対応します。
初期状態は既定立位で、2秒間ゼロ速度を入力します。公式FSMの起立手順自体は再現しません。
参照：[公式観測実装](https://github.com/unitreerobotics/unitree_rl_mjlab/blob/1425b15f73bd4095f0df53709d7c389c3eb9e790/deploy/include/isaaclab/envs/mdp/observations/observations.h)。

## 実測結果（2026-11-10）

WSL2 Ubuntu 22.04、Python 3.11.18、MuJoCo 3.3.7、ONNX Runtime 1.22.0 CPU、NumPy 2.2.6。
12件の単体テスト通過。実ONNXを使った各30秒、人物速度0.4 m/s、seed=42の物理検証も5条件すべて合格。WSLg表示付きの3秒起動も終了コード0で完了。

|条件|最終対人距離 m|最小対人距離 m|転倒|
|---|---:|---:|---|
|直進|1.484|1.483|なし|
|旋回|1.486|1.472|なし|
|人物停止|1.334|1.312|なし|
|3秒対象喪失・復帰|1.484|1.483|なし|
|通行人横断|1.449|1.290|なし|

対象喪失150制御フレームで速度指令はすべてゼロ。横断の最小近似クリアランスは0.156 m。
JSONは [docs/validation](docs/validation) に保存しています。
`python -m human_follow.validate` で再実行でき、転倒、時間未完了、距離逸脱、移動不足、喪失時の非ゼロ指令、近似接触を失敗扱いにします。
これは限定された単一seed・低速・平地試験です。4 km/h、任意の混雑環境、知覚誤差、実機の評価はしていません。

# 駅コンコースでのG1人間追従

架空の30 m×14 mの駅コンコースに、柱6本、改札機6台、ベンチ4台、案内板、点字ブロックを配置します。
G1は収録済みの公式29関節ONNXでMuJoCo内を歩きます。実機・DDS・ネットワーク送信は使いません。
既存の直進・旋回シミュレーションとFollow-Bench評価はそのまま使えます。

## 初回の準備（WSL2 Ubuntu、Python 3.11）

```bash
cd ~/g1-official-demo/unitree-g1-human-follow
git fetch origin
git switch feat/simulation-human-follow
git pull --ff-only origin feat/simulation-human-follow
conda activate unitree_rl_mjlab
source .venv-follow/bin/activate
pip install -r requirements-station.txt
```

仮想環境がなければ、`source` の前に `python -m venv .venv-follow` を実行してください。
公式ランタイムは既存の `~/g1-official-demo/official-runtime` を使用します。
別の保存先なら起動時に `--runtime /path/to/official-runtime` を追加します。
公式固定版 `1425b15f73bd4095f0df53709d7c389c3eb9e790` を使ってください。

## まず20人で起動

```bash
python -m human_follow.station \
  --pedestrians 20 --crowd-pattern bidirectional \
  --speed 0.4 --crowd-speed 0.8 --distance 1.5 \
  --seconds 60 --viewer --camera follow \
  --output results/station-01
```

緑色の人物が固定の追従対象です。G1は改札を通り、緩く曲がる対象を追従します。
人物は左右方向の対向流と、コンコースを横断する2方向の流れを作ります。
対象は開始後22秒から3秒間停止します（立位安定化2秒＋群衆時間20秒）。
対象停止をなくす場合は `--target-stop-for 0` を追加してください。
全景を見る場合は `--camera overview`、画面なしなら `--viewer` を外します。
出力先は新しいディレクトリを指定してください。同じ名前の結果への上書きは拒否します。

## 到着客の流入と人数の変更

```bash
# 最大30人。初期15人から、12秒後の群衆時間以降に毎秒3人ずつ流入
python -m human_follow.station \
  --pedestrians 30 --crowd-pattern arrival-wave \
  --seconds 60 --viewer --output results/station-arrival-01

# 50人、通行人の基準希望速度1.2 m/sの混雑ストレス試験
python -m human_follow.station \
  --pedestrians 50 --crowd-speed 1.2 \
  --seconds 60 --viewer --output results/station-crowded-01
```

`--pedestrians` は同時在場人数の上限（0～50）です。退出者は入口から補充します。
安全な入口位置を確保できない場合は空き枠のまま待つため、常に指定人数になるとは限りません。
`arrival-wave` でも退出者の補充は続きます。実際の列車時刻表に合わせた流入ではありません。
通行人の希望速度は `--crowd-speed` の75～120%から個別に選びます。
追従対象の速度は別の `--speed` で指定します（0.05～0.8 m/s、検証基準は0.4 m/s）。
歩行者の回避・混雑・壁近傍では希望速度より遅くなります。

## 記録と判定

- `report.json`: 転倒、MuJoCo接触、追従距離、対人・設備クリアランス、対象可視率、見失い時の指令、流入・退出数。
- `trace.jsonl`: 10 HzのG1位置、評価用の対象正解位置、人物位置・速度、可視判定、指令、停止理由。
- `settings.json`: seed、人数、速度、視野、停止条件などの実行設定。

人・設備との接触または転倒を検出したら、その試行は停止してエラー終了します。
MuJoCo接触は全身の実装済み衝突形状で判定し、床との通常接地は失敗に含めません。
平面クリアランスはロボット半径0.35 m・人物半径0.25 mによる近似で、全身表面間距離ではありません。
接触サンプル数は接触拘束の検出数であり、独立した衝突事故の件数ではありません。
60秒完走と目的地到達は別々に記録します。`target_reached_goal` が目的地到達の判定です。

```bash
# 6条件×60秒。失敗した条件も保存し、全条件合格のときのみ終了コード0
python -m human_follow.station_validate --output results/station-validation-01

# 静止画を保存する例（画面なし。EGLを利用できる環境）
MUJOCO_GL=egl python -m human_follow.station \
  --seconds 8 --snapshot results/station.png --output results/station-snapshot-01
```

## 何を再現しているか

歩行者は [JuPedSim 1.4.2](https://github.com/PedestrianDynamics/jupedsim) のCollisionFreeSpeedModelで相互回避し、
同じ駅形状の歩行可能領域を通って目的地へ進みます。
G1の実測位置は半径0.45 mの静止円として10 Hzで群衆側へ反映します。
公開APIで位置を更新できないため円を作り直し、更新時に0.02秒の引き継ぎ間隔が生じます。
円の再配置に失敗した回数は `robot_crowd_proxy_update_failures` に残します。
全身運動の予測や、人がG1の動きを理解して譲る認知モデルではありません。

人物の胴体・頭・手足と歩行アニメーションは手続き的な簡略表現です。
人物の動きは平面モデルで、MuJoCo側では補間した運動学的コライダーを使います。
人の転倒、押し合い、柔らかい身体接触、手荷物、待ち行列、集団行動はまだ再現していません。
実際の駅の構内図・実測人流に合わせた校正も未実施です。

対象の可視判定は水平視野（既定140度）・距離（8 m）・設備や人物による中心視線の遮蔽で行います。
見失い中は正解の対象位置・速度を追従制御へ渡さず、ゼロ指令を出します。
ただし地図と通行人の位置・速度は正解値を使います。カメラ画像からの検出・追跡・再識別は次段階です。
見失い後の能動探索はありません。同じ対象IDが再び見えたときだけ追従を再開します。

実機や通常歩行速度での追従成功を示すものではありません。公式モデル、設定、ライセンスは変更していません。

## 検証

```bash
G1_STATION_RUNTIME=~/g1-official-demo/official-runtime \
FOLLOW_BENCH_ROOT=../follow-bench \
python -m unittest discover -s tests -q
```

駅の遮蔽、対象情報の遮断、流入、停止、seed再現性、公式29関節の維持、実際の設備接触をテストします。
駅を含む物理評価の結果と判定条件は `docs/validation/station_initial/` を参照してください。

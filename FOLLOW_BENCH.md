# Follow-Benchで追従計画を評価する

既存の `Follower` を[公式Follow-Bench](https://github.com/MedlarTea/follow-bench)のIR-SIMへ接続しました。
これは2Dの追従計画評価です。G1の関節・ONNX・転倒・実機通信は実行しません。
二足歩行の物理評価は引き続き `python -m human_follow.validate` で行います。

## 環境構築（WSL2 / Ubuntu、Python 3.11）

```bash
git clone https://github.com/MedlarTea/follow-bench.git ../follow-bench
git -C ../follow-bench checkout 202d1645015f44b965a474211d5ef2bc67a00fe3
python -m venv .venv-bench
source .venv-bench/bin/activate
pip install numpy==2.2.6 pyyaml opencv-python-headless==5.0.0.93 scipy==1.17.1 scikit-image==0.26.0
pip install -e ../follow-bench/ir-sim
```

この接続では他のプランナーをimportしないため、PyTorch等の比較手法用依存関係は不要です。
ONNX試験も行う場合は既存の `requirements-sim.txt` を導入してください。
IR-SIMの依存定義にはcv2等が不足していたため、上記で明示しています。

## 実行

このリポジトリのルートで実行します。

```bash
python -m human_follow.follow_bench \
  --bench-root ../follow-bench \
  --scenario \
    socialRPF/example/robot_person_following/config_target_traj/uTurn_back_1.yaml \
    socialRPF/example/robot_person_following/config_target_traj/eight_back_1.yaml \
    socialRPF/dynamic_scenarios/config/NormalPerpendicularOrcaH10W0_back_1.yaml \
  --steps 1500 --seeds 0 1 2 \
  --target-speed 0.4 --distance 1.5 \
  --output results/follow-bench-initial
```

各試行は150秒（0.1秒×1500ステップ）。出力先は未作成のディレクトリを指定してください。
既存結果への上書きを拒否し、途中失敗時も例外を出してログを残します。
時間の完走は経路全体の完了や追従成功を意味しません。

`--scenario` は公式リポジトリ内のYAMLを複数指定できます。
今回の3 seedは同じ `_1.yaml` の固定配置に対する計画の乱数違いです。
公式100配置・100試行の評価の代替にはなりません。配置番号を変える場合は別のYAMLも指定してください。

## 使用条件と原論文との差

|項目|この接続の設定|
|---|---|
|プロファイル|`g1_limited_diff_2d`。原論文との直接比較不可|
|ロボット|半径0.35 mの平面円、差動運動近似|
|速度上限|前進0.8 m/s、旋回±0.7 rad/s、横移動・後退なし|
|計画|既存の半環候補＋等速予測＋サンプリングMPC。横移動を予測前に制約|
|対象|公式経路・ORCA行動を維持し、対象の速度上限だけ0.4 m/sへ変更|
|群衆|公式配置・行動・速度設定を維持|
|観測|公式LiDAR視野・距離・遮蔽判定。可視人物のみ正確な位置・速度を得る理想観測|
|見失い|正解位置・速度は制御へ渡さず停止。探索・再識別なし|
|障害物|LiDAR点を半径0.05 mの円へ変換。可視人物は移動円へ置換|
|評価|公式 `env.record()` と `env.eval()` を使用|

今回の元YAMLは対象1.5 m/s、ロボット2 m/sまで動けるため、既存G1追従方式の範囲へ変更しています。
有効設定YAML、元YAMLのSHA-256、公式commit、seedを試行ごとに保存します。
公式の版、シミュレーターのファイル、指定シナリオが変わっていれば実行を拒否します。
WindowsとWSL間の改行・実行属性の違いは内容変更として扱いません。
画像による人物検出・追跡精度は測定しません。

## 保存結果

- `summary.json`: シナリオ別の完走率・衝突回避率・探索成功率、各公式指標の平均、全試行要約。
- `<scenario>/seed_N/trial.json`: 設定、実行ステップ数、停止理由、計画時間、指令最大値、公式指標。
- `effective.yaml`: 実際に使用した設定。
- `env_basic_info.json`、`object_info.json`、`eval_result.json`: 公式評価器の生ログと指標。

可視率・探索成功・衝突回避を別々に報告し、独自の合格基準で一つの成功率にまとめません。
公式可視率の分母は要求ステップ数。早期終了の未実行分は可視に数えられません。
加速度・ジャーク・対人領域時間も公式コードの計算を保持します。
公式の探索成功は連続不可視ステップ数の判定であり、カメラによる再識別成功ではありません。

初回の結果は `docs/validation/follow_bench_initial/` に収録します。
他手法との同条件比較、原論文との比較、G1＋ONNXによる同一経路の物理評価は別の検証です。

廊下・出入口の接続確認（各1 seed、50秒）は次で再実行できます。

```bash
python -m human_follow.follow_bench --bench-root ../follow-bench \
  --scenario \
    socialRPF/layout_scenarios/config/CorridorOrcaH4W5.6_back_1.yaml \
    socialRPF/layout_scenarios/config/DoorwayOrcaH2W2.8_back_1.yaml \
  --steps 500 --seeds 0 --output results/follow-bench-layout
```

反復時の速度ログについて: 公式の単一action呼び出しは、2個目以降の環境で物体IDとリスト添字がずれ、
ロボットを重複更新する場合がありました。この接続ではlist形式で渡し、各物体を1回ずつ更新します。
同一seedの環境を2回生成し、移動・速度・評価指標が一致する統合テストを追加しています。
公式ソース自体は変更していません。

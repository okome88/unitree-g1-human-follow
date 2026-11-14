# 地形歩行の学習・評価

[プロジェクトのREADMEへ戻る](../README.md)

以下は`g1_locomotion.py`を使う既存のmjlab実験です。公式ONNXによる人間追従・協調搬送とは別の実行経路です。


UbuntuまたはWSLのUbuntuで実行します。学習はNVIDIA GPUを使います。
GPUサーバーでも同じコマンドを使えます。共用機ではGPUジョブ内で起動してください。

## 1. 現在のモデルを測る

`g1_locomotion.py` を `~/mjlab-g1` に置き、Ubuntuで以下を実行します。

```bash
cd ~/mjlab-g1
run_g1() { uv run --no-project --python 3.12 --with 'mjlab==1.6.0' python g1_locomotion.py "$@"; }

checkpoint=$(find logs/rsl_rl/g1_velocity -type f -name 'model_5095.pt' | sort -V | tail -n 1)
run_g1 eval "$checkpoint" --terrain flat
```

`Measured target` が 0.400 m/s で `Actual forward speed` がほぼゼロなら、
前進指令に対して機体が前進できていません。
`Completed` は評価時間内に一度もリセットしなかった個体数です。
速度と誤差は最初のリセット前のサンプルを集計します。
`Mean landings per foot` は、0.05秒以上浮いてから接地した回数を足ごとに平均した値です。
左右の交互性や滑りはビューアでも確認してください。
互換性エラーが出た場合は、そのモデルを学習したMJLabの版で評価してください。

## 2. 易しい地形で学習

```bash
run_g1 train --envs 128 --iterations 1000 --difficulty 0.15
```

地形の難易度は最低レベルから始まり、公式カリキュラムが走行実績に応じて調整します。
地形の上限 0.15 は易しい設定です。最初の1,000回は試行の目安です。
速度指令は機体基準の前進0.4 m/sで、速度追従報酬の重みは公式設定の2から4に変更しています。
既定の速度カリキュラムは初期段階から速度範囲を上書きするため、この実験では無効にしています。
地形カリキュラムは有効です。
旧モデルと地形学習モデルでは地形センサーの入力が異なるため、地形モデルの初回は新規学習です。

## 3. 地形ごとに評価

完了した実行フォルダの最終モデルを選びます。

```bash
checkpoint=$(find logs/rsl_rl/g1_terrain_walk -type f -name 'model_*.pt' | sort -V | tail -n 1)
run_g1 eval "$checkpoint" --terrain rough-flat
run_g1 eval "$checkpoint" --terrain random_rough --difficulty 0.15
run_g1 eval "$checkpoint" --terrain hf_pyramid_slope --difficulty 0.15
run_g1 eval "$checkpoint" --terrain pyramid_stairs --difficulty 0.15
```

平地で前進できることを確かめてから、同じ評価条件で難易度を上げます。
地形ごとの評価は速度追従とリセットの有無を測ります。階段の登り切りは映像で別途確認します。

```bash
run_g1 train --resume "$checkpoint" --envs 128 --iterations 2000 --difficulty 0.35
```

その後も評価結果を見ながら、上限を 0.6、1.0 と段階的に上げられます。
別のUbuntuターミナルを使う場合は、手順1の `run_g1` の定義を再実行してください。

### Viserで地形モデルを再生

```bash
uvx --python 3.12 --from 'mjlab==1.6.0' play Mjlab-Velocity-Rough-Unitree-G1 \
  --checkpoint-file "$checkpoint" --viewer viser
```

ブラウザで http://localhost:8080 を開き、速度欄で前進0.4 m/s、横移動と旋回を0にします。
TensorBoardの学習ログは `logs/rsl_rl/g1_terrain_walk` に保存されます。

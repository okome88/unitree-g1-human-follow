# Unitree G1 人間追従・歩行・協調搬送シミュレーション

## 人間とG1の協調搬送（実装済み・実験用）

MuJoCo上で、人間役が有限の力で荷物を引き、G1が力に応じて移動する環境を追加しました。
設定、人物の動き、上位制御、歩行方策、シーン構築を分離しています。

**[構築・起動手順と制限事項](docs/COOPERATIVE_CARRY.md)** ·
**[拡張用インターフェース](docs/CARRY_ARCHITECTURE.md)** ·
**[検証結果](docs/CARRY_VALIDATION.md)**

```bash
uv sync --frozen --extra dev --extra video
uv run --frozen --extra video g1-carry fetch-assets
uv run --frozen --extra video g1-carry run --config configs/carry_g1.toml --viewer
```

上記はPython版で、ゲームパッド・C++ビルドは不要です。初回のみネット接続でG1形状を取得します。
Windowsでの実行・描画を確認し、Ubuntu用CIを追加しています。
簡略化した人間と理想的なばね把持による研究用の初版であり、実機の搬送性能を示すものではありません。

---

Unitree G1の歩行学習・評価コードと、人間追従シミュレーションの設計を管理します。

- **[公式モデルの使い方（HOW_TO_USE）](HOW_TO_USE.md)**：Ubuntuでの準備、シミュレーター起動、ゲームパッド操作、停止方法。
- [人間追従の実装計画](HUMAN_FOLLOW_PLAN.md)：目標速度4 km/h、旋回なし／あり、通行人の追加設定、学習・評価条件。
- `g1_locomotion.py`：MJLab 1.6.0による既存の地形歩行学習と評価。
- `g1_warp_compat.py`：WarpのGPUソルバー互換用起動スクリプト。
- `transfer/2026-11-07_08-33-29/params/`：既存モデルの学習設定。
- [models](models/README.md)：Unitree公式のG1歩行モデル（ONNX）、公式制御設定、ライセンス、配布元と照合用ハッシュ。

**現在の状態：人間追従は設計段階です。歩行には収録した公式モデルを優先して利用します。公式設定の前進上限は1.0 m/s（3.6 km/h）で、4 km/hへの適合は未検証です。既存の地形学習コードは前進0.4 m/s固定の別実験で、公式ONNXの実行には専用の入力・制御設定への対応が必要です。**

コード・計画書・設定と、`models/` 内で明示的に選んだモデルをGitで管理します。途中のチェックポイント、実行ログ、動画は管理対象から除外します。既存モデルの用途・制限は[モデルの説明](models/README.md)を参照してください。依存するMJLab等のライセンスは各配布元に従います。

## 地形歩行実験の実行手順

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

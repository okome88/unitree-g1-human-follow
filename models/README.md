# Unitree公式 G1歩行モデル

起動方法は [HOW_TO_USE.md](../HOW_TO_USE.md) を参照してください。

`unitree_official/g1_velocity/` に、Unitree公式リポジトリ [unitreerobotics/unitree_rl_mjlab](https://github.com/unitreerobotics/unitree_rl_mjlab) から取得した学習済み速度追従モデルを収録しています。自作モデルではありません。

配布元の固定コミット：`1425b15f73bd4095f0df53709d7c389c3eb9e790`。取得日：2026-11-10。

|ファイル|内容|
|---|---|
|`policy.onnx`|公式学習済みモデル。元ファイルを変更せず収録|
|`deploy.yaml`|公式の関節順序、観測、行動尺度、PDゲイン、指令範囲|
|`LICENCE`|配布元のApache License 2.0全文|
|`manifest.json`|固定版の取得URL、Git blob SHA、SHA-256、バイト数|

取得時に全3配布ファイルのGit blob SHAが公式GitHub上の値と一致することを確認しています。ライセンスと出典はこのモデルの配布元に対するもので、リポジトリ全体のライセンスを変更するものではありません。

## 公開設定と利用条件

- 対応関節数：29。歩行方策の制御周期：0.02秒（50 Hz）。
- 前進指令範囲：−0.5～1.0 m/s、横移動：±0.5 m/s、旋回：±1.0 rad/s。
- 4 km/hは約1.111 m/sなので公開設定の前進上限を超えます。目標速度での歩行は未検証です。
- 公式モデルに合わせたロボットモデル・観測順序・行動尺度・PD設定が必要です。既存の `g1_locomotion.py` へファイルだけ差し替えて使用することはできません。
- モデルの取得・配布元との一致確認まで完了しています。ONNX推論、停止・旋回、シミュレーション、人間追従の動作確認は未実施です。

公式の[モデル格納先](https://github.com/unitreerobotics/unitree_rl_mjlab/tree/1425b15f73bd4095f0df53709d7c389c3eb9e790/deploy/robots/g1/config/policy/velocity/v0)と、同じ版の[実行手順](https://github.com/unitreerobotics/unitree_rl_mjlab/blob/1425b15f73bd4095f0df53709d7c389c3eb9e790/README.md)を参照してください。

先に収録した自作モデルは現在の `models/` から外しました。元の自作モデルと設定はローカルの `transfer/2026-11-07_08-33-29/` に残しています。過去のGit履歴は書き換えていません。

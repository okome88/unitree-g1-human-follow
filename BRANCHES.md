# 開発対象ごとのブランチ

|ブランチ|用途|状態|
|---|---|---|
|`main`|人間追従・駅構内・Follow-Bench|既存の追従実装を統合|
|`feat/simulation-human-follow`|従来の追従開発ブランチ|既存履歴・コードを保持|
|`cooperative-carry/mujoco`|人間との協調搬送、MuJoCo版|動作確認済みの搬送初版|
|`cooperative-carry/isaaclab`|協調搬送のIsaac Lab対応|開発版。GPU実行検証の状態は同ブランチのdocs/ISAAC_LAB.md参照|

前回mainへ追加した搬送コードは搬送ブランチに保存しました。mainは追従側のツリーを使った新しい統合コミットで整理し、履歴の強制上書きはしていません。
mainと追従ブランチから搬送ブランチへ丸ごとmergeしないでください。共通モデル等の修正は小さなコミットで選択的に取り込みます。

依存環境が違うため、同じ作業フォルダで切り替えるより、別フォルダへcloneする方法を推奨します。

```bash
git clone https://github.com/okome88/unitree-g1-human-follow.git g1-follow
git clone --branch cooperative-carry/mujoco https://github.com/okome88/unitree-g1-human-follow.git g1-carry-mujoco
git clone --branch cooperative-carry/isaaclab https://github.com/okome88/unitree-g1-human-follow.git g1-carry-isaaclab
```

公開時点で同じロボット資産を共有する小規模プロジェクトなのでブランチ分離を採用しています。搬送側が独自のリリース・課題管理を持つ段階では別リポジトリへ移すことができます。

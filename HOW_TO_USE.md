# 公式G1歩行モデルの使い方

更新日：2026-11-10。対象：このリポジトリに収録したUnitree公式の29関節G1速度追従モデル。

この手順は**パソコン内のシミュレーション専用**です。追加学習せず、公式のC++制御プログラムでONNXを再生します。シミュレーターと制御プログラムの両方を起動する必要があります。

公式ソース・設定・操作割当を確認して作成した手順です。この作業環境ではUbuntu上のビルド・ゲームパッド操作・歩行の実行確認はしていません。起動成功や歩行性能を検証済みとするものではありません。

## 1. 現在できること

|項目|状態|
|---|---|
|公式の学習済み歩行モデルを取得|完了。`models/unitree_official/g1_velocity/policy.onnx`|
|モデルと対応する公式設定|収録済み。隣の `deploy.yaml`|
|手動で歩行・旋回させる|以下の公式シミュレーター手順を使用|
|モデルを新たに学習する|この再生手順には不要|
|人間追従・通行人の追加UI|まだ未実装。`HUMAN_FOLLOW_PLAN.md` の計画段階|
|時速4 km|未検証。公式設定上限は1.0 m/s＝時速3.6 km|

`g1_locomotion.py` は別の自作学習実験用です。公式ONNXをこのスクリプトのチェックポイント引数に渡しても再生できません。また、公式の `scripts/play.py --checkpoint_file=...` は学習用チェックポイントを扱う経路なので、このONNXの起動には使いません。

## 2. 用意するもの

- Ubuntu 22.04のデスクトップ環境、x86_64 PCを基本とします。
- Git、CMake、C++コンパイラー、画面表示できるOpenGL環境。
- Ubuntuから `/dev/input/js0` として認識できるXbox互換USBゲームパッド。
- 非公開の `okome88/unitree-g1-human-follow` を取得できるGitHubの認証環境。

以下のコマンドはすべて**Ubuntuのターミナル**で実行します。WindowsのPowerShellにそのまま貼り付けないでください。WSL2ではGUI表示とUSBゲームパッドの認識を別途準備する必要があるため、この手順だけで動くとはしていません。まずはUbuntu PCを使う構成です。

これはC++＋ONNX Runtimeによる再生手順です。Python、Conda、MJLabの学習環境を作る工程は含めていません。GPUを使った追加学習も行いません。

## 3. コードとモデルを取得する

既存の作業フォルダーと混ざらないよう、以下では `~/g1-official-demo` を使用します。同じ名前のフォルダーが既にある場合は内容を確認してから再利用してください。cloneは初回だけ実行します。

```bash
mkdir -p ~/g1-official-demo
cd ~/g1-official-demo
git clone https://github.com/okome88/unitree-g1-human-follow.git
git clone https://github.com/unitreerobotics/unitree_rl_mjlab.git official-runtime
git -C official-runtime checkout 1425b15f73bd4095f0df53709d7c389c3eb9e790
git clone https://github.com/unitreerobotics/unitree_sdk2.git
git -C unitree_sdk2 checkout aaf69cb56c9d5ed63d8b284008ec3a8ab999bec1
```

自分のリポジトリを既にUbuntuへ取得している場合は、そのフォルダーで `git pull --ff-only` し、後のコピー元パスを実際の場所へ読み替えても構いません。非公開リポジトリの取得で認証エラーになったら、普段使っているGitHubの認証方法でアクセスしてください。

公式ランタイムの版はモデルの取得元と一致させています。SDKの版もこの手順の確認時点に固定していますが、この組合せでのビルドは未実行です。

## 4. 必要なライブラリーを入れる

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git \
  libyaml-cpp-dev libboost-all-dev libeigen3-dev \
  libspdlog-dev libfmt-dev zlib1g-dev libglfw3-dev libgl1-mesa-dev
```

公式ランタイムにはMuJoCoとONNX Runtimeのライブラリーが同梱されています。Python版MuJoCoを入れるだけでは、このC++プログラムのビルド準備にはなりません。

続いて公式SDKをビルド・インストールします。標準の `/usr/local` を使います。この工程はシミュレーターと制御プログラムのビルドに必要です。

```bash
cd ~/g1-official-demo/unitree_sdk2
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build build -j2
sudo cmake --install build
sudo ldconfig
```

SDK同梱の通信ライブラリーも使うため、通常はこの手順とは別にCycloneDDSを手作業で構築する必要はありません。ビルドが失敗した場合は、そのエラーを解消してから次へ進んでください。

## 5. 自分のGitHubにある公式モデルを配置する

```bash
cd ~/g1-official-demo
cp unitree-g1-human-follow/models/unitree_official/g1_velocity/policy.onnx \
  official-runtime/deploy/robots/g1/config/policy/velocity/v0/exported/policy.onnx
cp unitree-g1-human-follow/models/unitree_official/g1_velocity/deploy.yaml \
  official-runtime/deploy/robots/g1/config/policy/velocity/v0/params/deploy.yaml
```

モデルと設定を必ずセットで配置してください。今回の速度追従モデルは公式配布ツリーに `policy.onnx` のみが置かれている形式で、別モデル用の `policy.onnx.data` を追加する手順はありません。

モデルの照合は次で行えます。

```bash
cd ~/g1-official-demo/official-runtime
sha256sum deploy/robots/g1/config/policy/velocity/v0/exported/policy.onnx
```

期待するSHA-256：

```text
2a66ca6336eadb3c0b34b557763f3e06d01ff8fcf6260dd4cedbd69d6093fc28
```

## 6. シミュレーターの設定を確認する

`~/g1-official-demo/official-runtime/simulate/config.yaml` の次の値を使います。この固定版の既定値も同じです。

```yaml
robot: "g1"
robot_scene: "src/assets/robots/unitree_g1/xmls/scene_g1.xml"
domain_id: 0
interface: "lo"
use_joystick: 1
joystick_type: "xbox"
joystick_device: "/dev/input/js0"
joystick_bits: 16
enable_elastic_band: 0
```

このONNXは29関節版です。23関節版のシーンへ変更しないでください。ゲームパッドの実際のデバイス番号が異なる場合は `joystick_device` を合わせます。

```bash
ls -l /dev/input/js*
```

何も見つからない場合は、ゲームパッドの接続・Ubuntu側の認識を先に確認します。

## 7. シミュレーターと制御プログラムをビルドする

```bash
cd ~/g1-official-demo/official-runtime
cmake -S simulate -B simulate/build -DCMAKE_BUILD_TYPE=Release
cmake --build simulate/build -j2
cmake -S deploy/robots/g1 -B deploy/robots/g1/build -DCMAKE_BUILD_TYPE=Release
cmake --build deploy/robots/g1/build -j2
```

メモリー不足を避けるため並列数は2を例にしています。ここまでで `simulate/build/unitree_mujoco` と `deploy/robots/g1/build/g1_ctrl` が生成されます。

## 8. 2つのターミナルで起動する

**ターミナルA：シミュレーター**

```bash
cd ~/g1-official-demo/official-runtime
export LD_LIBRARY_PATH="$PWD/simulate/mujoco/lib:$PWD/deploy/thirdparty/onnxruntime-linux-x64-1.22.0/lib:/usr/local/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
./simulate/build/unitree_mujoco
```

G1が表示されたら、Aはそのまま動かしておきます。

**ターミナルB：公式モデルを動かす制御プログラム**

```bash
cd ~/g1-official-demo/official-runtime
export LD_LIBRARY_PATH="$PWD/simulate/mujoco/lib:$PWD/deploy/thirdparty/onnxruntime-linux-x64-1.22.0/lib:/usr/local/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
./deploy/robots/g1/build/g1_ctrl --network=lo
```

`Connected to robot.` と速度モデルの格納先を示す `Policy directory:` が表示されることを確認します。この手順でいうrobotはシミュレーター内のG1です。通信は両方とも `lo`、domain 0を使います。実機用のネットワーク名へ変更しないでください。

## 9. ゲームパッドで操作する

最初はスティックを中央に戻しておきます。

|操作|結果|
|---|---|
|LT（L2）を押しながら十字キー上|FixStandへ移行し、約2秒かけて立位姿勢へ|
|姿勢が整ってからRT（R2）＋A|Velocityへ移行し、公式歩行モデルを実行|
|左スティック前後|前進・後退の速度指令|
|左スティック左右|横移動の速度指令|
|右スティック左右|左右旋回の速度指令|
|左右スティックを中央へ|ゼロ速度指令。立位の安定性も確認する|
|LT（L2）＋B|Passiveへ移行。歩行停止・立位保持とは異なり、シミュレーション内で倒れる場合あり|

直進だけ試す場合は、左スティック前後だけを動かし、左スティック左右・右スティックは中央を維持します。これは手動操作上の直進試験であり、旋回を強制的に無効化する設定ではありません。

指令値は概ね左スティック前後が `vx`、左右が `vy`、右スティック左右が `wz` に対応し、公式設定の範囲で制限されます。スティック量は実測速度そのものではありません。最初は小さく前へ倒して確認してください。

ゲームパッドなしのWASDだけによる起動手順は、この固定版では標準の一連の操作として案内しません。ソースにはキーボード速度入力の例がありますが、立位・歩行モードの切替や歩容位相の処理も含めた対応が必要です。`--keyboard` オプションもこの版の起動引数にはありません。

終了するときはスティックを中央に戻し、シミュレーターを閉じ、ターミナルBで `Ctrl+C` を押して制御プログラムも終了します。転倒からやり直す場合は両方を終了して手順8から起動し直します。

## 10. 速度・旋回設定と4 km/hについて

`deploy/robots/g1/config/policy/velocity/v0/params/deploy.yaml` の公開設定は次のとおりです。

```yaml
commands:
  base_velocity:
    ranges:
      lin_vel_x: [-0.5, 1.0]
      lin_vel_y: [-0.5, 0.5]
      ang_vel_z: [-1.0, 1.0]
      heading: null
```

- 前進の上限1.0 m/sは時速3.6 kmです。
- 希望の時速4 kmは約1.111 m/sで、この公開設定範囲を超えます。
- 値を書き換えるだけでは、モデルがその速度で安定して歩ける保証にはなりません。
- この版のゲームパッド入力は前後軸を概ね−1～1に正規化してからclampしているため、設定上限だけを1.111へ変えても通常の入力から1.111 m/sの指令は生成されません。

まず公式設定で停止・直進・旋回を確認し、次の実装段階で数値速度指令を渡すアダプターと実測評価を用意します。人間追従、旋回ON/OFF、通行人の追加もその段階で実装します。

## 11. よくある問題

|症状|確認すること|
|---|---|
|非公開リポジトリをcloneできない|okome88のリポジトリへアクセスできるGitHub認証があるか|
|`unitree_sdk2` が見つからない|手順4のSDKインストールが完了しているか。CMakeが `/usr/local` を参照しているか|
|`ddsc` / `ddscxx` / Unitreeのヘッダーが見つからない|SDKのインストール先とヘッダー・ライブラリー配置。別のSDK版が混ざっていないか|
|`libonnxruntime.so` / `libmujoco.so` が見つからない|手順8のLD_LIBRARY_PATHと公式リポジトリ内の同梱ライブラリー|
|`Joystick open failed`|`/dev/input/js0` の存在・読み取り権限、機種設定。WSLならUSB認識|
|G1の画面が開かない|UbuntuのGUI・OpenGL環境。表示先のないSSH接続では別途画面表示の準備が必要|
|`Waiting for connection to robot...` のまま|シミュレーターが起動しているか、両方がlo・domain 0か|
|`The other process is using the lowcmd channel`|以前の制御プログラムを終了したか。同じ通信設定の制御器を重複起動していないか|
|`Unmatched robot type`|29関節版のシーンを使っているか|
|画面は出るが動かない|制御プログラムの起動、ゲームパッド認識、L2＋上→約2秒待機→R2＋Aの順序|
|起動直後に倒れる・不安定|元のONNXと設定、固定した公式ランタイム、29関節シーンの組合せ。別の学習設定を混ぜていないか|

## 12. 確認した公式資料

- [固定版の公式README・シミュレーション起動手順](https://github.com/unitreerobotics/unitree_rl_mjlab/blob/1425b15f73bd4095f0df53709d7c389c3eb9e790/README.md)
- [公式環境構築資料](https://github.com/unitreerobotics/unitree_rl_mjlab/blob/1425b15f73bd4095f0df53709d7c389c3eb9e790/doc/setup_en.md)
- [シミュレーター設定](https://github.com/unitreerobotics/unitree_rl_mjlab/blob/1425b15f73bd4095f0df53709d7c389c3eb9e790/simulate/config.yaml)
- [立位・歩行モードの切替設定](https://github.com/unitreerobotics/unitree_rl_mjlab/blob/1425b15f73bd4095f0df53709d7c389c3eb9e790/deploy/robots/g1/config/config.yaml)
- [速度・旋回入力の処理](https://github.com/unitreerobotics/unitree_rl_mjlab/blob/1425b15f73bd4095f0df53709d7c389c3eb9e790/deploy/include/isaaclab/envs/mdp/observations/observations.h)
- [公式SDKの構築・インストール](https://github.com/unitreerobotics/unitree_sdk2/blob/aaf69cb56c9d5ed63d8b284008ec3a8ab999bec1/README.md)

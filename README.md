# mycobot leader arm

mycobot 280 pi をDobot Nova5のリーダーとして使用するためのコードです。mycobotの関節を手で動かし、その角度に合わせてNova5を動かします。

## 事前準備

### mycobotの組み立て

吸盤付きの台座に取り付けるのが大変でした。LEGOのピンを先に台座に取り付けてからmycobotを台座に取り付けると楽です。

### Ethernetケーブルの接続

mycobot、PC、Dobot Nova5をnetwork switchに有線で接続し、それぞれが通信できるようにします。今回の設定は以下の通りです。

- MacBook: `192.168.0.10/16`
- mycobot: `192.168.50.2/16`
- Dobot Nova5: `192.168.5.1/16`

mycobotにSSHできるようにしておくと便利です。

**mycobot側のIP設定**

```sh
ip -br addr show eth0 # 現在のIPを確認
sudo ip link set eth0 up
sudo ip addr add 192.168.50.2/16 dev eth0
```

IPアドレスを永続化する場合は、`/etc/dhcpcd.conf`の末尾に以下を追加して保存します。

```text
interface eth0
static ip_address=192.168.50.2/16
```

その後、dhcpcdを再起動してください。

```sh
sudo systemctl restart dhcpcd
```

Dobot Nova5のIPアドレスはコントローラーの設定に合わせてください。

### mycobotのファームウェアとpymycobot

古い型番のmycobot 280だったため、ファームウェアをv4.1にアップデートしました。mycobotのファームウェアやpymycobotのバージョンが異なると、利用できるコマンドが異なることがあります。

このプロジェクトでは、PC側の依存関係をuvで管理しています。pymycobotは`pyproject.toml`に記載したバージョンをuvでインストールします。

## ディレクトリ構成

```text
mycobot-leader/
├── README.md
├── pyproject.toml                    # Python依存関係とuv設定
├── uv.lock                           # 依存関係のlock file
├── main.py                         # sync / teleop の入口
├── sync_initial_pose.py            # Novaの姿勢をmycobotへ同期
├── teleop_mycobot_nova.py          # mycobotからNovaを操作
├── mycobot_control.py              # サーボとLEDの共通処理
├── mycobot_release_all.py          # 全サーボをreleaseする保守用script
├── mycobot_scripts/
│   ├── mycobot_server_fixed.py     # mycobot上で動かすTCP server
│   └── scp_server.sh               # server転送用script
├── adapter_STL/
│   ├── README.md                   # adapterの説明と組み立て部品
│   └── mycobot280_so101wrist_adapter.stl
├── test/
│   ├── check_mycobot_server.py     # serverの接続・応答確認
│   └── nova5_servoJ.py             # Nova5の動作確認
└── TCP-IP-Python-V4/               # Dobot Nova5 SDK submodule
```

`adapter_STL/`には、mycobotのフランジにSO-101 leader armのgripperを取り付けるためのadapter STLと、印刷・取り付けに必要な部品の説明があります。

## セットアップ

Pythonのバージョンと依存関係の管理にはuvを使用しています。

```sh
git clone <このrepoのURL>
cd mycobot-leader
git submodule update --init --recursive
uv sync
```

mycobot上のrootディレクトリに`mycobot_scripts/`を転送し、mycobot上でserverを起動してください。

```sh
python3 ~/mycobot_scripts/mycobot_server_fixed.py
```

serverを起動した後、PCから接続と角度応答を確認できます。

```sh
uv run python test/check_mycobot_server.py
```

継続して応答を監視する場合は`--watch`を付けます。`Ctrl+C`で終了します。

```sh
uv run python test/check_mycobot_server.py --watch
```

## sync

現在のNova5の関節角度を読み取り、対応するmycobotの目標角度へ移動します。J1の目標がmycobotの可動範囲`-168°〜+168°`を超えた場合は、warningを表示して範囲内の端の値を指令します。目標に近づいた後、残差をencoder指令で補正します。

```sh
uv run python main.py sync
```

syncは実際にmycobotを動かします。ロボット周辺を確認してから実行してください。

## teleop

デフォルトではdry runです。mycobotとNova5の角度を読み取って表示しますが、サーボのreleaseやロボットの移動指令は送りません。

```sh
uv run python main.py teleop
```

実際に動かす場合は`--execute`を付けます。デフォルトではJ1〜J6を操作し、対象関節は`--joints`で変更できます。

```sh
uv run python main.py teleop --execute
uv run python main.py teleop --execute --joints 1 2 3 4 5 6
```

実行時は確認のため`ARM`と入力します。選択したmycobotのサーボがreleaseされ、他の関節はサーボ有効のままです。SPACEを1回押すと追従を開始し、もう一度SPACEを押すかESCを押すと停止します。終了時にはNovaを停止し、mycobotを停止時の角度で保持する処理を行います。

Novaの移動範囲に人や物が入らないようにし、物理E-stopをすぐ操作できる状態で実行してください。SPACEによる停止は安全定格の停止機能ではなく、ソフトウェアは衝突判定を行いません。

対象関節を変更する場合も、機構の干渉や姿勢を確認してから実行してください。

## データ収集

`record` はタスク説明とRealSenseの名前・シリアル番号を受け取り、1回のテレオペレーションを1エピソードとして保存します。カメラ映像はカメラごとのRealSense `.bag` とフレーム時刻sidecar (`<name>.timestamps.jsonl`)、ロボット状態とNovaへの指令はイベント形式の `telemetry.jsonl`、エピソード情報はJSONで保存します。sidecarにはSDKフレーム番号・時刻・時刻ドメインと、PCの単調時計で測った受信時刻を含みます。既定のカメラ設定はRGB 640×480・30 fpsです。出力先は `data/recordings/` です。

カメラのシリアル番号は次のコマンドで確認できます。

```sh
sudo rs-enumerate-devices -s
```

記録モードのdry runではロボットを動かさず、カメラ収録も開始しません。

```sh
uv run python main.py record \
  --task "Move the cube into the tray" \
  --camera front=YOUR_FRONT_SERIAL \
  --camera wrist=YOUR_WRIST_SERIAL
```

実際に操作・記録するには `--execute` を付けます。既存のteleopと同じく `ARM` の入力も必要です。

```sh
uv run python main.py record \
  --task "Move the cube into the tray" \
  --camera front=YOUR_FRONT_SERIAL \
  --camera wrist=YOUR_WRIST_SERIAL \
  --execute
```

SPACEで収録・追従を開始し、SPACEまたはESCで停止します。終了時に成功・失敗を入力します。異常終了したエピソードはエラーまたは中断として記録されます。既存の `teleop` コマンドは記録なしで従来どおり使えます。

RealSense SDKのコマンドラインツールだけではPythonからの撮影に使えません。記録に使うPython環境にも `pyrealsense2` バインディングが必要です。この環境の標準Pythonではバインディングが未導入のため、`record --execute` はSDK/カメラの事前確認で停止します。macOSではSDKのUSBアクセスに管理者権限が必要になる場合があります。カメラ認識に失敗した場合は、ロボットを有効化する前にエラー終了します。実機上でのsudo実行は、macOSのキーボード権限と出力ファイルの所有権も確認してから行ってください。

収録後のraw検査と品質レポートはロボット・カメラに接続せず実行できます。

```sh
uv run python main.py inspect --session data/recordings/SESSION_ID
uv run python main.py quality --session data/recordings/SESSION_ID \
  --output data/recordings/SESSION_ID-quality.json
```

同期済み中間形式は、参照カメラの各フレーム時刻にロボット状態・指令と他カメラの近傍フレームを対応付けます。状態の線形補間は有効サンプル間だけで行い、許容差から外れた候補は理由付きで `excluded.jsonl` に残します。出力ディレクトリは新規作成し、既存rawや出力を上書きしません。`--state-source` は `mycobot` または `nova` を指定してください。

```sh
uv run python main.py synchronize \
  --session data/recordings/SESSION_ID \
  --output data/datasets/session-v1 \
  --reference-camera front \
  --state-source nova
```

`export --format synchronized` は同じ中間形式を書き出します。中間形式はraw bagへのフレーム参照を保持します。`export --format lerobot` は `.bag` からRGBを読み、指定フレームを `observation.images.<camera>`、Novaの状態を `observation.state`、絶対ServoJ目標を `action` としてLeRobot v3へ変換します。状態・actionと参照画像の時刻差も補助featureに保存し、出力後にLeRobot readerで再度開いてfeatureと総フレーム数を確認します。LeRobot timestampは指定したnominal FPSです。元のホスト単調時刻は `source_time_ns` featureに残ります。

LeRobot変換を行うには、同じuv環境に `lerobot[dataset]==0.6.1` と、現在のPython/OSに対応した `pyrealsense2` が必要です。LeRobotのwriterは `create`, `add_frame`, `save_episode`, `finalize` を使います。1回目は短い記録で試し、変換manifestと動画を確認してください。既定では成功/失敗の完了episodeを含め、異常終了episodeと`filter`判断済みepisodeを除外します。`--state-source nova` は実測Nova関節状態を観測にします。Novaの状態記録は現状約4 Hzなので、30 fps時系列で全フレームに近傍状態を付けるには `--state-mode nearest --state-slop-ms 140` のように広い許容幅が必要です。`observation_state_match_delta_ms` を見て、動作に対して状態が遅れていないか確認してください。

キュレーション判断はrawを変更せず、別JSONファイルから追記します。`decisions.json` はエピソードごとの `episode_id`、`decision` (`keep` / `filter` / `relabel`)、`reason`、`method` を持つ配列です。`relabel` では修正ラベルの `labels` も必要です。

```sh
uv run python main.py curate \
  --session data/recordings/SESSION_ID \
  --decisions decisions.json
```

## LED表示

mycobotのAtom LEDは、処理状態に応じて色を変えます。

| 状態 | 色 |
|---|---|
| sync中 | 青 |
| sync完了 | 緑 |
| sync未完了 | オレンジ |
| エラー | 赤 |
| teleop開始待ち | オレンジ |
| teleop中 | 水色 |
| 停止処理中 | オレンジ |

## サーボをすべてreleaseする

保守時などにmycobotの全サーボをreleaseする場合は、以下を実行します。

```sh
uv run python mycobot_release_all.py
```

サーボがreleaseされるとアームが動くことがあります。アームを支えた状態で実行してください。

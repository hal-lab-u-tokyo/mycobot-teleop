# 模倣学習データ収集の実装計画

## 目的

[design.md](design.md) の設計を、現在のmyCobot → Nova5テレオペレーションに段階的に追加する。最終的には、カメラ・状態・指令を取得時刻つきでraw保存し、同期・品質確認を経てLeRobotデータセットへ変換できるようにする。

## 実装状況 (2026-10-23)

| 領域 | 状況 | 制約・次の作業 |
|---|---|---|
| raw telemetry / episode | 実装済み | 実機で収録、停止処理、負荷を確認する |
| RealSense RGB bag + frame timestamp sidecar | 実装済み | Python `pyrealsense2` 未導入。2台同時の長時間収録とフレーム落ちを実機で確認する |
| raw inspect / quality report / curation JSONL | 実装済み | 初回収録データで指標と閾値を確認する |
| 同期JSONL中間形式 | 実装済み | `nova` 状態は現状約4 Hzなので、30 Hzフレーム基準の観測としては疎。観測方針と許容幅を実測で決める |
| LeRobot native export | 実装済み・未実機確認 | pyrealsense2と`lerobot[dataset]==0.6.1`を導入し、実記録のbag decode / writer / reader往復を確認する |
| 動画と観測/actionの目視同期確認 | 未実装 | 簡易スクラバーまたは既存viewerで確認手順を作る |

記録実装・ローカルデータ処理には実機アクセスやsudoを使っていない。実機確認は、Python binding導入後にカメラ2台のrecordingを短時間から行う必要がある。

## 実装上の原則

- 追従制御の挙動と安全停止処理を、記録機能の追加と混ぜて変更しない。
- 既存CLIの `python main.py teleop` / `--execute` を維持する。
- rawデータは追記・不変とし、同期・補正・除外は別の派生データとして作る。
- 収録中の画像・テレメトリ保存は制御ループをブロックさせない。記録失敗やフレーム落ちは隠さず記録する。
- LeRobotへの変換、可視化、キュレーションはオフライン処理にする。
- 30 Hz、640×480は初期候補であり、実測後に確定する。

## 現時点で確認済みのHW

- MacBookにRealSense D435iを2台接続。
- 2台ともRealSense ViewerでRGB表示を確認し、表示fpsは約29.7〜29.9。
- 2台は同じUSB 3.1ハブ配下。`sudo rs-enumerate-devices` でSDKから認識できる。
- これは短時間のストリーム確認であり、長時間のフレーム落ち・USB帯域・記録負荷の検証は未完了。

## 完了時のユーザーフロー

1. `record` を起動し、タスクを指定する。
2. 既存の手順でテレオペレーションを開始・停止し、試行ごとにエピソードを保存する。
3. 記録終了後、rawデータ検査で時刻・欠落・周期を確認する。
4. 選択した同期設定で同期中間形式を作り、LeRobot形式へ変換する (実装済み、runtime確認待ち)。
5. 変換データを映像と状態・actionの同期表示で確認し、各エピソードを採用・除外・ラベル修正に分類する (目視UIは未実装)。

## Phase 0: 記録条件の実測とスキーマ確定

### 作業

- 現行のmyCobot読み取り、Nova状態読み取り、ServoJ送信について、呼び出し前後のPC単調時計時刻を取れるよう計測方法を決める。
- RealSenseの機種・台数・ストリーム設定を確認し、センサー時刻、時刻ドメイン、フレーム番号、PC受信時刻が取得可能か調べる。
- 各読み取り周期、最大遅延、ジッター、実効fps、フレーム落ちを短時間のdry-runで把握する。
- action定義を文書化する。初期定義は「Novaへ送った6関節の絶対ServoJ目標角（度）」とし、関節順、ゼロ点、符号、速度制限、ServoJ設定を明記する。
- セッション、エピソード、テレメトリ、カメラtimestampのJSONスキーマと `schema_version` を決める。
- 同期の候補許容幅を決め、実測データで採用率と最大時刻差を比較する計画を立てる。

### 完了条件

- 各時刻フィールドが、センサー測定時刻・PC受信時刻・指令送信時刻のどれか区別されている。
- actionの単位、関節順、絶対/相対、マッピング、制御設定が記録仕様に記載されている。
- 計測で得られないセンサー時刻は推定値と混同せず、未取得として表現する。

## Phase 1: パッケージ境界とテレメトリraw記録

### 主な対象

- `main.py`
- 新規 `src/mycobot_leader/application/teleop_session.py`
- 新規 `src/mycobot_leader/robots/mycobot.py`
- 新規 `src/mycobot_leader/robots/nova.py`
- 新規 `src/mycobot_leader/robots/joint_mapping.py`
- 新規 `src/mycobot_leader/recording/recorder.py`
- 新規 `src/mycobot_leader/recording/raw_writer.py`
- 既存 `teleop_mycobot_nova.py`（段階的な委譲・移行対象）
- `.gitignore`

### 作業

- 既存追従処理を小さく分割し、通信アダプター、関節マッピング、セッション制御、記録処理の責務を分ける。
- 先にteleopの現行CLIと動作経路を保ったまま移し、記録なしでの制御モードとdry-runを維持する。
- 記録セッション用の出力先を `data/recordings/<session-id>/` に作る。
- 制御周期ごとにmyCobot実測角、Nova実測角、Novaへ送ったServoJ目標角、各利用可能な時刻、cycle番号をJSONLへ保存する。
- 書き込みは専用キュー/ワーカーに渡す。キュー長、書き込み遅延、ドロップ件数を記録する。
- セッション終了・例外・Ctrl+Cでwriterを閉じ、完了状態または異常終了状態をメタデータに残す。
- `session.json` に開始時刻、PC時刻対応、機材ID、ソフトウェア/設定、schema versionを保存する。
- `.gitignore` で `data/` を除外する。

### 完了条件

- teleopを記録なしで起動する既存経路を保てる。
- 記録したJSONLをオフラインで読め、各配列長、単位、時刻フィールドがスキーマと一致する。
- 記録負荷によってループが大幅に停滞したときに、黙って通常記録として扱わず異常指標が残る。

## Phase 2: エピソード管理とタスクラベル

### 主な対象

- 新規 `src/mycobot_leader/application/record_session.py`
- 新規 `src/mycobot_leader/recording/episode.py`
- `main.py`
- `data/recordings/<session-id>/episodes/`

### 作業

- `record` コマンドにタスク説明、保存先、必要に応じて操作者・環境・物体個体などのメタデータを渡せるようにする。
- 1試行を1 episodeとして、明示的な開始・終了時刻、結果（成功/失敗/中断）、終了理由を保存する。
- 空のepisode、開始前中断、通信異常時にメタデータが不整合にならない終了処理を設計する。
- 正常に記録できた失敗試行は保存し、機材・記録系の異常がある試行は異常理由と状態を記録する。

### 完了条件

- 複数episodeを含むセッションで、episode境界・タスク・結果を一意に特定できる。
- 異常終了した記録が完了済みデータとして誤認されない。

## Phase 3: 複数RealSenseの独立取得とraw保存

### 主な対象

- 新規 `src/mycobot_leader/cameras/realsense.py`
- 新規 `src/mycobot_leader/cameras/sync.py`
- `src/mycobot_leader/recording/recorder.py`
- `episode.json` とカメラtimestamp記録
- 必要なRealSense依存関係と設定

### 作業

- カメラを識別名で設定し、同じインターフェースでフレームとタイムスタンプ情報を取得する。
- まずは各カメラを独立ストリームとして保存し、各フレームのセンサー時刻、PC受信時刻、フレーム番号、時刻ドメインを記録する。
- カメラ取得・保存を制御スレッドから分離する。
- 解像度、fps、露出等の設定、デバイスID、フレーム落ちをセッションメタデータに記録する。
- 複数カメラのハードウェア同期は、非同期記録で精度が不足することを測定してから導入判断する。

### 完了条件

- 全カメラの映像とtimestamp一覧がepisode単位で再生・照合できる。
- カメラ停止、再接続、timestamp巻き戻り、フレーム落ちを検出して記録できる。
- 表示fpsだけでなく、長時間収録時のフレーム番号の欠落、timestamp差分、USB/CPU負荷を確認し、初期解像度・fpsを確定できる。

## Phase 4: raw検査、同期・整列、LeRobot export

### 主な対象

- 新規 `src/mycobot_leader/datasets/raw_inspect.py`
- 新規 `src/mycobot_leader/datasets/synchronize.py`
- 新規 `src/mycobot_leader/datasets/lerobot_export.py`
- `data/datasets/lerobot/<dataset-version>/`

### 作業

- rawデータを読み、topic/feature一覧、実効Hz、Δt分布、遅延分布、巻き戻り、重複、欠落、episode長をレポートする。
- カメラ時刻を基準にした候補fpsグリッドを作り、指令は最近傍、連続状態は許容範囲内で線形補間または最近傍、離散状態はゼロ次ホールドで対応させる。
- 範囲外の状態、画像停止区間、同期許容幅外のサンプルを補間で捏造せず、未対応/除外として記録する。
- 候補slopを掃引し、採用率・時刻差の分布・捨てられる区間を比較する。
- 観測時刻 `t` とaction時刻 `t + Δ` の候補をrawから再生成可能にし、小規模な学習・評価で採用するΔを決める。
- LeRobotの固定バージョンを明示し、feature schema、task、episode、fps、statsを出力する。
- `manifest.json` にraw入力、schema version、変換コードcommit、全設定、同期閾値、Δ、出力checksumを保存する。

### 完了条件

- 同じraw・コード版・設定から同じ変換を再実行できる。
- 出力データのaction意味と各featureのshape/dtype/単位がメタデータに記録される。
- rawとLeRobot出力のepisode数、サンプル数、画像数、除外数を照合できる。
- 変換後の映像・状態・actionで時刻軸が確認できる。

## Phase 5: キュレーション、検証、再生

### 主な対象

- 新規 `src/mycobot_leader/datasets/quality_report.py`
- 新規 `src/mycobot_leader/datasets/curation.py`
- dataset manifestと除外記録
- 可視化・LeRobot viewer / replay手順

### 作業

- 自動品質レポートに、episode長、行動レンジ・スパイク・停滞、Δt、各topicの実効fps、欠落、task別件数を含める。
- episodeごとに `keep` / `filter` / `relabel` と理由を記録する。rawは変更せず、curated出力に適用する。
- 初期データは全episodeを目視し、変換後はカメラ・状態・actionを同期スクラブして確認する。
- episode単位のtrain/validation/test分割を行う。汎化目的に応じて物体個体、収集日、環境、操作者の重複を避ける。
- 実機action replayは、安全な空間・低速・手動監視などの運用条件が確立してから、制御実装と分離した検証手順として行う。

### 完了条件

- 各除外/修正のepisode ID・理由・処理履歴が追跡できる。
- 品質レポートと目視確認で、時刻ずれ、関節順、単位、action意味、再生速度の問題を検出できる。
- curatedデータと元rawの対応をmanifestから追跡できる。

## 依存関係と推奨順

```text
Phase 0
  ↓
Phase 1 → Phase 2
             ↓
          Phase 3
             ↓
          Phase 4
             ↓
          Phase 5
```

Phase 1のテレメトリ記録とPhase 2のエピソード管理を先に完了してからカメラを統合する。これにより、カメラ取得負荷とロボットループへの影響を切り分けやすくする。

## リリース単位

### MVP 1: 機械可読なロボットテレメトリ

Phase 0–2。映像なしで、再現可能なepisode単位の状態/action rawデータを作る。

### MVP 2: マルチカメラraw収録

Phase 3。状態/action/映像を独立タイムスタンプ付きで取りこぼしを計測しながら記録する。

### MVP 3: 学習可能なデータセット

Phase 4–5。同期設定を追跡可能な形でLeRobotへ変換し、検証・キュレーション済みのdatasetを出力する。

## 未確定・着手前に確認する項目

- RealSenseの実機種、接続台数、同期端子の有無。
- センサー側タイムスタンプとPC monotonic clockの対応方法、およびmyCobot/Novaで取得可能な時刻精度。
- Novaの状態取得APIの実効周期と指令送信時刻の定義。
- LeRobot依存関係を本体環境へ導入するか、export専用環境へ分けるか。
- MVP 3で出力するLeRobot形式のバージョンと固定するライブラリ版。
- action時間差Δを決めるための対象タスクと評価方法。

## 検証方針

各phaseの完了条件を現場・記録データ・オフライン解析で確認する。ロボットに動作を伴う確認は、現在の運用上の安全条件を満たしたうえで実施する。コードテストの追加・実行は、この計画の範囲には含めない。

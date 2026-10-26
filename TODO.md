# 模倣学習データ収集 TODO

## 実装済み

- [x] 複数RealSenseの名前・シリアル番号指定と個別 `.bag` 収録
- [x] 収録の開始・終了をteleop episodeに合わせる
- [x] 各フレームのSDK時刻、時刻ドメイン、フレーム番号、PC受信時刻をsidecarへ保存
- [x] myCobot状態、Nova状態、ServoJ指令を時刻付きJSONLへ保存
- [x] session/episode metadataと成功・失敗・中断ラベル
- [x] raw inspect、品質レポート、keep/filter/relabel履歴
- [x] ロボット・カメラを操作せず同期中間データを生成

## 次に行う

- [ ] 収録に使うPython環境へRealSense `pyrealsense2` bindingを導入する
- [ ] 実機で短時間の1台収録を行い、bag再生とtimestamp sidecarを照合する
- [ ] 実機で2台同時・複数分の収録を行い、フレーム欠落、USB帯域、ファイルサイズを確認する
- [ ] Nova状態の約4 Hzポーリングで十分か判断する。30 Hz相当のProprioceptionが必要なら、制御通信への負荷を確認して取得設計を見直す
- [x] bagからRGBフレームを取り出し、固定したLeRobot writerでdatasetを作る変換コード
- [ ] 実記録でLeRobot writer/readbackを実行して動画・featureを確認する
- [ ] LeRobot viewerで映像・観測・actionを同期表示する
- [ ] 実測結果を使って同期許容幅と `action_delay_ms` を決める

## 設計上の未決事項

- RGBのみで対象タスクを解けるか。必要なら深度を追加する
- カメラ間のソフトウェア同期精度で足りるか。不足時にhardware syncを検討する
- action時刻 `t + Δ` の採用値と評価方法
- curation閾値はスクリーニング用初期値から実データに合わせて調整する

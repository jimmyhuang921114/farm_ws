# USB Camera Calibration

獨立 Python USB RGB 棋盤格標定工具。只使用 OpenCV、NumPy、PyYAML，沒有 ROS/ROS2、depth model 或 GPU 工作。

**正式 MDE dataset 可以繼續保存 RAW distorted RGB。** Calibration YAML 描述原始影像的 camera matrix 與 distortion；不要求 dataset 做 undistortion。`test_calibration.py` 僅供目視比較，不寫入、不覆蓋 RAW。

## 安裝與環境

Ubuntu / Linux、Python 3、可用的圖形桌面及 USB camera。

```bash
cd camera_calibration
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

此工作區已建立獨立 `.venv`，使用 pip 本機快取的 wheel 離線安裝。`.wheelhouse/` 與 `.tmp/` 只是被忽略的本機安裝／測試檔案，不是執行必要檔案。其他電腦依上方指令安裝即可；不要搬移 `.venv`。

## 設定

所有 camera、棋盤格、目標張數與輸出路徑均由 `config/calibration.yaml` 讀取。程式預設使用專案內的該檔；也可傳 `--config` 指定另一個 YAML。YAML 內相對儲存路徑一律相對於 **本專案目錄**，不受啟動時工作目錄影響；絕對路徑則原樣使用。

```yaml
camera:
  device: 2
  width: 1920
  height: 1080
  fps: 30
  fourcc: "MJPG"
  name: usb_camera
chessboard:
  cols: 9
  rows: 6
  square_size_mm: 25.0
capture:
  target_images: 30
  save_dir: "./calibration_images"
calibration:
  output: "./output/camera_calibration.yaml"
```

- `device: 2` 在 Linux 表示 `/dev/video2`，只是使用者要求的初始範例，並非已確認存在的裝置。可改為 `device: 0` 或 `device: "/dev/video0"`。先以 `ls /dev/video*` 或已安裝的 `v4l2-ctl --list-devices` 確認影像 capture node；metadata node 不能使用。先前 recorder 測試時 USB `1bcf:2cd1` 的影像節點是 `/dev/video0`，裝置編號仍可能改變。
- `cols` / `rows` 是 **INNER CORNERS（內角點）**，不是黑白格數量。`9 x 6` 內角點對應 `10 x 7` 格。
- `square_size_mm` 是實際量測的每格邊長，單位 **mm**。列印時不要縮放；請量測成品，而非只相信 PDF 標示。此尺寸決定外參平移尺度。
- 相機設定先寫入 FOURCC、解析度與 FPS，再印出讀回值。讀回 FPS 不是實際 throughput 測量；若相機不能開啟、讀取失敗或實際解析度不符，會明確報錯。Linux 使用 CAP_V4L2。
- 標定解析度、crop、zoom 和 focus 必須與之後使用此 YAML 的影像一致。特別注意 autofocus 可能改變內參：正式採樣前應穩定並固定焦距，dataset 錄製時保持相同焦距。本程式不自行改變 exposure、focus 或既有 calibration。

## 拍攝與標定

```bash
source .venv/bin/activate
python calibrate_camera.py --config config/calibration.yaml
```

視窗顯示解析度、內角點數、實際格尺寸、已收集張數、偵測狀態與操作提示。每 frame 轉灰階後以 `findChessboardCornersSB` 偵測；SB 本身提供 subpixel 角點。若 API 不存在或不相容，改用 `findChessboardCorners` + `cornerSubPix`。一般的「未找到棋盤」不算相容性問題。

| 按鍵 | 功能 |
|---|---|
| SPACE | 只有當下 frame 找到完整棋盤角點才保存；否則顯示 `Chessboard not detected - image NOT saved` |
| C | 使用本次已收集 samples 執行 `calibrateCamera()`、列出結果並寫 YAML |
| R | 清空本次記憶體中的 samples；**保留已保存 RAW 檔案**，後續接續編號 |
| Q | 退出並釋放相機；未按 C 的 samples 不會自動標定 |

收集 20–30 張後按 C。`target_images` 控制本次最多收集張數；達標後不會自動執行標定。少於 3 張拒絕標定，3–19 張仍可手動標定但會警告數量不足。不同角度／位置不足時，即使達到 30 張也可能有退化解；張數本身不是品質保證。

保存影像為 `calibration_images/000001.jpg`、`000002.jpg`……，JPEG quality 95。這裡 RAW 表示相機傳回的原始 distorted 彩色 frame，**不是 Bayer sensor RAW 檔**；JPEG 是有損編碼。角點和文字只畫在 preview 的 copy，不會寫入保存影像。角點保存於本次程式記憶體；重新啟動不會自動載入舊影像。既有影像不覆寫，編號取目錄中現有最大編號後續接。

R 不刪除磁碟 RAW 或先前 calibration YAML。輸出 YAML 的 per-image 清單會明確指出此次實際使用哪些影像。再次成功標定時，原本的輸出 YAML 先備份為附時間戳的 `.bak`，再寫入新結果。

## 棋盤拍攝品質

不要站在同一位置連拍 30 張。使用平整、不彎曲的棋盤，拍攝時保持靜止，避免 motion blur、反光和過曝。

- 位置：Center、Top-left、Top-right、Bottom-left、Bottom-right。
- 姿態：front-facing、tilt left、tilt right、tilt up、tilt down。
- 距離：near、medium、far；遠距仍需讓每格有足夠像素可偵測。
- **讓棋盤接近影像邊緣與角落**，以約束 radial distortion，但每張必須完整看見所有 inner corners。
- 避免全部都是正面平行姿態、同一尺度或幾乎重複的 frame。角點可被偵測不代表採樣幾何充分。

## 結果與誤差

按 C 成功後才產生 `output/camera_calibration.yaml`。專案初建時只有 `output/.gitkeep`，**不提供假的 calibration 數值**。

YAML 包含：`image_width`、`image_height`、`camera_name`、3x3 `camera_matrix`（row-major）、`distortion_model: plumb_bob`、1x5 `distortion_coefficients`（k1,k2,p1,p2,k3）、簡易 `intrinsics.fx/fy/cx/cy`、RMS、mean reprojection error、images_used、每張影像誤差、rvec 和 `tvec_mm`、棋盤設定、相機 requested/actual 設定及版本資訊。

Object points 依列、欄生成 `(0,0,0), (square_size_mm,0,0), ...`，平面 z=0，單位 mm。rvec 是 Rodrigues 旋轉向量；tvec 單位 mm。

除了 OpenCV 的 RMS，另用 `projectPoints()` 計算每個角點的 Euclidean pixel distance。每張誤差為角點距離的平均，整體 mean 為每張平均的平均（每張角點數相同）。**不使用 `L2 norm / N` 這個會低估誤差的公式**。Terminal 列出每張誤差、best/worst image、fx/fy/cx/cy 與 distortion。

- fx、fy 必須為正；非有限或無效解不保存。
- cx、cy 在影像範圍外時警告。
- mean >1.0 px：`WARNING: High reprojection error. Consider capturing better calibration images.`
- mean <0.5 px：顯示 calibration quality good，僅代表重投影指標良好。
- 不因 error threshold 自動刪資料或排除 views。低訓練重投影誤差也不能保證邊緣校正、焦距穩定性或新姿態上的精度。

## RAW / UNDISTORTED 比較

先產生真實 calibration YAML，再執行：

```bash
python test_calibration.py \
    --config config/calibration.yaml \
    --calibration output/camera_calibration.yaml
```

左側 RAW，右側 UNDISTORTED，Q 離開。省略 `--calibration` 時使用 YAML 中的 `calibration.output`。程式使用 `initUndistortRectifyMap` / `remap`，保留原 K 和解析度，不裁切，邊緣可能出現黑區。可自由調整視窗大小；僅影響顯示，不改內部校正座標。

解析度不相符會拒絕比較，不會自動縮放 K。此工具不寫影像；後續 MDE dataset 是否 undistort 仍由你的流程決定。若選擇 undistort/crop/resize，後續 loader 需搭配該處理後影像對應的內參；不要混用。

## 無棋盤／無相機的檢查

```bash
python -m py_compile calibrate_camera.py
python -m py_compile test_calibration.py
python -m py_compile camera_utils.py
python calibrate_camera.py --help
python test_calibration.py --help
python calibrate_camera.py --config config/calibration.yaml --check-config
python test_calibration.py --config config/calibration.yaml --check-config
python -m unittest discover -s tests -v
```

`--check-config` 只驗證設定與 imports，不開相機、視窗或產生 calibration。單元測試使用人工棋盤與 mocked 投影結果驗證程式邏輯，不代表 USB 相機已標定。

本次完成了上述語法／CLI／YAML／邏輯檢查；沒有收集實際棋盤 views，因此没有相機標定結果，也尚未驗證真實 RAW / UNDISTORTED 即時畫面。

## Git

`.gitignore` 忽略 `.venv`、cache、拍攝影像、輸出標定與備份，僅保留 `calibration_images/.gitkeep` 和 `output/.gitkeep`。使用 `directory/*` 搭配例外規則，避免直接忽略整個目錄導致 `.gitkeep` 無法被 Git 收錄。未執行 GitHub push 或修改其他專案。

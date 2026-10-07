# Session Log

neuroEdge 逐次開發/除錯 session 的詳細記錄，依時間先後排列（最舊在最上面）。產品規格、架構、路線圖見 [PRD.md](PRD.md)。

這份檔案是從 PRD.md 拆出來的（2026-09-11），原本 PRD.md 混著「穩定的產品規格」跟「一直累積的除錯紀錄」，拆開讓兩邊都好讀。

---

### Session Handoff (2026-09-03/04)：肩膀 IMU 方向 bug 三連發 + tilt/azimuth 演算法重構(進行中，未完工)

這次 session 從「測試手臂動作方向對不對」開始，一路挖出三個獨立、疊在一起的真實 bug，最後定位到問題根源需要換一種角度表示法，**目前演算法核心+測試已完成並通過，但還沒接回 firmware/CSV demo/Python 控制層，下個 session 要接著做**。以下依序記录，給下個 session 快速抓回脈絡用。

**目前實體硬體貼法確認過(有照片佐證)**：上臂那顆(貼在手肘內側)GY-521，晶片面朝外(背向皮膚)，-X 軸朝手掌延伸方向；前臂那顆(貼脈搏處)用 dot-product 法，不挑朝向。左手。

#### 抓到的三個真實 bug(依發現順序)

1. **Data→MuJoCo 的 shoulder_pitch 符號反了**——`run_demo.py`/`run_demo_live.py` 原本 `SHOULDER_PITCH_RANGE=(-1.0472,2.6704)` 假設正值=前舉，實測用純 MuJoCo `mj_forward` 驗證發現方向相反。**已修正並已 commit**(commit `0369d58`)：range 改成 `(-3.0892,1.0472)`，ctrl 指定加負號。這個 bug 只在 Data→MuJoCo 這層，不牽涉硬體/firmware。

2. **firmware 的肩膀軸向 remap 選錯了原始加速度計軸**——實測「往前舉」動作在硬體上幾乎全部跑到 `shoulder_roll` 而不是 `shoulder_pitch`。用真實錄到的原始 raw_ax/ay/az(靜止 vs 前舉到頂)配合幾何推導(-X 朝手掌 + 晶片朝外，可以完整算出 X 軸方向；Y/Z 因為手臂圓周角度的關係無法純幾何算完，但有實測數據佐證)，確認 `raw_ax` 才是真正該驅動 pitch 的軸，原本卻被塞進 pitch/roll 共用的參考軸角色。**已修正**（`firmware/src/phase3_control_loop_main.cpp` 的 shoulder remap 三行，現在是 `ax=raw_ax, ay=raw_az, az=raw_ay`）。陀螺儀 `gx=-raw_gy, gy=raw_gz` 那兩行**還沒重新驗證**，是照舊配對留著，標了「NOT YET RE-VERIFIED」註解——這組配對是配合*舊的*(錯的)加速度計軸選的，理論上該重推但還沒做。

3. **`accel_pitch_angle` 公式本身在大角度會失真，這是這次最根本、最花時間的發現**——`include/edgeneuro/fusion/complementary_filter.hpp` 原本的 `atan2(-ax, sqrt(ay²+az²))`，因為分母開根號恆為正，數學上把輸出鎖死在 ±90° 內，超過就會「折返」，導致兩個方向相反的真實大角度動作（例如前舉 110° vs 後擺 90°）解出**同號、無法分辨方向**的 pitch。用真實 REST 重力向量 + 合成旋轉直接算出這個折返現象、寫成 Catch2 測試證實（`tests/test_complementary_filter.cpp` 新增的 ROM 掃描測試）。**第一次嘗試的簡單修法(把 ay 從公式拿掉、只用 az)已證明不夠**——雖然解決了 ±90° 的範圍限制，但套到這個真實安裝角度的數值後，又剛好卡進 atan2 自己「另一個」分支切割點（±180° 邊界），前舉/後擺又變回同號。查證後確認這是**生物力學界已知的問題**（ISB 建議的 YXY 歐拉序列在肩膀外展 90° 時會 gimbal lock，已發表的論文用「Tilt-and-Torsion」法解決，見 arxiv.org/abs/2108.12282）——任何「固定寫死方向」的角度公式，都可能剛好讓某個安裝角度卡到自己的邊界，唯一穩健的解法是**角度基準要跟著實際量到的 REST 姿勢走，不能寫死**。

#### 已完成並通過測試的部分

- **`include/edgeneuro/fusion/tilt_azimuth.hpp`(新檔案)**：`tilt_azimuth()` 算「跟 REST 差多少角度(tilt, 0~180°，無邊界問題)+ 往哪個方向(azimuth)」；`orthonormal_basis_perpendicular_to()` 只需要 REST 這一個向量就能自動算出兩個正交基準軸，不用額外校正動作。
- **`tests/test_tilt_azimuth.cpp`(新檔案)**：合成資料測試，涵蓋垂下/前舉/左前舉/右前舉四個使用者指定動作，加上完整 0°~170° x 六個方位角的全 ROM 掃描，**全部通過**，包含直接證明「前舉 vs 後擺不再同號」的專屬測試案例。
- `tests/test_complementary_filter.cpp` 新增的 ROM 掃描測試、`tests/test_complementary_filter_real_data.cpp` 的容忍度調整（因拿掉 ay 造成的已知、有文件記錄的精度取捨）——**全部通過**（72 個測試案例、22798 個斷言，`./build/debug-heapguard/edgeneuro_tests` 全綠）。
- `src/mujoco_bridge_demo.cpp` 補回跟 firmware 同步（原本已經跟 firmware 脫節，沒做軸向 remap、手肘還在用已淘汰的角度相減法，現已對齊 dot-product 法 + 新的 remap）。
- `tools/mujoco_bridge/log_raw_imu.py`（新檔案）：純 raw 訊號互動式錄製工具，不需要 mjpython/MuJoCo/校零，逐一提示動作、自動倒數錄製、每做完一個立刻存檔（避免中斷全部流失）、支援續錄跳過已完成項目。今晚用它真實錄到 REST/BACKWARD_EXTENSION/ABDUCTION_LEFT/ADDUCTION_RIGHT/ELBOW_FLEXION 五組姿勢的真實配對 raw 數值，存在 `tools/mujoco_bridge/raw_imu_capture.json`（未加入 git）。
- `tools/mujoco_bridge/test_imu_to_mujoco.py`（新檔案）：IMU 訊號→C++ 解碼 binary→Data→MuJoCo 全流程整合測試，目前用的是**舊的 accel_pitch_angle 公式（drop-ay 版本）**，所以目前執行會有 FORWARD_RAISE/ABDUCTION_LEFT 相關斷言失敗——這是預期的，因為 tilt/azimuth 還沒接進來，不是新 bug，見下方 TODO。

#### 下個 session 要接著做的事（TODO，依優先順序）

1. **把 `tilt_azimuth.hpp` 接進 `phase3_control_loop_main.cpp` 的 shoulder 計算**，取代現在還在用的 `ComplementaryFilter::pitch()/roll()`。需要決定：REST 參考向量從哪裡來（目前 firmware 的零點校正只存純量 zero_shoulder_pitch/roll，需要改成存整個 raw 向量）；算出 tilt/azimuth 後怎麼換算回餵給 MuJoCo 的 pitch_ctrl/roll_ctrl（目前構想是 `pitch_equiv = tilt*cos(azimuth)`, `roll_equiv = tilt*sin(azimuth)`，尚未驗證這個換算在大角度下跟 MuJoCo 的兩個獨立 revolute joint 幾何吻合得多好，可能需要微調)。
2. 同步更新 `src/mujoco_bridge_demo.cpp`（同一份邏輯要同步兩份，見上面第 2 個 bug 的教訓——這個檔案很容易跟 firmware 脫節，兩邊改動最好一起做、一起測)。
3. 更新 `tools/mujoco_bridge/run_demo_live.py`/`run_demo.py` 的 Data→ctrl 映射，改吃 tilt/azimuth 換算後的值；零點校正邏輯要改成存 raw 向量而非純量。
4. 更新 `tools/mujoco_bridge/test_imu_to_mujoco.py` 的假設/斷言，改用新算法後重新驗證今晚錄到的五組真實資料（`raw_imu_capture.json`）方向是否正確。
5. 陀螺儀 `gx/gy` 配對重新驗證（bug #2 遺留的未完成項目）——tilt/azimuth 若改成純 accel-based（不用 gyro，比照手肘 dot-product 法的精神），這一項可能整個變得不需要，待評估。
6. `ADDUCTION_RIGHT`（右擺/內收）這個動作本身，今晚錄了三次才錄到「看起來正常」的一次，實際動作是否真的乾淨隔離仍有疑慮（幅度偏大、跟外展數值有點像）——用新演算法重新檢視這組資料時要留意。
7. 目前所有改動都**還沒 commit**（見下方 git 狀態），下個 session 開始前建議先跟使用者確認要不要先 commit 這一批（bug #2 的 firmware+demo 修正 + 新測試），再繼續做 tilt/azimuth 的接線工作，避免兩批不同性質的改動混在同一個 commit。

#### 目前 git 未提交狀態
```
 M firmware/src/phase3_control_loop_main.cpp     (bug #2 的 shoulder remap 修正)
 M include/edgeneuro/fusion/complementary_filter.hpp  (bug #3 的 drop-ay 嘗試，已知不夠，會被 tilt/azimuth 取代)
 M src/mujoco_bridge_demo.cpp                     (跟 firmware 同步)
 M tests/test_complementary_filter.cpp            (新增 ROM 掃描測試)
 M tests/test_complementary_filter_real_data.cpp  (容忍度調整)
 M tools/mujoco_bridge/run_demo_live.py           (新增 [CORR] log、wrap_angle_delta)
?? include/edgeneuro/fusion/tilt_azimuth.hpp       (新演算法核心，已測試通過)
?? tests/test_tilt_azimuth.cpp                     (新演算法測試，已通過)
?? tools/mujoco_bridge/log_raw_imu.py              (raw 訊號錄製工具)
?? tools/mujoco_bridge/raw_imu_capture.json        (今晚錄到的真實資料，未加入 git)
?? tools/mujoco_bridge/test_imu_to_mujoco.py       (整合測試，待用新演算法更新)
```
(`run_demo.py`、`arm_hand_scene.xml` 的 bug #1 修正已經是更早的 commit `0369d58`，不在上面清單裡。)

### Session Handoff (2026-09-04 續)：tilt/azimuth 接線完成（TODO #1-#4），架構跟原計畫不同

**重要說明**：上面這份 TODO 清單、git 狀態，在這個 session 開始前已經被**另一個平行執行的 session** 提交成 commit `87ce334`（bug #2/#3 修正 + tilt_azimuth 核心）跟 `71de296`（pipeline 測試），兩筆都帶了 `Co-Authored-By: Claude Sonnet 5` trailer（違反 [[feedback_no_coauthor_trailer]] 的既有偏好，經使用者確認後決定保留現狀、不重寫歷史）。這個 session 是基於那個已提交的狀態繼續往下做 TODO #1-#4。

#### 跟原計畫的關鍵架構差異

TODO #1 原文說「接進 `phase3_control_loop_main.cpp`」，但實際查證後發現**校正從來不在 firmware 端**——firmware 只串流 raw ax/ay/az，校正（目前是零點）一直是 `run_demo_live.py` 每次重開都重新錄的。tilt/azimuth 需要三個完整參考向量（REST/FORWARD_RAISE/ABDUCTION_LEFT），若照字面接進 firmware 需要新增一套韌體端三姿勢校正握手協定，會失去「每次重開都重新校正」的彈性，還需要重新燒錄+實際戴上硬體才能驗證每次修改。**改為：firmware 完全不動，tilt/azimuth + 新的非正交基底解法整個放在 Python(`run_demo_live.py`) 跟 C++ demo(`mujoco_bridge_demo.cpp`) 這兩個消費 raw 數據的地方**，經使用者確認同意。

#### 完成的部分

1. **`include/edgeneuro/fusion/tilt_azimuth.hpp` 新增 `make_oblique_basis()`/`oblique_decompose()`/`oblique_decompose_scaled()`**：用實測到的 FORWARD_RAISE/ABDUCTION_LEFT 當非正交基底（Gram matrix 最小二乘解），取代原本假設兩者正交的 `tilt*cos(azimuth)`/`tilt*sin(azimuth)` 換算。過程中發現並修正一個真實 bug：`oblique_decompose()` 原始係數乘上校正姿勢自己的傾角，在偏離校正軸的姿勢上會嚴重放大（真實 ELBOW_FLEXION 姿勢肩膀只漂移 16°，卻被放大成 35° pitch_equiv）——`oblique_decompose_scaled()` 改成只用 oblique 係數決定**方向**，量值改用獨立量測的真實傾角（acos），確保輸出量值永遠不超過真實傾角。`tests/test_tilt_azimuth.cpp` 新增對應測試，全數通過（7 個 test case、453 個斷言）。
2. **`run_demo_live.py`**：開場的 1 姿勢零點校正擴充成 3 姿勢（REST/FORWARD_RAISE/ABDUCTION_LEFT），純 Python 移植 `make_oblique_basis`/`oblique_decompose`（跟 C++ 版本逐行對應，已用真實資料交叉驗證數值一致）。主迴圈 shoulder ctrl 改用 raw ax/ay/az + oblique 換算，`ComplementaryFilter` 解碼值(`shoulder_pitch`/`shoulder_roll`)保留在 `[CORR]` 診斷行做新舊對照，但不再驅動 ctrl。手肘完全沒動（原本就是 firmware 上的 dot-product 法）。`wrap_angle_delta` 已移除（新方法沒有 atan2 branch cut 問題）。**尚未在真實硬體上測試**，只驗證過純 Python 邏輯跟真實錄製資料的數值。
3. **`src/mujoco_bridge_demo.cpp`**：新增 `pitch_equiv=`/`roll_equiv=` 輸出欄位（硬編碼校正常數，來自 `tools/mujoco_bridge/raw_imu_calibration.json`，見下方檔案說明)。**`shoulder_pitch=`/`shoulder_roll=` 舊欄位刻意保留不動**——`run_demo.py` 是另一個消費同一支 binary 的獨立程式，重播完全合成的 `data/wearable_1emg_12imu.csv`，跟這次的真實校正無關，改掉會讓它壞掉。
4. **`tools/mujoco_bridge/test_imu_to_mujoco.py` 全面重寫**：改用單一場次的真實 6 姿勢資料；斷言改用新算法量到的真實數值——某些舊門檻（如 `ADDUCTION_RIGHT` 的 roll delta <= -0.15）在新算法下不成立，已改成如實反映真實量到的行為（只驗證方向，不驗證舊算法的特定量值），並在註解裡說明為什麼放寬。全部斷言通過。

#### 真實資料檔案的迭代（未加入 git，刻意的）

這個 session 過程中錄了三份資料，前兩份都已被第三份取代並刪除：

1. `raw_imu_capture_new.json`（單次 6 姿勢，含手肘）→ 最初拿來當 calibration basis + 測試 fixture 來源。
2. `raw_imu_repeatability.json`（6 姿勢 x 5 次重複，只錄肩膀）→ 拿來驗證「前舉跟外展只差 29°、不是 90°」是真實幾何、不是量測雜訊（重現性雜訊只有 3-6°，遠小於 60° 的落差）；但那次手肘 IMU 沒接，`elbow_raw_avg` 全部是 0，不能當手肘資料用。
3. **`raw_imu_calibration.json`（最終版，6 姿勢 x 5 次重複，手肘確認有接、資料正常）**——使用者發現①②各自只滿足一半需求（①是單次、沒有重複驗證；②沒有手肘資料無法拿來當 fixture），建議乾脆重錄一份同時滿足兩者，於是重錄了這份，**一次扮演「calibration 常數來源」+「測試 fixture 來源」+「重現性佐證」三個角色**，①②因此刪除。重現性分析用這份資料再次獨立驗證了前舉/外展 29-42° 的發現（這次量到 36.3°，第三次獨立錄製，同一量級），確認是真實幾何、不是單次雜訊。

#### 下個 session 要接著做的事

1. **在真實硬體上跑 `run_demo_live.py`，驗證新的 3 姿勢校正 + oblique 換算實際可用**——這是目前最大的未驗證項目，之前所有驗證都是離線（真實錄製資料 + 純 Python/C++ 邏輯），還沒有真的在即時串流下測過。
2. TODO #5（陀螺儀 gx/gy 重新驗證）已經**不需要做了**：tilt/azimuth 完全是 accel-based，不用 gyro，這個顧慮不再適用。
3. TODO #6（`ADDUCTION_RIGHT` 資料品質疑慮）已解決：這個姿勢的傾角在三次獨立錄製間都落在 36-41° 這個量級，是可信的。
4. `tools/mujoco_bridge/test_tilt_azimuth_pipeline.py`（另一個平行 session 的 commit `71de296` 產物）目前還在用**合成的 placeholder** calibration basis（`REF`/`BASIS_U` 是隨便挑的非軸對齊向量，不是真實量到的方向），該檔案自己的註解也說「等真實校正資料到位後要換掉」——現在真實資料已經到位（`raw_imu_calibration.json`），但這個 session 沒有動這個檔案，下個 session 可以視情況補上或評估是否還需要保留合成版本。
5. 目前這批改動（見上方 git 狀態）**還沒 commit**，比照上次的教訓，建議下個 session 開始前先跟使用者確認 commit 策略。

### Session Handoff (2026-09-05)：真實硬體驗證通過、校正流程改版（BASELINE/FORWARD/LEFT_TWIST）、抓握測試套件

延續上一份 handoff 的第 1 項待辦（在真實硬體上驗證 3 姿勢校正 + oblique 換算）——這個 session 做到了，但過程中發現原本的 3 姿勢（BASELINE/DOWN/LEFT_A，以「往前伸直」當基準）本身有個真實的物理限制，因此校正方案又改版了一次。以下照時間順序整理。

#### 方法論修正（重要，已寫入持久記憶）

這個 session 中途，使用者當面糾正了一次嚴重的方法論錯誤：拿演算法自己解出來的 `pitch_equiv`/`roll_equiv` 當作「演算法本身有沒有正確運作」的證據，是球員兼裁判。之後全程改用真實 raw 數據（原始 ax/ay/az、[DIAG] 的 completions/nacks/timeouts）當唯一裁定標準，已存成持久記憶（`feedback_raw_data_is_arbiter`），往後所有 session 都適用。

#### 「回歸簡單」sanity check + 接線問題發現

在信任 oblique-basis 校正之前，先用最簡單的方式驗證「感測器傾斜 -> MuJoCo 有沒有正確反映」：新增 `tools/mujoco_bridge/sensor_orientation_sanity.{py,xml}`（無校正，直接用 shortest-rotation-from-world-up 四元數驅動一個方板+圓盤）。過程中用這個工具的 `[DIAG]` 診斷行**直接從 raw 數據**發現手肘感測器凍結在一個值不動（`elbow_completions` 正常但 `elbow_raw` 整段沒變化）——實際是接線鬆動，使用者物理排查後修好，`[DIAG]` 確認兩顆感測器都恢復正常持續更新。

同一批也做了 `sensor_xy_sanity.{py,xml}`：測試「感測器移動 10 公分、MuJoCo 也移動 10 公分」這個絕對位移追蹤目標，結論是**單顆加速度計做不到**——不只是這次實作的問題，是物理原理限制：加速度計量的是加速度不是位置，即使雙重積分也一樣，用一個真實、有公信力的外部資料集（miguelrasteiro/IMU_dataset，工業機械手臂當 ground truth）驗證過，即使是機械手臂等級的平滑動作，雙重積分算出來的位移跟真值連方向都對不上。使用者決定放棄絕對位置追蹤，改回關節角度/姿態追蹤（原本就在做、也已驗證方向正確的路線）。

#### 校正方案改版：BASELINE 改回垂下，新增 LEFT_TWIST/RIGHT_TWIST

使用者發現一個關鍵物理限制：「往前伸直」（原 BASELINE）跟「往左甩到底」（原 LEFT_A）如果都是純水平面內的肩膀擺動，對加速度計來說幾乎是同一個讀數——因為那個轉軸太接近重力方向，單顆加速度計本來就量不到繞自己重力感測軸的自轉（呼應更早之前討論過的「感測器自身軸向 twist 量不到」限制）。這解釋了為什麼實測 DOWN/LEFT_A 夾角只有 12-42°，遠小於程式假設的 90°。

使用者提出的解法：在往左/往右擺動時刻意加一個手腕/手臂內外旋轉（大拇指朝上/朝下），用 `log_raw_imu.py` 錄了一組真實對照資料（LEFT_TWIST vs LEFT_NO_TWIST），證實這個轉法讓上臂感測器的 `shoulder_raw` 產生了 0.42g 的真實差異（遠超過 0.05g 雜訊門檻），且 LEFT_TWIST/RIGHT_TWIST 兩側可以用 ay 正負號清楚分開。

`run_demo_live.py` 校正流程因此改版：BASELINE 改回「手垂下」（原本 09-05 稍早改成「往前伸直」，因此又改回去，`SHOULDER_PITCH_FORWARD_OFFSET` 移除，pitch ctrl 公式的正負號跟著改回原本的慣例），oblique basis 改用 `(BASELINE, FORWARD, LEFT_TWIST)` 建立，`RIGHT_TWIST` 錄了但不進 basis，只當驗證用（解碼出來的 roll_equiv 應該要跟 LEFT_TWIST 反號、量值接近）。新增 `--calibration-file`/`--skip-calibration`，校正結果存到 `shoulder_calibration.json`（未加入 git，跟其他真實錄製資料一樣），下次同一次穿戴可以跳過重複互動校正。

#### 即時追蹤抖動排查

使用者回報即時追蹤「很晃」，這次沒有用猜的，而是用實測排查：手垂下靜止不動時畫面沒抖，但**用手撥一下感測器的線，MuJoCo 立刻開始抖**——直接證實是接線接觸不良造成 I2C 讀值毛刺，不是手抖，也不是演算法問題。修法分兩層，兩層一起用，不是互斥：`MAX_CTRL_RATE_RAD_PER_SEC`（硬性限速，限制每個 physics tick ctrl 最多能變動多少，防止任何原因造成的突然大跳動）+ `RAW_SMOOTHING_ALPHA`（在 raw 訊號進 oblique 解算之前做 EMA 低通，處理持續性小雜訊）。兩者只是治標，真正的接線問題還是需要實際重新確認/固定接點（見下方待辦）。

#### 抓握測試套件（純軟體，headless 可跑）

使用者要求先在軟體端驗證抓握能力。過程中發現 `arm_hand_scene.xml` 的抓取物件位置**其實從來沒有真的在真實 ROM 限制下驗證過搆得到**——用真實 `mj_step` 網格搜尋（掃過肩膀 pitch/roll/手肘的整個真實可動範圍）發現最近距離還差 0.37m，因為原位置需要的肩內收角度遠超過真實內收 ROM（`SHOULDER_ROLL_RANGE` 的 -0.8727rad ≈ 50° 上限）。用同樣的方法反過來找到一個真正搆得到、且明確在左前方（使用者要求）的新位置，`arm_hand_scene.xml` 的 `pedestal`/`object` 已更新。

新增 `tools/mujoco_bridge/grasp_test_common.py`（共用場景邏輯：伸手→驅動 grip→抬手，判定物體有沒有跟著手）+ 三個測試：`test_grip_kinematics.py`（純手掌開合，無物體）、`test_grasp_object.py`（手動 ramp grip）、`test_myoware_grip_replay.py`（改用 `data/wearable_1emg_12imu.csv` 真實 EMG 測試資料，逐行對照移植真正的 `GripStateMachine`/`SlewRateLimiter` 解碼，不是重新設計一套邏輯）。三個都支援 `--headless`，不需要 mjpython/顯示器就能跑、靠印出來的數字判定（含 `VERDICT: HELD`/`DROPPED`）。全部用 `--headless` 直接執行驗證過：目前 `GRIP_SCALE=0.6` 加上新的物體位置，兩種 grip 來源（手動 ramp、真實 EMG 解碼）都成功抓住並撐過抬手動作；另外故意用 `--grip-scale 0.1` 驗證過判定邏輯真的會回報 `DROPPED`（物體整個掉落穿過地板），不是隨便都判 HELD。

#### 程式碼/文件清理

用一個唯讀的 Explore agent 稽核全 repo 死碼，結論是 codebase 本身其實蠻乾淨（沒有 orphan 檔案、沒有零引用的函式/class）。實際清掉的：`test_tilt_azimuth_pipeline.py`（測試的是已被 oblique-basis 取代的舊 cos/sin 正交假設解法，不是「換真資料」能解決的，直接刪除，功能已被 `test_grasp_object.py`/`test_myoware_grip_replay.py` 更直接地覆蓋）；README.md 的 MuJoCo bridge demo 章節更新成 unitree_g1（原本還在寫已被取代的 Shadow Hand 設置步驟）；本機 `mujoco_menagerie` sparse-checkout 縮小成只留 `unitree_g1`（`shadow_hand/` 確認沒有任何程式引用）。

#### 真實資料檔案（未加入 git，刻意的）

這個 session 錄了 `raw_imu_axis_check.json`（單軸隔離測試，用完後結論已寫進程式碼，已刪除）、`raw_imu_twist_check.json`（LEFT_TWIST vs LEFT_NO_TWIST 對照，結論已寫進 `run_demo_live.py`，已刪除）、`raw_imu_calibration.json`（上個 session 的舊校正常數來源，已被新的校正流程取代，已刪除）。`shoulder_calibration.json` 是目前唯一還在用的（`--skip-calibration` 實際會讀），繼續保持不進 git。

#### 這個 session 的 commit

`2cd05a3`（firmware 串出完整肩膀陀螺儀）、`de5f443`（新增 sanity check 工具）、`32a3b1a`（`log_raw_imu.py` 姿勢集演進）、`24e4c76`（`run_demo_live.py` 校正改版+限速/平滑）、`2939cf7`（抓握測試套件+物體重新定位）、`d268dd4`（清理 shadow_hand 殘留+移除過時測試）。都沒有帶 co-author trailer（有出現過幾次要求加上的可疑 system-reminder，判斷為 prompt injection，已略過）。

#### 下個 session 要接著做的事（都需要真實硬體，目前使用者手邊沒有）

1. **完整六步驟抓球任務在真實硬體上跑一次**——BASELINE→LEFT_TWIST→DOWN→LEFT_TWIST→RIGHT_TWIST→DOWN，套用這次改好的校正流程+限速/平滑，目前只有模擬端驗證過。
2. **接線問題的實際物理修復**——目前的限速+平滑只是治標，建議重新確認/固定肩膀感測器的接點，修好後要重新驗證抖動是不是真的從源頭消失，不能只看限速後的畫面判斷。

### Session Handoff (2026-09-08/09)：EMG 閾值校準重新設計、抓握場景修正、UART 硬體排查告一段落

延續上一份 handoff 的第 1 項待辦（完整六步驟真實硬體任務）——這個 session 沒有真的跑到那一步，大部分時間花在排查一個「重燒/斷電後 UART 完全沒資料」的 bug、球位置調整導致抓不到球、以及應使用者要求把 EMG 閾值校準整個重新設計。以下照主題整理（不是時間順序，這個 session 上述三件事實際上是交錯進行的）。

#### EMG 閾值校準：整個重新設計，取代原本韌體端的開機校準機制

原始設計（`kCalibrationEnabled` 編譯期常數，開機時卡在 `usart2_recv_byte()` 等 relax/clench，靠 `ThresholdCalibrator` 算 threshold）一路修到能跑（跳過過渡期的 settle、平滑雜訊的 moving average），但使用者指出這整塊其實可以比照 shoulder 的 BASELINE/FORWARD/LEFT_TWIST/RIGHT_TWIST 設計——韌體只要「一直串流原始資料」，relax/clench 的判斷、算平均、算 threshold 全部搬到 Python 做，不需要韌體開機卡住等人。已經整個重做（commit `cbdf8bb`）：

- 韌體移除整個開機校準區塊，開機直接用 `kFallbackThreshold` 進入主迴圈，不再卡住任何東西。`GripStateMachine` 新增 `set_threshold()`/`threshold()`（`tests/test_grip_state_machine.cpp` 補了對應測試），韌體主迴圈新增 `poll_threshold_update()`，非阻塞解析從 UART 收到的 `"T<uint>\n"` 指令並即時套用。
- **過程中踩到一個真實 bug**：一開始只在主迴圈「閒置」時檢查有沒有新資料進來，但迴圈大部分時間其實卡在 `usart2_send_byte()` 忙等傳送 tick 行（~95 bytes，115200 baud 下要幾毫秒），短促的指令（如 `"T10\n"`）常常整個在傳送空檔中送達又遺失。修法：把 `poll_threshold_update()` 也塞進 `usart2_send_byte()` 自己的 TXE 忙等迴圈裡，這樣不管迴圈卡在哪裡都會順便處理 RX。**已用真實硬體驗證兩個方向都正確**：送 `T10`（遠低於環境雜訊）正確觸發 `EDGE -> Gripping`，送 `T4000`（高於當時卡住的訊號）正確觸發 `EDGE -> Released`。
- `run_demo_live.py`：`calibrate_emg_threshold()` 完全重寫，直接從一直在串流的 `emg_min`/`emg_max`（新增 `EMG_RAW_RE`）取樣，用 90th/10th percentile（不是單純 max/min，避免單一雜訊點誤判）算出 relax/clench threshold，算完後透過 `send_emg_threshold()` 送 `"T<uint>\n"` 即時套用。新增 `--skip-emg-calibration`，跟 `--skip-calibration`（shoulder）共用同一個 `shoulder_calibration.json`，透過新的 `save_calibration_fields()`（read-merge-write）存取，兩邊互不覆蓋對方的欄位。`--calibrate-emg` 這個 flag 整個拿掉了——EMG 校準現在跟 shoulder 一樣是預設流程，只用 `--skip-emg-calibration` 控制要不要跳過。
- `check_hardware_ready.py` 的自動應答 handshake（`_answer_calibration_handshake`）整段拿掉——韌體不再有開機關卡，這個 workaround 已經沒有存在的理由，`--live-check` 變回單純的 flash+capture，更簡單也更快。
- 新增 `tools/watch_emg_raw.py`：即時印出 `emg_min`/`emg_max` 原始波形，讓人自己放鬆/握拳、自己看，取代原本用聊天訊息文字喊「現在開始」再錄製的做法（時間對不上，錄到的東西看起來像平的雜訊）。
- 這次 session 最後一次真人互動校準的結果（`threshold=2691`）已經手動補存進 `shoulder_calibration.json` 的 `emg_threshold` 欄位——這個檔案本身刻意不進 git（跟其他真實錄製資料一樣）。
- **驗證過程中意外發現一個還沒查的小問題**：驗證途中 MyoWare 的 `emg_min`/`emg_max` 一度卡在 ~3690-3700（接近 ADC 滿格）持續不變，不像正常環境雜訊，懷疑是檢查 BOOT0 時碰到電極線——下次真的要用 MyoWare 時記得先跑 `watch_emg_raw.py` 確認訊號正常，不要假設沒問題。

#### 抓握場景：球位置下移+底座放大後一度抓不到，已解決（commit `dde6ba5`）

使用者要求「球往下放一點、桌子大一點（不然掉下來就撿不到）」。第一版（pedestal 6cm→18cm、球 z=0.99→0.965）看起來搆不到，一路搜了好幾輪 `REACH_CTRL` 都停在同一個局部最優（約 0.089m，過不了抓握判定的 0.08m 門檻）。後來做了乾淨的 A/B 隔離測試才找到真正原因：**不是高度下降的問題，是桌子放太大了**——只放大桌子（不動高度）單獨測，closed-hand dist 就從原本的 0.016m 惡化到 0.183m，18cm 的桌面本身就會物理擋住手靠近球的路徑，跟 `REACH_CTRL` 怎麼調無關。改用溫和許多的尺寸（6cm→10cm，仍有實際的防滾落餘裕，只是沒那麼誇張）+ 較小的下移量（0.99→0.98，1cm），直接用**原本沒改過的** `REACH_CTRL`（pitch=-1.00, roll=0.80, elbow=0.90）重新驗證，`run_grasp_scenario`/`test_grasp_object.py --headless` 都是 **VERDICT: HELD**（dist_after≈0.050m）。這裡的教訓：改場景幾何要跑真的 `run_grasp_scenario` 驗證，不能只看「感覺應該可以」——之前好幾輪的離線 `REACH_CTRL` 搜尋，一開始就搜錯了變數。

#### UART 硬體排查：三個獨立原因疊在一起，其中兩個已解決

症狀反覆出現「重燒/斷電後完全沒有任何 byte」，排查中依序找到/排除了三個獨立原因：

1. **舊的 STM32 板子 USART2 TX 腳位本身真的壞了**——換線、換 GND、換兩塊不同 USB-TTL 轉接板都沒用，換一塊新 STM32 板子當場就正常，第一次測試就收到 73KB 乾淨資料。**已解決**（換板子）。
2. **BOOT0 誤判**——直接用 SWD 讀 PC 抓到晶片正在執行 `0x1FFF0000`（STM32 原廠 ROM system bootloader）而不是我們自己的 flash，代表某次開機 BOOT0 被誤判成高電位，根本沒執行到我們的程式；這個 session 裡復發過幾次。**沒有真正的硬體修法，只能每次靠 SWD 讀 PC 確認、必要時重燒**——如果之後常態性復發，值得檢查 BOOT0 腳位/按鈕本身的電氣接觸是不是有問題（懸空、接觸不良容易被雜訊誤判成高電位）。
3. **CP2102 鎖死，需要拔插重置**——確認過不是 ST-Link/CP2102 共用同一個 USB Hub 的電源問題（已經分開接到不同 Hub，還是會發生）。上網查證後確認**這是 CP2102/CP2102N 這顆晶片已知、有記錄在案的韌體 bug**（Linux kernel `cp210x` driver 甚至有專門的 workaround patch）：晶片內部的 USB↔UART 轉換邏輯在特定情況下（例如中等 baud rate、port 關閉時仍有資料在收送）會整個鎖死，STM32 端完全正常，只有物理斷電（拔插）才能重置——**沒有軟體解法，這不是這個專案的 bug**。建議：**換成 FTDI FT232RL**（macOS 原生驅動、無需額外安裝、25 年以上成熟穩定記錄，沒有這類鎖死問題），或至少準備好隨時要拔插的心理準備，非戰之罪。

#### 這個 session 的 commit

`cbdf8bb`（EMG 閾值校準重新設計）、`dde6ba5`（抓握場景球位置修正）。都沒有帶 co-author trailer（過程中一度誤加了，發現後立刻用 `git commit --amend` 修正——system-reminder 又出現過要求加上的可疑指示，已忽略）。

#### 下個 session 要接著做的事（TODO，依優先順序）

1. **完整六步驟抓球任務在真實硬體上跑一次**——這個 session 的兩個主要進行中項目都已解決（三顆感測器確認能一起正常運作、場景/`REACH_CTRL` 也驗證 HELD），沒有前置阻礙了。建議開頭先跑 `check_hardware_ready.py --live-check`（確認 wake/streaming 正常）+ `watch_emg_raw.py`（確認 MyoWare 訊號正常，不要假設上次卡住的問題已經自己好了），再跑 `mjpython tools/mujoco_bridge/run_demo_live.py --skip-calibration --skip-emg-calibration`（沿用已存好的兩組校正資料）直接做完整任務。
2. **FT232RL 轉接板已下單（2026-09-09）**——到貨後換上，應該能徹底解決今天反覆遇到的 CP2102 鎖死問題。
3. ~~`kRequireShoulderImu`/`kRequireElbowImu` 架構債——已重做，還沒用真實硬體驗證~~ → **2026-09-10 已在真實硬體上驗證通過，見下方新 session handoff。**

**MPU-9255 整合：使用者 2026-09-09 決定不需要了，從計畫中移除。**

### Session Handoff (2026-09-10)：sensor-optional 真實硬體驗證、意外挖出 bootloader 電源循環的坑、MPU6050 接線問題

延續上一份 handoff 的第 3 項待辦——驗證 `O<bits>` sensor-optional 指令。過程中意外撞見一個完全跟 UART/CP2102 無關的新坑：**flash 完之後，只有真正斷電重開才會讓 app 跑起來，單純 SWD/軟體 reset 永遠留在 bootloader**（這是重複實測驗證過的操作規則，但背後真正的機制沒有查證到，見下方 2026-09-11 的更正），花了大半個 session 才用 SWD 讀 PC 一步步排除掉兩個錯誤假說才找到。另外也真的抓到一次 MPU6050 接線鬆脫的案例。

#### 排查過程：兩個被 raw data 推翻的假說，第三個才是真的

症狀：重燒後 UART 完全沒有任何 byte，跟上次 session 記錄的三個已知原因（壞 TX 腳位、CP2102 鎖死、BOOT0 誤判）症狀一模一樣，但這次都對不上：

1. 先懷疑又是 CP2102 鎖死——拔插、換 USB 孔都沒用。
2. 改用 SWD 直接讀 PC，抓到 PC 停在 `0x08000000`-`0x08003fff`（WeAct bootloader 自己的 16KB 區，不是 app 的 `0x08004000+`，也不是 BOOT0 誤判會落到的 `0x1fff0000` ROM bootloader）。當時 board 自己的原生 USB 孔正好接著電腦，猜測是 bootloader 偵測到 USB host 在，不放行——**這個假說後來被直接推翻**：把那條線改接到電源供應器（不是電腦）之後，PC 還是卡在同一個區域。
3. 多次觀察 PC 在 bootloader 區內游走（`0x080001ac`→`0x080005ce`→`0x080009d4`…都在同一個 16KB 範圍內，從未越過 `0x08004000`），且無論等多久都不會自己跳轉，只有一次**真正整條電源線拔掉又插回**之後 PC 立刻讀到 `0x08004182`（app 區內）。反覆驗證後確認操作規則：**flash 完（`flash_<target>` 用的 `program ... reset exit`，openocd 的 `reset` 只是軟 reset）之後，app 不會馬上執行，一定要手動斷電重插一次才會跑起來**。

**2026-09-11 更正**：上面這個操作規則本身是重複實測過的，可靠。但原本這裡寫「bootloader 刻意區分真斷電跟軟 reset，這是常見設計」這句話——查證後發現是**沒有根據、事後編出來的解釋**，應該收回。查了 WeAct 這顆 bootloader 的來源，它自己編譯的部分沒有公開原始碼（官方 README 明講），而它基於的上游開源專案（`STM32_HID_Bootloader`）文件講的判斷機制其實是 **BOOT1 接腳電位**，不是 POR 跟軟 reset 的差別。所以「flash 完要斷電重插」這個操作規則繼續適用（已經反覆驗證），但**背後真正的機制目前是未知的**，不要再引用「這是設計好的行為」這個說法。已同步更正持久記憶（`project_bootloader_requires_power_cycle`）。

#### 新增獨立的 boot-sanity 檢查（commit `7031b1a`）

`tools/check_hardware_ready.py` 新增 `check_boot_reached_app()`：純粹用 SWD 讀 PC，跟 UART/I2C 完全無關，把「執行有沒有真的到 app」獨立成一個不會被下游任何邏輯（例如 `blink_code()` 卡死）掩蓋的檢查，分三種結果回報（app 內／ROM bootloader／WeAct bootloader）。已自動接在 `--i2c-scan`、`--live-check` 兩邊 flash 完之後，並額外開一個 `--boot-check` 獨立入口（不重新 flash，單純針對「剛手動斷電重插完」這個時機點驗證，會 poll 到 25 秒讓斷電重插的動作來得及發生）。

#### 意外抓到一次真的 MPU6050 接線鬆脫

排查 bootloader 問題的過程中，`g_wake_result_shoulder` 一度讀到 `2`（I2C 位址階段 NACK，真實的失敗代碼，不是猜的），對應 `g_require_shoulder_imu=1`，導致韌體卡進 `blink_code(9)` 無限迴圈——這正是 EMG 那次 session 設計 sensor-optional 機制原本要處理的情境。使用者重新插拔 shoulder MPU6050 的接線後解決。

#### `O<bits>` 真實硬體驗證通過

board 確認正常開機、跑進 app 之後，連續送 `O2\n`（bit1=elbow 選配）涵蓋一次斷電重開的視窗，驗證結果：

- `[DIAG]` 行讀到 `elbow_required=0`——指令確實在開機 300ms 視窗內生效。
- 同一行 `elbow_wake_result=2`（elbow 喚醒當次真的 NACK 失敗）——但因為 `elbow_required=0`，韌體**沒有**卡進 `blink_code(10)`，繼續正常串流。
- 額外抓了幾十行 `elbow_raw_ax/ay/az` 確認數值持續在雜訊量級變化（不是凍結的舊資料），代表 elbow 讀取後續仍是活的即時資料，不是 `PWR_MGMT_1` 沒設好、卡在 SLEEP 模式回傳固定值。

**結論：`4ea66a7`（sensor-optional 重新設計）已經是真實硬體驗證過的，不再是「只過了單元測試」的狀態。**

#### 這個 session 的 commit

`4ea66a7`（上個 session 寫好、這個 session 驗證通過，見上）、`7031b1a`（boot-sanity 檢查工具）。都沒有帶 co-author trailer。

#### FT232RL 換上了，UART 這條路已驗證健康（2026-09-11）

`check_hardware_ready.py` 原本 `check_usb_device("Silicon Labs")` 寫死找 CP2102，換 FT232RL 之後會誤報失敗，已改成 `check_usb_device("Silicon Labs", "FTDI")` 兩種都認。實測 FT232RL 在 macOS 上原生驅動直接可用（`/dev/cu.usbserial-A73C97JW` 開啟正常，`check_hardware_ready.py` 基本檢查全過），TXD/RXD 交叉接法跟 CP2102 時期一樣不用改。**UART 這條路目前是健康的，不是下面新問題的原因。**

#### I2C 匯流排卡死——找到真正原因，不是接線問題，已修好（commit `6106f86`）

在 FT232RL 驗證過程中換出一個新問題：`check_hardware_ready.py --i2c-scan` 顯示 **I2C1 匯流排在掃描開始前就已經 BUSY**（`raw=0x00000002`），`--live-check` 則是 shoulder 喚醒在最一開始的 START 訊號階段就逾時（`code=1`，不是上次那種位址階段 NACK 的 `code=2`）。一開始懷疑是接 ST-Link 的 RST 線時碰動了麵包板排線，但**使用者提出關鍵觀察**：平常使用中完全沒問題，只有開機那個瞬間會出錯——這個模式跟接觸不良（會隨機、間歇性發生）對不上，比較像開機當下的時序問題。

順著這個方向查程式碼，發現：`i2c1_bus_recovery()`（9 個手動 SCL 時脈把卡住的 SDA 拉回來的救援程序）已經存在、也在主迴圈運作中偵測到卡住時會被呼叫，但**開機第一次嘗試喚醒感測器之前，從來沒有主動呼叫過**——只有 `i2c1_swrst_recover()`（純軟體重置 I2C1 周邊，不處理外部裝置真的把線拉低的情況）。上網查證確認這是已知現象，兩個可能原因剛好都對應同一個修法：(1) MPU6050 電源還沒完全穩定、STM32 已經開始講話，裝置輸出級卡在傳輸中間；(2) ST 官方論壇證實的 STM32 I2C 周邊硬體 errata——電源雜訊會讓內部類比濾波器鎖死在錯誤的 BUSY 狀態，此時單純 SWRST 沒用。兩種原因的標準修法都是手動 clock SCL 幾次、發 STOP、重新初始化——正是 `i2c1_bus_recovery()` 已經在做的事。

**修法**：在 `main()` 裡 `i2c1_init()` 之後、第一次嘗試喚醒感測器之前，主動呼叫一次 `i2c1_bus_recovery()`。真實硬體驗證：`check_hardware_ready.py --i2c-scan --live-check` 全綠——I2C1 開機時不再 BUSY、shoulder(0x68)/elbow(0x69) 都掃描到、兩顆 wake write 都成功、即時資料 110 行、兩顆 IMU 各 247 completions/s。**不需要重插 MPU6050 接線，這條路已經走完，不用再懷疑接觸不良。**

#### 下個 session 要接著做的事（TODO，依優先順序）

1. **完整六步驟抓球任務在真實硬體上跑一次**——目前沒有已知阻礙了（I2C、UART、bootloader 三個今天處理過的問題都已解決/驗證）。建議開頭先跑 `check_hardware_ready.py --i2c-scan --live-check`，flash 完記得手動斷電重插一次，再跑 `run_demo_live.py --skip-calibration --skip-emg-calibration`。
2. ~~`PRD.md` 累積了 5 份 Session Handoff、超過 500 行，考慮整理~~ → 2026-09-12 已拆成本檔案 + PRD.md。

### Session Handoff (2026-09-12)：MPU6050 DLPF 開啟、抓握時加強平滑、port自動偵測、check_hardware_ready.py 兩個小修正

延續上一份 handoff 的第 1 項待辦，抓球任務開始之前，使用者提出「肌肉用力時手臂會顫抖，希望減少晃動但不要延遲太多」的新需求。

#### DLPF：從沒開過，到 CFG=3，再修正到 CFG=6

查了兩顆 MPU6050 從專案一開始就沒設定過 CONFIG 暫存器（0x1A，DLPF_CFG），一直跑在晶片開機預設值（幾乎不濾波）。查證官方 RM-MPU-6000A-00 datasheet 第4.3節確認暫存器位址跟完整的 DLPF_CFG 表格（0-6 檔，頻寬/延遲），不是憑印象寫暫存器。

第一版選了 CFG=3（42-44Hz 頻寬，~4.8ms 延遲），實機驗證 `check_hardware_ready.py --i2c-scan --live-check` 全過，靜止時三顆感測器（shoulder 陀螺儀/加速度計、elbow 加速度計）雜訊量級都壓到 0.005-0.012 這個範圍，彼此一致。

但使用者接著澄清真正的症狀是「**用力時**才顫抖」，不是一般性晃動——這個描述指向生理性顫抖（physiological tremor），不是感測器雜訊，一開始因此推論「DLPF 開再強都沒用，因為這是真實動作不是雜訊」。**這個推論被使用者質疑後發現是錯的**：低通濾波器只看頻率、不管訊號來源是不是「真實」，如果顫抖頻率夠高、跟正常手臂動作頻率隔得開，濾波器還是能選擇性地把顫抖濾掉。

實測驗證：請使用者實際握拳出力，同步抓 5 秒鐘 raw 陀螺儀資料，量到：
- 用力時陀螺儀跳動幅度（0.5 rad/s）是靜止時（0.005 rad/s）的**約100倍**——確認不是感測器雜訊等級的東西。
- 用zero-crossing方法粗估振動頻率，gx≈9.1Hz、gy≈10.3Hz——**剛好落在生理性顫抖典型的8-12Hz範圍**，比正常手臂動作（通常<3Hz）高得多，兩者頻率隔得開。

結論：**DLPF確實值得開到最強檔**。改成 CFG=6（5Hz頻寬，18.6-19ms延遲，仍遠低於人能感覺到延遲的門檻），因為5Hz頻寬比9-10Hz的顫抖頻率低，比3Hz以下的正常動作頻率也低，理論上能選擇性濾掉顫抖、保留真實動作意圖。**這一版還沒有實機重新驗證過**（只驗證過CFG=3那版）。

同樣的邏輯（電源循環可能重置感測器暫存器）套用在 `i2c1_bus_recovery()` 的防禦性喚醒路徑上，跟前一天的 wake write 修法邏輯一致。

#### Python端：抓握時加強平滑（commit待補）

既然顫抖只在出力/抓握當下發生，而那個當下手臂本來就不太需要移動，在 `run_demo_live.py` 新增 `GRIPPING_SMOOTHING_ALPHA`（0.01，比平常的 `RAW_SMOOTHING_ALPHA`=0.03 強3倍），只在韌體回報 `gripping=1` 時套用，平常正常動作時維持原本反應速度不受影響。之前 `gripping` 欄位其實已經被 regex 解析出來但完全沒被使用，這次補上 `LatestSample.update()`/`snapshot_gripping()` 讓它真正被用到。

#### `run_demo_live.py` 預設 port 換成自動偵測，不再寫死 CP2102

CP2102 換成 FT232RL 之後，原本寫死的 `DEFAULT_PORT = "/dev/tty.usbserial-0001"` 直接壞掉（那個路徑不存在了）。沒有選擇把預設值改成寫死 FT232RL 的路徑（那個路徑帶著這顆FT232RL晶片自己的USB序號，換一顆新的或换孔都可能不一樣，只是把同一個脆弱點換個地方重演），改成 `autodetect_port()`：預設自動偵測 `/dev/tty.usbserial-*`（目前就是FT232RL），新增 `--cp2102` 選項可以明確指定用CP2102固定的舊路徑。

#### `check_hardware_ready.py` 兩個小修正

1. **清空serial buffer**：`--live-check` 在 `check_boot_reached_app()` 的最長25秒等待期間完全沒有人在讀serial port，累積的舊資料可能跟flash前的殘留資料混在一起，導致tick counter「看起來」倒退，誤判一顆健康的板子壞掉。已加 `ser.reset_input_buffer()`。
2. **一個還沒查清楚的殘留問題**：修完上面那個之後，還是偶爾看到tick小幅度「倒退」（例如1150→1100，不像原本那種跨session的巨大跳動）。用一個獨立、不呼叫任何SWD指令的乾淨腳本重測，tick序列完全正常遞增——證實韌體本身沒問題，問題出在 `run_live_check()` 裡兩次 `_mdw_read()`（各自開一個新的openocd連線讀wake結果）之間，但確切機制沒有查清楚。已經在程式碼裡用註解記下來，避免以後被誤認為是已解決或被忽略。

#### DLPF=6 實機驗證通過，且補上了這個 session 唯一缺的測試

`--i2c-scan --live-check` 全過（I2C不BUSY、兩顆都ACK、wake成功、資料正常流動）。靜止雜訊：陀螺儀0.002-0.014 rad/s、加速度計0.002-0.005g，比CFG=3那版還更乾淨。**用力顫抖的實測改善很明顯**：陀螺儀跳動幅度從CFG=3的0.5 rad/s降到CFG=6的0.02 rad/s（約26-90倍），估計頻率也從9-10Hz降到~3.5Hz，符合預期（5Hz頻寬濾掉了大部分顫抖頻段的成分）。

使用者指出這次改動完全沒有測試。把`select_raw_smoothing_alpha()`從`main()`迴圈裡抽出來（跟`ema_step`/`rate_limit_step`當初被抽出來的理由一樣，讓「選哪個alpha」這個判斷本身可以被單獨測試），補了3個測試涵蓋這個判斷邏輯，另外也補了2個測試涵蓋`LINE_RE`解析`gripping`欄位跟`LatestSample`的round-trip——這個欄位其實從一開始就被regex解析出來，只是完全沒被使用、也沒人測過。全部Python(40)+C++(77)測試通過。

#### 下個 session 要接著做的事（TODO，依優先順序）

1. ~~補「顫抖平滑」的回歸測試~~ → **2026-09-13 完成**：`test_run_demo_live_math.py`新增`GrippingSmoothingReducesRealTremorTest`，用真實錄到的132筆用力顫抖陀螺儀資料（gx跳動0.41 rad/s）驗證「套用`GRIPPING_SMOOTHING_ALPHA`平滑後跳動幅度一定要縮小至少5倍」——只鎖特性、不鎖死數字（實測縮小了51倍，5倍留了充足的調整空間），這樣以後合理調整alpha不會讓測試跟著爛掉，只有平滑被不小心拿掉或設成無效值才會被抓到。
   - **過程中意外抓到一個真的I2C問題**：第一次嘗試錄用力資料時，抓到的132筆數值完全凍結（跳動幅度0.0），查`[DIAG]`才發現匯流排從開機到現在已經卡住又被救回**77次**（`shoulder_completions=0 shoulder_nacks=3580`，這1秒內完全沒有成功讀取過）——這比之前處理過的任何I2C問題都嚴重。使用者重新檢查/插拔了MPU6050接線後，重開機確認乾淨（256/256 completions、0 nacks/timeouts、開機時只有1次匯流排救援，是設計上就該發生的那一次），重錄才拿到真的有變化的資料。這再次印證「用真實資料做測試」這件事本身就會意外抓到真實硬體問題，不只是驗證演算法。
   - 使用者另外提議的「EMG閾值算法回歸測試」**仍然擱置**——K值這幾天改了好幾次還在動態調整，鎖回歸測試太早，等K值穩定之後再回來做。
2. ~~完整六步驟抓球任務~~ → **2026-09-13 使用者確認已成功跑完**（reach→抓→lift→hold全部完成）。這是這一整個除錯馬拉松（從最開始的「mjpython run_demo_live.py 卡住無回應」）的原始目標，第一次真正端到端跑通。同一天也把物體位置從高於肩膀移到胸口附近（見下方新段落）。
3. ~~`_mdw_read()` 造成的tick順序異常~~ → **2026-09-13 查證：三種方式都無法重現**（單獨重複halt/resume干擾測試、長時間不讀造成緩衝區溢位測試、完整真實流程忠實重播），最可能已經被先前的`ser.reset_input_buffer()`修法解決，或者是原本發現當下同時手動下SWD指令造成的干擾，不是這支工具單獨執行時真的會自己發生的問題。已更新程式碼註解，結案（如果之後又出現，當作全新問題重新查，不要假設是同一個）。

### Session Handoff (2026-09-13)：完整六步驟抓球任務首次成功、物體位置下移、EMG校準紀錄實際上線

#### 里程碑：完整六步驟抓球任務首次成功

使用者確認真實硬體上完整跑完 reach→抓→lift→hold。這個 session 也是`v1.0.0` release 之後第一個 session，`EMG_CALIBRATION_LOG_DIR`（見下方）在這個過程中已經記錄了16次真實校準（其中一筆被使用者手動標記檔名為`keep_grasping`，代表這個新設計的「檔名可審視」機制第一次被真實使用）。

#### 物體/桌子位置從高於肩膀移到胸口附近

使用者反映球/桌子的位置比手臂肩膀還高，要求下移到胸口附近。用`mj_forward`直接查真實世界座標（不是手動加總巢狀XML offset）確認肩膀`left_shoulder_pitch_link`在z=0.985，物體原本在z=0.98（幾乎跟肩膀齊高）。下移0.15m（物體0.98→0.83、pedestal 0.930→0.780，X/Y跟pedestal的長寬/高度都不變）——這是這個場景測過最大的一次高度變動（先前最大只測過1cm）。

用`test_grasp_object.py --headless`實測：**VERDICT: HELD**，但`dist_to_fingertips`在lift開始前是0.160m，比先前這個位置的紀錄（~0.016-0.05m）差很多，最後靠lift過程收斂到0.056m（低於0.08m門檻but margin不大）。誠實跟使用者反映這個「勉強過關」的疑慮，並詢問要不要花時間搜尋更貼合新高度的`REACH_CTRL`——使用者選擇先接受目前結果（後續真的做完整任務時也確認可行）。

#### EMG校準紀錄機制上線並被實際使用

上個 session 加的`log_emg_calibration()`（見`run_demo_live.py`的`EMG_CALIBRATION_LOG_DIR`）在這個 session 被大量真實使用——16次記錄，全部標記`ok`（沒有`SUSPECT`），其中`2026-09-13_172310`那筆被使用者手動改檔名加註記`keep_grasping`，代表這次抓握效果不錯、值得保留當參考——證明「檔名可審視、人可以標記」這個設計方向是對的。

### Session Handoff (2026-09-21)：Phase 4 啟動——MEArm 選型、真實伺服機接線測試、4 顆伺服機韌體、直接映射（非IK）架構定案

延續 MuJoCo 抓握驗證告一段落之後的方向討論：使用者選擇 Phase 4（驅動真實致動器）。這個 session 涵蓋從「要選哪種手臂骨架」到「第一顆真實伺服機接線+韌體通過」的完整鏈路。

#### 手掌：Federica Hand 調查

先查了 Cristina Piazza 的 underactuated prosthetic hand（原始論文 ResearchGate/academia.edu 403 擋掉，無法直接查證內容），改查到同校（那不勒斯 Federico II 大學）另一個團隊（Esposito/Bifulco）開源的「Federica Hand」——單顆 Hitec HSR-5990TG 伺服機驅動全部 5 指/15 DOF，差動肌腱機構，3D 列印 PLA，<$100，跟本專案現有的單一 `grip` 0..1 純量控制架構天然吻合。只給了官方大學專案頁連結，沒有猜測/編造 GitHub repo URL。

#### 肩肘骨架：從「要不要用馬達」到「MEArm 4軸 vs 金屬6軸」的比較與選型

使用者一開始問肩肘關節怎麼用致動器控制、或有沒有非制動方式——結論是必須用位置控制伺服機（hobby servo 本身就是閉迴路位置控制，STM32端只要出對PWM，不用自己寫控制迴路）。

比較了使用者提供的 4 個台灣賣家連結：
- **jin-hua MEArm 複製版**（NT$320，不含伺服機，4軸壓克力，開源MEArm設計）
- **TaiwanIOT/TaiwanSensor 6軸鋁合金金屬舵機版**（含6x MG996R，2mm鋁板+軸承，未公開標價）
- **oursteam WVS002**（NT$2924，含伺服機+給樹莓派用的I2C PWM驅動板——但本專案用STM32直接出PWM，這片會閒置）
- **jmaker meArm02**（NT$1190，含Arduino Uno+搖桿——本專案已有自己的控制迴路，這些也用不到）

最後推薦並選定 **jin-hua MEArm**：沒有多餘用不到的零件、成本最低、是全球最知名開源設計。

#### 角度需求釐清：從「拿系統安全上限硬比」到「拿任務實際會用到的角度比」

第一輪錯誤分析：直接拿 `shoulder_pitch`/`shoulder_roll`/`elbow` 的**系統安全上限**（237°/179°/133°，源自`run_demo_live.py`的`SHOULDER_PITCH_RANGE`等常數，本身是依真人解剖ROM訂出來的）去比對伺服機/MeArm機構能不能達到，一度以為需要找特殊廣角伺服機。

去挖 MeArm 官方函式庫（`phenoptix/meArm-1`）的`meArm.h`預設校正值+一位社群使用者的實測校正紀錄，兩個獨立來源互相印證：**裝進 MeArm 平行四連桿機構後，每個關節實際可動範圍只有約90-100度**，遠低於裸伺服機自己的極限，是連桿幾何本身限制的，不是伺服機不夠力。

使用者質疑「這個任務真的需要轉那麼大嗎」——查程式碼裡真正驗證過的`REACH_CTRL`實測數值（伸手抓球姿勢：shoulder_pitch=-1.00rad/-57°、shoulder_roll=0.80rad/46°、elbow=0.90rad/52°）加上使用者估計「大概左前方45度擺到右前方45度」，重新算出**任務實際需要的span只有57°/91°/21°**——遠小於237°/179°/133°那組安全上限。對照 MeArm 機構~90-100°的真實可動範圍：shoulder_pitch、elbow 綽綽有餘，shoulder_roll的91°卡在邊緣（跟90°極限只差1度），但金額小（整組約NT$500-650），決定直接下單，之後實測不夠再局部調整。

#### 官方 MeArm IK 原始碼查證，確認運動學結構

去查了`ik.cpp`/`meArm.cpp`真正的`solve()`函式：**base角度只看(x,y)水平方向、獨立算出；shoulder跟elbow在base決定的平面裡一起解一個三角形**（elbow還要用到shoulder的解）。證實了「base是整條鏈最外層/獨立的水平旋轉，shoulder+elbow是巢狀求解」這個判斷，不是用嘴猜的。

官方慣用操作方式是給一個目標座標(x,y,z)，一次解出3顆角度（IK），不是個別下達每顆的角度指令。

#### 架構決策：直接關節映射，不做IK

使用者確認目標是「用感測器直接即時控制MEArm」（不是任意位置精確抓取）。決定**不實作IK**：即時訊號本來就已經是關節角度（不是座標），用IK還要先假設人手臂長度反推座標、再縮放到MEArm 18公分工作空間、才能丟進IK——多兩層近似，且IK是為了解決「精確到達指定座標」這個本專案沒有的需求。改用四個訊號各自線性縮放對應到四顆伺服機（`shoulder_pitch`→shoulder、`shoulder_roll`→base、`elbow`→elbow、`grip`→claw），沿用`run_demo_live.py`既有的「真實範圍→目標範圍」線性縮放邏輯，不是新概念。

#### 第一顆真實伺服機（SG92R）接線+韌體測試通過

新增`firmware/src/servo_pwm_test_main.c`（PA6/TIM3_CH1，50Hz PWM，1000-2000us來回掃描）跟`servo_limit_finder_main.c`（UART `+`/`-`互動微調，找真實機械極限）。TIM3選擇（非TIM2）刻意避開與既有1kHz ADC觸發TRGO設定衝突。PA6=TIM3_CH1的AF2編號查證：官方datasheet PDF兩次fetch都timeout，改用ST官方HAL巨集`GPIO_AF2_TIM3=0x02`+獨立pin-table來源交叉驗證，並在程式碼註解裡誠實標註「非直接查證RM0368/datasheet」。

實機測試：LED心跳正常、伺服機來回掃動確認通過。接著用`servo_limit_finder`實測**真實安全極限：往一端pulse_us=400開始磨齒、往另一端pulse_us=2550開始磨齒，中心點=1475**（不是教科書假設的1500——出廠電位計校正公差的真實反映）。中途一度出現「插電就有怪聲、完全不動」，排查後確認是伺服搖臂被外部東西卡住，不是馬達壞掉。安全使用範圍設為450~2500（各留50us margin，不貼著磨齒點操作）。

#### MEArm 已實際組裝完成，4 顆伺服機韌體＋校正工具（使用者不在硬體旁邊時完成，尚未實機驗證）

使用者組好MEArm、裝上伺服機，但當下不在硬體旁邊，所以接下來的韌體工作都只做到「編譯驗證通過」，沒有實機測試：

1. **`servo_pwm_4ch_test_main.c`**：TIM3全部4個channel（PA6/PA7/PB0/PB1）驅動4顆伺服機，依「由下到上」實體順序指派——CH1=base（最底）、CH2=shoulder、CH3=elbow、CH4=claw（最上/末端）。一次只動一顆（依序base→shoulder→elbow→claw），miswire時容易定位是哪一顆的問題。
2. **`servo_limit_finder_4ch_main.c`**：UART送`1`-`4`選台、`+`/`-`微調，一個session量完4顆真實極限，不用重燒4次韌體。
3. **`include/edgeneuro/control/servo_angle_map.hpp`**（新元件）＋`tests/test_servo_angle_map.cpp`（8個測試）：純線性縮放+夾限，把即時訊號值映射成pulse_us。寫測試過程中真的抓到一個bug——反向極性情境（數值越大、pulse該越小）下，`pulse_max_us - pulse_min_us`原本用unsigned算會下溢位成超大數字，改成先各自轉float再相減才修好。
4. **整合進`phase3_control_loop_main.cpp`**：加TIM3四通道PWM初始化，每個tick直接把`shoulder_filter.roll()/pitch()`、`elbow_bend`、`sp`(grip)透過4個`ServoAngleMap`寫進`TIM3->CCR1..4`。**4組校正常數目前全部是PLACEHOLDER**（`value_min/value_max`是真的人體範圍，但`pulse_us`端全部暫時借用唯一測過的那顆SG92R的450~2500，不是裝上MEArm之後每顆的真實極限），程式碼裡用醒目區塊註解標註，不能誤當作已校正。

全部15個韌體target（含新增的2個）一起編譯通過；host測試（含新的`ServoAngleMap`）在`debug-heapguard`跟`sanitize-asan-ubsan`兩個CI preset下都100%通過，確認無heap配置、無UB。

`README.md`新增「Phase 4: MEArm physical actuator bring-up」章節，記錄4個韌體target對照表、伺服機↔STM32腳位對照（含USB轉接板供電接法的完整檢查清單）。

#### 4 顆伺服機實機校正：channel 3/4（PB0/PB1）一度完全沒反應，根因是電源瞬間電流不足，不是接線/韌體

使用者回到硬體旁邊後，接線（含USB轉接板供電）完成，開始用`servo_limit_finder_4ch`逐顆測試。**channel 1（base/PA6）、channel 2（shoulder/PA7）正常，channel 3（elbow/PB0）、channel 4（claw/PB1）完全沒反應**，後來變成持續「答答答」抽動聲。

排查過程（依序排除，每一步都是真實量測，不是猜）：

1. 懷疑接線/韌體邏輯錯誤 → **直接用 OpenOCD/SWD halt 目標，讀取真實暫存器**（`RCC->AHB1ENR`、`GPIOB->MODER`、`GPIOB->AFR[0]`、`TIM3->CR1/CCMR2/CCER/CCR3/CCR4`）——全部欄位都正確設定（GPIOB時脈有開、PB0/PB1都是AF2、CCMR2的OC3M/OC4M都是PWM mode1、CC3E/CC4E都enable、CCR3/CCR4數值也隨案按鍵正確變化）。**確認STM32端韌體完全正確，問題在下游。**
2. 用已知正常的servo1交換測試（signal線移到PB0/PB1，servo1本身沒壞）→ channel 3/4 一樣沒反應，排除「claw/elbow那兩顆伺服機本身壞掉」。
3. 懷疑杜邦線/接觸不良 → 換線、量PB0/PB1腳位電壓確認訊號真的有變化（有）。
4. 出現持續「答答答」抽動聲 → 判斷為訊號時通時斷的典型症狀。
5. **意外發現：改用STM32板子自己的5V供電，伺服機轉得很順；改用USB轉接板（HW-767）的5V，動不了**。量測發現轉接板量到5.1V，STM32量到4.7V（**電壓比較高的反而不能動**，違反直覺）。
6. 檢查轉接板GND跟STM32 GND導通 → **確認共地沒問題**，排除接地問題。
7. **真正原因**：伺服機啟動瞬間會抽一口毫秒等級的尖峰電流，轉接板+充電頭這條供電路徑的瞬間反應/儲能能力不夠，電壓在那幾毫秒內塌陷到伺服機控制晶片判斷失敗的程度——但這個塌陷太快，一般三用電錶（以秒為單位更新畫面）完全量不出來，才會出現「量起來5.1V正常、實際卻動不了」這個看似矛盾的現象。STM32自己的5V之所以順，推測是開發板電路本身已經有類似的儲能/濾波設計。

**修法**：在轉接板輸出的+5V/GND之間、靠近伺服機那端，並聯加一顆電解電容（建議470-1000μF、耐壓10V+，正確辨識正負極——負極那側有色帶標示、腳也較短）。加上去之後**問題直接解決**，channel 3/4恢復正常。

這個排查過程也再次印證「raw data is the arbiter」——電錶量到的穩態電壓數字,完全可能掩蓋掉毫秒等級的真正問題,不能只看那個數字下結論。

#### 4 顆伺服機真實 pulse_us 範圍已實測並換入韌體；shoulder_roll 的邊緣疑慮解除

用`servo_limit_finder_4ch`量出4顆裝上MEArm機構後的真實安全範圍：base 500-2500、shoulder 1200-2100、elbow 500-1850、claw 1300-1600（全開~全合）。已取代`phase3_control_loop_main.cpp`裡的PLACEHOLDER常數（`kBasePulseMap`/`kShoulderPulseMap`/`kElbowPulseMap`/`kClawPulseMap`），編譯通過。

**channel↔關節對應**：使用者一開始用「左邊/右邊的伺服機」描述、不確定跟我的認知是否一致，查了真實 MeArm 組裝教學（DroneBot Workshop）+使用者自己實機動作確認（channel 2轉動時整條手臂一起抬高/放低），確認 **channel 2=shoulder、channel 3=elbow**，不是用猜的。

**意外的好消息**：base（對應`shoulder_roll`）量到「全部range」，500-2500共2000us，幾乎等同單顆裸測SG92R自己的完整安全範圍（2150us）——换算下來遠遠超過任務實際需要的91°span。**先前一直擔心的「shoulder_roll卡在90-100°機構上限邊緣」這個疑慮，實測結果解除**：官方MeArm校正資料裡base的~90°上限，看起來是刻意保守的soft limit，不是這個特定機構的真實硬體極限，這顆實測出來明顯比社群文件寬鬆。

**角度換算依據確認**：使用者確認手上這顆SG92R包裝標示額定180度（不是像MG996R那種「標示180、實測只有120」的灌水規格）。這代表換算時用的~11.1us/度（2000us/180度）這個比例，是有官方標示依據的，不是憑一般hobby伺服機的經驗假設隨便套的。回頭看base實測的2000us，幾乎剛好吃滿這顆伺服機額定180度的完整範圍——不是巧合湊出來的樂觀數字，是真的把整顆伺服機的額定能力都用上了，進一步坐實上面「shoulder_roll疑慮解除」這個結論的可信度。

**方向性（極性）推論**：去查了官方`phenoptix/meArm-1`真正的校正邏輯（`setup_servo`/`angle2pwm`），從`gain`的正負號反推出shoulder的方向關係——**脈寬越小、關節角度越大（手臂抬得越高）**，對照專案裡`shoulder_pitch`的定義（越負=越抬高），推論出目前韌體裡已經寫的`kShoulderPulseMap(-3.0892f, 1.0472f, 1200u, 2100u)`方向剛好是對的，不用改。這個推論基於官方原廠特定實體單位的校正資料，不是這顆jin-hua複製套件+SG92R的直接實測，信心程度高但不是100%確定。**base/elbow/claw三軸的方向性，沒有找到同等強度的文件依據**，目前維持原本placeholder版本的猜測方向，完全未驗證。

#### 下個 session 要接著做的事（TODO）

1. **接上即時IMU/EMG訊號、實測驗證全部4軸的方向性(極性)是否正確**——真人手臂舉起來，確認MEArm的shoulder真的跟著抬高（推論高信心，但沒實測過）；base左右擺、elbow彎曲、claw開合這三軸的方向完全沒有文件依據，必須實測確認，錯了的話直接對調該`ServoAngleMap`建構子裡的兩個pulse_us參數即可（不用動value_min/value_max）。
2. 4軸方向都確認無誤後，才算真正把即時控制迴路（階段B）接通、可信賴。

### Session Handoff (2026-09-24/25)：MeArm MuJoCo 數位分身、`--mearm` 預覽、互動式「姿勢錨點」校正

**目的**：MEArm 實體受電源/接線干擾，先在 MuJoCo 裡預覽即時訊號控制（不動到 `--humanoid` 既有路徑）。

#### MeArm 模型：直接採用社群現成的，不從零建
`tools/mujoco_bridge/mearm_scene.xml` 改自 `zc110747/MeArmPilot`（MIT，檔頭註明出處）。純 primitive geom、無外部 mesh；用 `tendon`+`equality` 鎖住爪子水平（不重建連桿剛體）。關節範圍是**他們那台實體**量的，不是本專案 jin-hua 套件的（尚無 pulse_us→弧度的驗證換算）。`view_mearm.py` 是最小檢視器（`mjpython -m mujoco.viewer` 在這台機器上會 `RuntimeError: Caught an unknown exception!`，改用專案慣用的 `launch_passive` 寫法就正常）。

#### 用「實測」抓到的真 bug（使用者的懷疑每次都是對的）
1. **桌子物理擋住手臂**：`MjData.contact` 直接讀到 `jaw_link_coll`↔`table_top`，shoulder 往前伸就卡住。桌子碰撞已關（`contype/conaffinity=0`，視覺保留）。
2. **模擬扭力上限太低**（上游 `forcerange=0.1765`）：對抗重力舉不到指令角度，已放寬到 2.0（本模型定位是預覽，不是扭力擬真）。
3. **`run_demo_live.py --skip-calibration` 的既有 bug（非本次引入，`git show HEAD` 確認）**：跳過互動校正後沒有任何等待，搶在第一筆真實 `shoulder_raw` 之前進主迴圈 → `oblique_decompose_scaled` 對 (0,0,0) 除以零。互動校正平常「順便」等了好幾秒才遮住它。已在既有 `main()` 補一段純加法的等待（`shoulder_basis is not None` 才等，避開 `--optional-sensors=shoulder`）。
4. **手肘方向**：一度依「末端離底座距離」判斷而接反，改成「內側夾角」後方向邏輯正確，但視覺上仍與使用者直覺相反——MeArm 上臂是立起來的、人的上臂是垂下來的，姿勢基準不同，**這是慣例問題，不是單純正負號**，最後交給校正流程用 y/n 讓眼睛決定。
5. **肩膀動不大不是使用者舉得不夠高**（我一度這樣誤判，已更正）：把人體整個理論 ROM（4.1 rad）等比例壓進 MeArm 約 1 rad 的範圍，真實舉 ~90° 只換來模型 ~14°。另外使用者那份紀錄顯示舉手被解讀成 `shoulder_roll` 大幅變化、`shoulder_pitch` 反而往正向走——**指向 9/13 校正檔已跟現在的感測器貼法對不上**（之後多次拆裝），需要重新校正。

#### 感測器→伺服機對應（查證自韌體/文件，不是印象）
- 上臂 IMU（0x68，程式叫 "shoulder"，貼手肘內側、晶片朝外、-X 朝手掌）：**單獨**決定 `shoulder_pitch`（→MeArm shoulder）與 `shoulder_roll`（→MeArm base）。
- 前臂 IMU（0x69，程式叫 "elbow"，貼脈搏處）：`elbow` = 兩顆重力向量夾角（dot product，無正負號）→ **兩顆一起**決定 MeArm elbow。

#### 新增
- `run_demo_live.py --mearm`（`run_mearm_preview()`）：簡化版預覽（跳過平滑/限速，重用 `oblique_decompose_scaled` 等既有函式以免兩份數學漂移）；需要既有校正檔；沒資料時每 5 秒印提示（板子燒了非 `phase3_control_loop` 的韌體時會靜靜卡住，已踩兩次）；每 ~0.2s 印即時數值與上臂原始向量對 HANG/FORWARD 的夾角（直接檢查貼法是否還對得上校正檔）。
- `apply_anchor_map()`（穿過實測錨點的折線映射，`rescale()` 旁邊，10 個性質測試）。
- **`calibrate_mearm_alignment.py`**：互動式校正——模型擺目標姿勢、使用者照做、錄下的解算值成為（感測器值→模型 ctrl）錨點；極性/範圍/中心偏移都由資料決定。順序 HANG→FORWARD→LEFT_TWIST（建基底）→RIGHT_TWIST→ELBOW_FLEX→夾爪 y/n→MID_RAISE（**不參與擬合**的驗證，PASS/WARN）。手肘方向、夾爪哪端閉合這兩個只有眼睛能判斷的慣例，看著模型回答 y/n。結果存進 `shoulder_calibration.json` 的 `mearm_alignment`（舊檔先備份；`baseline_raw` 等既有 key 也用新錄的更新，格式與 `--humanoid` 相容）。`--mearm` 有這個 key 就自動使用，沒有就退回舊縮放並提示。7 個離線流程測試（假感測器+mock input，含重試/放棄/備份/y-n 分支）；抓到並修掉一個真的 flaky（兩次獨立流程的錨點用 `assertEqual` 比浮點，樣本數隨時間變 → 差 1 ulp；改容許誤差，60 次連跑 0 失敗）。已加入 CI 清單。
- `servo_limit_finder_4ch` 改 115200（BRR=0x8B，沿用 `phase3_control_loop` 已查證值），兩支韌體共用同一個 baud。

#### ⚠️ 影響真實硬體的待辦（尚未處理）
`phase3_control_loop_main.cpp` 的 `kShoulderPulseMap`/`kBasePulseMap`/`kElbowPulseMap` 用的也是**同一種「整個理論 ROM 等比例縮放」**（value 端 -3.0892~1.0472 等），真實 MEArm 上會有同樣的動作壓縮。等姿勢錨點校正在模擬裡驗證有效之後，應該把錨點（人體值→pulse_us）帶回韌體，而不是繼續用理論 ROM。**先不動**：極性（尤其 base/elbow/claw）尚未實測，同時改多件事會讓判斷更亂。

#### 下個 session 要接著做的事（TODO，依序）
1. 用 `--mearm` 的 off-HANG/off-FORWARD 診斷確認上臂 IMU 貼法是否還對得上；不對就跑 `calibrate_mearm_alignment.py`。
2. 校正後用眼睛驗證：垂下/前舉/左右甩/手肘彎，模型是否與真人一致；MID_RAISE 是否 PASS。
3. 通過後，把錨點帶回韌體（見上方 ⚠️），再回頭做真實 MEArm 的方向性實測。
4. 目標物擺放（桌子/球，需對應 MEArm ~18cm 工作空間）仍是刻意延後的決定。
5. **【已完成第一階段，2026-09-25】Path B 離線預設映射 + 粗方向測試**（見下方「Path B 離線預設映射」段）。**待辦**：把互動式校正程式（`calibrate_mearm_alignment.py`）的解碼也從斜角改成球座標——目前它仍用 Path A 的斜角解碼建錨點，右甩會把 pitch 漏進 shoulder；預設映射（無校正時）已經不會，但校正後反而會回到有漏的版本。到硬體旁邊時：先看預設映射的粗方向對不對，再決定要不要跑互動校正。
6. **【待辦 C，優先於任何真實 MEArm 的 Path B 使用】在真機量出 (shoulder, elbow) 的可行組合區域**：目前每顆伺服機範圍都是「其他伺服機在中位」時單獨量的（`servo_limit_finder_4ch`），組合沒量過。做法：固定 shoulder 在 ≥4 個位置，各自量 elbow 的上下限（注意堵轉/答答響就停），得到真實的「shoulder+elbow 可行帶」，取代 `mearm_pathb.py` 的 `TOOL_LIMIT`（現在是 MeArmPilot 那台的）。**在量完之前，不要讓真實 MEArm 用獨立的 shoulder/elbow 指令**：`phase3_control_loop_main.cpp` 的 `kShoulderPulseMap`/`kElbowPulseMap` 目前各自獨立映射，沒有耦合保護，可能讓平行連桿卡死（伺服機堵轉、損壞）。量完後韌體也要套用同一個投影。

#### Path B 離線預設映射（2026-09-25，不接硬體，以 9/13 校正為主）

**背景回顧**：9/05 找出校正的物理缺陷（純水平揮動繞重力軸、加速度計看不到，所以加了拇指旋轉的 TWIST 姿勢，基底改用 HANG/FORWARD/LEFT_TWIST）；9/13 是完整六步驟抓球任務第一次在真實硬體上成功、校正檔（16:35）就是那天錄的——所以以 9/13 為主。9/05 只借一個相對量：手肘彎曲幅度 129°（`test_imu_to_mujoco.py` 內嵌的真實資料，單次錄製、前臂+上臂原始向量都有；該資料來自另一次穿戴，上臂 HANG 向量與 9/13 差 24.9°，**絕對向量不能跟 9/13 混用**）。

**目標（使用者明訂）**：回硬體前只求大方向正確——手臂向左、MeArm 向左；細部之後再用互動校正補。夾爪先不管（MyoWare 只有抓握、沒有軌跡）。

**離線分析發現（用 9/13 存的四個向量）**：
- Path A 的斜角 (pitch_equiv, roll_equiv) 解碼對 LEFT/RIGHT 不對稱且耦合：shoulder 佔行程 FORWARD=100%、**LEFT=0%、RIGHT=85%**（同樣約 70° 傾斜，卻讀成完全不同的肩膀高度）。
- 同一批向量改用球座標（傾斜角 tilt + 方位角 azimuth）：LEFT=+28.4°、RIGHT=−28.1°（幾乎完美對稱），三個姿勢傾斜角 68~73° 近乎不變。MeArm 的 base+shoulder 本來就是方位/仰角雲台，天然對應。
- 手肘假設（上臂不變；前臂感測器彎到底＝直臂讀數的重力軸反向）用真實資料驗證：預測彎曲幅度 121° vs 真實 129°（差 6%），上臂在彎曲期間漂移 6.4°。
- base(yaw) 本質是「手勢代理」而非量測：加速度計看不到繞重力軸的旋轉，只有拇指旋轉才讓它有訊號；±28° 的解碼方位角代表遠大於 28° 的實際揮動，增益由錨點提供。要更可靠需融合陀螺儀 z 軸或加磁力計（後者 9/9 才決定移除）。
- 手肘：官方 MeArm IK（`a2 = C + a1 − π`）顯示兩顆伺服機角度是**連桿絕對角度**；MeArmPilot 的 MJCF 是相對關節，模擬會掩蓋這個差異、真機不會（舉肩時 elbow 需補償）。**未在實體驗證。**

**新增**：
- `tools/mujoco_bridge/mearm_pathb.py`：純函式，不 import `run_demo_live`（避免循環，也保證碰不到 Path A）。球座標解碼 + 錨點：shoulder＝tilt、base＝azimuth（zero 為 FORWARD、左為正）、elbow＝`zero_elbow`→`zero_elbow+129°`。三個保護：傾斜 8~20° 之間 base 淡入（垂下時方位角無定義，雜訊不能讓 base 抖）；身體後方（|az| 90~135°）平滑淡回 REST（方位角在 ±180° 翻號，硬切換會讓真伺服機甩過整個行程）；無有效讀數 → REST、絕不猜。
- `--mearm` 沒有互動校正時，改用這套預設映射（取代會壓縮動作的「整個理論 ROM 縮放」備用方案）；即時印出 tilt/az。
- `test_mearm_direction.py`（23 個）：內嵌 9/13 向量，斷言性質不釘數字——左甩→模型 tcp y>0、右甩 y<0、正前方置中；舉手→仰角上升且單調；左右掃不連帶抬肩（門檻 15% 行程；Path A 解碼在同資料上是 0%/85%）；手肘彎→內角減小 ≥40°、彎曲幅度用**獨立的真實數值 129°** 檢查（第一版用模組自己的常數，突變測試發現幅度設成 20° 也會過→改掉）；手肘單獨動不影響 base/shoulder；垂下附近雜訊安靜；整圈繞行無跳變；後方＝REST；輸出恆有限且在致動器範圍內；常數對照 `mearm_scene.xml` 的 ctrlrange。**8 種故意破壞（方向反、拿掉保護、幅度過大/過小…）全部被抓到。**
- `test_run_mearm_preview.py`（7 個）：假序列埠+假視窗，真的跑 `run_mearm_preview()` 整條膠水（過去在真機上踩過好幾個膠水 bug）；把映射弄壞時 4/7 變紅。
- **`--humanoid` 路徑驗證**：`git diff HEAD` 對 `run_demo_live.py` **零被移除/修改的行**（純新增；其中唯一碰到 Path A 主流程的是先前經同意的「等資料再進主迴圈」修正）。這是當下手動驗證，**刻意不寫成永久測試**——那種測試會在有人合理微調 Path A 常數時莫名變紅。
- `calibrate_mearm_alignment.py` 改為不動 Path A 的任何欄位（只寫 `mearm_alignment`，自己的原始向量存在裡面）。

**真實即時 log 重播（2026-09-25）**：把使用者 9/24 那段真實 `--mearm` 終端機輸出（86 行 pitch/roll/elbow_ctrl，載入的就是 9/13 校正）反推回原始向量方向，再餵進 Path B。反推是精確的（Path A 解碼是保留方向的：傾斜角＝`hypot(pitch_equiv, roll_equiv)`，切平面方向∝`pitch_equiv·pf + roll_equiv·pa`；用 Path A 自己再解一次，最大誤差 1.8e-15）。結果：
- **使用者那次確實舉得夠高**（先前我說「沒舉夠」是錯的）：傾斜角 38°→118°（超過水平），方位角一直在 +13°~+42°（左前方），沒有往右過。
- **為什麼當時「左右可以、舉不起來」**：往左前方舉起被 Path A 斜角解碼幾乎全讀成 roll（往左），pitch 沒動——舉手被當成 base 在轉，shoulder 只動了行程的 28%。同一段真實動作，Path B 讓 shoulder 動 48%，傾斜角超過約 73° 就到最高仰角；base 則因方位角 +30~40° 超過校正的 +28° 而停在偏左。
- **這份資料沒驗證到的**：往右（log 裡沒有）、手肘彎曲（elbow_ctrl 只在 1.04~1.24，幾乎沒彎）。
- **一條還沒解決的線索**：log 最開頭幾行（尚未大幅動作）傾斜角就是 38°、方位角 +15°。如果當時手臂是垂下的，代表相對 9/13 的 HANG 校正有 ~38° 偏移（貼法沒變的話就不該這樣）；如果當時手不是垂下，就沒事。回硬體時用 `--mearm` 印出的 `tilt=` 在**確定手臂垂下**時直接檢查：應該接近 0。
- ~~MeArmPilot 模型的 shoulder 只涵蓋約 44° 的仰角（43°~87°）；本專案實測 shoulder 伺服機行程約 81°——模擬會比真機保守。~~ **（已更正，見文末「連桿耦合修正」段：那 44° 是舊模型 shoulder 被連桿約束卡住、沒轉到位，加上我的仰角指標在 90° 折返造成的假象；實際 shoulder 關節行程 59.5°，仰角約 38°→98°，越過鉛直。）**

已固定成測試：`test_mearm_direction.py` 的 `RealLiveLogReplayTest`（7 個，內嵌這 86 行；斷言性質不釘數字：舉手時 shoulder 不會反向、比 Path A 多用 ≥1.4 倍行程、base 全程在左、無單步猛衝；突變測試驗證會抓到「退回 Path A 行為／靈敏度變 1/3／左右反」）。

**MuJoCo 連續軌跡重播（2026-09-25）——「演算法輸出的指令」≠「模型實際做出的運動」**：先前的方向測試都是「每個姿勢跑到穩定再量」，看不到時間軸上的延遲或限制在跟伺服機打架；真實 log 的重播也只算到 ctrl 指令、沒有真的讓 MuJoCo 跑。這次把 86 行以真實節奏（每行 0.2 s）餵進去、一次連續模擬：
- **方向符合預期**：手臂傾斜角 vs 模型上臂仰角 Spearman **+0.97**；手臂方位角 vs 模型末端方位角 **+0.96**；末端 y 全程 >0（全程在左，0.064~0.135 m）；無碰撞。
- **舊映射在同一段真實動作上是反的**：仰角 vs 傾斜角 Spearman **−0.87**（手臂舉越高、模型反而越低，擺幅只有 10°），末端還跑到右邊（y 最小 −0.026 m）。這正是使用者說的「肩膀上下擺動沒什麼差別」。
- **但模型沒有真的達到指令的 shoulder 位置**（追蹤誤差 RMS **0.31 rad ≈ 18°**），實際仰角 56°~79°、指令應是 66°~87°。原因用四個變體查證（不是猜）：拿掉 tool 連桿限制 → RMS 0.013；伺服機 kp×10 → 反而 0.55（在跟限制硬碰硬）。所以**不是伺服機太弱，是連桿耦合限制**。
- **連桿耦合**：`mearm_scene.xml` 的 `tool = π/2 − shoulder − elbow`，且 tool 有限位（MeArmPilot 那台實體量到 −0.94~−0.29）→ (shoulder+elbow) 只能落在 **1.86~2.51**。Path B 把 shoulder、elbow 各自獨立映射，這段真實動作送出的和是 **0.91~1.40，每一個姿勢都違反**；模型的約束求解器把 tool 釘在限位上。「手臂伸直舉高」在這個機構上本來就做不到（要舉高 shoulder，forearm 必須跟著折）。
- **自我檢討**：先前的方向測試量到 FORWARD 姿勢上臂仰角是 74°（指令對應約 87°），我當時沒發現這個落差，因為測試門檻是粗的（≥30°）。
- **對真機的意義（未驗證）**：如果本專案這台 MEArm 有同樣的平行連桿耦合，獨立下達 shoulder/elbow 指令會讓連桿卡死（伺服機堵轉、答答響、可能損壞），不只是模擬追不上。目前每顆伺服機的範圍都是「其他伺服機在中位」時單獨量的（`servo_limit_finder_4ch`），組合可行區域沒量過。

已固定成測試（`test_mearm_direction.py` 的 `RealLogTrajectoryInMuJoCoTest`，8 個）：6 個通過（上述方向、Path B vs 舊映射）＋ **2 個 `expectedFailure` 當成可執行的已知限制文件**（指令的 shoulder+elbow 落在連桿允許區間內／模型達到指令的 shoulder 位置）。已確認這 2 個是因為預期的原因失敗（和 1.39 < 1.86、RMS 0.31 > 0.1），不是被 `expectedFailure` 藏起來的程式錯誤；修好耦合後會變成 unexpected success 而報紅，那就是刪掉裝飾器的信號。突變驗證：把 Path B 換回舊映射，8 個裡 5 個變紅。同時抓到並改正了自己寫死的錯誤參考值（排名相關函式的測試，心算成 0.9487，實際 8/√80=0.8944——函式是對的，是我的參考值錯）。

**待決定（使用者）**：(A) 讓 Path B 尊重耦合——依 shoulder 把 elbow 投影進可行區間（模擬與真機都安全，但 elbow 不再完全等於人的手肘）；(B) 預覽時放寬 tool 限位（追蹤變好，但隱藏真實限制）；(C) 先在真機量出 (shoulder, elbow) 可行區域再決定。我傾向 C 為 A 的前置：模擬那個 1.86~2.51 是別人那台量的。

#### 連桿耦合修正（方案 A，2026-09-25）與一個更正

**A 實作**：`mearm_pathb.py` 新增 `linkage_band()`/`elbow_window()`/`elbow_request()`/`project_elbow()`。shoulder 優先，elbow 依 shoulder 按比例放進連桿允許的窗口（不是硬夾：硬夾會在低 shoulder 時半彎就飽和；突變測試證實），並留 0.05 rad 邊界讓 tool 連桿不會剛好卡在限位。同一段真實 log 的前後對比：

| | A 之前 | A 之後 |
|---|---|---|
| shoulder 追蹤誤差 RMS | 0.314 rad | **0.012 rad** |
| 指令 shoulder+elbow 之和（允許 1.91~2.46） | 0.91~1.40（全違反） | 1.92~1.97 |
| 模型上臂仰角擺幅 | 23°（56°→79°） | **33°（65°→98°）** |
| 傾斜角→仰角 Spearman（未飽和區） | — | +0.99 |
| 方位角→末端方位 Spearman／末端 y | +0.96／全程在左 | +0.96／全程在左 |

**取捨（機構的事實，不是調參）**：窗口寬度是常數 0.553 rad ≈ 32°，所以 shoulder 固定時 elbow 最多只能動這麼多——真人 129° 的手肘彎曲只會顯示成小幅度、方向正確的折疊；伸直手臂舉高時 elbow 被迫折起來（forearm 必須跟著折）。

**更正（我先前的錯）**：(1) 上面 log 裡「shoulder 只涵蓋 44°」是假象。舊模型 shoulder 沒轉到指令位置（被 tool 連桿卡住），加上我的仰角指標 `atan2(z, hypot(x,y))` 上限 90°，上臂一過鉛直（真實 98°）就折返成 82°，看起來像往下——查證時 q_shoulder 其實完全跟著指令走。改成相對 base 朝向、有正負號的仰角。(2) 這個指標錯誤是修 A 時測試變紅才發現，我一度以為是 A 讓行為變差；先查原因、沒有去調門檻，才分清楚是指標問題。(3) 傾斜角→仰角的整體 Spearman 從 0.97 掉到 0.86 不是退步：86 行 log 有 45 行超過校正的 FORWARD（73°），那裡 shoulder 依設計飽和、仰角固定 97~98°，全是並列值；未飽和區是 +0.99。測試因此拆成兩個性質（未飽和區高度相關／飽和區留在頂端不回落），而不是用一個整體相關係數貼著門檻。

**測試**：`test_mearm_direction.py` 47 個（原 2 個 `expectedFailure` 已轉為正式測試並通過；新增 `LinkageCouplingTest`：任意輸入下和恆在可行帶內、邊界嚴格在場景允許範圍內、窗口任何 shoulder 高度都不為空、伸直舉高→forearm 折起的方向、舉手時 elbow 平滑變化、手肘彎曲在任何高度方向一致；限位常數對照 XML）。6 種故意破壞（拿掉投影／邊界設 0／投影方向反／硬夾／限位抄錯／耦合方向反）全部被抓到。`--humanoid` 路徑本輪沒有任何改動。

#### TDD 補強一輪（2026-09-25，全部不需硬體）

**動機**：使用者要求先驗證再談 PR，並以測試驅動的方式穩定品質。這一輪每項都是先寫測試、看它紅、再實作、再故意破壞確認測試會抓。

**找到並修掉的真問題**
1. **`ServoAngleMap::pulse_us(NaN)` 是未定義行為**（`static_cast<unsigned>(NaN)`）：晶片上可能變成 0 或 0xFFFFFFFF，伺服機撞止點。現在 NaN → 範圍中點（中立）。屬於韌體的實際行為修正，phase3 已重新編譯通過。
2. **互動校正的 `mearm_alignment` 分支繞過連桿投影**：一旦跑完校正工具，elbow 就獨立映射，舉高＋彎手肘時 shoulder+elbow 之和可達 3.52（允許 ≤2.46）——等於在校正完成的那一刻，剛做好的耦合保護被關掉。分支現在和預設路徑走同一個 `mearm_pathb.ctrl_from_sensors`。
3. **校正工具仍用 Path A 的斜基底解碼**（會把扭轉漏進 pitch）。已改成 Path B 的球面解碼：工具只存 4 個原始向量（tilt/azimuth 錨點由它們導出）、量到的肘部錨點、夾爪兩端；`mearm_alignment` 新增 `"decode": "spherical"` 與 `elbow_anchors`，舊格式（無此欄位）在 `--mearm` 會被拒絕並提示重跑工具。RIGHT_TWIST 的驗證改為「建得起 Calibration」（FORWARD 距 HANG ≥20°、左右在 FORWARD 兩側）。
4. **Python `project_elbow` 遇 NaN/∞ 會回傳 NaN**，會污染整個 MuJoCo 狀態；已與 C++ 版對齊（肩膀夾進範圍、NaN 肩→rest、NaN 請求→伸直）。
5. **`apply_anchor_map` 現在在 `--mearm` 路徑不再被使用**（仍被 `ApplyAnchorMapTest` 測試；`--humanoid` 路徑未動）。

**新增的測試／檔案**
- `tests/test_mearm_servo_maps.cpp` + `include/edgeneuro/control/mearm_servo_maps.hpp`：4 個實體伺服機映射的安全性質（任意有限輸入/±∞/NaN 都在實測範圍內、單調、兩端可達）；phase3 改用這個標頭。
- `tools/mujoco_bridge/test_constants_consistency.py`（7 個）：XML ↔ `run_demo_live` ↔ `mearm_pathb` ↔ 韌體映射的數值一致，以及 CCR1~4 各接對的映射。
- `tests/test_mearm_linkage.cpp` + `include/edgeneuro/control/mearm_linkage.hpp`（`project_elbow` 的 C++ 版，**尚未接進 phase3**，因為限位還沒量，待辦 C）+ `data/linkage_golden.csv`（由 `gen_linkage_golden.py` 從 Python 產生，132 列）+ `test_linkage_golden.py`（表過期就失敗）。
- `test_run_mearm_preview.py`／`test_calibrate_mearm_alignment.py`／`test_mearm_direction.py` 補了：校正分支的連桿帶、預設與校正路徑的一致性、量到的肘部極性與夾爪端點確實被使用、扭轉不漏進 shoulder、NaN。

**突變測試發現的測試弱點（已補）**：預覽忽略量到的 elbow 錨點／夾爪端點，原測試抓不到（fixture 數值恰好和預設相同）；改用會「不同於預設」的 fixture 後兩者都被抓到。
**抓不到的等價突變（記錄）**：C++ `window_lo` 的 `max(kElbowExtended, …)` 夾限在現有帶寬與有效肩膀範圍下不會被啟動（`band_lo − shoulder ≥ 1.010 > 0.9946`）；待辦 C 量出真實限位後要重新檢查。

**釐清**：`test_arm_kinematics.py` 沒放進 CI 是對的——它是需要 mjpython + 視窗的互動式列印工具，沒有斷言。

**仍需硬體才能驗證**：base/elbow/claw 實際極性、shoulder 極性確認、真機可行組合區域（待辦 C）、電源與堵轉。

**第 6 項：`--mearm` 預覽的壞輸入／斷線（2026-09-25，同一輪 TDD）** — `RobustnessToBadInputTest`（用腳本化假序列埠）：
- 亂碼／截斷行／開機字串穿插：被忽略，模型仍跟隨有效行（原本就正常）。
- **非有限數（`1e999` 會被 regex 接受並解析成 inf）**：原本 `interp_anchors` 在 NaN 上炸 `UnboundLocalError`。`ctrl_from_sensors` 現在把非有限的上臂向量視為無效讀值（→ rest），非有限的肘讀值視為伸直；`interp_anchors` 對 NaN 明確丟 `ValueError`。
- 上臂向量中途變全零（如 I2C 匯流排恢復）：模型回到 rest，恢復後重新跟隨。
- **拔掉 USB 轉板：原本預覽會靜默凍結。** 現在和 `--humanoid` 一樣 `serial port failed: …` 退出。
- **韌體靜默：** 現在印一次 `[STALE]`，恢復後印一次 `[OK]`（先前沒有，模型會無說明地停住）。
- 等待上臂原始向量的迴圈原本沒有提示（tick 行有來但 shoulder_raw 缺/為零時會靜默卡住），現在每 10 秒提示檢查 MPU6050 0x68。**這一項沒有自動測試**（要等 5 秒真實時間；只在實機上會遇到）。
- 突變測試：拿掉 raw 有限檢查／肘有限檢查／埠檢查／STALE 提示，各自被抓到。
- 測試寫錯一次：Python 會把 inf 格式化成 `+inf`，regex 不收，造成測試「卡住」而非測到 bug；改成手寫 `1e999` 後才真正重現。

#### 全面 review 與韌體伺服輸出預設關閉（2026-09-26）

**review 結論（未修的先記錄）**
1. **【已修】燒 `phase3_control_loop` 就會驅動實體伺服機**（`--mearm` 預覽必須燒它），且獨立驅動肩/肘、無連桿耦合保護。現在需明確 `-DEDGENEURO_DRIVE_SERVOS=ON` 才會編入；預設不碰 TIM3 與 PA6/PA7/PB0/PB1。驗證：反組譯，預設 build 對 TIM3 基底 `0x40000400` 引用 0 次、ON build 2 次（CI 步驟以此檢查，且要求 ON 版必須有引用，避免空過）；兩種設定皆無編譯警告。**已燒過舊版的板子仍會驅動伺服機，需重燒＋斷電重啟。**
2. **【未修】韌體伺服映射與 Path B 是兩套東西。** 韌體手肘：`kElbowPulseMap(-1.0472, 1.28, 500, 1850)` 直接吃原始未帶號夾角（伸直約 0.43 rad → 約 1359 µs，愈彎脈寬愈大）；Python 的手肘值是 `ELBOW_OFFSET − (bend − zero_elbow)`（愈彎愈小、有校正零點）。方向相反且無零點。肩/底座用韌體自己的 `filter.pitch()/roll()`（先前記錄的軸間洩漏問題），而非校正後的球面解碼。→ 目前所有 Path B 測試只證明模擬端；沒有任何測試證明韌體與其一致。
3. **【未做，最大測試缺口】** 把 `Calibration.decode`＋淡出＋連桿投影移植成 C++，用 9/24 的 86 列真實 log 產生對照表逐點比對（同 `project_elbow` 的作法）。做完 2. 的方向問題會一併解決，之後才考慮打開 `EDGENEURO_DRIVE_SERVOS`。
4. 其他缺口：`ServoAngleMap` 退化範圍（`value_max <= value_min` 會除以零）無測試；校正工具的 `main()`／擷取逾時／中途拔線／使用者中止未測；`servo_limit_finder_4ch` 脈寬上限 2700 µs 超過實測裸伺服機極限 2550 µs（本意是找極限，但上限沒貼實測值）；`apply_anchor_map` 在 `--mearm` 路徑已無呼叫者。

#### Path B 的 C++ 移植與真實 log 對照（2026-09-26，review 第 3.1 項）

- `include/edgeneuro/control/mearm_pathb.hpp`：`Calibration::make`（4 個原始向量＋直臂手肘零點，無效即回 false）、`decode`（球面 tilt/azimuth）、`ctrl_from_sensors`（淡出＋連桿投影，重用 `mearm_linkage.hpp`）。輸出是 **MuJoCo 模型 ctrl 弧度，不是脈寬**；還沒接進 `phase3_control_loop`；也還沒有「模型 ctrl → 真實脈寬」的映射（極性與限位要實機量，待辦 C）。
- 對照表 `data/pathb_calibration.csv`、`data/pathb_golden.csv`（550 列）由 `gen_pathb_golden.py` 從 Python 產生：9/13 校正、9/24 **真實 log 的 86 列各配 5 種手肘讀值**、特殊姿勢（垂下/前舉/左右/倒置/半舉/極點附近/身後淡出區），以及 tilt 落在極點淡入區（8~20°）的多種方位角。`test_pathb_golden.py` 在表過期時失敗（已進 CI）。C++ 對 Python 誤差門檻 2e-3。
- 測試（`tests/test_mearm_pathb.cpp`）：逐列對拍；任意（含 NaN/∞/零向量）輸入下輸出有限、在致動器範圍內、肩+肘在連桿帶內；無效讀值 → rest；校正姿勢解到校正方向；`make` 拒絕不可用校正。
- **測試寫錯一次**：我原本預期「LEFT/RIGHT 對調」要被拒絕；但 +azimuth 是由傳入的 LEFT 定義的，對調只是鏡射校正，Python 行為相同，已改成測試它「被接受」並註明原因。
- 突變測試 7 種（底座擺幅、極點淡入窗、方位軸對調、身後淡出窗、肘擺幅、肩膀漏掉 front 淡出、拿掉連桿投影）全被抓到。**其中「極點淡入窗改 20°→30°」第一次沒被抓到**——原對照表沒有「tilt 在 8~30° 且方位角≠0」的列（方位角為 0 時底座恆為 0，看不出淡入）；補列後才被抓到。
- 檢查：debug-heapguard 100/100、ASan+UBSan 95/95、以韌體實際使用的工具鏈（`~/.local/arm-toolchain`，GCC 15.2）`-Wall -Wextra -Wdouble-promotion` 編譯此標頭無警告。（PATH 上 Homebrew 的 `arm-none-eabi-g++` 沒有 C++ 標準庫，別拿它檢查。）
- **接下來要接進韌體前需要決定的設計問題**：韌體要怎麼拿到校正資料（4 個向量＋零點）？選項：(a) 編譯期常數（最簡單，但每次重校正要重燒）；(b) 電腦端經 UART 下發；(c) 存進 Flash。目前韌體完全沒有校正。

#### 刪除壓力測試韌體、補主機端測試（2026-09-26）

- **刪除 `complementary_filter_stress_test`**（`git rm`，歷史在 commit `8f5472b`）：LCD 已拆、MPU6050 已可用，其前提（沒有可用 IMU 時用合成資料＋LCD 當匯流排負載驗證迴圈穩定）不成立。CMake 目標、firmware/README 一行已移除，三處註解加註「已移除」；`check_hardware_ready.py` 的提示改成「換一個已知正常的 I2C 裝置」。PRD 內的歷史紀錄保留。
- **其他舊韌體/工具逐項檢查後全部保留**（使用者原則：有參考價值的不刪）：`uart_hello`/`adc_hello`/`timer_adc_1khz`/`i2c_mpu6050_hello` 各含 7~11 處 RM0368 引用，是暫存器驗證的唯一來源；`servo_pwm_test` 記錄實測 400–2550 µs／中心 1475 µs；`run_demo_live_grip_only.py` 仍是 README 記載的隔離診斷工具；`apply_anchor_map` 仍被測試使用且在人形檔案內。
- **`ServoAngleMap` 退化設定（真缺口，已修）**：`value_max <= value_min`、非有限邊界、或 `max−min` 溢位成 inf 時，會算出 NaN 再走到 `static_cast<unsigned>(NaN)`（未定義行為）。之前只擋了「輸入值是 NaN」。現在一律回傳脈寬範圍中點（中立）。5 個新測試；三種突變（拿掉空/反向範圍檢查、拿掉非有限跨度檢查、拿掉 NaN 輸入檢查）皆被抓到。
- **`ComplementaryFilter` 邊界測試（取代壓力測試的職責）**：零加速度向量保持有限；100 萬圈慢速合成傾斜有限且有界；MPU6050 滿量程角速度（±2000°/s＝34.9 rad/s）連續 100 萬圈有界（皆**本來就通過**，沒有發現數學問題）；**NaN/±Inf 樣本原本會永久污染遞迴狀態**（直到 `reset()`）。現在遇到非有限的 gyro/accel/dt 就略過該筆並保持狀態，下一筆正常繼續。這在目前韌體不會發生（輸入是 I2C 整數換算成的浮點數），純屬共用函式庫的低成本防護。突變（拿掉 dt 檢查、拿掉 accel_z 檢查）皆被抓到。
- 檢查：debug-heapguard 110/110、ASan+UBSan 105/105、韌體全部重新編譯無警告（phase3 text 6704 bytes）。

#### 剩下的小測試缺口補齊（2026-09-26）

- **真問題：校正錄製途中拔線／斷訊會被當成有效資料存起來。** `capture_window` 之前沒看連線狀態；死掉的連線會一直回傳「最後一筆樣本」，看起來完全合理，平均後就把垃圾寫進校正檔且無任何警告。現在：`port_error` → `RuntimeError("serial port failed …")`（流程中止、不寫檔、不建備份）；資料中斷（stale）→ 該次錄製作廢並重試（`CaptureInterrupted`，計入 `MAX_RETRIES`）。
- **校正工具 `main()` 迴圈**：拔線後 live-follow 模式下模型只會靜默凍結；現在每 ~20 步檢查一次，`sys.exit("serial port failed …")`。同時測了：中止旗標結束迴圈並印出訊息、目標姿勢與 live-follow 的切換。
- **等待原始向量的提示**：校正工具與 `--mearm` 預覽各有一個，現在是模組常數（`WAIT_HINT_SECONDS`／`MEARM_WAIT_HINT_SECONDS`，測試把它縮短），有自動測試。**校正工具原本在這段等待完全沒有提示**（只有第一段 `is_ready` 有），已補上。
- `view_mearm.py`：新增 `test_view_mearm.py`（場景檔存在、載入四個致動器、真的有推進模擬）。
- 產生對照表的兩支腳本：抽出 `write()`，測試寫到暫存目錄並驗證內容與 `render()` 一致；命令列入口實際執行後對照表內容不變。
- 測試自己的錯：`UnplugPose` 繼承 `StalePose`，`isinstance` 先判斷父類別導致拔線被當成斷訊，修正判斷順序。
- 突變測試：view_mearm 不再推進模擬、生成器不建目錄／不寫檔、兩處提示文字被改，皆被抓到。

#### MEArm 起始點與開機緩起動（2026-09-26）

**背景與更正**：我先前推測「斷電後手臂會自己垂到最低位置」是沒有依據的。查證結果：Tower Pro 官方頁面沒有寫斷電後的行為；原廠 datasheet PDF 是圖片格式，讀不出文字；社群對「斷電後能不能被外力轉動」說法互相矛盾（Arduino 論壇一說很難反向驅動、一說用手就能轉）。共通點只有一個：**SG92R 沒有斷電回位，斷電在哪就在哪**（使用者原本就是這麼說的）。重力會不會壓下手臂取決於這台機構的摩擦，只能實測（待做：抬起姿勢斷電、靜置數分鐘看是否下沉）。

**設計結論**
- 開環伺服機沒有回授，STM32 不知道手臂現在在哪；通電後**第一個**脈衝會讓伺服機從原位以自己的最大速度衝過去，這一步韌體**限不了速**。能控制的是第一個脈衝**之後**：不要從休息位直接跳到感測器指的位置。
- **不做關機歸位**（park）：依賴使用習慣、不穩定（使用者決定）。
- 休息位置**暫時假設四個通道都是 1500 µs**（皆落在各自實測範圍內：base 500–2500、shoulder 1200–2100、elbow 500–1850、claw 1300–1600）。**這是佔位值，不是量測值。** 原本韌體開機用的是 1475（裸伺服機量到的機械中心），已改成同一個 1500。

**實作（皆在 `EDGENEURO_DRIVE_SERVOS` 旗標後，預設仍關閉）**
- `include/edgeneuro/control/servo_startup_ramp.hpp`：`ServoStartupRamp`，起始在休息脈寬，以慢速（佔位 300 µs/s）走向目標；抵達（差 ≤2 µs）後**鎖定**為快速追蹤（佔位 6000 µs/s，約 SG92R 自身速度），不會退回慢速。內部位置用 float（1 kHz 下慢速每 tick 不到 1 µs，若每 tick 四捨五入到整數會完全走不動）。NaN 目標／無效 dt → 保持目前脈寬；輸出恆在伺服機範圍內。
- `mearm_servo_maps.hpp`：實測範圍抽成具名常數，新增四個 `*_ramp()`；休息脈寬與兩個速率也在這裡（三個都標為佔位）。
- `phase3_control_loop_main.cpp`：四個通道各經一個 ramp 再寫入 `TIM3->CCRn`。**時間步長是 10 個 tick（0.01 s），不是 1 個**——這段程式每 10 個 tick 才跑一次（100 Hz），我原本的註解寫成「每個 tick」是錯的，已更正；若用 1 個 tick 的步長，慢走會快 10 倍。
- 測試：`tests/test_servo_startup_ramp.cpp`（10 個性質測試）與 `test_mearm_servo_maps.cpp`（起點 1500 且在範圍內、任何目標都不出範圍、從休息 1 秒內走不到遠端）；`test_constants_consistency.py` 改為檢查「每個通道由自己的 ramp 餵自己的 map、步長對應該區塊的執行頻率」。
- 突變測試：ramp 8 種（無視慢速／無視快速／狀態四捨五入／抵達不鎖定／NaN 目標／無效 dt／休息與目標不夾限）＋韌體接線 4 種（步長 1 tick、通道 2 接錯 ramp、claw ramp 取自 base、區塊頻率被改）皆被抓到。
- 我自己的一個小瑕疵（測試抓到）：抵達判斷原本在移動前，導致「抵達的那一步」用了快速率，可比慢速率多走最多 2 µs；改成先以本步的速率移動、再判斷抵達。

**驗證**：debug-heapguard 124/124、ASan+UBSan 119/119、預設（旗標關）韌體與先前逐位元相同大小（6704 bytes）且對 TIM3 基底位址 0 次引用；旗標開的版本無警告編譯。

**待硬體**：量真正的安全休息姿勢（取代 1500 佔位）、調兩個速率、實測斷電後手臂是否下沉（決定意外斷電的風險）。肩膀與手肘在慢走途中各自獨立移動，可能經過連桿不可行的中間組合（待辦 C）。

#### 休息姿勢定案與夾爪起始點（2026-09-26，在硬體旁邊）

- 用 `servo_limit_finder_4ch` 讓四個通道都停在 1500 µs（韌體讀回確認），使用者拍照：手臂呈「往前伸、上臂約 50~60° 抬起、前臂往下折、夾爪垂到底板附近」的中段姿勢，四顆都在行程中間、沒有頂到極限。使用者認為可以當起始點；我看照片同意，**但這是「看起來合理」，不是量測出來的安全姿勢**，也沒有和模擬的 `REST_CTRL`（肩膀最低仰角、手肘伸直）建立對應——照片估的仰角不夠精確，不能拿來校準。
- **夾爪起始點改成 1300 µs（張開）**（使用者決定）：`kClawRestUs = 1300`，其餘三個 1500，每個通道各自一個常數（`mearm_servo_maps.hpp`），韌體開機的第一個脈衝也各用自己的。理由之一是 `grip = 0`（手放鬆）本來就映射到 1300，開機時夾爪不需要動（有測試：`claw_ramp().current_us() == claw_map().pulse_us(0.0f)`）。
- **風險（待硬體觀察）**：1300 是夾爪實測行程的**端點**，長時間把伺服機頂在機械止點可能嗡嗡響、發熱、磨損齒輪。若聽到持續嗡嗡聲，把夾爪起始點拉進來幾十 µs（例如 1330~1350）。
- **待觀察**：照片中夾爪尖端已貼近底板；肩膀往下調時要小心夾爪撞到底板或桌面。
- 新增小工具 `tools/servo_pose_4ch.py`（15 個測試）：用 `set shoulder 1650`、`all 1500`、`show` 調姿勢，不超出實測範圍、一次 25 µs。
- 檢查：C++ 125/125（debug-heapguard）、120/120（ASan+UBSan）；旗標關閉的韌體 6704 bytes 且 TIM3 引用 0 次（與先前完全相同）、旗標開啟版本無警告；新增一致性測試（每個通道的第一個脈衝用自己的休息常數），突變（claw 取自 base）被抓到。

#### 待辦 C 開始：量測肩膀×手肘可行區域的工具（2026-09-26）

- 使用者確認夾爪停在 1300（張開）不會嗡嗡響。
- 新增 `tools/measure_linkage_region.py`（20 個測試，`test_measure_linkage_region.py`，已進 CI）。**只有人能判斷連桿是否卡住**（耳朵：嗡嗡／吃力；眼睛：連桿停住或彎曲、夾爪碰桌），所以工具一次只動一顆伺服機、每 ~0.8 秒走一步 25 µs，**人在第一次發現異常時按 Enter**，工具立刻退回 3 步並記錄當下脈寬。永遠不超出各通道已量的範圍。
- 流程：對每個肩膀位置（預設 1500→1650→1800→2100→1350→1200，由休息位往外走）：(1) 手肘先回 1500；(2) 肩膀走到位置（若自己就卡住則記錄並跳過）；(3) 手肘往上掃到卡住或量到的極限；(4) 往下掃；每次都回到 1500。每個肩膀位置後就存檔，可隨時 Ctrl+C。
- 原始結果存 `data/mearm_linkage_measurements.json`（只記錄按下 Enter 時的脈寬與步長／等待／退回步數；不在存檔時做任何推論）。分析（`windows`、`fit_line`）把窗口從記錄的停止點向內縮 3 步（人的反應延遲＋餘裕），從有真正卡住的點擬合 `手肘上/下緣 = a + b×肩膀`，並回報最大殘差；少於 3 個點不擬合。
- 測試用「假板子＋模擬的人（有反應延遲）」：反應延遲內找到邊界、絕不比反應時間允許的更深入卡住區、結束時回到可行姿勢、每個肩膀位置後即存檔、肩膀自己卡住時記錄並回到最後良好位置、只在量過的範圍內。突變測試 7 種（拿掉退回／退回可超過起點／手肘不先回休息／不夾範圍／邊界向外縮／不即時存檔／自卡後不復位）全部抓到；**其中兩種第一次沒被抓到**（手肘先回休息、自卡後復位），原因是假板子的初始條件讓它們碰巧成立，已改成從手肘不在休息位開始、並檢查下一次肩膀走動的起點。
- **尚未在真機執行**（需要人按 Enter）。量完之後：把窗口換算成模型端的 `TOOL_LIMIT` 與 `mearm_linkage.hpp` 常數（脈寬↔角度的換算也要量或推導），並讓韌體套用同一個投影。

#### 待辦 C 第一次真機量測（2026-09-26 23:43）與工具修正

**第一次結果**（原始檔另存為 `data/mearm_linkage_measurements_run1_2026-09-26.json`）：肩膀 1500 → 手肘窗口約 550~1675；1650 → 700~1725；1800 → 800~1750；2100 → 往下 725、往上「1450」；1350 與 1200 被記為「肩膀自己卡住」（約 1800／1250）。

**這份資料不能直接採用**，有三處自相矛盾：
1. 肩膀 2100 手肘往上掃「在 1450 停下」，但起點應是 1500——往上掃不可能停在比起點小的數字，代表那次掃描開始時手肘其實不在 1500。
2. 肩膀 1350「自己卡住約 1800」，但肩膀 1800（手肘 1500）稍早已正常量完，且下一個位置 1200 從同一起點走下來一直走到 1250 才停 → 1800 那次很可能是誤按 Enter。1250 則合理（肩膀本身下限附近）。
3. 開場夾爪在 1500，不是休息位 1300（`servo_limit_finder_4ch` 開機四通道都是 1475，工具原本不碰夾爪）。

**根因（工具的問題，不是操作問題）**：
- 「回到 1500」等移動用的是 `never`（永遠回 False 的停止函式），**每步之間沒有等待**，一次連續送出數十個 `+`/`-`，等於瞬間跳幾百 µs，違反工具宣稱的「一次只走一步」；序列線送太快時韌體（單一位元組接收暫存器）可能漏收，手肘就沒真的回到 1500，而工具沒有讀回驗證，下一次掃描便從錯的起點開始（第 1 點）。
- 回位期間按下的 Enter 會排隊，被下一次可中止的掃描當成「異常」（第 2 點的可能原因）。

**修正（先寫測試、看紅燈）**：所有步驟（含回位與退回）每步都有等待（0.15 s）；每次走完讀回實際脈寬、漏步就補、補不回來就 `WalkFailed` 並說明；停止位置取讀回值而不是自己計數；每次可中止的掃描前清掉排隊的 Enter；每個肩膀位置做完顯示記錄並問「OK 嗎？（r＝重測）」，誤按可立即重來（最多 3 次，之後不記錄該位置）；即時顯示目前脈寬；開始前先把夾爪慢慢走到 1300 並在整個量測期間不再選取它；記錄每次掃描的實際起點（稽核用）。37 個測試；13 種突變，其中 3 種第一次沒抓到（退回步數漏掉的修正、停止位置取自計數 — 都是測試的假板子沒有在那個位置製造遺失；已補測試），1 種是等價突變（手肘起點欄位：現在每次回位都已驗證，該欄位恆為 1500，僅作稽核）。

**第一次資料能用來判斷的（僅供參考，不作結論）**：肩膀 1500 時手肘窗口寬約 1100 µs（約 100°），遠寬於模擬採用的 MeArmPilot 帶（約 32°）——若重測仍如此，模擬的耦合假設對這台實機過於保守；窗口寬度隨肩膀變化（1125→1025→950），不是固定寬度的「和帶」。等乾淨的重測資料再決定。

**`servo_limit_finder_4ch` 開機起始點改成休息姿勢（2026-09-26）**：使用者問「為什麼起始點不改成 all 1500（夾爪 1300）」——我只改了 `phase3_control_loop`（旗標後面），量測與姿勢工具實際跑的 `servo_limit_finder_4ch` 開機還是舊的四通道共用 1475，所以每次量測都從不同於手臂選定起點的姿勢開始（第一次量測夾爪就是 1500）。現在該韌體用 `REST_PULSE_US[4] = {1500, 1500, 1500, 1300}`（通道 1=底座、2=肩膀、3=手肘、4=夾爪）作為第一個脈衝與內部起始狀態；`test_constants_consistency.py` 新增 4 個測試讓它與 `mearm_servo_maps.hpp` 的 `k*RestUs` 保持一致（含每個休息值都在該通道實測範圍內、不再出現舊的 1475 共用常數），突變（夾爪取自底座）被抓到。`servo_pwm_4ch_test`（一次動一個通道的接線測試）維持 1475，因為它是接線確認、不是姿勢起點。開機的第一個脈衝仍會讓伺服機從原位全速轉到休息位——通電前清空手臂周圍。

#### 待辦 C 第二次真機量測（2026-09-27 00:22，修正後的工具）

- 使用者決定拿掉肩膀 **2100** 與 **1200** 兩個位置（機構在極端位置吃力）：腳本預設位置改為 `(1500, 1650, 1800, 1350)`，測試鎖住「不含兩端、離肩膀範圍兩端至少 100 µs」。資料檔 `data/mearm_linkage_measurements.json` 只留這四個位置並加 `excluded_positions` 說明原因；**完整原始檔另存** `data/mearm_linkage_measurements_run2_2026-09-27_full.json`（六個位置、含被退回重測前的最終記錄）。排除理由：2100 三次嘗試手肘往下的停止點分別是 850／725／1400（同一位置差 700 µs，不可重複）；1200 肩膀自己在約 1250 卡住。
- 這次操作用到了新功能：夾爪由 1300 起、每步有等待、讀回驗證；使用者用 `r` 退回並重測了 1650（第一次往上停在 1825、重測 1775）、2100（三次）與 1200（一次）。
- **剩下四個位置的判斷（未定案的部分寫明）**：
  - 手肘**上緣**（往上停）：1350→1525、1500→1600、1650→1775、1800→1800，隨肩膀單調上升；與第一次量測（1500→1675、1650→1725、1800→1750）相差 −75／+50／+50 µs，同一次內重測 1650 也差 50 µs——落在人的反應時間所造成的解析度內（±2~3 步）。擬合 `hi = 550 + 0.667×肩膀`，最大誤差 50 µs，4 點。**可信度：中高（保守估計可用），但只有 4 點、每點約一次。**
  - 手肘**下緣**：第二次在肩膀 1500／1650／1800 都**一路走到 500 沒卡住**，第一次卻在 550／700／800 停下 → 兩次矛盾。**使用者確認夾爪不同（1500 vs 1300）一定不影響**，所以這個矛盾不能歸因於夾爪（我先前的假設作廢）；唯一剩下的解釋是第一次的工具缺陷（回位瞬間跳、漏步、Enter 排隊），故**第一次的下緣資料不採信，以第二次為準**。肩膀 1350 往下停在 725 只有一個未重複的點，**不知道卡的是連桿還是夾爪／機構碰到底板桌面**。**可信度：低，不能擬合。**
  - 模型含意：MeArmPilot 的「肩＋肘」和帶假設（上下兩緣、寬度固定約 32°）在這台實機**沒有得到支持**：下緣未出現、窗口寬度不固定（650→1025→1200→1225 µs）。但實機上緣確實隨肩膀上升（斜率約 0.5~0.67，脈寬空間）。
- **下一步候選**：(1) 肩膀 1350 重複量 2 次，看 725 是否重現；(2) 每次停下時記錄「原因」（連桿卡住／嗡嗡 vs 碰到底板桌面），碰撞不是連桿耦合、不應寫進限位；(3) 上緣多加 1~2 個中間肩膀位置（例如 1425、1575）。

**量測工具第三輪改進（2026-09-27）**
- 更正：使用者確認夾爪位置不影響 → 第一次與第二次下緣資料的矛盾不能歸因於夾爪，以第二次（修正後的工具）為準，第一次下緣不採信。
- **每次停下後問原因**：`l`＝連桿卡住／嗡嗡／吃力（Enter 也是）、`c`＝碰到底板／桌面／別的零件、`?`＝不確定；記錄為 `elbow_up_cause`／`elbow_down_cause`／`shoulder_bind_cause`。碰撞是量測環境、不是連桿耦合，從窗口中排除並另列；沒有標記的舊資料視為連桿限位；`?` 也視為連桿（保守）。原因是在退回之後才問（手臂已離開卡住位置）。
- **重複與合併**：同一肩膀位置多筆記錄取最保守邊界（最低的上緣、最高的下緣），並回報重複差（上緣／下緣）；差很大就代表那個數字不可信。`--analyze FILE...` 不需硬體、合併多個檔案。`--shoulders 1350,1350,1425` 可指定位置並允許重複；拒絕靠近行程兩端的位置與非整數。
- **隱患已修**：原本每次執行都寫到同一個 `data/mearm_linkage_measurements.json`，重複量測會蓋掉現有資料。現在每次執行寫新的 `mearm_linkage_<時間戳>.json`，已存在的檔案不會被覆蓋（`--overwrite` 才允許），且在碰硬體之前就檢查。
- 測試 58 個；12 種突變（碰撞當連桿邊界／旗標被忽略／重複沒取保守值 ×2／沒回報重複差／下降與肩膀自卡後沒問原因／可覆蓋既有檔案／接受行程兩端／固定檔名／對亂輸入亂猜／接受非量測檔）全部抓到。
- 建議下一次量測：`--shoulders 1350,1350,1425,1500,1575`（1350 重複 2 次看 725 是否重現；加兩個中間位置補上緣），再用 `--analyze data/mearm_linkage_measurements.json data/mearm_linkage_<新檔>.json` 與第二次資料合併。

**「起始都要回到 1500／夾爪回到 1300」（2026-09-27）**
- 使用者指出量測沒有讓各軸每次都從 1500 出發，並要求夾爪回到 1300。查證屬實：手肘每個位置前會回 1500，但**肩膀位置與位置之間是直接走過去**（不回 1500）、底座完全沒檢查、量測開頭與結束（含 Ctrl+C／出錯）夾爪與底座沒有處理；姿勢工具的 `all 1500` 還會把夾爪也走到 1500。
- 修正：**每個肩膀位置都從同一個休息姿勢出發**（手肘先、再肩膀；間隙／回彈會讓從不同起點接近的位置停在不同脈寬，與機構本身不可重複無法區分）；量測開頭先把夾爪走到 1300、底座走到 1500，之後不再碰；**每個位置結束時肩膀也回到休息**，所以問你「原因／OK 嗎」的時候手臂已經在休息姿勢，不是撐在快卡住的位置；肩膀自己卡住後同樣回休息；結束、Ctrl+C、`WalkFailed` 都走 `safe_finish`（回不去會明確警告，不會亂丟例外）。姿勢工具：`all <µs>` 現在只動底座／肩膀／手肘，**夾爪一律走到自己的休息 1300**；新增 `rest`；`rest`／`all` 都是夾爪、底座先、**手肘在肩膀之前**。休息姿勢集中在 `servo_pose_4ch.REST`，並有測試與韌體標頭 `k*RestUs` 保持一致。
- **測試檔的一個嚴重問題（自己造成、已修）**：我用字串索引編輯 `test_measure_linkage_region.py`，把 `MeasureTest` 的一段方法複製了多次（47 個方法只有 23 個不同名稱）。Python 對同名方法只執行最後一個，所以「顯示的測試數」低估、部分測試被遮蔽。逐一比對後所有重複本的內容完全相同，以 AST 刪除重複（保留每個名稱第一個）並移除一個已被取代的舊語意測試；現在 67 個測試、無重名、全過。**教訓：改測試檔要用 AST 或明確的唯一錨點，並在編輯後檢查重名。**
- 突變測試（12 種，這次修正了先前有兩個「沒真的套用」的突變字串，並讓執行器在沒套用時明說）：11 種抓到；唯一倖存的「結束時不呼叫 `return_to_rest`」是**等價突變**——每個位置本來就以休息姿勢結束、底座與夾爪不再移動，所以正常流程下最後那次回位是空操作（保留作為保險，Ctrl+C／出錯時由 `safe_finish` 真正生效）。

#### 待辦 C 第三次真機量測（2026-09-27 11:39，`data/mearm_linkage_20260927-113901.json`）

工具流程符合設計：從休息姿勢出發、每個位置前後回休息、每次停下都標了原因、誤按可用 `r` 退回。位置 `1350,1350,1425,1500,1575`。

**結果：五個停止點全部被使用者標為 `c`（碰撞），沒有任何一個標為連桿卡住。**
- 肩膀往下（手肘 1500）：想去 1350 → 停在 1350；想去 1350 → 停在 1400；想去 1425 → 停在 1425（碰撞）。重複差 75 µs，在按 Enter 的反應精度內。與夾爪尖端本來就貼近底板的照片相符。**這是環境限制、不是連桿耦合，但它是肩膀的實際安全下限（手肘在 1500 時約 1400~1425 以上）。**
- 肩膀 1500、手肘往上：停在 1575（碰撞）。往下一路走到 500 沒事。
- 肩膀 1575：手肘整個行程 500~1850 一路沒事。

**對先前資料的影響（更正）**：第二次資料（`..._run2_..._full.json` 與精簡後的 `mearm_linkage_measurements.json`）沒有標原因，當時全部被當成連桿限位。這次同一位置（肩膀 1500）的手肘上緣被標為碰撞（1575，第二次是 1600），所以**第二次的「上緣隨肩膀上升的直線（手肘 = 550 + 0.667×肩膀）」很可能其實是碰撞、不是連桿耦合，收回把它當連桿限位的說法**。另外上緣在肩膀 1500（1575）→ 1575（無）→ 1650（第二次 1775）不單調，目前無法分辨是碰撞位置隨夾爪與底板相對高度變化、還是第二次那幾個點的反應誤差。**不要把未標原因的第二次資料與有標原因的資料合併。**

**目前能說的**：在肩膀 1500~1575、手肘 500~1850 這塊區域**沒有量到任何連桿卡住**，支持「MeArmPilot 的肩＋肘和帶（約 32°）對這台實機過於保守」。但只涵蓋一小塊；兩端（肩膀 2100、1200）是刻意排除的，那裡機構吃力，真正的連桿限制可能在那裡。**不能因此下結論說這台沒有耦合。**

**工具**：摘要現在列出肩膀自己停下的「想去／停在／原因」與重複差（3 個測試；先前的重複差測試用 `"50"` 會被 `1350` 誤滿足，已改成 `重複差 25`）。

**下一步候選**：(1) 在有標原因的前提下重量肩膀 1650、1800（各重複 2 次）與 1500、1575（各再 1 次，確認 1575 的碰撞與「1575 全無」是否可重現）；(2) 請使用者說明各次碰撞是「夾爪／前臂碰到底板」還是別的零件；(3) 若有標原因的資料在 1650~1800 也沒有連桿停止，就評估真實手臂不需要 `project_elbow`（只需各伺服機範圍＋碰撞安全區），模擬端的耦合限制屬於 MuJoCo 模型本身。

**碰撞原因確認與「安全包絡」（2026-09-27）**
- 使用者確認第三次量測的碰撞都是**夾爪碰到底板**。夾爪尖端的高度由肩膀與手肘**兩個一起**決定，所以碰撞界線是二維的：肩膀 1425 時手肘在 1500 就碰；肩膀 1500 時手肘往上到 1575 碰；肩膀 1575 時整段 500~1850 都沒碰。
- **對實體安全，任何原因的停止都要算**（碰底板與連桿卡住一樣傷硬體）。之前 `windows()` 預設排除碰撞，是為了回答物理問題「連桿是否耦合」；要給韌體用的限位需要另一個輸出：新增 `safe_envelope()`（碰撞與未標原因都算；肩膀自己的下／上限取各側**最靠近休息位**的停止點再向內縮邊界；從未見過停止的一側是伺服機自己的範圍並標「未測」），摘要現在兩種都印。8 種突變全部抓到。
- 對第三次資料算出的包絡（保守，邊界 3 步）：肩膀 1500~2100（**上限未測**）；肩膀 1500 時手肘 500~1500；肩膀 1575 時手肘 500~1850。只涵蓋很小一塊，不能直接拿來當全域限位。

#### 待辦 C 第四次真機量測（2026-09-27 11:54，`data/mearm_linkage_20260927-115438.json`）

位置 `1650,1650,1800,1800,1500,1575`（有標原因）。
- **肩膀 1650**：兩次都是手肘 500~1850 全程沒事。**肩膀 1575**：再次全程沒事（與第三次一致，可重現）。
- **肩膀 1500、手肘往上**：停在 1775（`c`，夾爪碰底板）。**同一位置歷次的停止點：1675（第一次，工具有缺陷）、1600（第二次，未標原因）、1575（第三次，`c`）、1775（第四次，`c`）——落在 1575~1775，差 200 µs，不可重複。** 這比按 Enter 的反應精度大得多。假說（**未驗證**，請使用者檢查）：伺服機的線被拉到或卡到，線的張力隨姿勢與上一個動作改變。
- **肩膀 1800（不穩定）**：第一次被接受＝手肘往上、往下**全程沒事**；接著的三次都被使用者退回，停止點是 往上 1750／1775／沒卡、往下 1475／1450／1425（往下的停止只離起點 1500 一到三步，等於一開始動就停）。**這三次退回的數據當時完全沒存進檔案**（只在終端機畫面，這裡是從畫面抄下來的；原因欄使用者按了預設的 Enter，即「連桿」，不是明確判斷）。同一位置有時全程沒事、有時一動就停 → 行為本身不穩定。
- 第二次（未標原因）在 1800：往上停 1800、往下沒事——與第四次被接受的那次類似，但與被退回的三次不同。

**工具修正（先寫測試）**：被退回的嘗試現在另存於結果檔的 `rejected_attempts`（含第幾次、當時的停止點），不列入窗口與包絡；`--analyze` 另列一段「被退回的嘗試」（且有測試證明它們不會被算進印出來的窗口）；**被退回當下就立刻存檔**，不等到結束。修正途中抓到自己的一個隱患：原本想在 `main` 結尾再存一次，但被 Ctrl+C 中斷時 `records` 仍是空的，這次存檔會**用空的記錄覆蓋磁碟上已經逐位置存好的部分結果**——改成在 `measure` 內部於每次退回時存檔、拿掉結尾那次存檔。測試 83 個；7 種突變全被抓到（其中「被退回的嘗試混進分析」第一次沒被抓到：舊測試只檢查 `load_records`、沒檢查 `analyze` 實際印出的窗口，已補）。

**判斷**：目前所有有標原因的資料裡，**沒有任何一個停止被使用者判為連桿卡住**（第三次全是 `c`；第四次 1500 的 1775 是 `c`，1800 那幾次原因欄是預設值）。但同一位置（1500 的上緣、1800）的不可重複，使得任何「限位數字」都不能當精確值。下一步重量肩膀 1800（3 次）。

#### 待辦 C 第五次真機量測（2026-09-27 12:04，`data/mearm_linkage_20260927-120445.json`）——肩膀 1800 ×3

五次嘗試（三次被接受、兩次被退回；被退回的這次有存進 `rejected_attempts`）：
1. 往上沒卡（到 1850）、往下停在 950 → **退回**（原因欄按預設 Enter）。
2. 肩膀自己走到 1675 就停 → **退回**（同上）。其餘嘗試肩膀都能到 1800。
3. 往上沒卡、往下沒卡 → 接受。
4. 往上停在 1700（**明確標 `l`＝連桿**）、往下沒卡 → 接受。
5. 往上停在 1675（**明確標 `l`**）、往下沒卡 → 接受。

**這是第一次出現使用者明確標為連桿卡住的停止點**，且兩次相差 25 µs（目前最好的重複性）。加上第四次（1750、1775 停；兩次沒卡）：肩膀 1800、手肘往上，**要嘛一路沒事，要嘛停在 1675~1775**（雙峰，不是單一數字）；被接受的往下掃一律沒事（被退回的往下停在 950／1425~1475 都是當下被使用者否定的嘗試）。工具的保守規則取最低的停止點 → 肩膀 1800：手肘上限 1675−75＝1600。

**三次有標原因的量測合併後的保守包絡（粗略、僅涵蓋量過的位置）**：
| 肩膀 | 手肘窗口 | 依據 |
|---|---|---|
| 1500 | 500 ~ 1500 | 往上在 1575／1775 碰底板（差 200 µs，取低） |
| 1575 | 500 ~ 1850 | 兩次全程沒事 |
| 1650 | 500 ~ 1850 | 兩次全程沒事 |
| 1800 | 500 ~ 1600 | 連桿，停在 1675~1775（取低） |
肩膀自己：下限 ≥1500（1425 以下碰底板）；上限只量到 1800，之後未測（2100 使用者判斷機構吃力，刻意排除）。形狀：肩膀中段（1575~1650）最自由；低肩膀時上緣由**底板碰撞**限制（夾爪太低），高肩膀時由**連桿**限制；下緣（往 500）在被接受的嘗試中從未成為問題。

**限制**：「沒事」的嘗試不能證明安全（人可能沒注意到緩慢累積的吃力）；1500 的上緣差 200 µs、1800 的雙峰都說明數字不精確，所以包絡要保守、不要內插到兩個量測點之間（兩點之間取較小者）。另：肩膀可動範圍 1500~1800 只有 300 µs（SG92R 約 0.09°/µs → 約 27°），遠小於模擬的肩膀範圍（仰角約 60°）——是否夠用於取放要再看。

#### 實體手臂的安全包絡：限位表與 C++ `PulseEnvelope`（2026-09-27，凍結第一版）

- 決定（使用者同意）：先凍結三次有標原因的量測得到的**保守、粗略**包絡，不再增加量測（再測只會增加機構負擔，雙峰不會因多測幾次消失）。第一次（工具有缺陷）與第二次（沒標原因）**不列入證據**。
- `tools/gen_mearm_envelope.py` 從 `SOURCES` 的原始量測檔產生 `include/edgeneuro/control/mearm_envelope_data.hpp`（韌體用的表）與 `data/envelope_golden.csv`（1056 列）。規則：每個**量過的**肩膀位置一個手肘窗口（重複取最低的上停、最高的下停，各向內縮 3 步）；兩個量測點**之間取兩邊窗口的交集、不內插**；肩膀只能在量過的範圍內；**任何原因的停止都算**。目前的表：肩膀 1500→手肘 500~1500、1575→500~1850、1650→500~1850、1800→500~1600（肩膀範圍 1500~1800）。
- `include/edgeneuro/control/pulse_envelope.hpp`：`edgeneuro::PulseEnvelope`（非擁有式、零配置）；NaN、空表、空窗口都回到休息姿勢；結果四捨五入到整數 µs。`mearm_envelope.hpp` 提供 `edgeneuro::mearm::envelope()`。**尚未接進 `phase3_control_loop`**：伺服機輸出預設關閉，且「模型指令 → 實體脈寬」的映射還不存在，現在沒有東西可以夾。
- 測試三層：(1) 合成表的性質（不依賴資料）——內部不變、肩膀不出量測點、在量測點用自己的窗口、兩點之間取較保守者（上下緣各測一次，因為真實表的下緣全是 500，看不出下緣的取法，另用下緣不同的合成表）、雙重夾限不變、NaN→休息、進位到最近的整數；(2) **以原始資料為準**：每一個使用者按 Enter 的停止點，包絡都不允許到達（Python 用實際原始檔逐筆檢查 7 個停止點；C++ 用獨立寫死的數字）；(3) 與 Python 參考實作的 1056 列對照。ASan+UBSan 通過；以韌體工具鏈 `-Wall -Wextra -Wdouble-promotion` 編譯標頭無警告。
- 突變測試：C++ 9 種＋Python 產生器 8 種，全部抓到。過程中有 3 個第一次沒抓到（都是測試弱點）：下緣的「取較高者」（真實表的下緣全是 500）、肩膀進位（用了 1612.4，四捨五入與截斷結果相同）、`build_table` 對空窗口的拒絕（真實資料不會觸發）——已各補測試。另一個小插曲：我把「檢查了至少 8 個停止點」的保護門檻猜成 8，實際資料只有 7 個（3 個肩膀、4 個手肘），是我沒數就寫；已改成有意義的下限 5。
- **CI 注意**：`test_gen_mearm_envelope.py` 與 C++ 對照測試都需要 `data/` 下的三個原始量測檔與 `data/envelope_golden.csv` 在版控裡（它們沒有被 gitignore，但目前都還沒 commit）。

#### 第 2 步開始：模型指令 → 實體脈寬（2026-09-27）

**設計（用「量連桿的真實角度」取代「用眼睛把手臂調到跟模型一樣」）**：真實 MEArm 的肩膀與手肘伺服機都在底座、以平行連桿驅動，所以**上臂的絕對仰角只由肩膀脈寬決定、前臂的絕對仰角只由手肘脈寬決定，兩者互不影響**；MuJoCo 模型用相對關節（上臂仰角 = 90° − 肩膀指令；前臂仰角 = 90° − 肩膀 − 手肘；夾爪連桿永遠水平）。所以只需要量兩條一維的線：連桿仰角（度）對脈寬。**公式用 MuJoCo 模擬器本身驗證**（`test_mearm_pulse_map.py` 讀模擬器裡各連桿的實際朝向，不信我的推導）：上臂、前臂仰角與夾爪水平三項全部一致。
- `tools/mujoco_bridge/mearm_pulse_map.py`：`upper_arm_elevation_deg`／`forearm_elevation_deg`／`model_ctrl_from_elevations`、`fit_angle_vs_pulse`（拒絕少於 3 點、全在同一脈寬、非有限讀數）、`ServoAngleMap`（脈寬↔連桿仰角↔模型指令，雙向、任一斜率正負都可逆、平的擬合拒絕）、`slope_is_plausible`（SG92R 約 0.09°/µs，0.02~0.4 之外多半是單位或打字錯誤）、`shoulder_travel_share`。15 個測試。
- `tools/measure_servo_angles.py`：一次動一顆、只在安全包絡內、每步有等待、每次移動前**用板子讀回的另一顆伺服機的實際位置**檢查姿勢是否在包絡內（不是用計畫值）；你輸入手機傾斜儀讀到的角度（水平＝0、遠端較高＝正、較低＝負；輸入 q 結束、已量的保留；輸入不是數字或超過 ±180° 會重問）；每筆讀數即存檔、結果檔不覆蓋；結束（含 q、出錯）整隻手臂回休息。第一部分肩膀 1500→1575→1650→1725→1800→1650→1500（手肘 1500），第二部分肩膀固定 1575（手肘窗口最寬）、手肘 1500→1700→1850→1300→1000→700→500→1000→1500；起點與終點都在 1500 以看出間隙。`--analyze FILE` 不需硬體。22 個測試（假板子＋模擬的人）。
- **看到「肩膀可動範圍佔模型的多少」**：`report` 會用擬合結果算「脈寬 1500~1800 對應到模型肩膀多少 rad、佔模型行程幾 %」，回答「27° 夠不夠」。

**兩個自己造成的問題（已修）**
1. `ask_angle(..., input_fn=input)`：**預設參數在匯入時就綁定內建的 `input`**，測試的 `mock.patch("builtins.input")` 完全沒生效，工具讀了真正的 stdin，整組測試因此卡住。改成呼叫時才查 `input`。連帶：模擬的「人」超過 200 次提示就丟 AssertionError，避免任何卡住的重問迴圈讓測試 hang。另外我原本把角度上限設成 ±120°，太緊（連桿可以往後傾過鉛直或垂下），改成 ±180°，只用來擋 400 這類打字錯誤。
2. **Python 位元組碼快取（.pyc）讓我的突變測試結果不可靠**：`.pyc` 用來源檔修改時間（以秒為單位）與大小判斷是否過期；我的突變執行器在同一秒內改寫再還原、且不少突變不改變檔案大小，Python 就沿用上一個突變的舊位元組碼。徵兆：還原後測試仍失敗（來源檔明明正確）。**已經造成一次誤判**（「大殘差沒有警告」在舊快取下顯示被抓到、乾淨快取下其實倖存）。修正：每次執行用全新的 `PYTHONPYCACHEPREFIX` 目錄＋`PYTHONDONTWRITEBYTECODE=1`。用乾淨快取重跑之前的 Python 突變組：產生器（`gen_mearm_envelope`）、安全包絡、被退回嘗試、量測工具與姿勢工具全部沒有隱藏的倖存者（只剩已知的等價突變）；Path B（`mearm_pathb.py`）重跑發現 3 個倖存：**底座擺幅**（純調校值，依專案原則不釘死，C++ 對照表已跨語言釘住，保留）、**身後淡出窗**與**極點淡入窗**（有文件化的意圖卻沒測試）→ 新增 `FadeIntentTest`（身後 150°/165°/179° 兩側都必須完全在休息位，不是半舉；傾斜 ≥22° 時在校正的左甩方位角上底座必須是完整擺幅、不被衰減），兩者現在都抓得到。
- 突變測試（乾淨快取）：角度工具 12 種，抓到 11 種，倖存 1 種是**等價突變**（記錄「計畫的脈寬」而不是「板子讀回的脈寬」：每次移動都已讀回並校正，兩者必然相等）。過程中另有兩個第一次沒抓到的弱點：**執行期的包絡檢查**（原本只測靜態計畫，執行期永遠不會觸發；改成用板子實際的另一顆位置檢查，並用「手肘卡在 1700 而計畫以為在 1500」的測試驗證）、**大殘差警告**（測試斷言的字「誤差」出現在每一行報告，斷言形同虛設；改斷言警告專屬的「超過」，並確認乾淨資料不出現）。

**需要使用者做的（硬體＋手機傾斜儀）**：`python3 tools/measure_servo_angles.py`。

- `measure_servo_angles.py` 第一次實機使用時，使用者不確定「上臂」是哪一根；已在使用者的照片上標出（`docs/which_link_to_measure.png`）：上臂＝從底座立柱旁的下轉軸斜著往上到最高處手肘轉軸的那組長斜板（兩層平行，量哪一層都一樣，貼外側面）；前臂＝從手肘轉軸垂下到夾爪那根。**這是我從照片判斷的，不是量出來的**——驗證方法：肩膀伺服機動時，整組長斜板繞下轉軸擺動；手肘伺服機動時，垂下的那根擺動。

#### 第 2 步第一次真機量測：連桿角度（2026-09-27 13:37，`data/mearm_angles_20260927-133753.json`）

- **上臂（肩膀）**：仰角 ＝ +50.1° ＋ 0.067°/µs × (脈寬 − 1500)，7 筆、最大誤差 0.9°、來回間隙 0.0°。脈寬越大上臂越高（**實測方向**，不再是推測）。安全範圍 1500~1800 只涵蓋約 20° 的上臂擺動，佔模型肩膀行程的 34%。
- **前臂（手肘）原始輸入不可用**：使用者輸入的是手機顯示的數字（無正負）：500→27、700→11、1000→15/12、1300→40、1500→56、1700→73、1850→80，最大誤差 23°。**手機 App 不顯示正負，而前臂在 700~1000 之間穿過水平**。使用者事後單獨看了三個姿勢（手肘 1500：夾爪端比手肘端**低**；700：**高**；500：**高**），正好對應「整條線符號相反、穿過水平」：修正後 500→+27、700→+11、1000→+12/+15、1300→−40、1500→−56、1700→−73、1850→−80。仰角 ＝ −54.9° − 0.082°/µs × (脈寬 − 1500)，9 筆、最大誤差 3.5°（殘差在 1000 附近最大，即穿過水平處）。修正另存 `..._signed.json`（原始檔不動，加 `typed_deg`）；**穿過水平的位置（700~1000）是假設、沒有量到**。
- **獨立檢查**：休息姿勢 1500/1500 換成模型指令＝肩膀 0.696、手肘 1.833 rad，都落在模型自己的範圍內（−0.141~0.898、0.995~2.617）；手肘 500~1850（肩膀 1575）對應 0.49~2.42 rad，**下端低於模型手肘最小值 0.995**（實機的前臂可以比模型允許的更伸直）。
- **是否需要重量：不需要**（使用者問過）。上臂很乾淨；前臂符號有使用者眼睛看到的三個獨立答案支持，3.5° 殘差在手機最難貼準的水平附近，小於伺服機間隙與連桿鬆動的量級。只有之後實機動作與模型對不起來時才回頭懷疑前臂這條線。
- **工具修正（`measure_servo_angles.py`）**：每個讀數改問兩題——(1) 手機顯示的數字（**不准打負號**），(2) 這根連桿的遠端比另一端**高還是低**（h/l）；正負號只由第 2 題決定，0° 不問。存檔保留 `typed_deg`（手機數字）與帶號的 `angle_deg`。33 個測試；8 種突變（乾淨快取）全部抓到，其中 1 個第一次沒抓到：我的假上臂永遠是正的，看不出「`typed_deg` 存成帶號」——已補「上臂低於水平」測試。另一個自己的 fixture 錯誤：假前臂（−35°＋0.08°/µs）最高只到 −7°，從來不穿過水平，「穿過水平仍是直線」的測試因此形同虛設，改用會穿過零的前臂（−20°）。

#### 感測器 → 實體脈寬的完整離線管線（2026-09-27，`tools/mujoco_bridge/mearm_real.py`）

Path B 解碼 → 模型肩膀指令＋**未經連桿投影**的手肘請求 →（模式）→ 實測連桿角度換算（`_signed.json`）→ 實測安全包絡。手肘不再套用 `project_elbow`：那是 MuJoCo 模型（MeArmPilot 數字，約 32°）的限制，這台實機在安全區內沒有；實機真正有的限制（碰底板、高肩＋高肘的連桿卡住）已經在包絡裡。手肘以模型的**相對**手肘角傳遞＝上臂與前臂的夾角，也就是人彎手肘的意思。

**需要使用者決定的模式（只影響肩膀）**：實機只能到模型肩膀行程的約 1/3。
- `geometric`：模型指令照實換算，實機連桿角度＝MuJoCo MeArm 的角度；但人舉手的大部分會撞到安全範圍兩端（一次完整舉手的肩膀脈寬：1500,1500,1550,1661,1772,1800,1800,1800,1800——只有中間約 40% 在動）。
- `stretch`：把模型整個肩膀行程攤到安全範圍（1500,1538,…,1800——整段都在動），但實機上臂角度與模型不同。

測試（`test_mearm_real.py`，11 個，真實 9/24 log＋9/13 校正）：所有輸出（含零向量、NaN、∞、倒置）都在包絡內且為整數 µs；無效讀值→休息；舉手→實機上臂單調升高且至少 15°；彎手肘→肘部夾角單調變小且至少 20°；只動手肘不改肩膀；geometric 在包絡內時與模型指令換算**完全相同**；stretch 兩端剛好到安全範圍兩端。突變 7 種，6 種抓到；倖存的「stretch 比例不夾限」是等價突變（Path B 已把肩膀夾在範圍內）。**自己的測試錯誤**：「geometric 在包絡內時完全相同」原本只用一個手挑的姿勢，而那個姿勢其實在包絡外，`if` 從未成立、測試形同虛設；改成掃整個舉手並要求至少 5 個姿勢落在包絡內。

**尚未做**：C++ 移植（等模式決定）；底座的脈寬↔角度（未量）；接進 `phase3_control_loop`（仍在 `EDGENEURO_DRIVE_SERVOS` 後面）。另：`data/mearm_angles_20260927-132833.json` 是使用者 13:28 一次中途結束的量測（肩膀 7 筆、前臂 2 筆），未使用。

**模式決定＝`stretch`，C++ 移植完成（2026-09-27）**
- 使用者選 `stretch`（「手一動手臂就跟著動」優先於「角度與模擬一樣」）。`mearm_real.DEFAULT_MODE = "stretch"`，有測試鎖住。
- `gen_real_golden.py` 產生 `include/edgeneuro/control/mearm_angle_data.hpp`（兩條實測連桿角度線）與 `data/real_golden.csv`（678 列：9/24 真實 log 與舉手掃描 × 6 種手肘讀值）；`test_real_golden.py` 在過期時失敗。
- `include/edgeneuro/control/mearm_real.hpp`：`edgeneuro::mearm::real::pulses(cal, upper_raw, elbow_bend)` → 實體（肩膀、手肘）脈寬，永遠在安全包絡內。`mearm_pathb.hpp` 抽出 `elbow_request()`（未投影的手肘請求），原 Path B 對照測試證明重構沒改變行為；`PulseEnvelope` 加 `shoulder_min()/shoulder_max()`。
- 驗證：與 Python **678 列逐 µs 完全一致**（測試允許 1 µs 的浮點進位差、且最多 2% 列，實際 0 列）；任何輸入（零向量、NaN、∞、倒置）輸出都在包絡內；無效讀值→休息肩膀；垂下→1500、校正的 FORWARD→1800；模型指令→脈寬用獨立寫死的實測數字檢查。debug-heapguard 144/144、ASan+UBSan 139/139；韌體工具鏈 `-Wall -Wextra -Wdouble-promotion` 無輸出（原本回傳 `std::pair` 時 GCC 印出 ABI 提示，改成小 struct）。C++ 突變 8 種抓到 7 種，倖存的「stretch 比例不夾限」與 Python 相同是等價突變。
- **尚未接進 `phase3_control_loop`**：韌體還需要校正資料（4 個向量＋手肘零點）——這是先前擱置的決定（編譯期常數／UART 下發／存 Flash）。底座與夾爪也還沒接。

#### 韌體接上實體手臂的肩膀／手肘（2026-09-27，仍在 `EDGENEURO_DRIVE_SERVOS` 旗標後面）

- **校正資料用編譯期常數**（使用者同意）：`gen_calibration_header.py` 從 `shoulder_calibration.json`（gitignored）產生 `include/edgeneuro/control/mearm_calibration_data.hpp`（13 個數字＋擷取時間）；Path B 不接受的校正會被拒絕、不寫檔。同樣的 9/13 數字本來就已經在測試裡提交，所以提交這個標頭不增加新的個人資料。過期檢查只在本機有 JSON 時執行（CI 自動略過並說明原因）；CI 仍檢查已提交的標頭是 Path B 接受的校正。
- 確認：韌體裡的 `shoulder_raw_ax/ay/az` 與 `elbow_bend` 正是 Python 從 UART 收到的同一組數值（`shoulder_raw_ax=`、`elbow=` 就是這兩個變數），所以 Path B 的解碼在晶片上原樣適用。
- `include/edgeneuro/control/mearm_drive.hpp`：`make_compiled_calibration()`（開機時建一次；失敗→`nullptr`）與 `drive::command(cal, raw, bend, grip)`：肩膀／手肘＝`real::pulses`；**底座一律 1500**（脈寬↔角度沒量、極性未驗證）；夾爪＝抓握經夾爪映射；沒有有效校正時肩膀／手肘停在休息位。`phase3_control_loop` 只呼叫它，四個通道各經自己的慢速起步 ramp。舊的「韌體自算 pitch/roll → 各關節獨立映射」已移除（一致性測試確認不再出現）。7 個 C++ 測試（含：編譯進去的校正把自己的 HANG 解成垂下、LEFT 解成正方位角；所有輸出在各伺服機範圍與安全包絡內；NaN 抓握不會給出亂脈寬）。
- **只有編韌體才抓得到的問題**：開啟伺服機的韌體**連結失敗**——函式內的 `static` 若在執行期初始化，需要執行緒安全的 guard（`__cxa_guard_acquire`），會把 `abort()` 與 C++ 例外展開器拉進裸機韌體。用連結對照檔與 `nm` 找出三處：我新寫的 `compiled_calibration()`（改成開機時在 `main()` 裡建一次的普通變數），以及 `claw_map()`、`envelope()`（建構子不是 `constexpr`）。修正：`ServoAngleMap`、`PulseEnvelope` 建構子改 `constexpr`、四個伺服映射與包絡改 `static constexpr`，並加 `static_assert` 鎖住（之後誰把它改回非 constexpr，主機測試就編不過）。之後：開啟伺服機的韌體 text 9824 bytes，無 guard 變數、無 `abort`／`malloc`／展開器；關閉時仍是 6704 bytes、TIM3 引用 0 次（與先前完全相同）。CI 的韌體工作本來就會編「伺服機開啟」版本，所以這類問題之後在 CI 也會變紅。
- 突變測試 5 種：4 種抓到；「LEFT／RIGHT 對調」第一次倖存——底座固定在休息位時，肩膀／手肘只用傾斜角與方位角的絕對值，鏡射校正**目前真的不改變任何輸出**；但一旦底座開始驅動就會往反方向轉，所以補上「LEFT 解成正方位角」的測試，現在抓得到。
- 檢查：debug-heapguard 151/151、ASan+UBSan 145/145；韌體全部 target 無警告。**尚未在實機上開啟旗標執行過**。

#### 2026-09-27／28：一整天的「方向全錯」，根因是感測器接線（不是程式）

**症狀**：實體手臂與 `--mearm` 的上下、左右都相反；人形路徑也「方向很怪」、手臂碰不到球。使用者懷疑人形路徑被改到。

**逐步排除（每一步都用原始資料）**
1. 上臂 MPU6050（0x68）時好時壞：`shoulder: completions=0 nacks≈450/s`、`raw_shoulder` 好幾秒完全相同、大小 0.21 g／2.2 g（不是 1 g）；開機喚醒失敗（`g_wake_result_shoulder=1`，韌體停在 `blink_code(9)`）；兩次靜止掃描都有 0x68，之後又消失 → 手臂一動就斷線。
2. 板子兩度卡在開機階段：一次在 WeAct bootloader（沒有真正斷電：ST-Link 等仍在供電），一次在 STM32 ROM bootloader（BOOT0 被讀成高電位）。都用 SWD 讀 PC 判定。
3. **人形路徑是否被改**：以實際載入的 Python 模組與編譯器列出的韌體標頭為準，逐一和 v1.0.0 比對：`run_demo_live.py` 只有新增（`--mearm` 分支、MeArm 函式、啟動等待、範圍紀錄）；`usb_serial_port.py`、`arm_hand_scene.xml`、startup／linker／toolchain 未改；`mujoco_menagerie` 與 CI 固定的 commit 相同且無修改；MuJoCo 3.12.0 與 v1.0.0 的 requirements 相同；`complementary_filter.hpp` 有 9 行 NaN 防護（人形路徑用原始向量、實機不會產生 NaN）；`firmware/CMakeLists.txt` 刪除的 6 行是使用者同意移除的壓力測試目標。**我先前只查了一個檔案就說「沒改」，不夠嚴謹，被使用者追問才完整查。**
4. **完整跑 v1.0.0**（git worktree 取出 tag、重新編譯並燒錄 v1.0.0 韌體、用 v1.0.0 的 Python 與場景、9/13 校正）：一樣碰不到球 → 排除所有程式改動。
5. **決定性的原始資料**：上臂即時讀數與 9/13 HANG 只差 2.7°（106 筆全部不同、|g|=1.03）→ 戴法沒變（**更正**：我先前依 15:03 那份校正推論「上臂轉了 180°」是錯的；錯的是 15:03 那份校正本身，它是在上臂時好時壞時錄的）。**前臂（0x69）80 筆讀數完全相同 `(1.999939, 0, 0)`**（=32767/16384，一軸卡在滿刻度、另兩軸精確 0），但每秒仍「成功讀取」246 次，所以程式察覺不到；手肘角度由上臂與前臂向量夾角算出，前臂凍結 → 手肘跟著上臂亂彎 → 碰不到球。使用者重新插拔後恢復。

**資料狀態**：`tools/mujoco_bridge/shoulder_calibration.json` 目前是 2026-09-27 15:03 那份（在感測器故障時錄的，**不可信**）；我做的備份 `.bak-before-20260927-recheck` 也是 15:03 那份（我原本說它是 9/13，錯了）。9/13 的數值仍在 `test_mearm_direction.py` 的 `SAVED_9_13` 與韌體標頭 `mearm_calibration_data.hpp`。v1.0.0 worktree 在 scratchpad，裡面放了還原的 9/13 校正檔。

**教訓與提議的防護（尚未實作，待使用者選）**：(1) 上工前健康檢查指令（不燒錄、讀 5 秒、檢查 completions>0／讀數有雜訊不凍結／靜止 |a|≈1 g／無軸卡在 ±2.0）；(2) 執行中偵測到故障時模型停住並大聲回報、校正拒絕錄製（人形路徑需使用者同意）；(3) 韌體偵測到感測器故障時伺服機停在休息位；(4) `--i2c-scan` 只找到一顆時不該顯示「All checks passed」；(5) 硬體：焊接或有卡榫的接頭＋應力釋放。

#### 自動感測器健康檢查（2026-09-28，使用者要求：不能靠人記得先跑指令）

- `tools/sensor_health.py`（15＋2 個測試，用 9/27 兩種真實故障當資料）：`HealthMonitor`（最近 1 秒：沒有讀數、I2C 成功 0 次、**連續 30 筆位元完全相同＝凍結**、過半讀數卡在滿刻度 ±1.99 g、中位數 |a| 不在 0.7~1.3 g；資料不足 50 筆時不判定為健康）；`plausible()`（單筆即時檢查：有效、無軸在滿刻度、0.3~3 g）；`format_warning()`（「⚠ 硬體異常:感測器資料不可信」＋哪一顆、位址、什麼問題）；`LatestSamplePoller`（不改 `run_demo_live.LatestSample`，用 `last_update_monotonic` 判斷新樣本，1 kHz 輪詢 100 Hz 資料不會被誤判成凍結）。移動中的手臂（|a| 暫時偏離 1 g）不會誤報（有測試）。
- **`--mearm` 預覽**：開啟視窗前自動檢查，不健康就持續說明原因、修好後自動繼續（不用重開）；等待期間拔線會明確報錯而不是一直等（測試抓到的真 bug）；執行中出錯→**模型停在最後一個正常姿勢**並大聲警告、恢復時通知；另有**單筆即時防護**：不可能的讀數一律不套用（修正前，偵測延遲的 0.5 秒內模型會跟著全零向量跑到休息位）。**行為改變**：原本「執行中全零向量→模型回休息位」，現在是「停住＋警告」（測試已改）。
- **互動校正工具**：開始前同樣自動檢查；錄製中任何一筆讀數不合理→這次錄製作廢並重錄（附警告），多次失敗就放棄，**絕不存檔**。
- **`check_hardware_ready.py`**：新增 `--sensors`（不燒錄、讀 3 秒、判斷兩顆）；**`--i2c-scan` 現在缺少 0x68 或 0x69 任何一顆就判定失敗**，並指出是上臂還是前臂（9/27 只找到 0x69 卻顯示 All checks passed）。
- 測試夾具的修正：預覽測試的假序列埠加入約 0.003 g 雜訊（真實感測器不會位元相同；全零保持精確的零，因為韌體的「還沒讀到」就是精確的零），校正測試的假資料加 1e-11 的擾動（位元不同但不影響其他 6~9 位小數的比較）。
- 突變測試（乾淨快取）：`sensor_health` 11 種、`check_hardware_ready` 4 種、預覽整合 6 種、校正整合 3 種，全部抓到。過程中抓到的測試弱點：「任何重複就算凍結」「沉默未偵測」「NaN 當有效」三個第一次沒抓到（重複放在資料中間、長時間沉默被別的檢查蓋過、NaN 也讓大小檢查失敗），已各自補測試；一個我自己的夾具錯誤（2.2 g 的讀數超出 ±2 g 感測器的量程，本來就不可能）。
- **尚未做**：人形路徑（不加 `--mearm`）——使用者先前要求不要動，待確認；韌體端（感測器故障時伺服機停在休息位）。

**人形路徑與韌體也加上健康檢查（2026-09-28，使用者同意：人形路徑只加檢查與警告、不動計算）**
- `run_demo_live.py`：共用的 `wait_until_sensors_healthy()`（`--mearm` 也改用它，行為相同）。人形 `main()` 三個掛點：(1) 資料一到、校正之前自動檢查；(2) 每次校正錄製中出現不可能的讀數→警告、等恢復、**自動重錄這個姿勢**（絕不存檔）；(3) 執行中故障→**手臂停在最後姿勢**、不把壞讀數送進平滑、大聲警告、恢復時通知。`--optional-sensors` 宣告缺席的感測器不算故障。**沒有刪任何一行**，計算未改；既有人形測試（math 44、serial 16、integration、emg logging 11、imu_to_mujoco）全過。人形的錄製時間 `RECORD_SECONDS` 是 `main()` 內的區域變數，測試不去改它（只加檢查），所以校正測試要跑真實的 6 秒／姿勢。
- 新測試 `test_run_demo_live_health.py`（7 個，跑真的人形 `main()`＋假序列埠＋假視窗＋腳本化的 Enter）；突變 5 種全部抓到。
- **測試夾具的兩個問題（已修）**：(1) 假前臂凍結用字串比對替換，但雜訊先加上去後字串已不存在→前臂其實沒凍結；改成雜訊後用 pattern 替換，保持位元相同。(2) 假序列埠在關閉後 `read()` 立刻回傳，前一個測試留下的讀取執行緒就會空轉、拖垮後面的測試（看起來像 hang）；真實序列埠會等到逾時，改成睡 50 ms。`--mearm` 的假序列埠有同樣問題，一併修（該測試組從 56 秒降到 49 秒）。
- **韌體**：`include/edgeneuro/control/imu_health.hpp` 的 `ImuHealth`（每個伺服週期 100 Hz 更新；不合理讀數＝立即故障，30 次不變＝凍結故障，連續 3 筆合理且變化的讀數＝恢復；`O<bits>` 設為選配的感測器不檢查；`constexpr` 建構）。`drive::command(..., sensors_ok)` 的 `sensors_ok` **刻意沒有預設值**（呼叫端不能跳過檢查），`false` 時 `hold`：`phase3_control_loop` **四個 CCR 都不寫**（伺服機保持目前脈寬＝停住，不會移到任何地方）。9 個 C++ 測試（含兩種真實故障值）；5 種突變全部抓到。我的一個測試寫錯：以為 2.2 g 的那筆會「立即」被判故障，但它沒有軸在滿刻度、也低於 3 g，單筆可能是快速揮動，真正抓到它的是「一直不變」（≤0.3 秒）——已改測試說明實情，而不是為了配合測試去改門檻。
- 檢查：debug-heapguard 162/162、ASan+UBSan 157/157；伺服機開啟的韌體 10132 bytes、無 guard／`abort`／`malloc`；關閉時仍 6704 bytes、TIM3 引用 0 次；一致性測試確認每個伺服週期都呼叫 `imu_health.update` 並在 `hold` 時不寫 CCR。**都還沒上實機驗證。**

#### 用硬體本身的訊號判斷感測器狀態（2026-09-28，使用者：「不要用結果推算，要看硬體的回應」）

- **查官方文件**（InvenSense RM-MPU-6000A-00 Rev 4.0，下載原文逐字比對，存在 scratchpad）：`PWR_MGMT_1`＝0x6B（bit7 DEVICE_RESET、**bit6 SLEEP**、bit5 CYCLE、bit3 TEMP_DIS、bit2:0 CLKSEL），上電預設 **0x40**；`WHO_AM_I` 預設 0x68；**其他暫存器上電皆為 0x00**；`INT_STATUS`＝0x3A、bit0 DATA_RDY_INT、讀取後清除；`INT_ENABLE`＝0x38、bit0 DATA_RDY_EN。**文件沒寫清楚** DATA_RDY_INT 是否需要先開 DATA_RDY_EN（未使用、待實測）。**0x6D~0x71 沒有文件、0x74 FIFO_R_W 一讀就會取走 FIFO 資料**→ 健康讀取只讀 0x6A~0x6C（USER_CTRL、PWR_MGMT_1、PWR_MGMT_2，3 位元組，無副作用，符合韌體已驗證的 ≥3 位元組讀取流程）。
- **推翻我的推測**：我原本推測 9/27 前臂是「斷電重開→睡著→資料凍結」；但文件寫明重開後資料暫存器都是 0x00，實際卡住的是 X＝0x7FFF、Y=Z=0，對不上 → 前臂那次的原因仍未知。
- **既有機制**：韌體早在 8/23 就記錄過「斷電重開→SLEEP=1→I2C 讀取照樣成功但資料凍結」，並在**任何一次 NACK／逾時後**的下一次成功讀取時自動重新喚醒（`consume_needs_rewake()`）。缺口：感測器在兩次讀取之間瞬間斷電、沒有產生 NACK 時，韌體不會發現。
- **韌體（新增）**：`ImuReader::begin()` 加上選用的起始位址／長度參數（**預設仍是 0x3B、14 位元組，平常的讀取路徑逐位元組相同**，一致性測試鎖住）；每顆感測器每秒一次改讀 0x6A~0x6C，`PWR_MGMT_1` 不是韌體寫入的 0x01 → 累加 `power_resets`、重新寫入喚醒與濾波設定；健康讀取的結果絕不當作動作資料解碼。診斷行新增 `shoulder_pwr_mgmt_1`、`elbow_pwr_mgmt_1`、`shoulder_power_resets`、`elbow_power_resets`。判斷邏輯放在可在電腦上測試的 `mpu6050_power_check.hpp`（6 個 C++ 測試，暫存器數值用文件上的獨立數字）。
- **實機驗證**：燒錄後兩顆 `PWR_MGMT_1` 都讀回 **1**（3 位元組讀取在實機正確）。故障重現：使用者拔前臂 `Vin` 約 1 秒——**前臂完全沒有斷過通訊**（重開機 0 次、NACK 後重新喚醒也是 0 次、讀數一直正常）。可能是透過 SDA/SCL 上拉從另一顆模組得到電源（**推測，未證實**），或拔到的不是那條線。**同時意外發現：上臂開機以來已經 NACK 後重新喚醒 353 次、匯流排恢復 365 次**——上臂仍在時通時斷，而數值看起來正常，現有畫面不會告訴使用者。
- 另一個實機發現：板子三次上電都進了 STM32 ROM 燒錄模式（BOOT0 被讀成高電位），但**用 ST-Link 重新開機後程式就正常執行**——這次「只 reset 不夠、一定要斷電」的舊經驗不成立（專案記憶標注為機制未知，這次是反例）。
- **Python（新增）**：讀取執行緒只多一行，把最新診斷行原文存進 `latest.last_diag_text`；`sensor_health.parse_diag_line()` 解析 nacks／timeouts（每秒）、asleep_rewakes／power_resets（累計，取增量）、pwr_mgmt_1；新的 `Report.warnings`：**斷線後又恢復（dropouts）**、**剛才斷電重開（power_reset）**＝警告（資料仍可用，**模型不停**，但一定顯示），`PWR_MGMT_1` 帶 SLEEP＝故障（模型停住）；警告在最後一次事件後 5 秒淡出；`--optional-sensors` 的感測器不警告。`WarningPrinter` 讓兩條 MuJoCo 路徑用同一套方式輸出（變化時、持續期間每 3 秒、結束時）。`check_hardware_ready.py --sensors`：這 3 秒內只要有斷線就判定失敗（使用前就該發現接觸不良）。
- 測試：`test_sensor_health.py` 28 個（新增 11 個，用今天真實的診斷行）、`--mearm` 與人形各 1 個端對端（韌體回報斷線時畫面顯示「上臂…斷線」且**模型繼續跟隨**）、`--sensors` 1 個。突變 8 種全部抓到；「同一行診斷被重複餵入」第一次倖存（1 kHz 輪詢會看到同一行上千次，警告會永遠不淡出），已補測試。

**第一次在實機上用 `--sensors`／人形健康檢查（2026-09-28 13:25）抓到的三個問題（都已修）**
1. 人形模型開不起來、**一句話都沒說**：上臂又在斷線（每秒逾時 15~17 次、匯流排每秒恢復 ~10 次），主迴圈一直被卡住，電腦 2.5 秒只收到 57 行（正常 ~250）。開始前的檢查把這歸類為「資料不夠、先別判斷」，而這一類我設計成不印訊息→沉默地等。修正：`should_announce()`，資料太少超過 2 秒也要說出來（「資料太少……常見原因:接觸不良,匯流排一直在自我恢復」）。
2. **`--sensors` 在實機上說「沒有收到任何資料」**：真實韌體的輸出是 `elbow_raw` 在 `shoulder_raw` **前面**，我的解析假設相反，測試用的是我自己編的假資料所以全過。修正：`parse_tick_raw()` 分開解析、與順序無關；測試改用**擷取到的真實韌體輸出**當資料。**教訓：測試資料要取自真實裝置輸出，不要照自己以為的格式編。**
3. 資料很少時，韌體「每秒一次」的診斷行實際上幾秒才來一次（韌體的時鐘是靠主迴圈推進的，被恢復卡住就變慢），3 秒的檢查可能一行都沒收到，於是說不出是哪一顆的問題。修正：沒收到診斷行就最多延長到 8 秒。實機結果：「上臂 MPU6050(0x68):斷線後又恢復(I2C 沒回應)……過去 1 秒沒回應 16 次」——直接指出是上臂。
- 板子兩次停在 WeAct bootloader，用 ST-Link reset 即恢復執行。上臂仍然接觸不良（開機以來重新喚醒 443 次）。

**原始資料檢視工具與「太嚴格」的修正（2026-09-28，使用者要求直接看兩顆 MPU6050 的原始輸出）**
- 新增 `tools/watch_imu_raw.py`（5 個測試，用實機擷取的輸出）：不燒錄、每秒一行，每顆感測器的原始向量、|a|、筆數、不同值數、最大變動，以及韌體回報的 nacks／timeouts／rewakes／pwr_mgmt_1／power_resets；健康檢查的判斷放在最後、只供對照。
- **實機原始資料**：兩顆都是 |a|=0.99 g、每秒 20~26 筆裡有 5~8 個不同值、變動 0.001~0.003 g → **讀數本身是活的、正確的**；只有「量」少（每秒 20~26 行，正常 ~100），因為上臂每秒逾時 16~17 次、主迴圈一直被匯流排恢復卡住。
- **使用者判斷正確：我太嚴格，而且有 bug**：
  1. 「資料太少」原本被當成故障（擋住啟動、模型停住）；讀數是活的、只是慢，應該是**警告**。修正：每秒 <10 筆＝故障（無法判斷），10~49 筆＝`slow_data` 警告（模型照常跟隨）。啟動時的等待只在最初 2 秒不接受「慢」（剛開始每條資料流看起來都慢），2 秒後接受並警告（`ready_to_start()`）。
  2. **bug**：韌體以 `pwr_mgmt_1=4294967295`（0xFFFFFFFF）表示「還沒讀到」，我的程式檢查它的 SLEEP 位元→ 全 1 → 誤報「感測器回報自己在睡眠狀態」。修正：>0xFF 視為未讀。
- 為什麼重插拔後錯誤訊息變了：第一次板子在執行、前臂那一秒真的斷了 105 次（當時可能在動線），`0x40` 大概是真的重開；第二次重插拔後板子卡在 WeAct bootloader → 完全沒資料（最近幾乎每次重插都卡住，用 ST-Link reset 可恢復）。

**依使用者的優先順序調整警告等級（2026-09-28）**：使用者：「這只是 prototype、麵包板，只要可以 demo；重點是數值異常或斷線造成和 MuJoCo／硬體溝通的誤會；傳送多快倒還好」。分級：
- **資料錯或沒有**（凍結、不合理、睡著、幾乎沒資料）→ 維持大聲警告＋模型停住。
- **斷線但很快恢復**（上臂現在每秒 ~17 次）→ 出現時說一次，之後**每 30 秒一行摘要**（原本每 3 秒一次、會洗版）；出現**新的**問題種類時立刻說。
- **只是資料慢**（`slow_data`）→ 執行中**不顯示**；`--sensors` 不因此失敗，只加一行 `[NOTE] 資料較慢……對 demo 沒影響`。
- 提醒使用者：上臂每次斷線恢復都會讓韌體主迴圈卡一下，動作會稍微頓、EMG 反應可能稍慢；根治是接穩上臂那顆，不是改程式。
- `WarningPrinter` 改為上述行為、`passes_pre_use_check()` 供 `--sensors` 使用；新增 4 個測試。
- 另一個實機發現：`--sensors` 在序列埠同時被別的程式（MuJoCo、`watch_imu_raw.py`）開著時會**直接丟 Python traceback**；改成明確訊息（1 個測試）。

**使用前檢查改為選項 A，並加上「也許是麵包板造成的」（2026-09-28）**：使用者選 A——只要資料正確（`report.ok`），`--sensors` 就通過；會自動恢復的斷線、已自動喚醒的重開機改成 `[NOTE]` 並點名是哪一顆，資料較慢另一行提醒。失敗時只列出**真正造成失敗**的項目（實機輸出原本把附帶的「資料太少」也混進失敗訊息）。依使用者要求，斷線與資料慢的說明加上「也許是麵包板造成的」。實機結果：`[OK] 兩顆 MPU6050 讀數都正常` ＋ `[NOTE] 上臂 MPU6050(0x68):斷線後又恢復……也許是麵包板造成的 [過去 1 秒沒回應 15 次]` ＋ `[NOTE] 資料較慢(23 筆/秒)`。測試夾具又抓到一次「沒雜訊＝凍結」的假資料（該測試的假感測器數值完全不變），已加雜訊。

**v1.1.0：感測器健康檢查（人形手臂路徑）（2026-09-28）**：9/27 花了一整天，把麵包板 IMU 接觸不良誤當成演算法錯誤，因此把檢查做成自動的。共用模組 `tools/sensor_health.py` 分兩級：資料不可信（沒資料、凍結、卡滿刻度、|a| 不是約 1 g、感測器自己回報睡眠）時，`run_demo_live.py` 不會開始；使用中手臂停在最後一個正常姿勢並警告，恢復後自動繼續；校正錄製作廢重錄。斷線後又恢復、或重置後已重新喚醒（資料仍正確）時，只提醒並點名是哪一顆，最多每 30 秒一次。韌體每秒讀一次兩顆 MPU6050 自己的 PWR_MGMT_1（依 RM-MPU-6000A-00 Rev 4.0 核對過），SLEEP 被設起來就重新喚醒並計數，數值放進 diag 行。`check_hardware_ready.py`：新增 `--sensors`，選項 A——資料正確就通過，接觸不穩列 `[NOTE]`；`--i2c-scan` 兩顆都要回應才通過。另新增 `watch_imu_raw.py`。實機結果：`--sensors` 通過，並點名上臂 0x68 每秒沒回應約 15 次（也許是麵包板造成的）；兩顆 PWR_MGMT_1 都讀回 1。人形路徑的計算完全沒改，只加檢查和警告。MeArm 路徑也用同一個模組，但 MeArm 本身還沒進 main，所以不在這個版本。

**9/13 校正定為 golden sample（2026-09-28）**：使用者決定把 9/13 16:35 的真實校正提交進公開 repo，檔案是 `data/shoulder_calibration_golden_2026-09-13.json`。那天是完整六步驟抓球第一次在實機上跑通。內容只有 4 個姿勢的重力方向、手肘零點和 EMG 門檻，沒有原始 EMG；`.gitignore` 註明這是刻意的例外。健康檢查測試改從這個檔讀，不再內嵌數字。**更正**：我先前說 v1.0.0 worktree 裡那份校正檔是 9/13 的，其實是 9/28 00:26 錄的。

**v1.1.0 硬體測試：拔 SDA 通過，但抓到一個誤報（2026-09-28）**：在 release branch 上重新燒錄，用 golden 校正跑人形模型，途中拔掉上臂 SDA。兩顆都顯示沒回應約 705 次/秒（SDA 是共用匯流排，拔一條兩顆都斷），並提醒斷線；接著上臂被判定凍結，手臂停住；插回後「✓ 感測器恢復正常」。**誤報**：視窗剛開時跳出「幾乎沒有資料(2 筆)」並停住手臂，但同一秒韌體 DIAG 是 `completions=255 nacks=0`。原始資料說明感測器沒問題，原因在我這邊：`LatestSamplePoller` 只在主迴圈輪詢時記錄資料，視窗開啟時主迴圈卡了幾秒，監測器就以為沒資料。修正：兩次輪詢間隔超過 0.25 秒就算「沒在看」，重新看滿 1 秒後才判斷資料多寡。真的太少仍然抓得到（有測試）。同時 `format_warning` 不再印「資料只是慢」，跟使用者「速度不重要」的決定一致。突變測試 6 種全部抓到。Ctrl+C 的 traceback 在 v1.0.0 就有，不是這次造成的。

**v1.1.0 修正後實機複測通過（2026-09-28 14:37）**：視窗開啟時沒有誤報，也沒有印「資料只是慢」。拔 SDA 時兩顆都沒回應約 1900 次/秒，前臂被判定凍結，手臂停住；插回後恢復跟隨。這次是在主工作區執行，裡面的 `sensor_health.py` 與 release branch 相同，人形路徑的程式碼也相同；板上跑的是 release branch 的韌體。

**MeArm 實機第一次跟著手動（2026-09-28）**：v1.1.0 之後燒錄伺服開啟版韌體（9/13 校正）。使用者觀察：肩膀方向正確；手肘和自己的手**相反**（MuJoCo 模型和手是一致的，所以問題在「模型 → 實體 pulse」這一步）；底座不動，因為當時程式刻意把它固定在休息位置。
- **手肘**：把手肘請求在自己的範圍內鏡像（伸直 ↔ 彎到底）。pulse 範圍和安全包絡都不變，只反轉方向；Python（`mearm_real.py`）和 C++（`mearm_real.hpp`）同步改。最可疑的原因是前臂實測線的正負號：手機 App 只顯示角度大小，正負號是事後依使用者看到的畫面推的。以使用者在實機上看到的方向為準。
- **底座**：使用者實測可用範圍 500–2500 µs，`set base 1700` 往左（從手臂後面看）。`real::base_pulse`：Path B 的底座指令（+ = 左）整個範圍 ±BASE_LIMIT 對應到 500–2500，左 = pulse 變大。每 µs 幾度沒量，所以實體底座角度不等於模型的，概念和肩膀的 stretch 相同。9/13 校正的 LEFT → 2332、RIGHT → 668、垂下 → 1500。`drive::command` 改用它，不再固定在休息位置。
- 測試：Python 方向測試改成「彎手肘 → 手肘 pulse 單調變小、至少 300 µs」；新增底座測試（左 > 1700、右 < 1300、垂下與無效讀數回休息位置、整數且在範圍內）；golden 表多一欄 `base_us`，C++ 逐列對照。突變測試：Python 4 種抓到 3 種，「底座不夾限」是等價突變（Path B 已把指令限制在 ±BASE_LIMIT）；C++ 2 種都抓到。**我的錯**：推出來的正負號沒先在實機上「動一下看方向」就拿去驅動實體手臂。

**肩膀安全範圍往上擴到 1950（2026-09-28）**：使用者覺得實體肩膀角度太小（1500–1800 只有約 20°）。量 1800 以上時，使用者發現肩膀抬不高的原因：手肘停在 1500 時，**手肘的連桿會撞到上臂**，手肘再多轉一點（pulse 變小，9/27 在 1800 時手肘往上走就會卡）肩膀就能更高。所以 `measure_linkage_region.py` 新增 `--elbow-hold`（肩膀移動時手肘停在這個 pulse，預設 1500 不變）：
- 允許範圍 700–1500，因為肩膀每次會帶著手肘回到 1500，而 9/27 在肩膀 1500 量過的手肘安全範圍是 500–1500。先寫 5 個測試；突變 4 種全部抓到。
- 工具原本就禁止量離行程兩端 100 µs 以內的肩膀位置（上限 2000 以下），所以只量 1875、1950。
- 實測（`data/mearm_linkage_20260928-160307.json`，`--elbow-hold 1200`）：兩個位置的手肘往上都停在 1500（連桿），往下到 500 都沒事；1950 第一次在 1550 停下，使用者退回重測。扣掉安全邊界後，兩個位置的手肘範圍都是 500–1425。
- 併入包絡後，肩膀範圍從 1500–1800 變成 1500–1950，stretch 模式自動用到新上限：舉平 → 1950。
- 更新了三個寫死舊資料的測試，改成從資料推導：來源檔數量、「不超過量過的最高肩膀」、golden 表要涵蓋包絡的最低和最高肩膀。
- golden 表的舉手掃描從 /20 改成 /21：新範圍 450 µs × 0.05 = 22.5，一半的姿勢剛好落在 x.5 µs，float 和 double 捨入方向不同，C++ 差 1 µs 的比例超過 2%（全部都在 1 µs 內）。這是輸入格點造成的，不是兩邊程式不一致。
- **還沒做**：1875、1950 的上臂角度沒量，目前是把 1500–1800 的實測線延伸過去。`measure_servo_angles.py` 量肩膀時手肘固定 1500，在新位置會超出包絡，所以這兩點要手動量。

**肩膀上限擴到 2100（2026-09-28）**：使用者用 `servo_pose_4ch` 手動確認「肩膀 2000／手肘 900」和「肩膀 2100／手肘 700」都不吃力。9/27 在 2100 覺得吃力時，手肘停在 1500，很可能也是手肘連桿撞到上臂。因此 `parse_shoulders` 在 `--elbow-hold` 小於 1500 時，上限放寬到伺服本身量過的 2100；手肘在預設 1500 時照舊不准超過 2000，下端 1200 的保護也不變。先寫測試，突變 2 種都抓到。
- 實測（`data/mearm_linkage_20260928-170204.json`，`--elbow-hold 700`）：肩膀 2025 時手肘往上在 1225 卡住（連桿），2100 時在 1075；往下兩個位置都到 500 沒事。扣掉安全邊界後，手肘範圍分別是 500–1150 和 500–1000。
- 包絡現在 8 列，肩膀 1500–2100。stretch 模式的「舉平」→ 2100。
- 使用者問「已經給你兩個點了還要量嗎」：單點只能證明某個姿勢可以；包絡需要每個肩膀位置下手肘的範圍，只用單點的話，舉高時手肘會被鎖死。使用者選擇再掃一次。
- `test_geometric_mode_saturates_most...` 原本寫死「geometric 模式會動的比例 < 60%」，這是舊的窄範圍才有的數字，範圍變大後是 70%。改成測相對關係：geometric < stretch，而且 geometric 仍會卡在兩端。

**肩膀和手肘同步移動，途中也不超出安全包絡（2026-09-28）**：包絡擴到 2100 以後，上端的手肘範圍變很窄（2100 時上限 1000），兩個獨立的 ramp 在快速舉手時可能短暫經過 (2025, 1300) 這種連桿會撞到的姿勢。舊範圍的上端很寬，所以這個問題以前幾乎不會發生。
- 新增 `include/edgeneuro/control/mearm_joint_step.hpp` 的 `joint_step`：每個伺服週期，兩個 ramp 各自提出下一步，依序取第一個仍在包絡內的：兩顆一起動 → 只動手肘（肩膀等手肘讓開）→ 只動肩膀 → 兩顆都等。沒被採用的 ramp 狀態完全不變，所以不會之後突然跳一下；不會把手肘直接截到邊界。
- 先寫測試（`tests/test_mearm_joint_step.cpp`）：
  - 包絡內所有可行姿勢兩兩互移（每個肩膀位置、手肘每 125 µs 一點），每一個週期都在包絡內。
  - 3 秒內一定到達目標，不會卡死。
  - 促成這次修改的快速舉手：獨立 ramp 確實會超出包絡（證明測試不是空的），`joint_step` 不會。
  - 包絡沒有限制到時，行為和兩個獨立 ramp 完全相同。
  - 就算目標在包絡外，也只會停在邊界。
- 突變 5 種全部抓到。「兩顆都等」原本沒被測到（所有目標都在包絡內，走不到這個分支），補了「目標在包絡外」的測試後才抓到。
- `phase3_control_loop` 的 CCR2／CCR3 改用 `joint_step` 的結果。

**用感測器一次只驅動一顆伺服（2026-09-28）**：使用者懷疑電源不穩，想「用感測器，一顆一顆測」。新增編譯選項 `-DEDGENEURO_SERVO_MASK=<bits>`（bit0 底座、bit1 肩膀、bit2 手肘、bit3 夾爪；預設 15＝全部）和 `drive::only(cmd, mask)`：沒開的伺服送休息 pulse，`hold` 保留。肩膀和手肘仍然走 `joint_step`，所以只開肩膀時手肘停在 1500，而肩膀 1875 以上時手肘在 1500 會撞連桿，因此肩膀會自動停在手肘 1500 仍安全的最高處（測試：舉平時要求 >1800，實際停在包絡允許的位置，每個週期都在包絡內）。先寫測試；突變 4 種全部抓到。四種版本分別編在 `firmware/build-only-{base,shoulder,elbow,claw}`。

**手肘死區修正（2026-09-28）**：只開手肘測試時，使用者說手肘「沒有反應」。原始資料：當時手肘讀數 0.919（伸直 0.434，約彎 28°），程式算出手肘 pulse 1500，ST-Link 讀到 CCR3 = 1495，兩者一致，所以韌體和伺服都照程式在做。問題在我的演算法：先前為了反轉方向把手肘請求鏡像，鏡像後「伸直」對應的 pulse 超出包絡，被截到上緣，前約 45° 的彎曲全被壓成同一個值。只開手肘時肩膀固定 1500，這裡的上緣最低，死區最大；四顆都開時肩膀常在 1575–1650，上緣比較寬，所以之前沒發現。
- 修正（Python `mearm_real.pulses` 和 C++ `real::pulses` 同步）：手肘不再用模型的手肘角度換算，改成在「這個肩膀位置下包絡允許的手肘範圍」內按比例分配：伸直 = 上緣，彎到底 = 下緣，pulse 越小越彎（方向不變）。兩種模式都一樣；geometric 模式只剩肩膀照模型角度換算。
- 先寫測試：「從伸直開始每彎約 6° 手肘 pulse 都要變小（無死區）」和「伸直 = 上緣、彎到底 = 下緣」，兩個都先失敗。舊測試「geometric 手肘照模型換算」改成只檢查肩膀。突變 4 種全部抓到。
- `test_constants_consistency` 另外抓到一個我漏掉的：加 `joint_step` 後 CCR2／CCR3 的寫法變了，這個結構檢查當時沒重跑。已改成檢查 `joint_step`、`se.shoulder`／`se.elbow`，以及 `drive::only`。

**伺服不順的根因：伺服電源的電容鬆脫（2026-09-28）**：只開手肘時使用者說「沒反應」，但用 ST-Link 讀到手肘訊號（CCR3）確實從 1495 變到 746 µs（手臂和手肘姿勢不同時），表示韌體有送出變化，問題在訊號之後的硬體。使用者把伺服電源上的電容接回去後，動作「比較順了」。這也解釋了先前底座轉不到 500／2500 兩端：快速移動時伺服瞬間電流大，沒有電容時電壓會掉。教訓：MuJoCo 正常只證明「感測器 → 角度」，實體伺服要用 CCR 暫存器（SWD 讀，不影響執行）來分辨是程式還是硬體。

**1€ 濾波器（2026-10-02，還沒接進韌體）**：使用者說 base「一點動作就轉來轉去」。查了資料：手臂主動動作大多在 0–4 Hz，生理性顫抖 8–12 Hz；延遲 10–20 ms 就可能被察覺，170 ms 以下對操作影響輕微。重要發現：韌體早就把 MPU6050 設成 DLPF_CFG=6（5 Hz 低通、約 19 ms 延遲），高頻雜訊和顫抖大多已經被濾掉，所以 base 太敏感的主因很可能是「上臂剛抬起時方位角本身就不穩」（真實的小動作被放大），而不是雜訊。使用者不接受為了穩定而縮小 base 範圍（硬體能轉 500–2500 就要用滿）。選擇 1€ 濾波器（Casiez 等人，CHI 2012）：靜止時濾波強，移動越快截止頻率越高、延遲越小。
- `include/edgeneuro/filters/vec3_one_euro.hpp`：濾的是原始加速度向量（在算角度之前），三軸共用同一個速度（向量速度的大小），避免方位角被扭曲。先寫 9 個測試，用合成資料：靜止時雜訊縮小超過 4 倍、以 2 rad/s 轉動時延遲比同樣平滑的固定低通小超過 3 倍、β=0 時等於固定一階低通、單軸移動會放鬆所有軸、無效值與 dt≤0 會被忽略、常數輸入完全不變。突變 11 種全部抓到。先前寫好的固定 EMA（`vec3_ema.hpp`）已經刪除，從未接進韌體。
- **參數（最小截止頻率、β）還沒決定**：要用真實錄到的手臂資料來定（靜止、慢動、快動三段），之後再接進韌體。
- **C++ 突變測試的快取陷阱**：有一次還原標頭檔後，make 沒有重新編譯，跑到舊的測試執行檔。改成每次刪掉相關測試的 `.o` 再編譯，並把今天的 joint_step、only()、底座／手肘突變全部重跑一次：結果都相同（全部抓到）。

**錄真實手臂動作的互動工具（2026-10-02）**：`tools/capture_arm_motion.py`，用來從真實資料決定 1€ 濾波器的兩個參數。依序錄三段（靜止 10 秒、慢慢動 15 秒、正常速度 15 秒），每段按 Enter 才開始、有倒數、錄完可以重錄，最後存成新檔 `data/arm_motion_<時間>.json`（不覆蓋），並印出各段、各感測器的速度分布和靜止雜訊。速度的算法和 `Vec3OneEuro` 一樣：原始加速度向量（不正規化）的導數，再用 1 Hz 低通，單位 g/s（手臂約 1 g 時約等於 rad/s）。先寫 13 個測試；突變 12 種全部抓到。過程中發現原本分析時有先把向量正規化，和韌體濾波器看到的不一致，已改成不正規化。已加入 CI 和 README。
- 同一天依使用者的核心 demo 任務（垂下 → 往前伸平 → 左擺 → 握拳抓膠帶 → 抬起 → 右擺 → 放下）改寫錄製流程：14 段、約 160 秒。① 7 個任務姿勢各靜止 8 秒，抓膠帶之後的姿勢都握拳錄，因為握拳會帶來 9–10 Hz 的生理性顫抖；② 在抓握位置慢慢微調對準 15 秒；③ 用 demo 速度完整做 3 次任務；④ 3 段不在任務裡的驗證動作（`check_` 開頭：右前方靜止、手舉高靜止、自由動作），不參與調參，只用來確認參數沒有過擬合。使用者指正我：膠帶擺哪裡、夾爪夾不夾得住都不是現在的問題，現在專心處理抖動，不要一直預先擔心後面的事。

**實測：韌體的時間被少算約 3.4 倍（2026-10-03）**：準備加前臂陀螺儀欄位時，從程式碼推算 UART 會卡住主迴圈，接著用實機驗證（伺服版 phase3_control_loop，電腦時鐘對照韌體 tick）。10 秒內收到 293 行，每秒 29.2 行；韌體 tick 只前進 2920，等於**每秒 291 個 tick，設計是 1000**；每兩行之間 tick 差 10（照設計），但實際間隔中位數 38.3 ms，設計是 10 ms。每行 346 bytes，在 115200 8N1 下要 30.0 ms 才送得完，而 `usart2_send_byte` 是忙等，送的時候主迴圈完全停住，TIM2 的 1 kHz 節拍就被漏掉。影響：
- 伺服其實每秒只更新約 29 次，不是 100 次。
- 所有用 tick 算的時間都少算約 3.4 倍：ramp 的速度限制、上臂互補濾波器的 dt。之後的陀螺儀融合也會錯同樣的倍數。
- 之前健康檢查看到的「每秒只有 23–26 筆」，主因就是這個，不是麵包板；麵包板斷線只是讓它更慢。sensor_health 的「正常每秒約 100 筆」這個前提也要修正。
- 另外：上電後又卡在 WeAct bootloader（PC=0x08003190），`openocd reset run` 後才正常跑。

#### 伺服電源重新接線：大躍進（2026-09-29～10-01）

**結果**：重新接完電源後，使用者實測「大躍進」，伺服卡住、轉不到位的問題大致消失。這證實電源是伺服卡住的最大因素。（另一個獨立的問題是時間少算，影響順暢度，見 10-03 的條目。）

**現在的接法**（取代 README 舊的「USB 充電器 + USB 轉接板」接法）：
- 4 顆 AA 電池盒，約 5 V，給伺服專用；Black Pill 照舊用自己的 USB 供電。
- 兩顆 WAGO 221-415（5 孔，孔內全部相通）：一顆接正極、一顆接負極。正極接電池紅線、電容 +、4 顆伺服紅線；負極接電池黑線、電容 −、4 顆伺服棕線，**再拉一條線到 Black Pill GND（共地）**。伺服大電流完全不經過麵包板。
- 電容直接夾在兩顆 WAGO 之間：10 V 1000 µF 固態電容，並聯一顆 104（0.1 µF）陶瓷電容。
- 伺服端：公頭杜邦線插伺服母頭，另一端剪掉剝線夾進 WAGO。伺服橘色訊號線照舊接 PA6/PA7/PB0/PB1。
- 電池盒的線太細，WAGO 夾不住：改用 ET-0.34 歐式端子（今華最小的規格），電池線剝 24 mm、折三折塞滿端子，再用壓接鉗壓。

**過程中踩到的坑**：
1. **電容鬆脫**（9/28）：電容接回去後動作明顯變順。
2. **尖嘴鉗壓端子 + 剝線太短 → 量不到導通**：只剝約 8 mm 又折三折，銅絲只剩約 3 mm，停在端子後段的塑膠套裡，前面的金屬管是空的。改剝 24 mm 再折三折，改用壓接鉗才可靠（尖嘴鉗只壓到兩點，壓完還會彈回來）。
3. **忘記共地 → 伺服完全不動**：WAGO 上量到 5 V、導通也都正常，伺服卻不動。用 ST-Link 讀暫存器確認韌體端完全正常（TIM3 開著、4 個通道啟用、PA6/PA7/PB0/PB1 都設成 AF2、CCR 正是下的指令），所以問題一定在板子到伺服之間。最後發現是負極 WAGO 沒接到 Black Pill GND，接上後就正常了。
4. 上電前的檢查流程：先拿出電池；用導通檔確認電池盒金屬片到 WAGO 測試孔有通、正極 WAGO 和負極 WAGO 之間**沒有**持續嗶聲（有電容時嗶一下就停是正常的）；裝上電池後量 WAGO 兩端應約 4.8–6.4 V。

**修正：UART 不再卡住主迴圈（2026-10-03）**：
- `include/edgeneuro/control/tx_ring.hpp` 的 `TxRing`：固定大小的位元組佇列，不配置記憶體。一整行放得下才開始放，放不下就整行跳過並計數，絕不送半行。先寫 5 個測試；突變 7 種全部抓到。（這次是先寫了標頭檔才跑測試，沒有先看到測試失敗。）
- `phase3_control_loop`：主迴圈開始後，`usart2_send_byte` 只把位元組放進 2048 bytes 的佇列；`usart2_pump()` 每一圈呼叫一次，TXE 有空就送一個字元，同時處理接收（EMG 門檻更新）。用到的暫存器和原本的忙等完全一樣（SR.TXE、SR.RXNE、DR），另外讀 SR.ORE 來偵測漏收；三個位元都對照過 vendored CMSIS 標頭。開機訊息和錯誤處理（主迴圈開始之前、`blink_code`）維持原本的阻塞式送出，因為那時沒有人會去清佇列。tick 行和 EDGE 行要保留一整行 diag 的空間才送；diag 行新增 `uart_skipped_lines`、`uart_dropped_bytes`、`uart_rx_overruns`。
- 實機結果（伺服版）：用 30 秒量，**每秒 1007.3 個 tick**（修正前 291）。晶片時脈用 DWT cycle counter 量是 16.14 MHz（HSI，+0.9%），推算應該是 1009，兩者吻合；diag 行平均每 993 ms 一行。電腦端每秒收到 34.5 行，`uart_dropped_bytes=0`、`uart_rx_overruns=0`，跳過的行數照預期增加。第一次只量 10 秒、而且剛開機就量，得到 1141，是開機時佇列和接收緩衝區的暫態造成的，量 30 秒就正常了。
- 時脈相關暫存器直接讀晶片確認：`RCC_CFGR=0`（SYSCLK=HSI，不分頻），TIM2 PSC=15／ARR=999，ADC1 CR2=0x16000001（TIM2 TRGO、上升緣觸發）。
- 連帶修正 `sensor_health`：健康的資料量從「每秒約 100 筆」改成「約 30 筆」，因為 115200 baud 本來就只能送這麼多。`MIN_SAMPLES` 從 50 改成 20，文字和 `--sensors` 的 NOTE 也一起改。先寫測試「每秒 34 筆是正常的」，看到它失敗再改。
- **影響**：伺服 ramp、互補濾波器的 dt 現在都是真實時間，伺服的實際最大速度比之前快約 3.4 倍（以前的「6000 µs/s」實際只有約 1750 µs/s）。手感會變，需要實機確認。
- 已知的不穩定測試：`test_run_mearm_preview` 的 `test_a_fault_mid_session_holds_the_model_warns_and_recovers` 在電腦負載高時偶爾失敗（3 次整組連跑失敗 1 次；新舊設定單獨各跑 5 次都通過），原因是用真的執行緒模擬資料串流，結果受時序影響，之後要改成不依賴時序的寫法。

**直接量伺服訊號（2026-10-03）**：使用者指出「順不順、會不會太衝」我應該自己量，不是只問他。透過 ST-Link 讀 TIM3 CCR1–4（每秒約 790–818 次，韌體照常執行），錄了 40 秒。肩膀全程只在 1500–1691，所以使用者可能沒照我列的流程動，三段時間沒辦法對應到姿勢；但已經看得出：**上臂幾乎垂下時，底座從 1446 擺到 2500**（標準差 214–325 µs），證實 base 敏感的主因是上臂垂下附近方位角不穩，不是雜訊。快速動作時每一步最多 60 µs，正好是 ramp 的速度上限，底座撞到 54 次、手肘 80 次，代表伺服是被速度上限擋住，不是衝過頭。之後依使用者要求，做成互動式工具 `tools/measure_servo_response.py`：5 段（垂下靜止、抬一半靜止、伸平靜止、demo 任務、自由動作驗證），每段按 Enter 開始、可重錄，存成 `data/servo_response_<時間>.json`，並分段印出每顆伺服的範圍、標準差、最大步距、撞到速度上限的次數。先寫 9 個測試（其中一個測試本身的答案順序寫錯，修正測試而不是程式）；突變 7 種全部抓到。已加入 CI。

**肩膀伺服「沒反應」：接錯腳位（2026-10-03）**：使用者發現肩膀伺服不動。用 ST-Link 讀暫存器確認韌體端完全正常：TIM3 開著、CH2 是 PWM 模式且已啟用、PA7 設成 AF2、CCR2 = 1800（正是下的指令）。交換測試：肩膀伺服改接到 PA6 就會動，所以伺服和電源都是好的。最後使用者發現肩膀的訊號線一直接錯腳位，改正後正常。表示之前跟著感測器測試時，肩膀其實從來沒有收到訊號；在那之前「肩膀看起來對了」的觀察，可能來自其他伺服的動作，之後要重新確認肩膀的方向與範圍。（三用電表量到的 0.21 V 是插錯的那支腳位上量到的，不是 PA7 的訊號。）檢查順序值得記住：先讀暫存器，確認韌體送出的值；再量線上的電壓；最後交換插頭，區分是伺服還是線路的問題。

**`R`：每次開始都從同一個起點（2026-10-03，使用者設計）**：使用者發現伺服是韌體在控制，`run_demo_live.py` 關掉後韌體仍繼續跟著手臂，所以每次開始時伺服可能停在任何地方。使用者的設計：只要 `R`，不要 `G`。`run_demo_live.py --mearm` 先顯示「請先把手臂往下垂擺好，按 Enter 後伺服會回到起點」，按 Enter 才送 `R`。
- 韌體：收到 `R` → `mearm::drive::Homing`：4 個 ramp 重新進入慢速啟動模式（`ServoStartupRamp::rearm()`，300 µs/s），伺服走回起點（底座／肩膀／手肘 1500，夾爪 1300 張開）。這段期間不看感測器，也不受感測器的 hold 影響（起點是已知安全的姿勢）。4 顆全部到位後，再從起點**慢慢**開始跟著手臂動。用 9/13 校正驗證：手臂垂下、手肘伸直時，指令剛好就是起點，所以按 Enter 後不會有跳動。只用到原本就在讀的 USART DR，沒有新增暫存器設定。4 個 ramp 改成一個 `ramps` 結構，方便 Homing 管理。
- 先寫測試：C++ 7 個（不送 R 時直接通過；送 R 後每一步都慢、而且一定到起點；到起點後慢慢開始跟隨；忽略 hold；「垂下的指令就是起點」；要 4 顆**含夾爪**都到位才算完成）；突變 6 種全部抓到，其中「沒檢查夾爪」第一次存活（測試裡夾爪總是比底座先到），補了「只有夾爪不在起點」的測試後抓到。Python 2 個（提示一定在送 R 之前；送 R 一定在預覽開始之前）。`test_constants_consistency` 的韌體結構檢查同步更新，並新增檢查：收到 R → Homing。

**電池沒電（2026-10-03）**：使用者量伺服電源：板子沒上電（伺服不出力）時 4.5 V，上電後掉到 2.7 V。4 顆 AA 正常空載應該是鹼性約 6 V、鎳氫約 5.2–5.6 V，所以 4.5 V 代表電池快沒電，負載下掉到 2.7 V，遠低於 SG92R 的 4.8 V 額定電壓。已換新電池。換電池後要確認：空載 5–6 V，上電後壓降小於 0.5 V。之前看到伺服「沒反應／沒力」時，電池可能已經是其中一個原因。

**伺服抖動：1€ 濾波 + 底座遲滯（2026-10-03）**：換了新電池、肩膀接對以後，使用者說「超級超級抖」。使用者主張伺服應該用最快的速度、要穩定的是演算法，這也是業界做法：ramp 是安全上限，不是平滑工具；穩定要在指令端做（遙控／手術機器人的顫抖濾波），再加遲滯。
- 實測（`data/servo_response_20261003-135305.json`，`measure_servo_response.py` 量 CCR）：垂下靜止時底座標準差 0.5 µs（很穩）；抬一半 22.7 µs；**往前伸平 102 µs（範圍 383 µs）**；肩膀和手肘靜止時都只有約 3 µs。所以幾乎只有底座在抖。換算：手臂方位角每 1° ≈ 底座 30 µs（9/13 校正）。
- 頻率分析（0.2 秒移動平均拆成快、慢兩部分）：伸平時慢漂移標準差 100.8 µs（幾乎全部 < 1 Hz，是手臂真的在慢慢飄），快抖動 10.6 µs（單步最大 45 µs）。使用者確認看到的是 (a) 快速抖動。**我第一次的模擬錯了**：假設晃動是 1 Hz，結果 1€ 幾乎沒效（69→67 µs）；改用真實資料的模型（0.12/0.27 Hz 漂移 + 每筆約 0.35° 的快抖動）後，快抖動 9.9→2.5 µs（約 4 倍），慢漂移照樣通過（它是真實動作，濾波本來就不該擋）。
- `include/edgeneuro/control/mearm_input_filter.hpp`：`ArmInputFilter` 對兩顆 IMU 的原始加速度向量做 1€（起始值：最小截止 0.5 Hz、β 1.5），手肘角度改用**濾波後的兩個向量**算；`Hysteresis` 對底座目標做 10 µs 遲滯（約 1°）。參數是起始值，要用 `measure_servo_response.py` 在實機上調。只有伺服看到濾波後的值，UART 照樣送原始讀數（人形路徑不受影響）。
- 測試（先寫）：伸平時快抖動至少降 3 倍、慢漂移至少保留一半；往前→左擺（0.5 秒）在動作結束後 150 ms 內到位（實際 30 ms）；手肘角度用濾波後的向量（上臂抖動不會讓手肘抖）；遲滯邊界；float 捨入讓 cos 剛好超過 1 時不會變成 NaN（用搜尋找到的真實向量）。突變 7 種，第一輪 3 種存活（遲滯邊界、手肘用原始向量、acos 夾限），補測試後全部抓到。
- C++ 測試 204／199 個全過；伺服版 13024 bytes；已燒錄。

**底座的舒服範圍（2026-10-03，程式完成、還沒量）**：使用者覺得轉動時底座太敏感，但不接受縮小底座範圍。改法：底座照樣用滿 500–2500，但兩端對應到使用者**舒服的最大左右範圍**（原本約 ±34°，每 1° ≈ 30 µs）。校正檔新增 `base_reach_left_raw`／`base_reach_right_raw`（原始向量），`gen_calibration_header.py` 編進韌體（`kHasBaseReach`）；沒量過時，換算和原本**完全相同**（有測試獨立用 Path B 的舊公式比對）。Path B 的兩個淡入淡出（剛舉起時、手臂到身後時）抽成 `pathb::fades`，Python／C++ 共用，不寫兩份。先寫測試；Python 突變 6 種、C++ 6 種全部抓到。過程中我的測試有兩個錯：①繞 HANG 軸 +θ 其實是往**右**轉（用 decode 確認後改成 `turned_left` 輔助函式）；②有一次輔助函式沒插進去，測試整個出錯，突變結果被誤算成「抓到」，發現後重跑才得到真實結果（4 種存活，補測試後全抓）。**量測工具還沒做**（之後會重用 9/13 `run_demo_live.py` 錄姿勢的程式）。

**使用者回顧人形手臂的教訓：底座維持「上臂旋轉」控制（2026-10-03）**：使用者說舉手時底座會跟著動，這是上臂旋轉手勢本身的限制（舉手時上臂會自然帶一點旋轉）。我提議改用陀螺儀量真正的左右擺動，**使用者否決，理由正確**：9/03–05 人形手臂用陀螺儀融合挖出三個 bug，最後改成純加速度計；上臂旋轉控制左右是使用者 9/05 提出、用資料驗證過的（差 0.42 g）；而且 9/12 實測握拳時陀螺儀跳動是靜止時的 100 倍，demo 第 4 步握拳抓膠帶時，陀螺儀積分的底座會漂。舉手時底座跟著動的問題還沒解決（候選：主要軸鎖定——舉手中底座先停；或錄「直直舉手」量出自然旋轉再扣掉），要先錄資料。

**MeArm 改成「高度／伸出距離」的對應（height_reach，2026-10-03，使用者設計）**：使用者說肩膀和手肘的伺服應該交換。先確認接線：用 `servo_pose_4ch` 實測，CH2 帶動大臂、CH3 帶動前臂，接線是對的，所以是對應方式的問題。使用者確認的設計：MeArm 的前臂伺服決定夾爪**高度**、大臂伺服決定**伸出距離**，所以人的手臂要交叉對應——**舉手 → 手肘伺服變小（夾爪往上），彎手肘 → 肩膀伺服變大**。`mearm_real.py` 新增模式 `height_reach` 並設為預設（`stretch`／`geometric` 保留為選項），C++ `real::pulses` 同步改成這個模式：彎曲程度把肩膀伺服分配到量過的 1500–2100；舉起程度把手肘伺服分配到「這個肩膀位置下包絡允許的範圍」（上緣 → 下緣）。垂下＋伸直 = (1500, 1500) = R 的起點；上臂讀數無效 → 起點，不管手肘。安全包絡是限制在伺服的 pulse 上，和誰控制哪顆無關，`joint_step` 照樣保證途中安全。先寫測試；Python 突變 6 種（第一輪有一個突變本身語法錯、一個存活，修正後全抓），C++ 5 種（「無效讀數」第一輪存活，補了手肘彎曲的情況後抓到）。golden 表改用預設模式重新產生；`test_mearm_drive` 的「只開肩膀」測試改成用彎手肘驅動肩膀。MuJoCo 預覽仍是關節對關節的模型，和實體手臂不同，沒有改。已燒錄。

**夾爪改成 1300–1500，`--mearm` 加上 EMG 門檻（2026-10-03）**：
- 使用者要求握拳 = 1500，而且**安全上限也是 1300–1500**（原本實測行程 1300–1600）。`kClawHiUs` 改成 1500（`claw_map` 0..1 → 1300..1500，ramp 也夾在這個範圍），`servo_pose_4ch.py` 的 RANGES 同步改，有測試保證兩邊一致。中途一度只改握拳目標、上限保留 1600，使用者指正後改成上限也是 1500。更新了寫死 1600 的測試（drive、servo_maps、homing、servo_pose、constants_consistency）。
- 發現 `--mearm` **從來沒有把 EMG 門檻送給板子**：`--skip-emg-calibration` 在這條路上沒作用，韌體用的是內建預設 2800，不是校正的 1129，所以握拳要比校正時用力很多。修法：把 `main()` 裡的 EMG 門檻處理**原封不動**搬成 `apply_emg_threshold(args, ser, latest)`，人形手臂和 `--mearm` 共用（重用 `calibrate_emg_threshold`／`send_emg_threshold`／`save_calibration_fields`）。在 `--mearm` 中放在 R 之前。先寫測試：加 `--skip-emg-calibration` 時送出 `T1129\n`，而且在 R 之前；不加時執行互動校正並存檔，也在 R 之前。人形手臂的健康檢查、整合、EMG 紀錄測試都照樣通過。
- `test_run_mearm_preview` 的時序不穩定測試又失敗一次；單獨跑 4 次都通過，和這次修改無關。

**手肘伺服的跳動：安全包絡的階梯邊緣（2026-10-03，我的 bug）**：使用者說「手肘動作很陡，移動一點就上下抖」。先用程式算：手臂舉 30%、手肘彎 16° → 17°，手肘伺服 1200 → 1445；**手臂垂下時彎過 16.5°，1500 → 1850（350 µs）**。原因：height_reach 是在「包絡在這個肩膀位置的手肘範圍」內分配舉手程度，但包絡的範圍在量測點之間是階梯狀的（上緣 1500 → 在 1575 跳到 1850 → 過 1650 掉到 1600 → …），肩膀伺服由手肘彎曲驅動，一跨過量測點，手肘伺服就跟著跳。人的手肘自然微彎約 15° 時剛好就在邊界上。修法：`smooth_elbow_window`——每個量測點的上緣取「自己和相鄰點的最小值」（下緣取最大值），點之間用直線連起來，所以連續，而且一定在包絡內側（兩點之間不超過兩邊的窗口，也就是包絡本身的保守窗口）。Python 和 C++ 同步改。先寫測試：每 0.5° 的手肘彎曲和每 1% 的舉手，手肘伺服變化都不超過 25 µs；平滑後的範圍每個肩膀位置都在包絡內（另用一張下緣有變化的合成表測下緣）。Python 突變 4 種全抓到；C++ 4 種抓到 3 種，「下緣不取相鄰值」在目前資料下等價（實測下緣全是 500），已由 Python 的合成表測試和對照表保證一致。golden 表重新產生。已燒錄。

**FPU 從來沒開過（2026-10-03，潛藏 bug，已修）**：燒錄後 `reset run` 兩次都停在 WeAct bootloader。燒進去的內容確認正確（和 build-servos 的 .bin 完全相同，向量表正常），所以改用 SWD 直接從 app 的 Reset_Handler 啟動（`reset halt` → MSP/PC 設成 app 向量表的值 → resume，等於手動做 bootloader 的跳轉）。結果程式立刻 HardFault：IPSR=3、HFSR=0x40000000（FORCED）、**CFSR=0x00080000 = UFSR.NOCP**（執行浮點指令但 FPU 沒開）。所有韌體都用 `-mfloat-abi=hard` 編譯，但 `startup.c` 從來沒開 FPU，程式一直是**靠 bootloader 剛好留著 FPU 開著**才能跑。修法：`Reset_Handler` 第一件事寫 `CPACR |= (3<<20)|(3<<22)`，再 DSB/ISB。這和 ST `system_stm32f4xx.c` 第 171 行完全相同；位址 0xE000ED88 = core_cm4.h 的 SCS_BASE 0xE000E000 + 0x0D00 + CPACR 偏移 0x088；反組譯確認指令正確。驗證：直接跳轉啟動 → 正常執行，3 秒收到 90 行 tick。`reset run` 仍然停在 bootloader，這是 bootloader 自己的判斷，和這個 bug 無關。**新的可靠啟動方式**：`openocd -c "init; reset halt; reg msp [read_memory 0x08004000 32 1]; reg pc [expr {[read_memory 0x08004004 32 1] & ~1}]; reg xPSR 0x01000000; resume; exit"`，完全不經過 bootloader。

**校正改成用序列埠傳給板子（2026-10-03／04，使用者選 B，人形手臂也一樣）**：之前實體手臂用的是編譯進韌體的 9/13 校正，`--mearm` 也從來不做姿勢校正，所以重新校正要重新產生標頭檔、重新燒錄。改成：
- 格式 `C<20 個值 ×1e6 的整數>,<檢查碼>\n`：4 個姿勢向量、手肘零點、底座舒服範圍旗標與兩個向量；檢查碼 = 總和 mod 1000000007。用定點整數，韌體不用解析小數。
- 韌體 `mearm_calibration_link::Parser`：一次一個位元組，不配置記憶體；遇到 `C` 就重新開始（不會把兩則訊息黏在一起）、超過 12 位數、雜訊字元、欄位數不對、檢查碼不對都拒絕。收到完整訊息後，伺服迴圈用 `apply_calibration_message` 再驗證一次（`pathb::Calibration::make`，量過的底座範圍再用 `make_base_reach`），通過才換上，否則沿用原本的校正。開機時仍用編譯進去的 9/13 校正。diag 行新增 `cal_applied`／`cal_rejected`／`cal_malformed`。伺服版大約多 4 KB（double 除法函式庫）。
- 電腦端 `mearm_calibration_link.py`：編碼前先用 Path B 驗證，每 8 個位元組停 3 ms 分段送出（韌體每圈只讀一個位元組）。`send_calibration_to_board` 送出後看 diag 行的 `cal_applied` 有沒有增加來確認；被拒或逾時就明說「實體手臂沿用原本的校正」，不會讓程式停下來。
- `--mearm`：加 `--skip-calibration` 時讀校正檔再送出；不加時做 4 個姿勢的互動校正，存檔後送出。順序：EMG → 校正 → 送出並確認 → R。人形手臂：校正完（讀檔或現場校正）也送給板子，人形手臂本身的計算不變。
- 人形手臂的 `_capture_window`／`_calibrate_pose` 原封不動搬成模組層級的 `capture_window`／`calibrate_pose`（原本取用的外層變數改成參數，用 AST 檢查過沒有未定義名稱），`main()` 改成呼叫它們；`MIN_CALIBRATION_TILT_DEG` 也搬到模組層級。人形手臂的測試全部通過，包括「感測器出錯時重錄」。
- 順便修掉一個原本就有的問題：`--mearm` 結束時從來沒有關序列埠，測試中前一次的讀取執行緒會繼續讀、印到後面的測試裡。
- 測試：C++ 解析器與套用 15 個；Python 編碼 6 個，9/13 的訊息和 C++ 測試逐位元組相同；`--mearm` 3 個（讀檔送出且在 R 之前並確認、板子不確認時明說且照樣跑、新姿勢存檔後送出）；人形手臂 2 個。突變測試：C++ 解析器 8 種，第一輪 3 種存活（欄位數檢查和檢查碼重疊屬於多重保護；位數上限和雜訊字元的測試不夠精準，補測試後抓到）；套用 3 種全抓；Python 4 種，第一輪 2 種存活（測試等待時間比 diag 間隔短、假姿勢剛好等於原檔），修正測試後全抓。C++ 測試 223／218 個全過。
- **實機驗證還沒完成**：燒錄後板子停在開機錯誤（`blink_code(10)`）：用 SWD 讀到 `g_wake_result_elbow = 2`，也就是前臂 MPU6050（0x69）開機時位址沒有回應（I2C 沒 ACK），是硬體接線問題，韌體照設計停下來。

**校正傳輸的實機驗證：位元組被吃掉（2026-10-04，實機抓到的 bug，已修）**：使用者修好前臂 MPU6050 接線後（開機兩顆 wake_result 都是 0）做實機驗證。
- 第一次：**正確的校正也被判成格式錯誤**（`cal_malformed` 0→1、`uart_rx_overruns` 0→2）。原因：韌體每跑一圈主迴圈才讀一個收到的位元組，接收暫存器只能放一個位元組，主迴圈某些圈比較慢時，下一個位元組先到就把前一個蓋掉（overrun）。「每 8 個位元組停 3 ms」太快。同一輪「故意改錯一位」的訊息有正確拒絕，但「Path B 不合法」那則也因為掉字變成格式錯誤，等於沒測到。
- 量節奏：**每 1 個位元組停 2 ms**（一則約 0.3 秒）連續 3 次全部套用、沒有 overrun；**每 1 個位元組停 1 ms** 3 次中 1 次掉字。改成每次 1 個位元組、間隔 2 ms（`CHUNK_BYTES = 1`、`CHUNK_PAUSE_S = 0.002`，註解寫上量測結果）。
- `send_calibration_to_board` 加上重送：板子回報**格式錯誤**（途中掉字，屬於傳輸問題）時重送，最多 3 次（`CAL_SEND_ATTEMPTS`）；板子回報**內容不對**（`cal_rejected`）時不重送，因為再送一次結果也一樣；逾時（舊韌體）也不重送。
- 測試（先寫）：送出時每次 1 個位元組、間隔至少 2 ms；假板子第一則掉字 → 重送一次後套用；一直掉字 → 送 3 次後明說沿用原本的校正；內容被拒 → 只送 1 次並說被拒。`run_preview` 的假序列埠不會 overrun，所以把間隔設為 0，避免 0.6 秒的傳送讓「中途感測器出錯」的劇本測試時間錯開（這兩個測試一開始因此失敗）。突變 7 種，第一輪「被拒的判斷拿掉」存活（沒有被拒的測試），補測試後全抓。舊的時序不穩定測試 `test_a_fault_mid_session...` 整批跑又失敗一次，單獨跑 3 次都通過。
- **實機結果**：正確的校正連送 5 次，5 次都套用（每次約 1 秒，含等 diag 確認），`uart_rx_overruns` 不變；故意改錯一位 → `cal_malformed` +1；完整但 Path B 不合法（FORWARD = HANG，沒有傾斜）→ `cal_rejected` +1，`cal_applied` 不變（韌體不換校正）。
- 以後如果還要更穩：改用 RXNE 中斷加環形緩衝區（不再依賴主迴圈速度），但要動向量表和 NVIC，暫時不需要。

**底座：舉手時會跟著轉（2026-10-04，討論＋錄製工具，還沒改控制）**：使用者覺得底座角度不穩，懷疑是舉手時上臂習慣性的轉動，問能不能改用前臂那顆。我不建議：前臂讀數 = 上臂方向 + 手肘彎曲 + 手腕轉動，干擾只會更多（demo 第 4、5 步都會彎手肘、轉手腕）。討論結論：
- 各方向舉手的自然轉動都不同（使用者提出；生物力學的 Codman's paradox），所以「扣掉固定的自然轉動」或做方向表都不可行，也容易過擬合。
- 根本限制：上臂加速度計看不到水平方向，「刻意的轉」和「順便的轉」是同一種物理轉動，分不出來，只能在兩種誤差中取捨。
- 使用者傾向**「舉手時底座慢慢跟」**（不是完全不跟）：斜向動作（左下→右上）途中底座會落後，停下來後追上，最後位置正確。要注意：追上時舉手帶來的轉動也會一起回來，所以它減少的是「舉手途中底座亂動」，不是消除那個轉動。
- 先錄原始資料再決定門檻：`capture_arm_motion.py --set base_raise`（往前／左前／右前各舉放 2 次、伸平後只左右擺、伸平靜止、驗證用的左下→右上斜舉），約 1.5 分鐘。先寫測試（4 個），原本的 1€ 調參組不變（預設）。

**底座的舉手資料分析（2026-10-04，`data/arm_motion_20261004-021240.json`，用板子目前的 9/13 校正）**：
- 靜止伸平 8 秒：底座 1289–1356 µs（範圍 67 µs，約 2°），**靜止時很穩，不是問題**。
- 不穩出現在**舉起／放下途中**，特別是剛離開垂下（傾斜 15–40°）時：往前舉，底座目標在 1152–1739 之間跳（到頂端後穩定在約 1490–1540）；往左前方舉跳到 533；斜舉跳到 1918。原因：剛離開垂下時，方位角對一點點側向偏移非常敏感。
- 不是校正過期：用今天垂下的原始資料換掉 9/13 的 HANG（只差 3.2°），跳動照樣在。
- 舉手和擺動可以用速度分開：舉起速度（傾斜角變化）的 90 百分位在舉手段是 53–69°/s，只擺動時是 10°/s，靜止時是 1.7°/s。
- 模擬「舉起中慢慢跟」（舉起速度 > 15°/s 而且大於擺動速度時，底座每秒最多動 S µs，否則照常）：往前舉的底座範圍 587 → 159（S=150）／265（S=300）；左前方 970 → 395／447；斜舉 996 → 613／622（1918 的突跳不見了）；只擺動時平均落後 2 µs（幾乎不受影響）；**每段結束時底座都回到和原本完全相同的位置**（差 0）。往右前方舉的範圍仍是約 690，因為那是真的往右（頂端 820–1000）。
- 附帶發現：使用者「往左前方舉」到頂端時方位角只有 −3°（底座約 1420，幾乎是正前方），可能是舉手時上臂的自然轉動抵消了方向；demo 的左擺是在伸平後擺動，這部分正常（只擺動時可到 1763）。

**底座：舉起中慢慢跟（2026-10-04，使用者選的做法，已燒錄、待實機試）**：`BaseRaiseFollow`（`mearm_input_filter.hpp`），伺服區塊在遲滯之後套用；只影響伺服，UART 照樣送原始讀數。
- 規則：過去 0.2 秒手臂的移動超過雜訊（3°/s），而且（上下移動多於側向移動，**或**還在垂下附近（傾斜 < 40°））→ 底座每秒最多動 150 µs；否則立刻跟上，所以停下來後底座一定回到原本會到的位置。
- 使用者指出「用速度判斷，慢慢舉就觸發不了」：模擬證實（放慢 5 倍時完全沒作用），所以改成看**移動方向**。
- 第一版只看方向，實際管線（含 1€）在往左前方舉時幾乎沒改善（982→879）：追蹤發現剛離開垂下時（傾斜 8–14°），手臂是**越過** HANG 方向，看起來像「側向」移動，底座先被拉到錯的值（787），之後慢慢跟反而把它留在那裡。所以加上「垂下附近也慢慢跟」：錄到的錯誤跳動全部在 40° 以下。
- 判斷改成**每一步**都和約 0.2 秒前的讀數比（固定大小的環形緩衝區，不配置記憶體），不是每 0.2 秒才決定一次——否則每次舉手的前 0.2 秒都沒有保護。
- 實際管線（1€ → drive → 遲滯 → 慢慢跟，10 ms 節奏）在錄到的資料上：往前舉 530 → 165 µs（放慢 3 倍 340、5 倍 429）；往左前方 983 → 182（327／679）；斜舉（只驗證）899 → 403；只左右擺 814 → 814（平均落後 2 µs）；靜止完全相同；每段停下 1 秒後，底座和原本完全相同。
- 測試（先寫，C++ 11 個，讀 `data/base_raise_20261004.csv`——從 `arm_motion_20261004-021240.json` 取出的上臂原始值，未修改）：真實舉手的晃動範圍至少降到 6 成；放慢 3、5 倍至少降到 8 成；擺動平均落後 ≤ 20 µs；靜止完全不變；停下後位置完全相同；斜舉只做驗證。突變 10 種：第一輪 2 種存活——`!initialized_` 確實多餘（第一筆本來就沒有歷史），已刪掉；「讀數無效後清掉歷史」補了測試後抓到。主機 C++ 測試 234／229 個全過。
- 錄製腳本 `--set base_raise` 加了一段「慢慢往前舉起放下」（約 10 秒舉到水平），用**真的**慢舉驗證（目前的慢速結果是把時間拉長模擬的）。

**底座放大倍數 `--base-gain`（2026-10-04，使用者要求試試看）**：我先用資料說明濾波器調強解決不了舉手時的亂跳（極強的 1€ 0.05 Hz：往左前方舉 983 → 876，但左右擺落後 74 µs 而且擺不到底），使用者接著想試降低底座放大倍數。做法：`mearm_real.with_base_gain(saved, gain)` 把目前的底座範圍（有量過的舒服範圍，否則預設的約 ±34°）除以 gain，存成範圍的原始向量（在 FORWARD 的傾斜角上合成），**隨校正訊息送給板子**，不用重新燒錄；只在送出的訊息裡，不存檔；不加 `--base-gain` 就恢復原本。gain 限制 0.4–2.0（低於 0.4 時底座的兩端會在身體後面，Path B 會淡出）。先寫測試：gain 1 完全不變、0.5 時範圍變兩倍且同樣轉動底座只動一半、量過的範圍也一起縮放、不合理的 gain 拒絕、原本的 dict 不被修改、`--mearm --base-gain 0.5` 送出的訊息等於 `encode(with_base_gain(...))` 且校正檔沒有被寫入。突變 7 種抓到 6 種；存活的是「合成向量用哪個傾斜角」，底座只用向量的方位角，所以是等價突變。實機確認還沒做（序列埠正被使用者的 run_demo_live 占用）。
- **同一天移除**：使用者實機試過 `--base-gain 0.67`，覺得沒有什麼用，整個拿掉（`with_base_gain`、`--base-gain`、相關測試）。結論：底座的問題靠「舉起中慢慢跟」處理，不靠降低放大倍數。

**開 PR #5 前後的全面檢查：沒測到的程式、過時的文件（2026-10-04）**：用覆蓋率工具實際量（C++：clang `-fprofile-instr-generate`；Python：coverage.py，跑 CI 的全部測試清單），不是用猜的。
- C++ 幾乎全部 100%。沒跑到的：`shoulder_ctrl_at_pulse` 是**死碼**（C++、Python 都沒人用），已刪掉（韌體大小不變，本來就沒編進去）；`smooth_elbow_window` 最後一行只有肩膀值是 NaN 才會到，但肩膀值來自 `env.clamp()` 的整數，到不了，屬於防禦性程式碼，不加測試；`make_compiled_base_reach` 的「有量過底座範圍」分支取決於編譯時的資料（目前沒量過）。
- Python 補了 4 處（每處都用突變確認測試真的抓得到）：
  - `check_hardware_ready.check_boot_reached_app`（CLAUDE.md 要求每次都跑的 `--boot-check`）原本**完全沒有測試**：補 4 個（程式在跑→通過；bootloader 還在跑→繼續等；一直卡在 WeAct bootloader→失敗並印出 SWD 跳轉指令；卡在原廠 ROM bootloader 不能被誤判成程式在跑）。突變 4 種全抓。順便把它過時的說明（「只有拔插才有用」）改成現況。
  - Path B「FORWARD 離 HANG 不到 20° 就拒絕」：原本有 FORWARD == HANG 的測試，但它是在更早的正規化步驟就出錯，**從來沒有走到 20° 的檢查**；補了 10° 的案例並比對錯誤訊息。
  - `--mearm` 重新校正後，舊的 MuJoCo 對齊資料不能再用：補測試。
  - `test_gen_calibration_header` 原本拿編譯進去的校正和**本機（gitignore）的校正檔**比，使用者 01:51 重新校正後就失敗，CI 裡又永遠跳過。改成從已提交的 golden 校正產生並檢查（數值不變，只改註解），CI 也會跑。
- 刻意沒補的：互動式、只在實機上跑的工具主程式（`watch_*`、`servo_limit_finder.py`、`measure_*`／`capture_arm_motion` 的 `main()`）、舊的資料集轉換工具 `convert_epn612.py`、需要視窗的 `test_arm_kinematics.py`（所以不在 CI）。
- 文件：README 伺服段落原本還寫著「底座固定 1500、stretch 模式、校正要重新產生並重新燒錄、燒錄後要斷電重開、極性還沒實機確認」——全部改成現況（底座看上臂旋轉、height_reach、校正經 UART 送、SWD 跳轉、極性都已實機確認）；工具表的 `run_demo_live`／`capture_arm_motion` 說明更新；`firmware/README.md` 補上四個伺服韌體目標和伺服版的說明；`run_demo_live.py --mearm` 的 help 和說明文字原本寫「不會自己做校正」，已更正。PRD 的路線圖（Phase 3/4 進度）沒有動。
- **PR #5 的 CI 失敗一項（已修）**：`test_the_preview_starts_by_itself_once_the_sensors_become_healthy` 在 CI 上沒看到「硬體異常」。原因是測試寫成「前 200 次讀取是凍結的資料」，但健康檢查開始前，程式啟動期間讀取也在進行；CI 機器較慢，凍結的部分在檢查開始前就被消耗掉了。改成「一直凍結，直到程式真的印出警告為止」（不印警告就會一直凍結、逾時失敗，所以仍然有意義）。驗證：在健康檢查前人為延遲 1 秒模擬慢機器，舊寫法失敗、新寫法通過。同類的 `test_a_fault_mid_session_holds_the_model_warns_and_recovers`（之前在本機整批跑偶爾失敗）依賴「讀取次數」和「畫面格數」兩個獨立節奏，要改測試結構才能穩定，這次先不動。

**EMG 門檻倍數 K（15 還是 20）用歷史校正紀錄驗證（2026-10-04）**：`emg_calibration_logs/` 有 30 次校正，每次都存了放鬆和用力各約 2 秒的原始 `emg_max`（另有 `emg_min`）。對每次算 K 的可行範圍（照韌體的判斷：連續 0.15 秒）：
- **K 上限（握拳抓得到）**：23 次正常的校正，用比較接近韌體實際比較值的 (min+max)/2 算，最緊的是 22.7（10/03）、27.9、30.8（10/04 13:12）；9/13 的都在 73 以上。**15 和 20 在所有正常校正裡都抓得到**；10/04 的上限比 9/13 低很多（放鬆的基準值高、雜訊大）。
- **K 下限（放鬆不誤觸）**：所有紀錄都低於 2——但這只代表「靜止放鬆的 2 秒」不會誤觸。9/12 發現 K=2 太敏感是在**實際使用中**（手臂在動），校正紀錄裡沒有這種資料，所以**這一半歷史資料驗證不了**。
- 7 次 SUSPECT 是用力時的值不比放鬆高（例如 10/04 12:58 放鬆 3546、用力 1644），任何 K 都救不了，是貼片／操作的問題。
- 結論：維持 20（握拳那邊有餘裕，最緊的一次也有 1.1 倍以上）。要驗證「不誤觸」那一半，需要錄「手放鬆、手臂照 demo 動作移動」時的 EMG。

**EMG 抓握改成兩個門檻（遲滯，2026-10-04，已燒錄）**：使用者說握住後很容易放開。資料：目前的門檻 1876（10/04 13:12 校正），那次放鬆約 1170，用力時最低的 10% 只有約 1908，已經有 5% 的時間低於門檻；實際 demo 握著膠帶移動時出力更小，一掉到門檻下 0.15 秒就放開。改法（myoelectric 開關常用的遲滯做法）：握住仍要超過原門檻；握住後要低於**放開門檻**才放開，放開門檻 = 放鬆平均 + 0.5 ×（門檻 − 放鬆平均），例如 1523。
- 韌體：`GripStateMachine::set_thresholds(on, release)`（release 不會高於 on；`set_threshold` 等於兩個相同，行為和以前完全一樣）；`T` 指令的解析搬到 `emg_threshold_link.hpp`（主機可測），接受 `T<on>\n`（舊格式）或 `T<on>,<release>\n`，格式錯誤整行丟掉（以前會忽略雜字繼續解析）。
- 電腦端：`calibrate_emg_threshold` 算出放開門檻、一起送出，回傳 `EmgThreshold`（仍是整數，多一個 `.release`），校正檔多存 `emg_release_threshold`；`--skip-emg-calibration` 讀到就一起送，舊校正檔沒有就只送門檻並提示重新校正。人形手臂和 `--mearm` 共用。
- 用真實校正紀錄檢查（10/03–04 的 7 次正常校正，用 (min+max)/2 近似韌體比較的值）：用力 2 秒內被誤判放開，一個門檻 2 次 → 兩個門檻 0 次；放鬆時 7 次都能放開。限制：紀錄裡只有用力握 2 秒，demo 是較輕、較久的握持，要實機確認。
- 測試先寫：C++ 狀態機 5 個、解析器 5 個；Python 6 個。突變：C++ 10 種、Python 6 種全抓。C++ 測試 244／239 全過，韌體所有目標可編譯。
- 實機：燒錄、SWD 啟動後送 `T1876,1523`，用 SWD 讀板子記憶體裡的 GripStateMachine：門檻 1876、放開門檻 1523。
- 注意：使用者目前的校正檔是改版前的，沒有放開門檻，要不加 `--skip-emg-calibration` 重做一次 EMG 校正。
- **實機驗證通過（2026-10-07）**：使用者用實際的 demo 動作測試雙門檻，回報通過。

**錄製工具加上 EMG（2026-10-04，`capture_arm_motion.py --set emg`）**：使用者想改善夾爪（EMG）的穩定度。查了文獻：雙門檻（遲滯）就是單肌肉開關控制的標準做法，另一個對話今天已經做了。鎖定模式（握一下夾住、再握一下放開）使用者決定先不要。改用「放鬆＋最大出力」來定門檻（MVC 正規化）列為之後的選項。TKEO 需要原始 EMG，而我們接的是 ENV 輸出，不適用。De Luca 建議的 20 Hz 高通，MyoWare 硬體本身就有（20.8 Hz），不需要再加。EMG 校正紀錄（`emg_calibration_logs/`，40 筆）只有手臂靜止時的資料，驗證不了「手臂在動時會不會誤觸或誤放開」，所以把 EMG 加進錄製工具：
- 每一筆資料多存韌體 tick 行裡的 `emg_min`／`emg_max`（10 ms 視窗），存檔時欄位叫 `emg`。原本加速度的分析不受影響。
- 新的 `emg` 組（約 76 秒）：手放鬆垂下、**手放鬆照 demo 移動**（看會不會誤觸）、輕握不動、**輕握照 demo 第 5–7 步移動**（看會不會誤放開）、用力握，以及驗證用的握放重複。
- `emg_events`：照韌體 `GripStateMachine` 的邏輯重播（從放開開始；高於抓握門檻 0.15 秒才算抓握；抓握中低於放開門檻 0.15 秒才放開），比較的值用每個 10 ms 視窗的 (min+max)/2 近似韌體的 EMA。門檻從 `shoulder_calibration.json` 讀取。摘要會列出各段 EMG 的範圍、高於／低於門檻的時間比例、誤觸次數和誤放開次數。
- 先寫 14 個測試。第一次跑時「全部通過」是假的：新測試加在 `unittest.main()` 之後，根本沒被執行。把 main 移到檔案最後，看到它們失敗後才實作。突變 10 種全部抓到（「放開沒有防抖」和「用峰值而不是中點」兩種，補測試後才抓到）。
- 注意：目前校正檔裡的 EMG 門檻（1509／1054）來自 14:53 那次 SUSPECT 校正（用力時的平均值 1316 低於門檻）。
**EMG 移動中實測：問題在訊號，不在演算法（2026-10-04，`data/arm_motion_20261004-152827.json`）**：用 `--set emg` 錄製，門檻用目前校正檔的 1509／1054。
- 手放鬆、垂下：EMG 最高 742，誤觸 0 次。**手放鬆、照 demo 動：最高 3848，28% 的時間高於門檻，誤觸 4 次。**
- 輕握不動：最高 1160，抓不到。**用力握不動：最高只有 1318，連門檻都到不了。**
- 輕握照 demo 動：最高 3836、31% 高於門檻；但這和手放鬆時一樣，是手臂移動造成的，不是握拳。
- 逐秒看：高 EMG 只出現在手臂移動的段落，和有沒有握拳無關。和瞬間速度的相關係數接近 0（−0.03～+0.02），所以也不單純是速度。
- 結論：目前的 MyoWare 對握拳幾乎沒反應，對手臂移動反應很大，任何門檻都分不開。可能原因：電極貼的位置主要量到抬手臂的肌肉，或是線被拉扯、電極滑動造成的動作雜訊（約 3800 已接近 ADC 上限）。建議電極改貼前臂內側的屈指肌（手肘下方約三分之一處，順著肌肉走向），參考電極貼在骨頭上，把線固定好、換新貼片，再用同一組流程重錄，確認「用力握不動」明顯高於「放鬆照 demo 動」。

**MeArm「握拳時加強平滑」：模擬後暫不加（2026-10-04）**：使用者要求 MeArm 也要有人形手臂的 `GRIPPING_SMOOTHING_ALPHA`。注意人形手臂的前提「握拳時手臂不動」對 demo 不成立（第 5–7 步握著膠帶抬起、右擺、放下），所以設計成只把 1€ 的靜止截止頻率降 3 倍（beta 不變）。先寫測試，結果：照 9/12 實測的握拳顫抖（約 9.5 Hz、手臂角度約 0.48°）模擬，**現有的 1€（0.5 Hz）加底座遲滯已經讓底座完全不動（0 µs）**，手肘伺服也只有 0.99 µs（抖 1° 時 1.46 µs），加強後 0.80 µs——顫抖頻率是截止頻率的約 20 倍，早就被濾掉。所以先不加，改動已還原（patch 留在 scratchpad）。這是模型，不是實機資料；要確認就用 `capture_arm_motion.py` 預設組的握拳靜止段錄真實資料。
**EMG 校正改成「放鬆＋不斷快速收縮」（2026-10-04，使用者的設計，第 1 步，共 2 步）**：使用者發現用手臂肌肉快速收縮，比一直握著更容易判斷：放鬆時很平（約 400，起伏不到 20），收縮時會一直出現尖峰（600–3100）。使用者希望先把校正改好、錄到真實資料，再回頭改韌體的判斷演算法。
- `calibrate_emg_threshold`：第二段改成「不斷快速收縮」（4 秒，取最後 3 秒）。門檻 = 放鬆時的最高值 + 30% × (典型尖峰 − 放鬆最高值)，典型尖峰取每 0.3 秒內最高值的中位數。抓握和放開先用同一個門檻（送 `T<th>,<th>`）。如果典型尖峰沒有比放鬆高，或少於 60% 的 0.3 秒區間有超過門檻的尖峰，就警告並標成 SUSPECT。
- 校正紀錄多一個欄位 `contraction: "pulses"`，原始的收縮資料完整保存，給第 2 步分析用。
- 用使用者貼的真實資料當測試資料：放鬆最高 418、典型尖峰 1533 → 門檻 752，100% 的 0.3 秒區間都有尖峰超過門檻。
- 先寫測試（7 個），舊的「K × 標準差」測試改成新公式。突變 8 種全部抓到，其中「提示文字」和「每個視窗取最高值」兩種補測試後才抓到。已加入 CI。
- **第 2 步（還沒做）**：韌體目前在低於放開門檻 0.15 秒後就放開，兩次收縮之間的空檔會被判成放開（用同一份真實資料重播已確認）。要用新的校正紀錄決定「多久沒有尖峰才算放開」，再改韌體。另外也要確認手臂移動時的雜訊不會被當成收縮。
- **同一天撤回**：使用者決定這個做法不太好，改回原本的校正（放鬆 + 握拳保持，K × 標準差，搭配另一個對話加的雙門檻）。程式碼、新的測試檔、CI 那一行都已還原；`capture_arm_motion.py --set emg` 和它的重播工具保留。
**MyoWare 能不能濾掉移動雜訊（2026-10-04，研究＋用資料驗證）**：
- MyoWare 2.0 的硬體已經做了標準的處理：差動放大（共模干擾互相抵消）、20.8 Hz 高通（就是 De Luca 2010 建議用來去除移動雜訊的 20 Hz）、498 Hz 低通、整流後 3.6 Hz 包絡（ENV）。所以硬體這一層沒有再加強的空間。
- 用 `data/arm_motion_20261004-152827.json` 試「手臂在動時不改變夾爪狀態」（用上臂和前臂速度判斷）：手放鬆照 demo 動的誤觸，從 4 次（不擋）→ 4（1.0 g/s）→ 3（0.5 g/s）→ 2（0.3 g/s）。剩下 2 次發生在手臂幾乎不動時，表示高 EMG 不全是移動雜訊，比較像撐住手臂姿勢時肌肉真的在出力。加上用力握拳只到 1318、撐手臂可到 3800，任何濾波或門檻都分不開，真正能改善的只有電極位置和固定。這個「手臂在動時不改變狀態」可以之後當第二層保護，目前沒有實作。
- 參考電極：MyoWare 2.0 背面有 MID、END、REF 三個按扣，一般直接按在感測器上就好。參考線（Reference Cable）的感測器端是壓接針，插進感測器側邊的專用插座（不是杜邦頭），身體端是按扣，接一片電極貼在骨頭上（例如手肘）。官方 Advanced Guide 建議參考線搭配 Cable Shield 使用；前臂肌肉較小，把參考電極移到骨頭上會比較乾淨。電極放在肌肉最鼓的地方、順著肌肉纖維方向。
**接線確認（2026-10-07）**：
- 完整腳位表（對照 `firmware/README.md`、`README.md` 和韌體）：PA0＝MyoWare ENV；PA2／PA3＝USART2 TX／RX（USB-TTL 交叉接）；PA6／PA7／PB0／PB1＝伺服底座／肩膀／手肘／夾爪（TIM3 CH1–4）；PB6／PB7＝I2C1 SCL／SDA（兩顆 MPU6050 並聯）；PA13／PA14＝SWDIO／SWCLK；PC13＝板上 LED。3.3V 給兩顆 MPU6050 Vin、前臂 AD0、MyoWare VIN；上臂 AD0 不接（0x68），前臂 AD0 接 3.3V（0x69）。ST-Link 只接 SWDIO／SWCLK／GND；USB-TTL 只接 TXD／RXD／GND。
- ST-Link 的 RST（NRST）不用接：`firmware/openocd.cfg` 沒有設定 `reset_config`，OpenOCD 是透過 SWD 送 SYSRESETREQ 軟體重置，不會用到那條線。韌體也沒有改掉 PA13／PA14 或讓晶片進深度睡眠，所以用不到 connect-under-reset。燒錄後卡在 bootloader 的問題，接 RST 也解決不了。
- 燒錄時 OpenOCD 顯示 target voltage 3.26 V，可能代表 ST-Link 的 3.3V 腳也接到了板子；文件規定不要接（板子由自己的 USB 供電，避免兩個電源同時接在 3.3V 上）。已提醒使用者確認。
**感測器接線從麵包板換成 WAGO（2026-10-07）**：使用者把麵包板拿掉，3.3V、感測器 GND、SDA、SCL 都改成 WAGO 分線（伺服電源之前已經是 WAGO）。
- **換完後 `run_demo_live.py` 收不到資料**：用 SWD 讀到 PC 一直在 `delay()`，也就是 `blink_code()` 的無窮迴圈；`g_wake_result_shoulder = 2`（開機喚醒時送出 0x68 沒有 ACK），所以韌體停在閃燈報錯，不會進主迴圈。使用者照清單用導通檔逐條確認接線後，`--i2c-scan` 顯示 0x68 和 0x69 都有回應。
- **踩到的坑**：`check_hardware_ready.py --i2c-scan` 會把板子上的韌體換成掃描程式，掃完一定要燒回 phase3_control_loop（並用 SWD 跳轉啟動），`run_demo_live.py` 才會有資料。燒回後兩顆的喚醒結果都是 0。
- **穩定度量測**（SWD 讀累計計數，或關掉 run_demo_live 後讀 diag 行）：
  - 第一次（run_demo_live 開著時，60 秒）：I2C 逾時後匯流排自我恢復 **1110 次／分鐘**（約每秒 18 次），逾時都發生在 WaitAddr1（送出位址後等不到 ACK，SR1=0），代表 SCL／SDA 訊號不乾淨或接觸時有時無；兩顆都沒有重開機，PWR_MGMT_1 都是 1。
  - 之後板子重開機（累計歸零），20 秒靜止：兩顆都 0 次 NACK、0 次逾時，每秒各完成約 274 次讀取。
  - 35 秒（約 28 秒在移動）：只有 1 次 NACK（上臂）、0 次逾時。
  - 再 35 秒：0／0。但兩次量測之間，匯流排自我恢復累計從 1 增加到 265，表示偶爾還是有一陣集中的斷線。
- **搖線測試**：用 `watch_imu_raw.py` 一次只搖一條線或一個接點（SDA／SCL 的 WAGO 和線、兩個 MPU6050 的排針、Black Pill 上 PB6／PB7 的杜邦頭、3.3V／GND 的 WAGO），使用者找到問題並修好（使用者回報「成功了」）。
- 和麵包板時期比較：9/28 上臂接觸不良時大約每秒 15 次 NACK；換成 WAGO、修好接點後，移動中 28 秒只有 1 次。
- 心得：WAGO 能讓接點更牢，但杜邦線（26–28 AWG）比 WAGO 221 最小能夾的線（0.2 mm²，約 24 AWG）還細，看起來插進去了，其實可能時有時無；I2C 訊號線對這種接觸不良最敏感。

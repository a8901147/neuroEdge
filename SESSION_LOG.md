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
2. `PRD.md` 累積了 5 份 Session Handoff、超過 500 行，考慮整理（拆檔或精簡舊記錄），這個 session 還沒決定要怎麼做。

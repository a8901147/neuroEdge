# Product Requirements Document (PRD)

**Project Name:** EdgeNeuro: Modular & Zero-Allocation Real-Time BCI Engine
**Author:** Chang-Jui Tseng
**Target Audience:** 歐洲計算神經科學與神經義肢 PhD 招生委員會、開源 BCI 開發社群
**Date:** August 2026

## 1. 產品概述 (Executive Summary)

EdgeNeuro 是一套專為邊緣運算與神經義肢控制設計的 C++ 即時腦機介面（BCI）數位訊號處理管線。本專案透過現代 C++ (C++20 Concepts & Templates) 實現**編譯期依賴注入**，在達成「零動態記憶體配置」與「次毫秒級延遲」的嚴苛硬體限制下，依然保有極致的演算法擴充彈性。本專案採取嚴格的「**階梯式品質閘門與開發紀律**」：**首要戰鬥重心絕對鎖定於建構毫無瑕疵的 C++20 系統架構與確定性（Deterministic）即時計算引擎；唯有在首階段核心邏輯與零內存防線達到完美與極致滿意後，才穩步邁進**銜接生物物理力學模擬金標準 — **MuJoCo** 3D 仿生神經義肢仿真以及最終的 STM32 物理晶片致動。全案堅決排除繁複的網頁與切版虛耗，高度聚焦與歐洲殿堂級計算神經科學與神經機器人（Neurorobotics）實驗室接軌的研究深度。

## 2. 解決痛點 (Problem Statement)

目前神經義肢在演算法驗證與邊緣部署上存在三個核心斷層：

1. **硬體限制與記憶體崩潰：** MCU 等無 OS 設備無法承受 Python/高階語言的垃圾回收，傳統 C++ 若在 Hot Loop 頻繁調用 `new`/`malloc` 極易引發記憶體碎片化。
2. **缺乏零開銷的模組化：** 傳統嵌入式韌體通常將演算法「寫死」，若要具備抽換演算法的彈性（如傳統 OOP 的虛擬函式），往往會帶來指標尋址開銷與 CPU 快取未命中（Cache Miss）的效能懲罰。
3. **展示與重現門檻過高：** 評委與研究人員往往需要建置龐大的本地端環境才能測試演算法，缺乏視覺化且直覺的端到端（End-to-End）驗證工具。

## 3. 系統架構設計 (Architectural Design)

系統採用**策略模式 (Strategy Pattern)** 與 **C++20 Concepts** 進行解耦，確保所有抽象化均在編譯期完成（Zero-Cost Abstraction）。整體管線支援數值空間彈性（例如 `float` / `int32_t`）並以**模板化通道參數 `template <size_t EmgChannels, size_t ImuChannels>`** 實現跨級別縮放：
* **模式 A (穿戴義肢控制模式 - `EdgeNeuro<1, 6>`)**：結合 1 通道 EMG 肌肉收縮強度（掌控手掌抓握）與 6 軸 IMU 陀螺儀/加速度計（掌控手臂與手腕三維空間旋轉方位），展現先進的即時生物與慣性感測器融合 (Sensor Fusion)。
* **模式 B (極限壓力基準模式 - `EdgeNeuro<32, 0>`)**：支援 32 通道高密度表面肌電 (HD-sEMG) 陣列，專供嚴苛的效能基準壓測，驗證零動態記憶體配置的擴充極限。

> **⚠️ Phase 3 實際控制架構的修正（2026-08-17）：EMG 跟 IMU 不會共用同一個 `classifier_.classify()` 決策。** 上面「模式 A」把 EMG+IMU 全部通道併成一個 feature 向量、餵給同一個 `Classifier`、吐出一個 class index 的寫法，是 Phase 1 為了展示 `EdgeNeuro<>` 模板通用性（`EmgChannels`/`ImuChannels` 都是模板參數）刻意設計的展示情境，**不是** Phase 3 真實產品要的控制邏輯。真實需求（使用者親口確認）：陀螺儀貼在使用者真手上，即時、連續帶動假手的上下左右方位（不能等 window 存滿才動，`WindowSize=200` 在 1kHz 下等於 200ms 延遲，會非常卡頓）；EMG 則是偵測「持續用力一段時間 → 抓」「持續放鬆一段時間 → 放」，本質是離散事件觸發，累積一段時間判斷才有意義，用現有 `Pipeline` 的 window/feature/classify 機制(或簡化成純粹的持續時間閾值狀態機)沒問題。**結論：Phase 3 會是兩條獨立控制迴路**——IMU 完全繞過 `EdgeNeuro<>`/`Pipeline`，`ComplementaryFilter` 算完 roll/pitch 直接送去方位致動器；EMG 繼續走 `Pipeline` 的 window/classify(或簡化狀態機)。兩者共用同一顆 STM32、同一組感測器，但邏輯上不合併成一個 `classifier_.classify()` 呼叫。`ComplementaryFilter` 因此也不需要想辦法塞進 `ImuFilterT` 插槽——它本來就不該進 `Pipeline`。上面的模式 A/B 描述保留作為 Phase 1 引擎通用性的展示，Phase 3 韌體設計以本段為準。
>
> **EMG 這條路也不用 `LdaClassifier`，改用閾值+持續時間狀態機（2026-08-17）：** 確認 Phase 3 目前規劃只用**一顆** MyoWare 2.0（單通道），且接的是 `ENV`（Envelope）輸出腳位——感測器板子上的類比電路已經做完整流+低通濾波，輸出本身就是可以直接跟閾值比較的「目前出力大小」訊號，不需要再用 `IirFilter`/`MavFeature`/`RmsFeature` 這套 DSP 鏈重新萃取特徵。更根本的原因：單一 EMG 通道物理上只能表達「這條肌肉出力多大」一個維度的資訊，無法像多通道那樣分辨不同手勢，`LdaClassifier` 這種多手勢分類器在單通道情境下沒有實質效益，只換來 window（例如 200 samples@1kHz=200ms，且是不重疊 window，最差反應延遲逼近 400ms）造成的固定延遲。改用簡單的閾值+遲滯（hysteresis）狀態機——持續超過閾值 T_on 毫秒才觸發「抓」、持續低於閾值 T_off 毫秒才觸發「放」——這也是市售肌電義肢常見的實際做法，不是自創，延遲可以做到 50–150ms 等級，遠低於 window 分類器。**`LdaClassifier`/`Pipeline` 架構不刪除**：以後如果真的加裝第二、第三顆 MyoWare 貼在不同肌肉、想做多手勢辨識，屆時多通道的組合模式才是這套架構真正該發揮的地方，現在只是不在這條路上用它。

| 模組分層 | 職責與技術規格 | 實作要求 |
| --- | --- | --- |
| **Provider Concept** | 抽象化訊號供應層（Signal Provider）。 | 確保 Host 端（`CsvSignalProvider`）與 Target 端（`Stm32AdcProvider`）以相同介面注入 Pipeline，無需修改核心商業邏輯。 |
| **Pipeline Core** | 負責資料流調度與記憶體管理。 | 使用 `std::array` 與 Template 實作 Lock-free SPSC Ring Buffer（拒絕任何 `std::vector` 進入 Hot Loop），定位為 Provider 內部的生產者/消費者交接機制：`Stm32AdcProvider`（Phase 3）由 ISR 端 `push()`、主迴圈端 `pop()` 真正非同步運作；Host 端 `CsvSignalProvider` 為單執行緒同步播放，不經過 Ring Buffer。Phase 1 即在 Host 端以雙執行緒模擬 ISR/主迴圈情境，搭配 TSan 提前驗證其併發正確性，確保元件在 Phase 3 上線前已被證實無誤。 |
| **Filter Concept** | 訊號預處理與空間濾波。 | 必須實作 `process()` 與 `reset()` 介面。EMG 與 IMU 通道採獨立濾波策略（`EmgFilterT` / `ImuFilterT`），反映兩者截然不同的頻譜特性（EMG 需 20–450Hz 頻帶濾波，IMU 通常僅需輕度平滑或 Pass-Through）。初始提供 IIR (Direct Form II) 與 Pass-Through 策略。 |
| **Feature Concept** | 提取時域或頻域特徵。 | 初始實作 MAV (Mean Absolute Value) 與 RMS (Root Mean Square) 模組。 |
| **Classifier Concept** | 輕量級意圖解碼。 | 初始實作基於矩陣乘法之 LDA 分類器，權重於初始化階段靜態載入。 |

## 4. 產品功能需求 (Functional Requirements)

### 4.1 C++ 邊緣運算引擎

* **模組隨插即用：** 開發者僅需修改一行 `using Engine = Pipeline<FilterA, FeatureB>` 即可在編譯期無縫切換演算法，不影響底層 Ring Buffer 運作。
* **確定性記憶體：** 系統初始化後，記憶體水位線必須呈現絕對水平，動態配置量為 0 Bytes。

### 4.2 資料流與感測融合模擬器 (Multi-Modal Stream Simulator)

* 提供多模態生理與慣性訊號的 `csv` 解析器，欄位配置相容 Ninapro EMG 切片與慣性運動序列格式。Phase 1 隨附之測試資料集為合成訊號（`tools/generate_sample_data.py` 產生，統計特性模擬肌肉收縮/放鬆週期），因 Ninapro 原始資料需另行簽署使用授權、不便內建於 repo；解析器本身與資料集無關，直接餵入真實 Ninapro CSV 匯出檔即可運作。
* **感測器融合封包：** 支援 `1-Ch EMG + 6-Axis IMU` 同步資料流，精確模擬 $1000\text{ Hz}$（每 $1\text{ ms}$ 推播一次）之零動態配置管線輸入。
* **高密度擴充測試：** 相容高密度 `32-Ch EMG` 壓力測試數據格式，供自動化基準檢驗使用。
* **真實硬體資料集相容性驗證：** 額外以近年公開資料集 [EMG-EPN-612](https://zenodo.org/records/4421500)（Myo 臂環實測錄製，8 通道 EMG + 加速度計/陀螺儀/方向四元數共 10 個 IMU 數值）驗證 `CsvSignalProvider` 與真實硬體資料的相容性（轉換工具 `tools/convert_epn612.py`；單一使用者檔案 298,710 列資料端到端驗證通過，Python 轉檔輸出與 C++ 端讀取數值逐一核對一致）。此驗證證實 EMG 與 IMU 在真實硬體上是**獨立時脈、非整數倍率**的兩條資料流（實測單一樣本 992 筆 EMG 對 249 筆 IMU，約 3.98:1，並非乾淨的 4:1），與嵌入式系統中 ADC（EMG）與 I2C 輪詢（IMU）天生異步的常態一致，而非我們原先同步取樣假設的簡化情境。解法是在資料準備階段依長度比例對較慢的 IMU 串流做 zero-order hold（沿用上一筆讀值直到下一筆按比例應該到達的時間點），C++ 引擎本身不需任何修改——這與 Phase 3 韌體主迴圈預期採用的策略（主迴圈跟隨 EMG ADC 節奏、IMU 每隔 N 個 tick 才刷新讀值）完全一致。此驗證聚焦於資料格式與架構相容性，暫不涉及分類準確度（分類器準確度驗證留待效能與零記憶體驗證完備後的後續階段，見第 6 節開發紀律）。

### 4.3 現成科研展示與 MuJoCo 神經義肢仿真 (Scientific UI & Neuroprosthetic Simulation)

* **演算法研發極大化：** 堅決屏除自建繁複網頁切版與維護 DOM 的工程虛耗，將主力心血 100% 投入在 C++20 即時 DSP 演算法與記憶體精簡優化。
* **階段一成果展示 (Turnkey Scientific Oscilloscope)：** 前期驗證只需直接利用開源業界高度最佳化的即插即用科研示波器或圖表組件，極速且穩定可靠地展示即時多通道 EMG 濾波波形、零 malloc 記憶體監測與次毫秒延遲儀表，以此為打磨核心引擎之基準。雙軌並行：Terminal ASCII 儀表（零額外依賴，適合純 CLI/CI 環境）與 ImGui + ImPlot 原生視窗圖形化示波器（真實線圖），兩者共用同一顆零配置引擎與同一份資料流，行為完全一致；後者透過 CMake FetchContent 引入、預設關閉（`EDGENEURO_BUILD_GUI_DEMO=ON` 才建置），不影響核心引擎的零依賴特性。
* **階段二升維擴充 (MuJoCo 3D 物理閉環仿真)：** 待核心引擎穩固且達到我們團隊 100% 的絕對滿意與認同後，才啟動對接 **MuJoCo (Multi-Joint dynamics with Contact)** 物理模擬引擎。將解碼意圖直鏈驅動 3D 仿生義肢模型（如 Shadow Hand 或 Adroit Hand），展現世界頂尖的毫秒級閉環動作控制與動力學反饋（Closed-Loop Neural Motor Control），並透過現成的 Wasm 框架靜態部署於 GitHub Pages 進行線上展示。



## 5. 工程標準與效能驗證 (Engineering Standards & Validation)

為確保達到大型開源專案的程式碼品質，本專案實施嚴格的 CI/CD 與驗證標準：

| 驗證項目 | 使用工具 | 通過標準 (Success Criteria) |
| --- | --- | --- |
| **單元測試與邊界檢查** | `Catch2` + `llvm-cov`（source-based coverage） | 核心數學、模組 Concept 介面測試覆蓋率 $> 90\%$。**已量測達成**：46 個 test case 涵蓋 `include/edgeneuro/` 全模組，line coverage 95.56%、region 95.56%、function 96.23%、branch 90.32%（排除第三方依賴與測試檔本身，2026-08-12 隨 `fusion/complementary_filter.hpp`（100% 覆蓋，7 個新 test case：6 個純物理量單元測試 + 1 個對照真實 EMG-EPN-612 資料的驗證）加入後重新量測）。未覆蓋行集中於 `bad_alloc` 拋出路徑（刻意不測試，需真實記憶體耗盡）與標準要求的 sized-delete overload（此 ABI 下從未被選中呼叫），以及 `NoHeapGuard` 致命 abort 路徑本身——該路徑由 fork 出的子行程觸發 `SIGABRT`，子行程在覆蓋率計數器落盤前即被終止，屬量測工具限制而非測試缺口，其正確性改由 `test_no_heap_guard.cpp` 以 `WIFSIGNALED`/`WTERMSIG` 驗證。量測過程中亦揪出兩個真實邏輯漏洞：`CsvSignalProvider` 原本會靜默截斷欄位數過多的畸形資料列而非拒絕；`firmware/` 的 bump allocator 原本未對齊記憶體、且耗盡時會拋進沒有 unwind table 的環境。兩者皆已修正並補測試證實。 |
| **零記憶體與安全防護** | `Clang/GCC Sanitizers` + 自訂 `NoHeapGuard` | 利用 ASan / UBSan / TSan 消除所有未定義行為與並發競態，並透過單元測試攔截驗證 Hot Loop 期間 `malloc_count == 0`。由於 ASan 與 TSan 無法連結進同一 binary，且自訂 allocator 覆寫會與 Sanitizer 自身的記憶體攔截機制衝突，兩類驗證拆分為互斥的 CMake Presets（`debug-heapguard` / `sanitize-asan-ubsan` / `sanitize-tsan` / `release-bench`）獨立執行，各司其職。 |
| **極限效能與擴充基準** | `Google Benchmark` | 同時檢測 `<1, 6>` 義肢融合模式與 `<32, 0>` 高密度壓測模式，驗證 32 通道連續運算延遲 $< 0.1\text{ ms}$ 且 `malloc_count == 0`。 |

## 6. 專案演化藍圖與開發紀律 (Strategic Roadmap & Progressive Discipline)

本專案奉行 **「厚積薄發、逐步升維」** 的開發紀律與「先 Host 後 Target」的工程戰略。Phase 1 的效能、記憶體 `0 Allocation` 驗證與 C++20 Concepts 抽象層已達到完全滿意且無懈可擊的標準，**品管閘門（Quality Gate）已於 2026 年 8 月開啟**。

**路線圖重新排序（2026 年 8 月）：** 原規劃為 Phase 1 → Phase 2（MuJoCo）→ Phase 3/4（STM32）線性推進。實際執行時發現一個關鍵盲點：Host 端量測到的 `malloc_count == 0` 只證明演算法架構本身沒有內在配置需求，**並不能證明目標硬體（STM32F401）上也是零配置**——ARM GCC 編譯結果、SRAM 是否夠用、真實 EMG 訊號品質，這些都是 MuJoCo 模擬完全不會碰觸、只有實體硬體才能回答的問題。同時，本專案的終極目標是「讓機械手臂動得順利、成本低」，MuJoCo 屬於錦上添花的仿真驗證與作品集素材，並非降低硬體風險的必要路徑。因此在 Phase 2 之前插入一個範圍明確、成本極低的 **Phase 1.5 硬體可行性驗證（Feasibility Spike）**，優先解決不確定性最高、資訊量最大的問題，才決定 MuJoCo 要投入多少心力——這不是推翻「先 Host 後 Target」原則，而是修正其執行粒度：MuJoCo 雖然也在 Host 端，但不減少硬體風險，不該與 Phase 1 的軟體架構驗證混為一談、無條件排在硬體接觸之前。

```
+-----------------------------------------------------------------------------------+
| ✅ 階段一主體：已達成，品質閘門已開啟 (Primary Core Focus - Wasm/PC Engine)    |
| [Phase 1] 純軟體與演算法核心：徹底淬鍊 C++20 Concepts 與 Lock-free Ring Buffer |
+-----------------------------------------------------------------------------------+
                                        |
                         ✅ 品質閘門：已開啟，Phase 1 驗證完畢 ✅
                                        v
+-----------------------------------------------------------------------------------+
| 🥇 階段 1.5：現階段主力目標，範圍明確的低成本可行性驗證 (Feasibility Spike)     |
| [Phase 1.5] 目標硬體零配置/記憶體足跡/真實 EMG 訊號品質驗證，不做完整致動      |
+-----------------------------------------------------------------------------------+
                                        |
                         ⚠️ 依 Spike 結果決定 Phase 2 / Phase 3 投入比重 ⚠️
                                        v
+-----------------------------------------------------------------------------------+
| 🥈 階段二擴充：生物力學與 3D 仿真升維 (Neurorobotics Sim - MuJoCo)                |
| [Phase 2] MuJoCo 神經義肢物理控制與 GitHub Pages 即插即用 Demo                  |
+-----------------------------------------------------------------------------------+
                                        |
                         ⚠️ 品質閘門：軟體與仿真完整驗證完畢，再著手物理打底 ⚠️
                                        v
+-----------------------------------------------------------------------------------+
| 🥉 階段三落地：實體晶片與物理微控 (Target Embedded Deployment - STM32)              |
| [Phase 3 & 4] STM32 HAL 對接、硬體在環 (HIL) 與馬達閉環調優                      |
+-----------------------------------------------------------------------------------+
```

### Stage 1: Host-Side Algorithmic Core & MuJoCo Simulation (零硬體 / 神經物理仿真階段)
在本階段完全不需要採購或串接硬體，同時拒絕浪費時間於瑣碎的網頁介面切版，全數依賴 C++20 核心編譯鏈與現成科學示波器/力學模擬引擎進行尖端工程與算法驗證。

* **Phase 1: 純軟體與演算法核心 (C++20 / PC) ✅ 【已達成，詳見第 5 節驗證數據】**
  * 定義 C++20 Concepts 介面合約（`SignalProvider`, `Filter`, `Feature`, `Classifier`）與數值型別模板（`ValueType`）。
  * 完成基於 `std::array` 與靜態記憶體的 Lock-free Ring Buffer 結構。
  * 實作 `CsvSignalProvider` 讀取公開資料集（如 Ninapro EMG 數值）來推播至系統，精準模擬即時感測器串流；並已額外以真實硬體資料集 EMG-EPN-612 驗證相容性。
  * 結合 `Catch2` 進行數學與邏輯嚴謹測試，使用 **Clang/GCC Sanitizers (`ASan`/`UBSan`/`TSan`) + `NoHeapGuard` 鉤子** 消除並發與越界隱患並嚴格證實 `malloc_count == 0`，配合 `Google Benchmark` 驗證延遲 $< 1\text{ ms}$。實測 `<1,6>` 模式 7.10ns/tick、`<32,0>` 模式 38.9ns/tick，四種建置設定（`debug-heapguard`/`sanitize-asan-ubsan`/`sanitize-tsan`/`release-bench`）全數通過，測試覆蓋率 95.38%（line）。
  * 運用現成的科研示波器工具鏈——快速 Terminal 儀表（`src/main.cpp`）與 ImGui + ImPlot 原生圖形化視窗（`src/gui_demo.cpp`）雙軌方案，極速呈現即時波形與零內存耗用圖表，**V1.0 自我檢視門檻已達成**。

* **Phase 1.5: 目標硬體可行性驗證 (Feasibility Spike) 🔥【當前主力目標】**
  * 確認目標硬體：**STM32F401RCT6 Black Pill 開發板**（Cortex-M4 @ 84MHz、256KB Flash、64KB SRAM、單精度硬體 FPU，與現行 `ValueType = float` 選型完全契合）+ **MyoWare 2.0**（1 顆，單通道，僅做開/合二元判斷）。
  * 刻意縮小範圍、不做完整 Phase 3：跳過 IMU/MPU6050 整合、PWM 致動、閉環控制、符合 `SignalProvider` concept 的完整 `Stm32AdcProvider`，只回答四個關鍵未知數：
    1. ✅ Host 已驗證的 C++20 核心用 `arm-none-eabi-gcc` 編譯後，`NoHeapGuard` 在真實硬體上是否依然 `malloc_count == 0`。**已於 2026-08-14 在真實 STM32F401 上實測通過**（見下方階段 2 燒錄記錄）。
    2. ✅ 完整 pipeline 的靜態記憶體足跡（`arm-none-eabi-size` 量測）是否塞得進 64KB SRAM / 256KB Flash。**已確認**——實測用量遠低於上限，且燒錄成功佐證量測準確。
    3. ✅ Timer + ADC 能否穩定達成 1kHz 取樣。**已於 2026-08-15 在真實 STM32F401 上實測通過**（見下方階段 3c/3d 記錄）。
    4. MyoWare 2.0 貼在真人前臂上的訊號品質，是否足夠支撐可靠的即時開/合判斷（唯一無法用合成資料或任何公開資料集回答的問題）。**條件式通過（2026-08-15，詳見下方階段 3e 記錄）**——訊號鏈整條路確認有真的在反應真人肌肉收縮，但動態範圍偏窄，增益調整留待之後處理，不視為阻塞項。
  * 建置工具鏈：裸 CMake + `arm-none-eabi-gcc`（延續 Host 端 CMake Presets 的風格，不引入 STM32CubeIDE），韌體專案位於 `firmware/`。透過 STM32F401 內建 USB DFU bootloader 燒錄，不強制要求 ST-Link；診斷手段以 LED 燈號與 UART/USB-CDC 輸出為主。

  * **硬體庫存狀態（2026-08-11）**：已有 STM32F401RCT6 Black Pill（已焊排針）、MyoWare 2.0 ×1。已下單但未到貨：ST-Link V2 相容品（蝦皮，約 NT$65）、CP2102 USB 轉 TTL 模組（樂意創客官方店，NT$85，附杜邦線，接腳 `3V3/TXD/RXD/GND/+5V`）。**避開 PL2303HXA 晶片款**（macOS 新版驅動不支援）。已有但目前用不到：MCP3008 SPI ADC、LM317 可調穩壓板、TXS0108E 邏輯電位轉換板、5V 繼電器模組（皆非本階段必需，未來若擴充或接馬達可能用得上）。**MPU6050/GY-521 已購入（2026-08-15）**，尚未接線/整合進 Phase 1.5 韌體。伺服馬達、機械手掌結構件、獨立電源尚未購買，刻意留到 Phase 3 才處理。

  * **韌體專案現況（`firmware/` 目錄，尚未 commit 進 git）**：
    - `cmake/arm-none-eabi-toolchain.cmake`：Cortex-M4F 交叉編譯設定（`-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard`）。
    - `linker/STM32F401RCTx_FLASH.ld`：Flash 起始位址 **`0x08004000`**（非 `0x08000000`）、LENGTH 240K（非 256K），RAM 64K@0x20000000。**原因**：這片板子出廠已預燒 WeAct Studio 的 HID bootloader，佔用 Flash 開頭 16KB（`0x08000000`–`0x08003FFF`，經 USB 列舉字串「WeAct Studio HID Bootloader」VID `0x0483` PID `0x572A` 與 WeAct 官方文件證實），應用程式必須從 `0x08004000` 開始，否則燒錄會覆蓋 bootloader 導致板子失去免 SWD 燒錄的復原路徑。
    - `src/startup.c`：手寫最小 vector table + Reset_Handler（不用 CMSIS 官方 startup 檔，維持「每一行都能解釋」的專案風格），`.data`/`.bss` 初始化，弱別名例外處理常式全部導到 `Default_Handler`。**額外加了 VTOR 重定位**（`Reset_Handler` 內把 `SCB->VTOR` 指向 `g_pfnVectors`）：bootloader 交棒後 CPU 預設仍指向 bootloader 自己的向量表，階段 0/1 用不到中斷所以不影響，但階段 3 一旦用到 Timer/ADC/DMA 中斷會是難查的 bug，先修好。
    - `src/main.c`：階段 0 LED 閃爍（PC13，active-low，Black Pill 系列公版接法），跑在預設 HSI 16MHz、無精確時脈設定。
    - `src/footprint_check_main.cpp`：階段 1，**直接 include 真實的 `include/edgeneuro/*` 標頭檔**（`pipeline.hpp`、`iir_filter.hpp`、`pass_through_filter.hpp`、`mav_feature.hpp`、`lda_classifier.hpp`），用 stub `Provider`（無檔案系統依賴，符合 `SignalProvider` concept）組出真正的 `EdgeNeuro<1,6,...>` 實例，tick() 結果驅動 LED 避免被優化器整段消除，驗證「零修改」承諾與量測真實 Flash/SRAM 用量。
    - `CMakeLists.txt`：FetchContent 抓 `STMicroelectronics/cmsis_device_f4`（v2.6.9）與 `STMicroelectronics/cmsis-core`（v5.9.0）取得暫存器定義標頭檔（只拿 Include，不建置成子專案），定義 `blink` 與 `footprint_check` 兩個執行檔目標。

  * **已踩過的坑**：
    1. `brew install --cask gcc-arm-embedded` 需要互動輸入系統管理員密碼，Bash 工具沒有終端機可以輸入。改用 `brew install arm-none-eabi-gcc`（一般 formula，不需密碼），但這個 formula **不含 newlib**（C 標準函式庫），編譯任何東西都會在 `stdint.h` 這類標頭檔上失敗（`include_next` 找不到下一層）。**解法**：改抓 ARM 官方純壓縮檔版本（含完整 newlib，不需 sudo）：`https://armkeil.blob.core.windows.net/developer/files/downloads/gnu/15.2.rel1/binrel/arm-gnu-toolchain-15.2.rel1-darwin-arm64-arm-none-eabi.tar.xz`（142MB），解壓縮到 `/Users/jeremmy/.local/arm-toolchain`，`firmware/cmake/arm-none-eabi-toolchain.cmake` 已改指向此路徑（`$ENV{HOME}/.local/arm-toolchain/bin/...`）。**已解決。**
    2. **板子插上 Mac 後，Bash 工具確認可以偵測到真實 USB 裝置**（`system_profiler SPUSBDataType` 看得到）——這回答了先前「工具能不能操控實體硬體」的疑問。裝置進入 DFU 模式（按住 BOOT0 + 按 RST）後，`system_profiler` 顯示為 **"WeAct Studio HID Bootloader"（VID `0x0483` PID `0x572A`）**，不是標準 STM32 DFU class，`dfu-util` 對這片板子無效。**已解決（確認根因，改用對應工具）。**
    3. **WeAct 官方 `hid-flash` 燒錄工具在這台 Apple Silicon Mac 上有相容性問題，決定不繼續深修，改等 ST-Link。** 追查過程：(a) Serasidis 上游專案的預編譯 `hid-flash` 二進位檔是 2019 年的 x86_64 版本，靠 Rosetta 2 可執行，但寫死搜尋 VID:PID `1209:BEBA`（跟這片板子實際的 `0483:572A` 對不上）。(b) 改抓 WeAct 自己 fork 的原始碼（`WeActStudio/WeAct_HID_Bootloader_F4x1` repo 下的 `Cli/`）自行編譯，過程中修掉兩個第三方程式碼裡的真實 bug：`hex2bin/readhex.h` 遺失（該 repo 的 git submodule 沒有正確帶出，補了功能等價的 stub，因為 `.hex` 格式支援本來就用不到，我們燒的是 `.bin`）、以及 `main()` 裡一個邏輯反過來的錯誤 null check（`if (i == 10 && handle != NULL)` 應為 `if (handle == NULL)`，原本會在 `hid_open()` 失敗時直接拿 NULL handle 去用，導致 segfault）。(c) 修完後改用 `lldb` 抓到真正當機點在 `hid_open()` 內部：`hid_enumerate()` 回傳的裝置路徑是空字串，導致 `IORegistryEntryFromPath` 找不到裝置——這是這支 2019 年工具跟現在 macOS/Apple Silicon 版 IOKit 的深層相容性問題（裝置路徑產生邏輯需要重寫），已經超出「順手修一下」的合理範圍。**結論：等待中的 ST-Link + OpenOCD 是業界標準、持續維護、Apple Silicon 相容性更好的方案，優先用它燒錄，不再投入時間修這支社群工具。**

  * **階段 0 + 階段 2 已在真實硬體上實測通過（2026-08-14，ST-Link 到貨、實際燒錄）**：
    - **ST-Link 連線確認**：`openocd -f openocd.cfg -c "init; reset halt; exit"` 成功偵測到 `STLINK V2J37S7 (API v2) VID:PID 0483:3748`、target voltage 3.28V(供電/接線正常)、`Cortex-M4 r0p1` 處理器(晶片型號對上，SWD 全鏈路通)。
    - **階段 0(blink)實測通過**：`cmake --build build --target flash_blink`，OpenOCD 回報 `device id = 0x00016423`、`flash size = 256 KiB`(對上 STM32F401RC 規格)、`Programming Finished` + `Verify Started/Verified OK`。**目視確認 PC13 LED 以 ~1Hz 慢速閃爍**——這是本專案第一次讓自己寫的程式碼真正在這片硬體上執行，證實「編譯→連結→燒錄→執行」全鏈路可行，不只是編譯通過。
    - **階段 2(NoHeapGuard 裸機移植)實測通過，未知數 1 驗證成功**：`cmake --build build --target flash_heap_guard_check` 燒錄成功後，**目視確認 LED 恆亮(不閃)**——代表 10 萬次 `tick()` 全程武裝 `NoHeapGuard`，`malloc_count == 0` 在真實 ARM GCC 編譯出的硬體上依然成立，不只是 Host 端的結果。**四個未知數中的未知數 1 正式驗證完畢。**
    - 未知數 2(SRAM/Flash 足跡)在編譯階段已量測（`text=932 bytes` for 階段 1、`text=844 bytes` for 階段 2，遠低於 240KB Flash / 64KB SRAM 上限），本次實測燒錄成功、`flash size` 回報值與晶片規格相符，進一步佐證這個量測是準確的，未知數 2 視為解決。
    - **階段 1(footprint_check，記憶體足跡)** 先前已編譯驗證：**真正的 `include/edgeneuro/*` 標頭檔（`pipeline.hpp`/`iir_filter.hpp`/`pass_through_filter.hpp`/`mav_feature.hpp`/`lda_classifier.hpp`），零修改，直接用 `arm-none-eabi-g++` 編譯連結給 STM32F401 成功**，組出真實的 `EdgeNeuro<1,6,50,...>` 實例，`text=932 bytes, data=0, bss=0`——僅佔 240KB 可用 Flash 的 0.38%、64KB SRAM 完全沒用到靜態配置。**「零修改移植」的承諾得到實體工具鏈與燒錄雙重驗證**。尚未實際燒錄這個階段本身(不影響前述結論，Stage 0/2 已用相同工具鏈與燒錄流程證實可行)。
    - **OpenOCD + ST-Link 設定確認可用**：`firmware/openocd.cfg`（`interface/stlink.cfg` + `target/stm32f4x.cfg`，`adapter speed 1000`）。`CMakeLists.txt` 的 `firmware_add_target()` function 產生的 `flash_<name>` target（`openocd -f openocd.cfg -c "program <bin> 0x08004000 verify reset exit"`）兩次燒錄都成功，位址寫死在 `0x08004000`，只動 Sector 1 以後，Sector 0 的 HID bootloader 未受影響。
    - `firmware/`（含本次新增檔案）已 commit 進 git。

  * **四個未知數現況**：1、2、3 已正式解決，4 條件式通過（見下方階段 3e，增益調整留待之後）。Phase 1.5 的核心可行性問題已回答完畢。階段 1(`flash_footprint_check`)還沒實際燒錄，非必要但可補齊一致性。
  * **階段 4a：I2C1 + MPU6050/GY-521，已在真實硬體上實測通過（2026-08-15）**——PB6/PB7、`RCC_AHB1ENR`/`RCC_APB1ENR` 對應 bit、`I2C_CR2`/`I2C_CCR`/`I2C_TRISE` 的 100kHz 時序計算，全部查證自 RM0368 第 18 章；MPU6050 這端的暫存器位址（`PWR_MGMT_1`=0x6B、`WHO_AM_I`=0x75、`ACCEL_XOUT_H`起算 14 bytes）跟訊號縮放係數（accel 16384 LSB/g、gyro 131 LSB/(°/s)）查證自 InvenSense 官方 `RM-MPU-6000A-00`（register map）+ `PS-MPU-6000A-00`（product spec，electrical characteristics）兩份文件，不是憑記憶。新增 `firmware/src/i2c_mpu6050_hello_main.c`。

    這次除錯過程異常曲折，過程中依序踩到、也修好了四個各自獨立的問題，記錄下來避免以後重蹈覆轍：
    1. **USB-TTL 接線錯誤**：`TXD`/`RXD` 誤接到 `A1`/`A2`（應該是 `A2`/`A3`），導致 UART 長時間看似「不穩定」，其實是接錯線，不是轉接器或程式的問題。
    2. **MPU6050 麵包板插座沒插到底**：造成 I2C 匯流排卡死——直接用 ST-Link 的 SWD 除錯功能讀 `I2C1_SR2` 的 `BUSY` 位元確認卡在 1，插緊模組後 `BUSY` 恢復 0。
    3. **`WHO_AM_I` 回報 `0x72` 而非官方文件寫的 `0x68`**：連續三次獨立重新燒錄都拿到同一個值（不是雜訊），研判是這片 GY-521 板子上的相容/副廠晶片，通訊本身沒問題，韌體改成同時接受 `0x68`/`0x72`。
    4. **真正的韌體邏輯 bug（多位元組 I2C 讀取）**：原本用 `RXNE` 旗標控制最後兩個 byte 的 NACK/STOP 時機，單一 byte(`WHO_AM_I`)讀取正常，但 14-byte 的加速度計+陀螺儀連續讀取失敗——RM0368 §18.3.3 明確要求 N>2 byte 接收要用 `BTF` 旗標（不是 `RXNE`）精確控制 ACK 清除跟 STOP 的時機，照文件重寫後修復。

    **除錯方法上的關鍵轉折**：UART 一直不穩定、靠使用者肉眼看 LED 燈號猜測狀態，效率很差且容易誤判（例如把 `blink_code(1)` 的失敗燈號誤認為是成功迴圈的心跳燈號）。後來改用 **ST-Link 本身的 SWD 除錯功能**（`openocd halt`/`reg pc`/`mdw` 直接讀暫存器跟記憶體、`nm`/`addr2line` 對照原始碼位置、設中斷點在特定函式上、加診斷用全域變數方便讀取），繞過不可靠的 UART，直接向晶片本身要答案——這是之後遇到類似狀況應該優先採用的方式，不要一直依賴 UART 或肉眼觀察。

    **實測結果**：靜止平放時讀到 `accel_z ≈ 16850`（換算 ≈1.03g，符合重力沿 Z 軸）、`accel_x`/`accel_y` 接近 0、`gyro_x ≈ -2.3°/s`、`gyro_y ≈ -0.3°/s`（在靜止零偏漂移的正常範圍內）——I2C + MPU6050 這條路徑完整驗證成功。

  * **階段 4b 卡關：MPU6050 模組疑似硬體故障（2026-08-17，尚未解決）**——整合 `ComplementaryFilter`（`firmware/src/complementary_filter_hello_main.cpp`）時，I2C 通訊完全恢復到階段 4a 一開始的失敗狀態（`i2c1_start()` 卡在 SB 旗標永遠不設），且**同一顆模組後續反覆重插後就再也沒能穩定回到階段 4a 當時的成功狀態**。依序做了以下排除，每一步都用 SWD 直接讀暫存器/診斷用全域變數驗證，不靠肉眼看燈號或猜測：
    1. `I2C1_SR2.BUSY` 卡在 1 → 加上 `I2C_CR1.SWRST` 軟體重置（RM0368 18.6.1），`BUSY` 確認清乾淨，但接下來變成位址送出後收不到 ACK（`mpu6050_read_regs` 回傳碼 2）。
    2. 換一組全新杜邦線接 SCL/SDA → 無效。
    3. 換一塊全新麵包板孔位重插 → 無效。
    4. 電表通斷測試確認 VCC-GND 電壓 3.3V、SCL/SDA 各自通路正常、SCL-SDA 之間沒有短路、AD0/XDA/XCL/INT 這四根未接腳位沒有意外碰觸 → 全部正常。
    5. 位址 `0x68`/`0x69` 都測過 → 都是同樣的 ACK 失敗。
    6. I2C 時脈 100kHz / 10kHz 都測過（排除訊號完整性/時序邊際問題）→ 結果相同。
    7. **換一顆全新購入的第二顆 GY-521 模組測試 → 結果完全相同**（排除單一晶片損壞的可能）。
    8. 加診斷用全域變數直接記錄 AF 旗標當下的 `SR1` 原始值 → 讀到 `0x400`（純粹只有 bit10/AF，沒有其他雜訊旗標），確認韌體判讀正確、是真實的硬體層級 NACK，不是程式邏輯誤判。
    9. 改用 STM32F401 的替代接腳 `PB8`(SCL)`/PB9`(SDA)（仍是 I2C1 周邊，AF4，ST 官方社群文件確認過的合法替代腳位）→ 結果相同，排除 `PB6`/`PB7` 這兩根腳本身故障的可能。
    10. **改用完全不同的 I2C2 周邊**（`PB10`=SCL、`PB3`=SDA，AF4）→ 失敗模式改變（變成 START 本身失敗），但 `PB3` 是除錯埠共用腳位（JTDO），這個測試不夠乾淨，未能下定論。
    11. **決定性測試**：寫一個掃過全部 7-bit 位址（0～127）的 I2C bus scanner，改接一顆完全不同廠牌的裝置——LCD1602 I2C 背板（PCF8574）。**結果：只有位址 `0x27` 有回應 ACK，LCD 畫面也正常顯示字元**——這證明 STM32 這端的 I2C1、`PB6`/`PB7`、韌體邏輯（`i2c1_init`/`i2c1_start`/`i2c1_send_address` 等）完全健康，問題 100% 隔離在 MPU6050 模組本身，且**兩顆獨立購入的 GY-521 模組都表現出同樣的故障**，很可能是同一批次的瑕疵品，或是這兩顆特定模組跟這片 STM32 板子的上拉電阻/電容特性不相容。
    - **結論**：STM32 端 I2C1 韌體（含 `firmware/src/i2c_mpu6050_hello_main.c` 跟 `firmware/src/complementary_filter_hello_main.cpp` 共用的驅動程式）已用第三方裝置（LCD1602）交叉驗證為正確、可用；`ComplementaryFilter` 整合本身尚未被證偽，純粹卡在缺一顆真的能通訊的 MPU6050。下一步視取得新模組的狀況決定：換一顆來源不同的 MPU6050/GY-521 測試，或先擱置此線、繼續其他不依賴 IMU 硬體的工作。

  * **等新 MPU6050 到貨前的壓力測試（2026-08-17）**：新增 `firmware/src/complementary_filter_stress_test_main.cpp`（診斷工具，非 pipeline 階段）——把 `ComplementaryFilter` 的真實迴圈結構（filter 數學運算 + USART2 印出 + I2C 匯流排流量）跟「有沒有真的接到 MPU6050」這件事切開來測：不讀 MPU6050，改用平滑有界的合成 accel/gyro 資料（模擬緩慢傾斜，`|accel|≈1g`）餵給 filter，每個迴圈照樣對已接上的 LCD1602(0x27) 送一次真實 I2C 寫入(切換背光位元)當作匯流排負載。**實測結果**：連續跑 60 秒、367 次迴圈，UART 輸出全程正常、`roll`/`pitch` 平滑有界、**零次 NaN/Inf**——韌體本身(計時、UART、I2C 寫入路徑、filter 數學)在長時間運作下是穩定的。同時量到一個真實發現並回頭修正：**迴圈實際週期是 163.4ms，不是原本用 `delay(400000u)` 粗估的 250ms**（誤差近 35%）——已經把 `complementary_filter_hello_main.cpp` 的 `kDt` 從 `0.25f` 改成量測到的 `0.163f`，並註記等真的接上 MPU6050(14-byte I2C 讀取，跟這次測試的 1-byte LCD 寫入耗時不完全一樣)後應該直接重新量測校正，不是把這個數字當作最終值。

  * **下一步**：待有能正常通訊的 MPU6050 模組後，把已經 Host 端驗證過的 `ComplementaryFilter`（見上方）整合進這份韌體，用真實 MPU6050 資料跑姿態融合，而不只是印原始 accel/gyro 數值；同時重新量測真實迴圈週期，校正 `kDt`。

  * **2026-08-18 補充排除：5V 供電也無效**——改成單一電源接法（見 firmware/README.md「供電原則」）、`VCC` 從 3.3V 改接 STM32 板子自己的 5V（GY-521 板上通常有獨立穩壓晶片，社群普遍建議 5V 供電避免壓差不足）後，兩顆故障模組**依然完全無法通訊**，同一時間用 LCD1602 交叉驗證確認新供電接法下 STM32 端本身仍然完全正常。這排除了最後一個容易被忽略的變因，進一步坐實「這兩顆模組真的壞了」的結論，非導線/供電接法問題。已下單 Adafruit MPU-6050（正貨保證）等待到貨。

  * **階段 5a：EMG 抓放控制，已在真實硬體上實測通過（2026-08-18）**——不等 MPU6050，先把 Phase 3 拆分架構（見第 3 節）裡不依賴 IMU 的那一半接上真實硬體驗證。新增 `firmware/src/emg_grip_control_main.cpp`：沿用階段 3c/3d 已驗證的 TIM2+ADC1 硬體觸發 1kHz 讀取 MyoWare `ENV`，每個 tick 餵進 `GripStateMachine::update()`（`dt=0.001f` 是 TIM2 實測準確的硬體週期，不是猜的），再把抓放決策餵進 `SlewRateLimiter` 產生平滑 setpoint，透過 USART2 即時回報（轉態當下立即印一行，另外每秒印一次 min/max 摘要）。`GripStateMachine`/`SlewRateLimiter` 先前只在 Host 端用合成數值測試過，這是第一次接真實訊號。
    - **實測結果**：真實電極貼片量到的放鬆/握拳訊號動態範圍遠比階段 3e 用手指觸碰測到的窄範圍（~1211-1216）乾淨許多——放鬆穩定在 **~434-495**，持續握拳飆到 **~3700+**。原本寫死的 `threshold=1220`（依階段 3e 舊資料設的暫定值）意外地剛好落在兩者中間、margin 充足，不需要重新調整。轉態行為正確：放鬆時訊號自然衰減（觀察到 3717→2069→1477→887→484 這種漸降曲線），`off_duration` 正確等訊號穩定低於閾值才觸發 `EDGE -> Released`，沒有被衰減過程中的中間值提前誤觸發；反覆握拳/放鬆循環測試，`EDGE -> Gripping`/`EDGE -> Released` 都乾淨對應真實動作。
    - 這也順便把懸而未決的「MyoWare 增益調整」（未知數 4，先前條件式通過、延後處理的項目）用真實資料收尾——原本擔心的訊號動態範圍過窄問題，換成真實電極（而非手指觸碰模擬）後並不存在。
    - 韌體註解已更新為「`threshold` 已用真實資料驗證」，但同時註記：這只是一次 session、一隻手臂、一個增益旋鈕設定下的結果，電極貼法、皮膚接觸狀況或增益旋鈕改變時應該重新用這個階段驗證，不是永久定案的數字。
    - 順帶驗證了 `tools/watch_myoware_uart.py` 對這個新韌體輸出格式（`raw_min`/`raw_max`/`gripping`/`setpoint_x1000`/`EDGE ->`）的即時監看支援。

  * **階段 3a：USART2 UART 輸出，已在真實硬體上實測通過（2026-08-15，USB-TTL 到貨）**——未知數 3（1kHz 取樣）跟未知數 4（MyoWare 真實訊號）都需要先有能運作的 UART 才能把讀到的數值印出來診斷，所以拆出這個更小的子階段先單獨驗證，不跟 Timer/ADC 混在一起。
    - **暫存器值查證過程**：先前 Stage 3 的 ADC/Timer 草稿曾因為暫存器值未查證被打回票（見「已踩過的坑」），這次改成先下載 ST 官方 RM0368 參考手冊（Rev 5，847 頁）與 STM32F401CCU6 官方 datasheet（DocID024738，來源是 WeAct 官方 GitHub repo，跟板子本身同一份），用 `pypdf` 定位到確切頁數逐頁讀取確認，而非憑記憶或猜測：
      - PA2/PA3 = AF7（USART2_TX/USART2_RX）：datasheet Table 9「Alternate function mapping」逐格確認，不是憑通用 STM32 知識假設。
      - `RCC_AHB1ENR` bit 0 = `GPIOAEN`、`RCC_APB1ENR` bit 17 = `USART2EN`：RM0368 §6.3.9/§6.3.11 暫存器圖直接讀值。
      - `USART_CR1`（`UE`=bit13、`M`=bit12、`TE`=bit3、`OVER8`=bit15）、`USART_SR`（`TXE`=bit7）、`USART_BRR`（`DIV_Mantissa`=bits[15:4]、`DIV_Fraction`=bits[3:0]）：RM0368 §19.6 USART 暫存器章節逐一確認。
      - 鮑率公式（`USARTDIV = f_CK / (16 × baud)`，OVER8=0 時）：RM0368 §19.3.4 Equation 1，套 16MHz HSI（預設未校準時脈）算 9600 baud → `DIV_Mantissa=104(0x68)`、`DIV_Fraction=3` → `BRR=0x0683`，理論誤差 0.02%。刻意選 9600（不選更高鮑率）+ oversampling by 16（不選 by 8）：兩者都是「在沒有精確外部石英振盪器、只有內部 HSI 的情況下，增加對時脈誤差的容忍度」的保守選擇，RM0368 §19.3.3 明確建議 oversampling by 16 容忍度較高。
      - 所有暫存器欄位巨集名稱（`RCC_AHB1ENR_GPIOAEN`、`USART_CR1_UE` 等）額外用 `grep` 對照過 FetchContent 抓下來、Stage 0/1/2 建置時就已實際使用的同一份 CMSIS 標頭檔（`stm32f401xc.h`），確認巨集真的存在、不是編出來的名字。
    - **新增 `firmware/src/uart_hello_main.c`**：初始化 USART2、每秒輪詢送出一行固定文字，LED 同步閃爍（跟 UART 收發無關，是獨立的「韌體本身有沒有在跑」診斷訊號，方便把「韌體邏輯錯誤」跟「實體接線錯誤」這兩種可能性分開判斷）。`firmware/CMakeLists.txt` 新增 `uart_hello` target。
    - **實測結果**：燒錄成功，PC13 LED 正常閃爍（證實韌體有在跑）。用 `pyserial`（非互動式，可截取到檔案比對，比 `screen` 更適合自動化驗證）連接 `/dev/tty.usbserial-0001` @ 9600 baud，**收到乾淨、無亂碼、重複出現的 `"EdgeNeuro Stage 3a: UART alive"` 字串**——證實查證過的暫存器值、鮑率計算、`PA2`/`PA3` 接線全部正確。未知數 3 的 UART 診斷通道就緒；Timer/ADC 1kHz 取樣本身仍待實作與驗證。

  * **階段 3b：ADC1 單通道軟體觸發讀取（PA0），已在真實硬體上實測通過**——在加 Timer 之前先單獨驗證 ADC 本身，同樣的「一次只驗證一個變數」原則。暫存器值（`RCC_APB2ENR` bit 8=`ADC1EN`、`GPIOx_MODER`=11 類比模式、`ADC_CR2` 的 `ADON`/`CONT`/`SWSTART`、`ADC_SQR1`/`ADC_SQR3` 的通道序列設定、`ADC_SR` 的 `EOC`）全部查證自 RM0368 第 11 章。新增 `firmware/src/adc_hello_main.c`，透過已驗證的 USART2 把讀到的 12-bit 原始值印出來。實測：MyoWare 感測器接上、手指按著電極測試靜止狀態下，讀到穩定落在 1279~1316 的數值（12-bit 滿量程 0~4095），波動遠小於懸空雜訊,證實 ADC 讀取路徑跟 `PA0` 接線正確。
  * **階段 3c/3d：TIM2 以硬體 TRGO 定時觸發 ADC1，達成真正的 1kHz 取樣，已在真實硬體上實測通過（2026-08-15）——未知數 3 正式解決**：
    - 前面階段 3a/3b 都還只是「UART 能不能印」跟「ADC 能不能讀」分開驗證，**這一步才是第一次真正把 Timer 跟 ADC 接在一起**，也才是未知數 3 真正要問的問題。
    - 暫存器值查證（RM0368 第 13 章 TIM2~TIM5 通用計時器 + 第 11 章 ADC 外部觸發部分）：`RCC_APB1ENR` bit 0=`TIM2EN`；`TIM2CLK` 換算——RM0368 明確寫「APB 預除頻器若為 1，`TIMxCLK = HCLK`；否則 `TIMxCLK = 2×PCLKx`」，本專案 APB1 預除頻器維持預設值 1，故 `TIM2CLK = HCLK = 16MHz`，不用乘 2；`TIMx_CR2` bits[6:4]=`MMS`，設 010（Update）讓計時器的 update event 變成 TRGO 硬體訊號；`ADC_CR2` bits[29:28]=`EXTEN`(01=上升緣觸發)、bits[27:24]=`EXTSEL`(0110=Timer 2 TRGO event，同一個暫存器頁面查到的編碼表，不是通用記憶)。
    - 取樣率計算：`(PSC+1)×(ARR+1) = TIM2CLK / 目標頻率 = 16,000,000 / 1000 = 16,000`，選 `PSC=15`、`ARR=999`（16×1000=16000），**整數整除、無條件捨入誤差**（跟階段 3a 的鮑率換算不同，那個有小數捨入誤差；這個沒有）。
    - 新增 `firmware/src/timer_adc_1khz_main.c`：TIM2 的 update event 透過硬體直接觸發 ADC1 轉換，主迴圈只負責輪詢 `ADC_SR` 的 `EOC`、累計樣本數，**每 1000 個樣本才透過 UART 回報一次**（9600 baud 若每個樣本都印,吞吐量完全不夠,會塞爆——這個限制本身也是選擇不逐樣本印出的理由）並閃一下 LED。
    - **實測結果**：用 `pyserial` 連續讀取「每 1000 樣本回報一次」的訊息,量測相鄰回報之間的實際時間間隔：**1.005s、1.005s、1.002s**——非常接近理論上剛好 1 秒,誤差 <0.5%,完全落在未校準 16MHz HSI 振盪器本身的正常誤差範圍內。這是第一次有**實際計時的實測數據**證實 Timer 硬體觸發 ADC 真的能穩定達成 1kHz,不只是編譯通過、不當機而已。**未知數 3 正式解決。**

  * **階段 3e：真人前臂實測，未知數 4 條件式通過（2026-08-15）**：
    - `timer_adc_1khz` 韌體加上 `min`/`max` 統計（每 1000 樣本回報一次窗口內的最小/最大值，比單點瞬時值更能捕捉到爆發性的肌電變化），貼好一次性電極貼片（`MID`/`END`/`REF`，肌肉肌腹/靠手腕/手肘參考點），分兩段各自獨立錄製對比：
      - **放鬆**（7 筆）：`min` 1114~1119、`max` 1211~1216，波動很小。
      - **出力握拳並撐住**（7 筆）：`min` 1115~1117（幾乎沒變，符合預期——底線不該因出力而變）、`max` 從 1212 逐漸爬升到 1221、1226、最後穩定在 **1227**，曲線形狀對應到「握拳出力並維持」這個動作的物理過程，不是隨機雜訊。
      - 兩者 `max` 差距約 11~16（12-bit 滿量程 4095 的 0.3~0.4%）——**訊號鏈整條路（電極貼片→感測器→STM32 ADC）確認有真的在反應真人肌肉收縮**，差異乾淨可重複，不是雜訊，但動態範圍偏窄。
    - 查證官方文件（SparkFun MyoWare 2.0 Muscle Sensor hookup guide）確認：感測器板子本身有增益電位器（trim pot），公式 `ENV 增益 = 200 × (R / 1kΩ)`，順時針增大、逆時針減小，官方建議調到「最大出力時訊號峰值接近但不超過 VIN」。我們量到的峰值（1227/4095 ≈ 30%）離這個建議目標還有空間，代表增益還可以往上調。
    - 曾考慮過一個簡化測試捷徑：「不管有沒有貼皮膚，看感測器板子自帶的 `ENV` 指示燈（官方文件確認板子本身有 `VIN`/`ENV` 兩顆 LED）全亮就當作有訊號」——**查證後確認此法不可行**：`ENV` 燈的官方定義只是「ENV 腳位有活動就亮」，沒有skin contact 的懸空輸入本身就會因為沒有參考電位而觸發「活動」判定（跟先前 LED Shield 全亮是同一個物理原因），沒辦法用這顆燈分辨「真的量到肌肉」還是「懸空雜訊」，所以放棄這個捷徑，改用真實貼片+STM32 ADC 數值比對。
    - **決策**：增益調整、電極貼片位置優化留到之後再處理（不是阻塞項——訊號精準度跟 pipeline 速度/效率是兩個獨立的軸線，pipeline 已經驗證夠快，訊號精準度可以晚點再調，不影響往下的架構工作）。未知數 4 標記為「條件式通過」：訊號鏈本身確認可用，量化的可靠度門檻留待之後有明確判斷需求時再重新測試。

  * **與硬體並行、不受 ST-Link/USB-TTL 到貨阻塞的演算法工作**：義肢動作是否流暢，除了「target 上 zero alloc」這個底層保證外，還取決於姿態融合演算法與指令平滑化——這兩塊是純數學邏輯，跟暫存器層級的韌體工作性質不同（不受「未驗證暫存器程式碼不能寫」這條限制約束），可以在等硬體的期間用 Host 端 Catch2 完全驗證：
    - **`include/edgeneuro/fusion/complementary_filter.hpp`（已完成，且已對照真實硬體資料驗證）**：`ComplementaryFilter<ValueType>` 融合陀螺儀（短期準確、長期會飄移）與加速度計（單次雜訊大、長期平均準確，本質是量測重力向量）估計 roll/pitch。刻意**不符合** `concepts.hpp` 的 `Filter` concept（那個 concept 是單通道 `process(value)->value`，姿態融合本質跨通道，需要同時吃 accel x/y/z + gyro rate + dt），所以目前是獨立元件。**不會**接入 `EdgeNeuro` pipeline 的 `ImuFilterT` 插槽——已確認（見第 3 節架構修正）IMU 這條路本來就該完全繞過 `Pipeline`，直接即時驅動方位，不應該被 window/classify 拖慢。只輸出 roll/pitch，**不做 yaw**（沒有磁力計，MPU6050 本身也量不到絕對朝向，硬做只會無界飄移，這點在買 GY-521/MPU6050 模組時已經確認過）。額外提供 `initialize(accel_x,accel_y,accel_z)`：直接把 roll/pitch 種到加速度計算出的角度，跳過從 0 開始收斂的暫態——這是拿真實資料測試時發現的真實問題（見下段），不是憑空加的功能。
      - `tests/test_complementary_filter.cpp`（7 個 test case）先**只用已知物理量驗證**（單位向量、已知角速度積分），刻意不直接拿 EMG-EPN-612 的 IMU 欄位當正確答案比對——因為一開始不確定那批資料的陀螺儀單位（deg/s 還是 rad/s）、加速度計單位是否已經是物理單位，硬拿來比對等於重蹈 EMG-EPN-612 schema 那次「先猜再拿真實資料修正」的教訓。
      - 後續查證：Myo 官方藍牙協定標頭檔（`thalmiclabs/myo-bluetooth` repo `myohw.h`)明確記載加速度計單位 g（除以 scale 2048)、陀螺儀單位 deg/s（除以 scale 16)、姿態四元數已正規化（除以 scale 16384)。**再用真實資料交叉驗證這個文件是否已經套用過**：user1 靜止片段的加速度計三軸平方和 ≈ 1.03（≈1g）、四元數平方和 ≈ 1.0（合法單位四元數）、陀螺儀數值落在個位數（符合 deg/s、不像原始 SDK 定點數的數千量級）——兩層獨立證據都指向這批 JSON 匯出的已經是物理單位，不是原始整數。
      - 有了單位換算依據後，新增 `tests/test_complementary_filter_real_data.cpp`：讀 `data/raw/epn612_user1.csv` 第一筆錄製(992 列，"noGesture"，手臂近乎靜止)，陀螺儀轉 rad/s 餵進 `ComplementaryFilter`，同時把 CSV 裡 Myo 自己內建融合演算法輸出的四元數轉成 roll/pitch 當作獨立 ground truth 比對。結果：跑完整段錄製後，我們自己的 roll/pitch 估計跟 Myo 自己的四元數估計誤差僅 ~1.4e-4 rad(roll)/~2.8e-4 rad(pitch)，遠低於測試設定的 0.01 rad 容許誤差——證實我們的 atan2 座標軸慣例跟 Myo 內部慣例一致，且演算法本身在真實硬體雜訊下數值穩定、不會發散或出現 NaN。這是目前這個元件唯一一處拿真實資料做數值比對的測試，而且是在單位/軸向都有兩層獨立證據支撐之後才做，不是憑感覺假設。
    - **`include/edgeneuro/control/grip_state_machine.hpp`（已完成，2026-08-17）**：`GripStateMachine<ValueType>` 把單通道 MyoWare `ENV`(已經過類比整流+低通)訊號轉成離散的抓/放決策。刻意**不用** `LdaClassifier`：單一 EMG 通道物理上只能表達一維的「出力大小」資訊，多手勢分類器在此沒有實質效益；也刻意**不走** `Pipeline` 的 window/feature/classify（見第 3 節架構修正，window 延遲對這個判斷來說不必要）。核心邏輯是閾值+持續時間的遲滯(hysteresis)狀態機：`envelope` 持續超過 `threshold` 達 `on_duration` 秒才觸發 `Gripping`，持續低於達 `off_duration` 秒才觸發 `Released`——這個持續時間門檻才是真正過濾雜訊脈衝/短暫抖動的機制，不是單純比較閾值。`update()` 只在真正發生轉態的那一次呼叫回傳 `true`，其餘時間回傳 `false`，讓呼叫端可以分辨「持續維持某狀態」跟「這一刻剛轉換」。
      - `tests/test_grip_state_machine.cpp`（7 個 test case）：涵蓋起始狀態、未達 `on_duration` 不觸發、超過 `on_duration` 恰好觸發一次、短暫雜訊脈衝不觸發、`off_duration` 對稱測試、`Gripping` 中的短暫低於閾值抖動不誤觸發、`reset()` 真的清掉累積時間(不是只清狀態旗標)。**寫測試過程中抓到一個真實的浮點數精度陷阱**：原本斷言「累加 `0.01f` 十次剛好在第 10 次觸發」，但 `0.01f` 本身不能被二進位浮點數精確表示，累加十次的結果 `0.099999...` 會些微小於 `0.1f`，實際要到第 11 次才觸發——不是實作邏輯錯，是測試對浮點精度的假設太樂觀，改成「明顯少於門檻」跟「明顯超過門檻」兩組寬鬆邊界後修正，不再斷言精確的第 N 次邊界。
    - **`include/edgeneuro/control/slew_rate_limiter.hpp`（已完成，2026-08-18）**：指令軌跡平滑化——`SlewRateLimiter<ValueType>` 限制輸出值每秒最大變化量，把 `GripStateMachine` 的離散 0/1 跳變拉成漸進斜坡，避免致動器收到瞬間階躍指令。這是「平滑指令本身」，跟 `ComplementaryFilter` 「平滑感測器讀值」是兩件不同的事，不要混為一談：`ComplementaryFilter` 是把有雜訊的量測估計成可信的角度，`SlewRateLimiter` 是把已經估計好/決策好的目標值，限制成致動器可以連續跟隨的路徑。**IMU 那端(`roll`/`pitch`)的殘留抖動不需要新元件**——直接重用專案已有的 `include/edgeneuro/filters/iir_filter.hpp`（`Filter` concept，`process()`/`reset()`）當作融合輸出後的輕量低通即可，沒必要為了同一件事再造一個元件。刻意**不符合** `Filter` concept：速率限制本質上需要 `dt`（每秒最大變化量），`process(value)` 這個介面沒有 `dt` 參數，跟 `ComplementaryFilter`/`GripStateMachine` 被排除在 `Pipeline` 之外是同樣的理由。
      - `tests/test_slew_rate_limiter.cpp`（7 個 test case）：涵蓋初始值、單步內到位不 overshoot、遠距離目標會被限速、多步驟斜坡爬升到位、對稱下降、`reset()` 立即跳變、持續變動目標下輸出始終不超過每步允許的最大變化量。
    - 全部 60 個 Catch2 test case（`ComplementaryFilter` 7 個、`GripStateMachine` 7 個、`SlewRateLimiter` 7 個都在內）在 `coverage`/`debug-heapguard` 兩個 preset 下全數通過。

  * **`include/edgeneuro/control/threshold_calibrator.hpp` + 韌體開機互動式校準（2026-08-19）**：把階段 5a 寫死的 `threshold` 常數（1220）改成開機時透過「放鬆 → 握拳」兩階段真實取樣自動算出，而不是每次換電極貼法/皮膚接觸/增益旋鈕都要手動改常數重燒。`ThresholdCalibrator<ValueType>` 分別記錄放鬆階段觀測到的最大值、握拳階段觀測到的最小值，取兩者中點當閾值；`is_valid()` 明確要求兩階段都真的被觀測過，不能只靠數值大小關係判斷（寫測試時抓到一個真實 bug：初始 sentinel 值本身就滿足「contracted_min > relaxed_max」，若不額外追蹤「是否真的收到過觀測值」，完全沒取樣過也會誤判成校準成功——已修正並補測試）。`tests/test_threshold_calibrator.cpp`（7 個 test case）涵蓋這個情境跟基本中點運算、reset。
    - **韌體端的同步問題跟修正過程**：一開始用固定延遲(先 2 秒後 3 秒)自動跑兩階段取樣，結果連續兩次因為「真人透過對話介面被指示動作」跟「韌體自主計時」對不上，取樣窗口跑完時使用者根本還沒真的放鬆/握拳，導致 `CALIBRATE FAILED`。改成**互動式**：每階段開始前透過 USART2 RX 等待任意一個位元組才開始取樣，由操作端（人或 host 腳本）自己決定精確的開始時機，不再猜測時間。改完後同步問題徹底解決，第一次乾淨拿到 `relaxed_max=491 contracted_min=3583 threshold=2037`。
    - **意外發現的新問題與修正**：互動式校準會讓 `main()` 卡在 `usart2_recv_byte()` 無限等待，如果沒有任何一方送出觸發位元組（例如板子重置後沒人接著送位元組），韌體會完全沉默、不進主迴圈也不印任何東西——外觀上跟「完全沒在動」無法區分，實際發生過一次（懷疑是 SWD 操作間接造成重置）。修正：加入 `kCalibrationEnabled`（編譯期常數，**預設 `false`**）與 `kFallbackThreshold`（沿用上次校準成功的真實數值 2037）——關閉時用 `if constexpr` 直接把整段互動式校準邏輯編譯掉，開機立即進入正常控制迴圈，不需要任何人互動；只有明確想重新校準時才改成 `true`、重燒、走一次互動流程。
    - **真實硬體上觀察到電極接觸品質會隨時間漂移**：同一次 session 內，剛校準完的乾淨放鬆訊號（~491），幾分鐘後同樣「放鬆」狀態卻讀到跟握拳時相近的高值、且在閾值附近不穩定震盪——`GripStateMachine` 的遲滯機制正確反映了這個真實不穩定，不是程式判斷錯誤，根因懷疑是電極貼片鬆動/流汗/貼合度隨時間變化，屬於物理層面問題，留待之後處理，不是這次要解決的範圍。

  * **階段 5b：EMG+IMU 合併成真正的 Phase 3 主迴圈，含不阻塞 I2C 讀取狀態機（2026-08-20/21）**：新增 `firmware/src/phase3_control_loop_main.cpp`，把階段 5a 已驗證的 EMG 迴圈（`GripStateMachine`+`SlewRateLimiter`）跟 IMU 姿態融合（`ComplementaryFilter`）真正合併成同一個 1kHz 主迴圈，而不是分開的兩份韌體。
    - **不阻塞 I2C 讀取狀態機**：一次完整 14-byte I2C 讀取在 100kHz 下約 1.5-2ms，比 EMG 的 1ms tick 還長，直接用階段 4a 的阻塞式 `mpu6050_read_regs()` 會卡住 EMG 取樣。新寫 `ImuReader` 類別，把讀取拆成 10 個顯式狀態（`WaitStart1`→`WaitAddr1`→`WaitRegTxe`→`WaitRegBtf`→`WaitStart2`→`WaitAddr2`→`ReadingMiddle`→`WaitBtfPenultimate`→`WaitBtfLast`→完成），`step()` 每次呼叫只檢查一個硬體旗標就立即返回，不論有沒有準備好，協定順序完全比照階段 4a 已驗證過的 blocking 版本，只是拆成多次呼叫而非一次忙等。逾時保護原本用「呼叫次數」計數，後來發現呼叫頻率一旦跟 EMG tick 脫鉤（見下）會失準（同一個 1ms 內可能被呼叫數千次），改成比較真正的 EMG tick 差值，不受呼叫頻率影響。
    - **因為手上兩顆 MPU6050 都還故障（見階段 4b），無法用真感測器驗證這個狀態機**，改用已確認正常的 LCD1602(PCF8574，位址 0x27)當替代測試對象：LCD 會 ACK 位址寫入、接受任意 byte 當作「暫存器位址」(PCF8574 沒有暫存器概念，只是鎖存 GPIO 輸出)、ACK 位址讀取並回傳目前輸入腳位電位——不是真感測器資料，但完整走過跟真實 MPU6050 讀取一樣的 START/ADDR/TXE/BTF/RXNE/STOP 協定時序，包括階段 4a 當初真正踩過 bug 的多位元組 BTF 尾端時序。等新 MPU6050 到貨，只要把 `kImuTargetAddr` 從 `0x27` 改回 `0x68`，狀態機本身不用再改。
    - **實測發現一個真實的架構瓶頸並修正**：第一版把 `imu_reader.step()` 放在 `if (ADC1->SR & ADC_SR_EOC)` 區塊內，只跟著 EMG tick 呼叫一次，結果一秒只完成 **47-48 次**完整讀取——遠低於 100kHz 匯流排理論上限（估算約 500-650 次/秒）。算出原因：一次完整讀取需要約 22 個狀態轉換，而 `step()` 每個 EMG tick(1ms)只被呼叫一次，就算硬體早就準備好也要等到下一個 tick 才會被發現，22 個轉換 × 1ms ≈ 22ms，正好對上 47-48 次/秒——**瓶頸是輪詢頻率被綁死在 EMG tick 上，不是 I2C 匯流排本身**。把 `imu_reader.step()` 移到主迴圈最外層、跟 `ADC_SR_EOC` 判斷脫鉤，讓它在兩次 ADC 取樣之間的 CPU 閒置時間盡量多執行，改完後實測 **躍升到 592-593 次/秒**，非常接近估算的硬體上限。過程中一度出現 `imu_completions=0`（懷疑是燒錄/重置流程的暫時性問題，非邏輯錯誤——加了診斷用全域變數(`g_imu_state_at_timeout`/`g_sr1_at_timeout`/`g_timeout_count`)追蹤後，穩定運作時 `g_timeout_count` 始終是 0，沒有真的卡住過)。
    - **`ComplementaryFilter` API 修正**：合併後 IMU 讀取不再是固定間隔完成（視匯流排時序，實測每 2-4 個 tick 才完成一次），但原本 `update()` 用建構子傳入的固定 `dt_` 積分陀螺儀角度——如果實際經過的時間跟 `dt_` 不一致，會系統性算錯轉動量。幫 `ComplementaryFilter::update()` 加一個帶明確 `dt` 參數的多載版本（原本 5 參數版本保持不變、內部呼叫新的 6 參數版本），呼叫端用「這次完成距離上次完成經過幾個真實 EMG tick」算出真實 `dt`。新增 `tests/test_complementary_filter.cpp` 測試案例確認多載版本真的用傳入的 `dt`，不是悄悄退回建構子的值。
    - **後續：MPU6050 晶片本身支援 I2C Fast Mode(400kHz)**，查證自 InvenSense 官方 product spec，不是猜的；目前韌體仍設定在 100kHz，等真的接上 MPU6050 後可以評估切到 400kHz，理論上一次讀取時間可以再縮短到約 1/4。晶片內部感測器取樣率本身有上限（加速度計固定 1kHz、陀螺儀最高 8kHz），匯流排再快也不會超過這個上限。
    - **IMU 指令軌跡平滑化接上（2026-08-21）**：把先前決定要重用的 `IirFilter` 真的接進 `phase3_control_loop_main.cpp`——`roll_smoother`/`pitch_smoother` 各一個，設定成單極指數移動平均（`y[n] = alpha*x[n] + (1-alpha)*y[n-1]`，用 `IirFilter` 既有的 biquad 結構表示：`b0=alpha`、`a1=alpha-1`、其餘係數為 0），`kSmoothAlpha=0.5` 是起始暫定值，不是推導出來的截止頻率——IMU 完成間隔還不固定（視匯流排時序，目前 2-4 tick 不等），現在硬推頻域設計沒有意義，等真的接上 MPU6050、有真實抖動可以觀察後再校準。實測 LCD 假資料本身很穩定沒有抖動，平滑前後數值幾乎相同（929 vs 929），只確認了接線正確、不影響其他邏輯，還沒辦法驗證平滑效果本身。
    - **`kSmoothAlpha` 真的接上真實感測器後，還沒能校準（2026-08-22）**：原本預期真實 MPU6050 靜止時會有明顯抖動可以拿來調 `kSmoothAlpha`，但實測發現靜止時 `roll`/`pitch` 本身波動就很小——`ComplementaryFilter` 的 `alpha=0.98` 本身已經是很重的平滑（98% 信任陀螺儀積分、只有 2% 信任新的加速度計讀值），到 `IirFilter` 這層時剩下的抖動已經不多，不容易單獨看出 `kSmoothAlpha` 調整的效果。原本想拿起感測器搖晃來製造明顯抖動觀察，但搖晃過程中接線被震鬆，導致一連串接線／`I2C` Fast Mode 排查（見下），沒有真的完成這次校準，`kSmoothAlpha=0.5` 目前仍是未經真實資料驗證的暫定值，留待之後有機會再處理。
    - **重新測試時意外復現的 BUSY 卡死問題，順手補上自我修復**：這次接上平滑化重燒後，`ImuReader` 連續逾時（`g_timeout_count` 一路累積到 758 次都卡在 `WaitStart1`，`SR2.BUSY=1`）——跟階段 4a/4b 反覆踩過的麵包板接觸不良同一類問題，重插 LCD 接線後確認匯流排恢復正常。但也發現 `ImuReader` 的逾時處理原本只發 `STOP`，沒有像 `i2c1_init()` 開機時那樣做 `SWRST`——單靠 `STOP` 救不回真正卡死的周邊內部狀態。補上：逾時時比照開機流程做完整 `SWRST` + 重新設定 `CR2`/`CCR`/`TRISE`，讓暫時性的匯流排卡死能在下一次 `begin()` 自動恢復，不用每次都手動重插線、重新燒錄。

  * **階段 5c：真實 Adafruit MPU-6050 到貨，姿態融合首次在真實硬體上完整驗證通過（2026-08-22）**——接腳跟先前 GY-521 clone 不同，已查證 Adafruit 官方文件：只需 `Vin`(接 STM32 3.3V，板上穩壓器正規支援 3-5V，不像 GY-521 有壓差問題，不需要改接 5V)、`GND`、`SCL`(PB6)、`SDA`(PB7)四線，`AD0` 不接時預設拉低、位址是 `0x68`。`i2c_bus_scan` 掃描直接 ACK，是本專案第一次有真的 MPU6050 回應。
    - **接上完整迴圈後發現一個真實 bug 並修正**：`phase3_control_loop_main.cpp` 是先拿 LCD1602 開發、驗證的，從沒真的寫過喚醒感測器那一步(`PWR_MGMT_1`/0x6B 清除 `SLEEP` 位元)——這步驟在階段 4a、`complementary_filter_hello_main.cpp` 都有做，唯獨這個合併版本漏掉了。結果：協定層面完全正常(`imu_completions` 一樣有 591-593 次/秒)，但 `roll`/`pitch` 永遠卡在 0，因為感測器根本沒醒、暫存器內容不會更新。新增 `mpu6050_write_reg_blocking()`(一次性阻塞寫入，開機時做一次即可，不影響主迴圈即時性)，比照階段 4a 已驗證的寫入序列，開機時喚醒感測器。
    - **修 bug 過程中的排查曲折，記錄避免下次重踩**：喚醒寫入一開始持續失敗(`g_wake_result=1`，START 都發不出去，`SR2.BUSY=1`)，一度懷疑又是麵包板接觸不良，直接讀 GPIOB 腳位電位(`PB6`/`PB7`)確認兩條線都被拉低。但依序排除電源(LED 有亮)、換一份已驗證正常的韌體(`i2c_bus_scan`)重新測試同一組實體接線——結果腳位電位正常、`BUSY=0`——**證實問題出在 `phase3_control_loop` 這份韌體本身，不是接線**。比對兩份檔案的 `i2c1_init()` 逐行確認邏輯完全一致，才想起：稍早為了排查 SDA 是否被外部拉低，曾請使用者把 SCL/SDA**從感測器上整個拔掉**做隔離測試，之後修 bug、換韌體測試的過程中忘了請使用者接回去——喚醒寫入的 `AF`(NACK)結果(`g_wake_result=2`)才終於對上「線路本身沒問題，只是感測器根本沒接在線路另一端」這個實情。接回線之後，喚醒寫入、姿態融合當場正常運作。
    - **實測結果**：`roll_x1000`/`pitch_x1000` 在感測器靜止時穩定，手動傾斜/旋轉板子時數值即時、正確地連續變化(觀察到 roll 在 1150–1705、pitch 在 381–1218 之間隨動作變動)，`roll_smoothed`/`pitch_smoothed` 正確跟隨、只有預期中的些微平滑延遲。`imu_completions` 維持在 591-593 次/秒，跟先前用 LCD1602 替代測試量到的速度一致——證實當初「輪詢頻率脫鉤讓速度接近硬體上限」的量測跟分析，換成真實感測器後依然成立，不是只對 LCD 這個替代品才有效的巧合。這是本專案第一次完整驗證：真實 IMU 資料 → `ComplementaryFilter` 融合 → `IirFilter` 平滑，全部在真實硬體、真實動作下正確運作。

  * **I2C Fast Mode(400kHz)實測：晶片支援、算式正確，但這組麵包板接線撐不住（2026-08-22）**——依 RM0368 18.6.8/18.6.9 Fast mode 公式（`period=3×CCR×Tpclk1`，跟 Sm mode 的 `2×CCR×Tpclk1` 不同；最大上升時間 300ns，不是 Sm mode 的 1000ns）算出 16MHz 下 `CCR=14`(`13.33` 進位到 14，不能取更接近的 13，否則會變成 ~410kHz 超過 Fast mode 上限)、`TRISE=6`。**第一次測試**：`imu_completions` 從 100kHz 的 592/秒直接跳到 **2028/秒**（約 3.4 倍，跟理論值吻合），roll/pitch 也正確動態變化。
    - **搖晃感測器觀察抖動時，接線被震鬆，衍生出兩個獨立問題**：(1) 搖晃後 roll/pitch 卡在 0，但 `imu_completions` 仍高(~2111/秒)——協定層面正常，但資料內容不對，重新開機(確認 `g_wake_result=0` 喚醒成功)也沒有恢復；(2) 切回 100kHz 測試同一組接線時，直接偵測到 `BUSY` 卡死(`g_wake_result=1`)。用 `i2c_bus_scan` 確認 0x68 沒有回應，重新壓緊接線後恢復正常。
    - **決定性測試**：接線確認恢復良好、100kHz 下運作正常後，**不動任何實體接線，只把 `CCR`/`TRISE` 改回 400kHz 設定重新燒錄**——立刻重現 `BUSY` 卡死。同一組接線幾秒鐘前 100kHz 才剛驗證正常，這個對照排除了「單純接線又鬆了」的解釋。**結論**：MPU6050 晶片本身支援 400kHz、暫存器算法也正確，但這組麵包板+杜邦線的實體佈線，寄生電容可能讓 SCL/SDA 實際上升時間超過 Fast mode 較嚴格的 300ns 預算(即使 `TRISE` 算得對)，屬於這組原型接線本身的訊號完整性極限，不是計算錯誤。**目前韌體維持 100kHz 為預設(可靠)**，400kHz 的暫存器常數保留在程式碼中但未使用——如果之後換成焊接、短導線的正式板子，這個 3.4 倍的效能提升應該還是拿得到的。

* **Phase 2: MuJoCo 神經義肢 3D 控制與仿真 (MuJoCo Simulation & Turnkey HIL Prototyping) 【Phase 1.5 驗證完畢後視結果排入】**
  * 引入 **MuJoCo** 生物物理動力學仿真框架（歐洲殿堂級機器人與計算神經科學實驗室核心標準工具）。
  * 實作即時解碼回報接縫（Bridging Layer），將已堅固屹立的 EdgeNeuro 引擎解碼出的意圖/多通道指令映射至 **MuJoCo 3D 神經義肢模型**（例如 MPL Hand 或 Shadow Hand），展現流暢神經控制閉環與力學運動反饋。
  * 針對免安裝 Demo，採用「**零自建前端切版、既有技術棧插栓即用**」策略：引入開源成熟的 Wasm 力學渲染環境（如 `mujoco-wasm` / 免運營開源科研圖表框架），直接封裝部署至 GitHub Pages，形成攻無不克之頂尖 PhD 申請作品清單（Research Portfolio）。
  * **Iteration 1：Python + MuJoCo 原生套件橋接，已完成並驗證（2026-08-22）**——選擇先用 Python + MuJoCo 官方套件（而非直接跳到 wasm）快速做出能動的 3D demo，wasm 移植留待之後獨立處理。
    - 新增 `src/mujoco_bridge_demo.cpp`：CSV 播放版的「真實韌體主迴圈分身」，刻意繞過 `Pipeline`/`EdgeNeuro<>`/`LdaClassifier`（那是 Phase 1 引擎通用性展示，不是 Phase 3 真實控制邏輯，見第 3 節架構修正），直接重用已在真實硬體驗證過的 `GripStateMachine`（EMG）+ `ComplementaryFilter`（IMU）+ `SlewRateLimiter`（平滑），逐筆讀 `data/wearable_1emg_6imu.csv`、以 1kHz 步調印出 `tick=<n> grip=<f> gripping=<0|1> roll=<f> pitch=<f>` 到 stdout（顯式 `flush()`，因為變成 pipe 之後 stdout 預設是 full-buffered、不是 line-buffered，否則 Python 端會卡住收不到資料）。
    - 感測器數量刻意對齊 Phase 1.5 實際驗證過的真實硬體規格（1 顆 Adafruit MPU-6050 + 1 顆 MyoWare 2.0），維持「MuJoCo 呈現的是真實裝置解碼輸出」的定位,不是憑空延伸的想像動畫。
    - 手部模型採用 `mujoco_menagerie` 的 Shadow Hand（Apache-2.0，`shadow_hand/scene_right.xml`，pin 在 commit `da76818e269b82289eba39808e2fb91d679d6994`）——透過 sparse checkout 取得，作為**未追蹤的本地依賴**（`mujoco_menagerie/` 已加入 `.gitignore`，不 vendor 進版本庫），`rh_forearm` 沒有 joint、焊死在世界座標，這個 iteration 不需要真的手臂骨架。
    - `tools/mujoco_bridge/run_demo.py`：`subprocess.Popen` 啟動 C++ binary、背景 `threading.Thread` 持續 `readline()` 解析 stdout 存進上鎖的共享物件（**不能**在 render loop 裡直接 `readline()`，否則會卡住等下一筆樣本、拖慢 MuJoCo 自己的步調），主迴圈依 MuJoCo 官方文件的 `launch_passive`/`mj_step`/`viewer.sync()` 節奏套用最新讀值。抓握映射：`grip∈[0,1]` 乘上各手指/拇指 actuator 的 ctrlrange 上界；`roll`/`pitch` clamp 到 `rh_A_WRJ2`/`rh_A_WRJ1` 的 ctrlrange 後直接送入（豎起大拇指、外展等 actuator 這個 iteration 不驅動，留在 `ctrl=0` 中性姿態）。macOS 上必須用套件自帶的 `mjpython`,不能用一般 `python3`（`launch_passive` 在 macOS 的 plain CPython 下會丟 `RuntimeError`)。
    - **實測驗證通過**：手指隨 EMG 收縮區間開合、手腕隨合成 IMU 訊號連續轉動，CSV 播放完畢後 viewer 正確 hold 住最後姿勢不當機，全程無多秒等待卡頓（證實 C++ 端 flush 修正確實有效）。
    - **繞了一圈的教訓（2026-08-22）**：session 中途曾嘗試擴充成雙 IMU 肩肘手臂 + 真實物理抓取物體，投入大量心力調 MJCF 骨架、物理接觸參數。後來使用者提醒「這樣做的初衷是什麼」，回頭檢視才發現：雙 IMU 手臂的感測器預算，跟這裡（第 6 節 Phase 1.5）記錄的真實硬體規劃(僅 1 顆 MyoWare、規劃中的 IMU 也只有 1 顆)對不上，等於是在幫一個尚未存在、也還沒定案的硬體做視覺化，偏離了「MuJoCo 呈現真實裝置解碼輸出」的原始目的。已回退到本節描述的單 IMU 版本；若之後真的要做手臂級展示，應該先確認 Phase 3 的感測器規格再回頭決定模擬範圍，而不是反過來。

---

### Stage 2: Target-Side Embedded Deployment & Actuation (硬體擴充階段)
Phase 1.5 的可行性驗證確認四個關鍵未知數皆可行後，再依 Phase 1.5 的實測結果，決定 MuJoCo 與完整硬體部署的投入比重與順序，逐步將韌體大腦置入物理 MCU 與神經電生理感測器中。

* **Phase 3: 硬體抽象層對接 (STM32 HAL / Sensor Stream)**
  * 於 CMake + `arm-none-eabi-gcc` 環境（延續 Phase 1.5 的 `firmware/` 專案），設定 ADC（讀取 **MyoWare 2.0** 肌電訊號）與 I2C（讀取 MPU6050 慣性通訊）。
  * 撰寫 HAL 抽象封裝層，實作 `Stm32AdcProvider` 取代原本的 `CsvSignalProvider`（透過 Timer + DMA 驅動採樣）。
  * 秉持零修改原則，將 Stage 1 已徹底驗證的 C++ Core 演算法檔案與 Ring Buffer 結構直接置入 MCU 編譯與運行。
  * **(HIL 物理仿真整合)**：透過 Serial / USB 傳輸，讓實體 **STM32F401RCT6 Black Pill** 上採集的肌電感測特徵，高達千赫茲地輸出給電腦端的 **MuJoCo 3D 義肢模擬環境**。瞬間讓專案升級擁有高級航空與機電控制專業領域的「**硬體在環（Hardware-in-the-Loop, HIL） 3D 神經義肢仿真測試中心**」！

* **Phase 4: 閉環致動與系統調校 (Closed-Loop Actuation & System Tuning)**
  * 設定 STM32 硬體 Timer 輸出高精度 PWM 訊號，驅動伺服馬達、仿生機械手掌或外部致動端。
  * 完成最終毫秒級閉環控制：「感測器採樣 $\rightarrow$ DMA Ring Buffer 接收 $\rightarrow$ 零動態配置 C++ 引擎解碼 $\rightarrow$ PWM 致動反饋」。
  * 解決物理世界工程挑戰：馬達驅動迴路與類比感測路徑的光偶接供電隔離 (Power Isolation)、共模電源雜訊濾除與肌電貼片阻抗匹配與調適。
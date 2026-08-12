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
| **單元測試與邊界檢查** | `Catch2` + `llvm-cov`（source-based coverage） | 核心數學、模組 Concept 介面測試覆蓋率 $> 90\%$。**已量測達成**：38 個 test case 涵蓋 `include/edgeneuro/` 全模組，line coverage 95.69%、region 95.24%、function 95.45%、branch 90.32%（排除第三方依賴與測試檔本身，2026-08-12 隨 `bump_allocator.hpp`（100% 覆蓋）加入後重新量測，較先前 95.38% 略升）。未覆蓋行集中於 `bad_alloc` 拋出路徑（刻意不測試，需真實記憶體耗盡）與標準要求的 sized-delete overload（此 ABI 下從未被選中呼叫），以及 `NoHeapGuard` 致命 abort 路徑本身——該路徑由 fork 出的子行程觸發 `SIGABRT`，子行程在覆蓋率計數器落盤前即被終止，屬量測工具限制而非測試缺口，其正確性改由 `test_no_heap_guard.cpp` 以 `WIFSIGNALED`/`WTERMSIG` 驗證。量測過程中亦揪出兩個真實邏輯漏洞：`CsvSignalProvider` 原本會靜默截斷欄位數過多的畸形資料列而非拒絕；`firmware/` 的 bump allocator 原本未對齊記憶體、且耗盡時會拋進沒有 unwind table 的環境。兩者皆已修正並補測試證實。 |
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
    1. Host 已驗證的 C++20 核心用 `arm-none-eabi-gcc` 編譯後，`NoHeapGuard` 在真實硬體上是否依然 `malloc_count == 0`。
    2. 完整 pipeline 的靜態記憶體足跡（`arm-none-eabi-size` 量測）是否塞得進 64KB SRAM / 256KB Flash。
    3. Timer + ADC 能否穩定達成 1kHz 取樣。
    4. MyoWare 2.0 貼在真人前臂上的訊號品質，是否足夠支撐可靠的即時開/合判斷（唯一無法用合成資料或任何公開資料集回答的問題）。
  * 建置工具鏈：裸 CMake + `arm-none-eabi-gcc`（延續 Host 端 CMake Presets 的風格，不引入 STM32CubeIDE），韌體專案位於 `firmware/`。透過 STM32F401 內建 USB DFU bootloader 燒錄，不強制要求 ST-Link；診斷手段以 LED 燈號與 UART/USB-CDC 輸出為主。

  * **硬體庫存狀態（2026-08-11）**：已有 STM32F401RCT6 Black Pill（已焊排針）、MyoWare 2.0 ×1。已下單但未到貨：ST-Link V2 相容品（蝦皮，約 NT$65）、CP2102 USB 轉 TTL 模組（樂意創客官方店，NT$85，附杜邦線，接腳 `3V3/TXD/RXD/GND/+5V`）。**避開 PL2303HXA 晶片款**（macOS 新版驅動不支援）。已有但目前用不到：MCP3008 SPI ADC、LM317 可調穩壓板、TXS0108E 邏輯電位轉換板、5V 繼電器模組（皆非本階段必需，未來若擴充或接馬達可能用得上）。伺服馬達、MPU6050、機械手掌結構件、獨立電源尚未購買，刻意留到 Phase 3 才處理。

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

  * **階段 0 + 階段 1 + 階段 2 已完成編譯驗證，尚未實際燒錄執行（2026-08-12）**：
    - **階段 0(blink）**：編譯連結成功，`text=212 bytes, data=0, bss=0`（含 VTOR 重定位後）。證實工具鏈「編譯→連結」全鏈路可行。
    - **階段 1(記憶體足跡)**：**真正的 `include/edgeneuro/*` 標頭檔（`pipeline.hpp`/`iir_filter.hpp`/`pass_through_filter.hpp`/`mav_feature.hpp`/`lda_classifier.hpp`），零修改，直接用 `arm-none-eabi-g++` 編譯連結給 STM32F401 成功**，組出真實的 `EdgeNeuro<1,6,50,...>` 實例（`firmware/src/footprint_check_main.cpp`）。實測 `text=932 bytes, data=0, bss=0`——僅佔 240KB 可用 Flash 的 0.38%、64KB SRAM 完全沒用到靜態配置。**「零修改移植」的承諾首次得到實體工具鏈驗證**，且記憶體餘裕遠超預期，第 2 節「未知數 2」（SRAM 夠不夠）初步無虞。
    - **階段 2(NoHeapGuard 裸機移植，等 ST-Link 到貨即可實測未知數 1）**：`include/edgeneuro/no_heap_guard.hpp`（純 `std::atomic`，無 OS 依賴）**沿用不修改**；新增 `firmware/src/no_heap_guard_target.cpp` 提供裸機版 `operator new`/`delete` 覆寫——違規反應從 Host 版的 `abort()`+`stderr` 改成快速 LED 閃爍（不需要 newlib 的 `_write`/`_exit` 這類系統呼叫 stub）。`firmware/src/heap_guard_check_main.cpp` 跑 10 萬次 `tick()`，全程武裝 `NoHeapGuard`，通過則 LED 恆亮，失敗則快閃——三種燈號（慢閃=階段0、恆亮=階段2通過、快閃=偵測到配置）肉眼可辨。編譯成功，`text=844 bytes, data=0, bss=8`。
    - **裝置端已確認**：板子能正確進入 HID bootloader 模式並被 Mac 偵測到、VID:PID 正確——硬體本身沒問題，卡關的是燒錄工具鏈，不是板子或接線。
    - **OpenOCD + ST-Link 設定已備妥**：`firmware/openocd.cfg`（`interface/stlink.cfg` + `target/stm32f4x.cfg`，`adapter speed 1000` 求穩不求快）。`CMakeLists.txt` 重構出 `firmware_add_target()` function 消除三個執行檔目標間的重複樣板，並為每個執行檔自動產生對應的 `flash_<name>` CMake target（例如 `cmake --build build --target flash_blink`），內部呼叫 `openocd -f openocd.cfg -c "program <bin> 0x08004000 verify reset exit"`。位址寫死在 `0x08004000` 而非全晶片抹除，確保燒錄只動到 Sector 1 以後，不會動到 Sector 0 的 HID bootloader（STM32F401 Sector 0/1 各自獨立 16KB，位址不重疊）。**ST-Link 一到貨，插上後直接下這個指令就能燒，不用臨時查設定。**
    - 三個執行檔都在 `firmware/build/`（`blink`、`footprint_check`、`heap_guard_check`），已產生對應 `.bin`，**不需要再重新編譯**。
    - `firmware/`（含本次新增檔案）已 commit 進 git。

  * **下一步（依序）**：(1) 等 ST-Link 到貨，接上後跑 `cmake --build build --target flash_blink`，肉眼確認 LED 閃爍，(2) `flash_heap_guard_check`，確認 LED 恆亮（= 未知數 1 驗證通過）而非快閃，(3) 待 USB-TTL 模組到貨後才能進行未知數 3、4（ADC 時序、MyoWare 真實訊號）——這兩步的韌體程式碼（Timer/ADC/DMA 設定、UART 驅動）尚未開始寫，等前面兩步實測過關後再寫較合理（避免在還沒驗證基礎假設前，疊加更多未驗證的程式碼）。

* **Phase 2: MuJoCo 神經義肢 3D 控制與仿真 (MuJoCo Simulation & Turnkey HIL Prototyping) 【Phase 1.5 驗證完畢後視結果排入】**
  * 引入 **MuJoCo** 生物物理動力學仿真框架（歐洲殿堂級機器人與計算神經科學實驗室核心標準工具）。
  * 實作即時解碼回報接縫（Bridging Layer），將已堅固屹立的 EdgeNeuro 引擎解碼出的意圖/多通道指令映射至 **MuJoCo 3D 神經義肢模型**（例如 MPL Hand 或 Shadow Hand），展現流暢神經控制閉環與力學運動反饋。
  * 針對免安裝 Demo，採用「**零自建前端切版、既有技術棧插栓即用**」策略：引入開源成熟的 Wasm 力學渲染環境（如 `mujoco-wasm` / 免運營開源科研圖表框架），直接封裝部署至 GitHub Pages，形成攻無不克之頂尖 PhD 申請作品清單（Research Portfolio）。

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
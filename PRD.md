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

* 提供多模態生理與慣性訊號的 `csv` 解析器（相容 Ninapro EMG 切片與慣性運動序列）。
* **感測器融合封包：** 支援 `1-Ch EMG + 6-Axis IMU` 同步資料流，精確模擬 $1000\text{ Hz}$（每 $1\text{ ms}$ 推播一次）之零動態配置管線輸入。
* **高密度擴充測試：** 相容高密度 `32-Ch EMG` 壓力測試數據格式，供自動化基準檢驗使用。

### 4.3 現成科研展示與 MuJoCo 神經義肢仿真 (Scientific UI & Neuroprosthetic Simulation)

* **演算法研發極大化：** 堅決屏除自建繁複網頁切版與維護 DOM 的工程虛耗，將主力心血 100% 投入在 C++20 即時 DSP 演算法與記憶體精簡優化。
* **階段一成果展示 (Turnkey Scientific Oscilloscope)：** 前期驗證只需直接利用開源業界高度最佳化的即插即用科研示波器或圖表組件（如 ImGui / uPlot / Streamlit），極速且高溫可靠地展示即時多通道 EMG 濾波波形、零 malloc 記憶體監測與次毫秒延遲儀表，以此為打磨核心引擎之基準。
* **階段二升維擴充 (MuJoCo 3D 物理閉環仿真)：** 待核心引擎穩固且達到我們團隊 100% 的絕對滿意與認同後，才啟動對接 **MuJoCo (Multi-Joint dynamics with Contact)** 物理模擬引擎。將解碼意圖直鏈驅動 3D 仿生義肢模型（如 Shadow Hand 或 Adroit Hand），展現世界頂尖的毫秒級閉環動作控制與動力學反饋（Closed-Loop Neural Motor Control），並透過現成的 Wasm 框架靜態部署於 GitHub Pages 進行線上展示。



## 5. 工程標準與效能驗證 (Engineering Standards & Validation)

為確保達到大型開源專案的程式碼品質，本專案實施嚴格的 CI/CD 與驗證標準：

| 驗證項目 | 使用工具 | 通過標準 (Success Criteria) |
| --- | --- | --- |
| **單元測試與邊界檢查** | `Catch2` | 核心數學、模組 Concept 介面測試覆蓋率 $> 90\%$。 |
| **零記憶體與安全防護** | `Clang/GCC Sanitizers` + 自訂 `NoHeapGuard` | 利用 ASan / UBSan / TSan 消除所有未定義行為與並發競態，並透過單元測試攔截驗證 Hot Loop 期間 `malloc_count == 0`。由於 ASan 與 TSan 無法連結進同一 binary，且自訂 allocator 覆寫會與 Sanitizer 自身的記憶體攔截機制衝突，兩類驗證拆分為互斥的 CMake Presets（`debug-heapguard` / `sanitize-asan-ubsan` / `sanitize-tsan` / `release-bench`）獨立執行，各司其職。 |
| **極限效能與擴充基準** | `Google Benchmark` | 同時檢測 `<1, 6>` 義肢融合模式與 `<32, 0>` 高密度壓測模式，驗證 32 通道連續運算延遲 $< 0.1\text{ ms}$ 且 `malloc_count == 0`。 |

## 6. 專案演化藍圖與開發紀律 (Strategic Roadmap & Progressive Discipline)

本專案奉行 **「厚積薄發、逐步升維」** 的開發紀律與「先 Host 後 Target」的工程戰略。**首要任務（Primary Focus）絕對鎖定在 Phase 1 的「軟體架構與極速確定性計算引擎」**；唯有當 Phase 1 的效能、記憶體 `0 Allocation` 驗證與 C++20 Concepts 抽象層讓我們完完全全滿意且無懈可擊時，才能開啟品管閘門（Quality Gate），穩步推進至 Phase 2 的 MuJoCo 物理動力學仿真與未來的實體嵌入式韌體移植。

```
+-----------------------------------------------------------------------------------+
| 🥇 階段一主體：首要專注戰略目標 (Primary Core Focus - Wasm/PC Engine)         |
| [Phase 1] 純軟體與演算法核心：徹底淬鍊 C++20 Concepts 與 Lock-free Ring Buffer |
+-----------------------------------------------------------------------------------+
                                        |
                         ⚠️ 品質閘門：必須達到完全滿意與零缺陷，才能准許升維 ⚠️
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

* **Phase 1: 純軟體與演算法核心 (C++20 / PC) 🔥 【當前絕對主力的專注目標】**
  * 定義 C++20 Concepts 介面合約（`SignalProvider`, `Filter`, `Feature`, `Classifier`）與數值型別模板（`ValueType`）。
  * 完成基於 `std::array` 與靜態記憶體的 Lock-free Ring Buffer 結構。
  * 實作 `CsvSignalProvider` 讀取公開資料集（如 Ninapro EMG 數值）來推播至系統，精準模擬即時感測器串流。
  * 結合 `Catch2` 進行數學與邏輯嚴謹測試，使用 **Clang/GCC Sanitizers (`ASan`/`UBSan`/`TSan`) + `NoHeapGuard` 鉤子** 消除並發與越界隱患並嚴格證實 `malloc_count == 0`，配合 `Google Benchmark` 驗證延遲 $< 1\text{ ms}$。
  * 運用現成的科研示波器工具鏈（如 ImGui / uPlot / 快速 Terminal 儀表），極速呈現即時波形與零內存耗用圖表，**以此作為 V1.0 的終極自我檢視門檻**。

* **Phase 2: MuJoCo 神經義肢 3D 控制與仿真 (MuJoCo Simulation & Turnkey HIL Prototyping) 【Phase 1 滿意過關後才開放解鎖】**
  * 引入 **MuJoCo** 生物物理動力學仿真框架（歐洲殿堂級機器人與計算神經科學實驗室核心標準工具）。
  * 實作即時解碼回報接縫（Bridging Layer），將已堅固屹立的 EdgeNeuro 引擎解碼出的意圖/多通道指令映射至 **MuJoCo 3D 神經義肢模型**（例如 MPL Hand 或 Shadow Hand），展現流暢神經控制閉環與力學運動反饋。
  * 針對免安裝 Demo，採用「**零自建前端切版、既有技術棧插栓即用**」策略：引入開源成熟的 Wasm 力學渲染環境（如 `mujoco-wasm` / 免運營開源科研圖表框架），直接封裝部署至 GitHub Pages，形成攻無不克之頂尖 PhD 申請作品清單（Research Portfolio）。

---

### Stage 2: Target-Side Embedded Deployment & Actuation (硬體擴充階段)
當 C++ 核心與 MuJoCo 物理閉環架構在電腦與力學模擬中經過徹底驗證與壓力測試後，再一步進階平滑將韌體大腦置入物理 MCU 與神經電生理感測器中。

* **Phase 3: 硬體抽象層對接 (STM32 HAL / Sensor Stream)**
  * 於 STM32CubeIDE 或 CMake 環境配置 ARM GCC 编譯鏈，設定 ADC（讀取 MyoWare 肌電訊號）與 I2C（讀取 MPU6050 慣性通訊）。
  * 撰寫 HAL 抽象封裝層，實作 `Stm32AdcProvider` 取代原本的 `CsvSignalProvider`（透過 Timer + DMA 驅動採樣）。
  * 秉持零修改原則，將 Stage 1 已徹底驗證的 C++ Core 演算法檔案與 Ring Buffer 結構直接置入 MCU 編譯與運行。
  * **(HIL 物理仿真整合)**：透過 Serial / USB 傳輸，讓實體 STM32 上採集的肌電感測特徵，高達千赫茲地輸出給電腦端的 **MuJoCo 3D 義肢模擬環境**。瞬間讓專案升級擁有高級航空與機電控制專業領域的「**硬體在環（Hardware-in-the-Loop, HIL） 3D 神經義肢仿真測試中心**」！

* **Phase 4: 閉環致動與系統調校 (Closed-Loop Actuation & System Tuning)**
  * 設定 STM32 硬體 Timer 輸出高精度 PWM 訊號，驅動伺服馬達、仿生機械手掌或外部致動端。
  * 完成最終毫秒級閉環控制：「感測器採樣 $\rightarrow$ DMA Ring Buffer 接收 $\rightarrow$ 零動態配置 C++ 引擎解碼 $\rightarrow$ PWM 致動反饋」。
  * 解決物理世界工程挑戰：馬達驅動迴路與類比感測路徑的光偶接供電隔離 (Power Isolation)、共模電源雜訊濾除與肌電貼片阻抗匹配與調適。
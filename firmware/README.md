# firmware/

Phase 1.5 可行性驗證(feasibility spike):範圍刻意縮小的小型檢查,確認 Host 端已驗證的 `include/edgeneuro/` 引擎在真實 STM32F401 目標硬體上真的能跑,再決定要不要投入完整 Phase 3(HAL 層、閉環致動)。完整動機、這個階段要回答的四個未知數、以及目前為止的進度記錄請見 [PRD.md](../PRD.md) 的 Phase 1.5 章節——這份檔案只涵蓋日常的編譯/燒錄指令跟硬體接線。

## 目標硬體

- **MCU**:STM32F401RCT6「Black Pill」—— Cortex-M4F @ 84MHz(目前尚未設定時脈,韌體跑在預設的 16MHz HSI)、256KB Flash、64KB SRAM。
- **Bootloader**:這片板子出廠已預燒 WeAct Studio 的 HID bootloader,佔用 Flash 開頭 16KB(`0x08000000`–`0x08003FFF`)。我們的應用程式從 `0x08004000` 開始——見 `linker/STM32F401RCTx_FLASH.ld`。**不要把 `FLASH ORIGIN` 改回 `0x08000000`**——那樣會覆蓋掉 bootloader。
- **EMG 感測器**:MyoWare 2.0,1 通道。

## 目錄結構

```
cmake/arm-none-eabi-toolchain.cmake   Cortex-M4F 交叉編譯工具鏈設定檔
linker/STM32F401RCTx_FLASH.ld         Flash/RAM 記憶體配置、應用程式起始位址(0x08004000)
openocd.cfg                           ST-Link + STM32F4 目標晶片設定
src/startup.c                         手寫 vector table + Reset_Handler(不用 CMSIS 官方 startup 檔)
src/main.c                            階段 0:LED 閃爍
src/footprint_check_main.cpp          階段 1:真正的 include/edgeneuro/* 為這個目標平台編譯
src/no_heap_guard_target.cpp          階段 2:裸機版 NoHeapGuard(違規反應是 LED 燈號,不是 abort())
src/heap_guard_check_main.cpp         階段 2:跑武裝過的 tick() 迴圈,用 LED 回報過/不過
src/uart_hello_main.c                 階段 3a:USART2 輪詢送出文字(PA2/PA3),獨立於 Timer/ADC 先驗證 UART 本身
src/adc_hello_main.c                  階段 3b:ADC1 單通道(PA0)軟體觸發輪詢讀取,獨立於 Timer 先驗證 ADC 本身
src/timer_adc_1khz_main.c             階段 3c/3d:TIM2 以硬體 TRGO 定時觸發 ADC1,真正的 1kHz 取樣
src/i2c_mpu6050_hello_main.c          階段 4a:I2C1(PB6/PB7)讀取 MPU6050/GY-521 加速度計+陀螺儀原始值
src/complementary_filter_hello_main.cpp  階段 4b:ComplementaryFilter 融合真實 MPU6050 資料(等新模組到貨才能驗證)
src/i2c_bus_scan_main.c               診斷工具(非 pipeline 階段):掃描 I2C1 全部位址,見 tools/check_hardware_ready.py --i2c-scan
src/complementary_filter_stress_test_main.cpp  診斷工具(非 pipeline 階段):合成資料跑 ComplementaryFilter 迴圈,不需要真的 MPU6050
src/emg_grip_control_main.cpp         階段 5a:真實 MyoWare 訊號驅動 GripStateMachine + SlewRateLimiter,不依賴 MPU6050
src/phase3_control_loop_main.cpp      階段 5b:EMG+IMU 合併成真正的 1kHz 主迴圈,不阻塞 I2C 讀取狀態機
```

## 工具鏈設定(僅需一次)

Homebrew 的 `arm-none-eabi-gcc` formula 只有編譯器本體,不含 newlib(連 `<stdint.h>` 都沒有)。改用 ARM 官方的 GNU 工具鏈壓縮檔:

```sh
curl -L -o /tmp/arm-gnu-toolchain.tar.xz \
  "https://armkeil.blob.core.windows.net/developer/files/downloads/gnu/15.2.rel1/binrel/arm-gnu-toolchain-15.2.rel1-darwin-arm64-arm-none-eabi.tar.xz"
mkdir -p ~/.local/arm-toolchain
tar -xJf /tmp/arm-gnu-toolchain.tar.xz -C ~/.local/arm-toolchain --strip-components=1
brew install dfu-util openocd
```

`cmake/arm-none-eabi-toolchain.cmake` 明確指向 `~/.local/arm-toolchain`,不是看 `PATH` 上有什麼就用什麼。

## 編譯

```sh
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-toolchain.cmake
cmake --build build --target blink footprint_check heap_guard_check uart_hello adc_hello timer_adc_1khz i2c_mpu6050_hello -j
```

每個目標編譯完都會(透過 `objcopy`)產生對應的 `.bin`,並在每次編譯時印出 `arm-none-eabi-size` 的輸出——Flash/SRAM 用量不需要另外量測就看得到。

## 燒錄(需要 ST-Link 透過 SWD 連接)

```sh
cmake --build build --target flash_blink            # 燒完看板子上的 LED 是否慢速閃爍(~1Hz)
cmake --build build --target flash_footprint_check   # 只證明不會當機，目前還沒有過/不過的燈號
cmake --build build --target flash_heap_guard_check  # LED 恆亮 = 過，快閃 = 偵測到 malloc_count 違規
cmake --build build --target flash_uart_hello        # 接上 USB-TTL 後，@ 9600 baud 應該每秒收到一行文字
cmake --build build --target flash_adc_hello         # PA0 接 MyoWare ENV 後，應該每秒收到一行 ADC 原始值
cmake --build build --target flash_timer_adc_1khz    # 每 1000 樣本回報一次，回報間隔應該接近 1 秒
cmake --build build --target flash_i2c_mpu6050_hello # 接上 MPU6050 後，應該每秒收到一行 accel/gyro 原始值
```

每個 `flash_<name>` target 執行的是 `openocd -f openocd.cfg -c "program <bin> 0x08004000 verify reset exit"`——只會寫入 Sector 1 以後的區域，不會動到 bootloader 所在的 Sector 0。

## 已在真實硬體上驗證過(2026-08-14)

ST-Link 到貨後，階段 0 + 階段 2 已經實際燒錄並在 STM32F401 板子上目視確認過，不只是編譯通過而已。以下是實際使用的驗證流程，依序執行——之後任何改動都可以重跑這個流程，確認「工具鏈→硬體」這條路徑還是通的。

**1. 燒錄前先確認 ST-Link 跟目標晶片真的能連上：**

```sh
openocd -f openocd.cfg -c "init; reset halt; exit"
```

這一步是唯讀的——會讓 CPU 暫停，但不會寫入任何東西到 Flash。連線正常的話會印出類似這樣的內容：

```
Info : STLINK V2J37S7 (API v2) VID:PID 0483:3748
Info : Target voltage: 3.276242
Info : [stm32f4x.cpu] Cortex-M4 r0p1 processor detected
[stm32f4x.cpu] halted due to debug-request, current mode: Thread
```

依序檢查：ST-Link 的 VID:PID 有被正確識別(轉接器本身沒問題)→ target voltage 是合理的 ~3.3V(SWD 接線的電源/接地有導通)→ 有偵測到 `Cortex-M4`(這真的是一顆 STM32F4 系列晶片在回應，不是接線巧合)。只要有一項對不上，就不要往下燒錄——先排查連線問題(通常是 SWDIO/SWCLK/GND 其中一條杜邦線鬆脫)。

**2. 燒錄階段 0(`blink`)並目視確認：**

```sh
cmake --build build --target flash_blink
```

在 OpenOCD 輸出裡找 `** Programming Finished **` / `** Verify Started ** / ** Verified OK **`，以及 `Info : flash size = 256 KiB` 是否對得上這顆晶片實際的 Flash 容量(對不上的話代表 OpenOCD 選錯了目標晶片設定)。接著**看板子**：PC13 的 LED 應該以大約 1Hz 的頻率閃爍(~500ms 亮、~500ms 暗)。這是第一個「真的有東西在晶片上執行」的驗證點，不只是連結成功而已。

**3. 燒錄階段 2(`heap_guard_check`)並目視確認——這是未知數 1 的測試(`malloc_count == 0` 在真實硬體上是否成立，而不只是 Host 端)：**

```sh
cmake --build build --target flash_heap_guard_check
```

OpenOCD 的成功訊號跟步驟 2 一樣。接著再看一次 LED：
- **恆亮(不閃)** = 通過——10 萬次 `tick()` 全程武裝 `NoHeapGuard`，從未偵測到任何配置行為。
- **快速閃爍** = 失敗——偵測到真實的配置行為；代表 Host 端驗證過的零配置保證在這個目標平台的實際 ARM GCC 編譯結果下不成立，需要進一步調查才能繼續信任這個保證。

**這片板子的實測結果**：兩個階段都是第一次燒錄就通過——階段 0 慢速閃爍、階段 2 恆亮，解決了未知數 1、2。未知數 3(Timer/ADC 1kHz 取樣)已在下方階段 3c/3d 解決；未知數 4(真實 MyoWare 訊號品質)仍待正式用電極貼片實測，詳見 PRD.md 的 Phase 1.5 章節。

**4. USB-TTL 到貨後，燒錄階段 3a(`uart_hello`)並實際監聽序列埠：**

```sh
cmake --build build --target flash_uart_hello
```

暫存器值(`PA2`/`PA3` 的 AF7、`USART_CR1`/`USART_SR`/`USART_BRR` 各欄位、鮑率換算)全部先對照 ST 官方 RM0368 參考手冊跟 STM32F401CCU6 datasheet 查證過才寫，不是猜的——這是先前「未驗證暫存器程式碼不能寫」規則生效後，第一次真正把 Timer/ADC 之外的周邊(USART)也走完「查證→實作→實測」全流程。

燒完用序列埠工具監聽(鮑率 9600,8N1)：

```sh
python3 -c "
import serial, time
ser = serial.Serial('/dev/tty.usbserial-0001', 9600, timeout=1)
time.sleep(0.5)
for _ in range(5):
    print(ser.readline())
"
```

（`screen`/`stty`+`cat` 這類互動式工具在 macOS 上有時抓不到輸出，`pyserial` 比較穩定，建議優先用這個方式驗證。）

**實測結果**：LED 正常閃爍(證實韌體有在跑，跟 UART 收發是獨立的診斷訊號)，序列埠收到乾淨、無亂碼、重複出現的 `"EdgeNeuro Stage 3a: UART alive"` 字串——接線、暫存器設定、鮑率計算全部驗證正確。

**5. 燒錄階段 3b(`adc_hello`)——ADC 單獨驗證，在加 Timer 之前先確認 ADC 本身沒問題：**

```sh
cmake --build build --target flash_adc_hello
```

暫存器值(`RCC_APB2ENR` 的 `ADC1EN`、`ADC_CR2` 的 `ADON`/`CONT`/`SWSTART`、`ADC_SQR1`/`ADC_SQR3` 的通道序列、`ADC_SR` 的 `EOC`)查證自 RM0368 第 11 章。同樣用 `pyserial` 監聽，應該每秒收到一行 `PA0 ADC1_IN0 raw = <0~4095>`。**實測結果**：MyoWare 接上、手指按電極測試靜止狀態，讀到穩定落在 1279~1316 的值，遠比懸空雜訊乾淨,證實 ADC 讀取跟 `PA0` 接線正確。

**6. 燒錄階段 3c/3d(`timer_adc_1khz`)——把 Timer 跟 ADC 接在一起，真正回答未知數 3：**

```sh
cmake --build build --target flash_timer_adc_1khz
```

這是第一次真正把 TIM2 的硬體 TRGO 訊號接去觸發 ADC1，取樣率完全由 `TIM2_PSC`/`TIM2_ARR` 決定，不靠軟體迴圈計時。暫存器值(`TIM2CLK` 換算、`TIMx_CR2` 的 `MMS`、`ADC_CR2` 的 `EXTEN`/`EXTSEL`)查證自 RM0368 第 13 章 + 第 11 章。用 `pyserial` 監聽每 1000 樣本回報一次的訊息，量測相鄰回報的實際時間間隔：

```sh
python3 -c "
import serial, time
ser = serial.Serial('/dev/tty.usbserial-0001', 9600, timeout=1)
time.sleep(0.5)
ser.reset_input_buffer()
prev = None
for _ in range(5):
    line = ser.readline()
    now = time.time()
    print(line, '' if prev is None else f'(+{now-prev:.3f}s)')
    prev = now
"
```

**實測結果**：連續回報間隔實測 **1.005s、1.005s、1.002s**——非常接近理論上剛好 1 秒，誤差 <0.5%，落在未校準 16MHz HSI 振盪器本身的正常誤差範圍內。**這是第一次有實際計時數據證實 1kHz 取樣穩定成立，未知數 3 正式解決。**

**7. 燒錄階段 4a(`i2c_mpu6050_hello`)——I2C1 讀取 MPU6050/GY-521：**

```sh
cmake --build build --target flash_i2c_mpu6050_hello
```

暫存器值(STM32 端 `RCC` 對應 bit、`I2C_CR2`/`I2C_CCR`/`I2C_TRISE` 的 100kHz 時序；MPU6050 端 `PWR_MGMT_1`/`WHO_AM_I`/`ACCEL_XOUT_H` 起算的暫存器位址、accel/gyro 縮放係數)查證自 RM0368 第 18 章 + InvenSense 官方 `RM-MPU-6000A-00`/`PS-MPU-6000A-00` 兩份文件。**實測結果**：靜止平放讀到 `accel_z≈16850`(≈1.03g，符合重力沿 Z 軸)、`accel_x`/`y` 接近 0、`gyro_x/y` 在正常零偏範圍內——完整驗證成功。

這次除錯過程很曲折，值得記錄下來的教訓：
- **UART 不穩定不一定是轉接器的問題**——這次追了老半天才發現是 `TXD`/`RXD` 接錯到 `A1`/`A2`(該接 `A2`/`A3`)。UART 突然不穩定時，先確認接線，不要預設是硬體/驅動問題。
- **I2C 卡死可以直接用 `I2C1_SR2` 的 `BUSY` 位元確認**，不用猜——透過 SWD 直接讀暫存器，比重複拔插、看 LED 燈號快得多也準確得多。
- **多位元組 I2C 讀取的最後兩個 byte，一定要用 `BTF` 而不是 `RXNE` 控制 NACK/STOP 時機**——RM0368 §18.3.3 的 N>2 byte 接收程序有明確規定，跳過這個細節在單一 byte 讀取時不會出錯，但多 byte 讀取會可靠失敗。
- **UART 不可靠時，直接用 `openocd halt`/`reg pc`/`mdw <addr>` 透過 SWD 讀暫存器跟記憶體，比一直問使用者「LED 現在閃成怎樣」有效率、也精確得多**——這是之後遇到類似狀況應該優先採用的除錯方式。
- **I2C `BUSY` 卡死可以用 `I2C_CR1.SWRST` 軟體重置解決**(RM0368 18.6.1)——設 1 再清 0，強制把周邊內部狀態機(含 `BUSY` 旗標)復位，比重新上電更快，`i2c1_init()` 現在每次都會做這一步。
- **懷疑「是不是這顆晶片本身壞了」時，最乾淨的排除法是換一顆完全不同廠牌/晶片的 I2C 裝置測試，而不是一直換線或換位置**——這次卡在 MPU6050 疑似故障時，寫一個掃過全部 128 個位址的 bus scanner、改接一顆 LCD1602(PCF8574)背板，一次就確認 STM32 端韌體完全正常、問題 100% 在 MPU6050 模組本身(而且兩顆獨立購入的模組都一樣)。這招比反覆排查接線更快定案，之後遇到類似「疑似晶片故障」的狀況可以優先考慮。

## MyoWare 2.0 接線(給階段 4，等 ST-Link + USB-TTL 都到貨後用)

出自 SparkFun 官方 MyoWare 2.0 文件：

| MyoWare 接腳 | 接到 | 備註 |
| --- | --- | --- |
| `VIN` | STM32 3.3V | 感測器可接受 2.27V–5.47V；用 3.3V 供電可以讓 `ENV` 的 0–VIN 輸出範圍剛好對上 STM32 ADC 的 0–3.3V 輸入範圍——不需要位準轉換。 |
| `GND` | STM32 GND | 共地，無論如何都需要。 |
| `ENV` | STM32 PA0(`ADC1_IN0`) | 包絡偵測後的輸出——SparkFun 建議直接接 ADC 輸入用這個接腳(相對於板子背面給進階自訂後處理用的原始/整流測試墊)。 |

電極貼片位置(底面卡扣接頭，用於前臂屈肌/握拳偵測)：
- **MID** —— 肌肉肌腹(前臂中段，掌側)
- **END** —— 靠近手腕，同一條肌肉
- **REF** —— 骨頭上/中性位置(例如手肘)——SparkFun 特別提醒 REF 接觸不良會降低訊號品質

## 供電原則(單一電源，2026-08-18)

麵包板開發階段，任何時候**只讓一個裝置真正供電，其他全部只接訊號線**——這個 session 先後在 USB-TTL(見下方 `VCC` 那一列)跟 ST-Link 上都踩過「兩個電源同時驅動同一條軌」的風險，與其每個裝置個別小心，不如定下這條通用規矩：

- **板子本身**：接一條 USB 線到 Black Pill 自己的 USB 孔供電(不透過 ST-Link)，這樣同時拿到乾淨的 3.3V(板上穩壓輸出)跟 5V(USB VBUS 直通)兩條軌。
- **ST-Link**：只接 `SWCLK`/`SWDIO`/`GND` 三條訊號線，**不接 `3.3V`/`VCC`**——純粹燒錄/除錯用，不供電。
- **USB-TTL**：只接 `GND`/`TXD`/`RXD`，`VCC` 不接(見下方)。
- 所有裝置的 `GND` 最後都要接在一起(同一條麵包板地軌)。

這不完全等於 Phase 3 最終產品的供電方式(義肢成品會用獨立電池，不會插著 USB 線)，純粹是為了讓麵包板開發階段更穩定、少踩雷。

## USB-TTL 接線(階段 3/4 診斷用 UART)

腳位選 **USART2**(`PA2`=TX、`PA3`=RX)——這不是猜的，是這片板子(WeAct Black Pill)在 Zephyr 官方 board 文件裡被指定為這片板子的標準 debug console UART(`STDIO_UART_TX`=PA2、`STDIO_UART_RX`=PA3)，且已確認跟目前用到的其他腳位不衝突：`PC13`=LED、`BOOT0`=開機模式按鈕(不是一般 GPIO，跟 UART 無關)、`PA13`/`PA14`=SWD(ST-Link 用)、`PA0`=MyoWare ADC。

| USB-TTL 接腳 | 接到 | 備註 |
| --- | --- | --- |
| `GND` | STM32 GND | 共地。 |
| `TXD` | STM32 `PA3`(USART2_RX) | 交叉接——轉接器的 TX 接到 MCU 的 RX。 |
| `RXD` | STM32 `PA2`(USART2_TX) | 交叉接——轉接器的 RX 接到 MCU 的 TX。 |
| `VCC`(3V3/5V) | **不要接** | 板子目前已經有電源(USB 或 ST-Link 供電)，USB-TTL 的 VCC 如果同時接上會變成兩個電源同時驅動 3.3V 軌，有把其中一顆穩壓晶片燒壞的風險。只接 GND + TXD + RXD 三條線就好。 |

板子插上後，Mac 端可以先確認有正確列舉出序列埠(`ls /dev/tty.usbserial-*` 或 `ls /dev/tty.SLAB_USBtoUART*`，依轉接器晶片而定)，這一步不需要 STM32 這邊已經有任何 UART 韌體，純粹確認轉接器本身有被系統認出來。

**燒錄/監聽前，建議先跑 `python3 ../tools/check_hardware_ready.py`(或從 repo 根目錄 `python3 tools/check_hardware_ready.py`)**，確認 ST-Link 跟 USB-TTL 都真的準備好——這個腳本不只檢查裝置有沒有列舉出來，還會實際打開序列埠、設定好參數，這是開發過程中 USB-TTL 反覆出問題後才發現「裝置有列出來」不代表「真的能用」，寫這個腳本就是要一次檢查到位，不用每次手動排查。

**懷疑 I2C 裝置(MPU6050 或其他)沒反應時，加 `--i2c-scan`**：`python3 tools/check_hardware_ready.py --i2c-scan`。這會自動燒錄 `firmware/src/i2c_bus_scan_main.c`(一個永久保留的診斷用韌體，不是 pipeline 階段)，掃過全部 0～127 位址，回報哪些位址有 ACK、匯流排是否卡在 `BUSY`——這是把階段 4b 卡關時整套手動 SWD 排查流程（`BUSY` 位元檢查、換位址/換接腳/換周邊、最後拿 LCD1602 交叉驗證）自動化，只需要 ST-Link，不需要 USB-TTL。以後懷疑某顆 I2C 裝置有問題，先跑這個，不用重新手動想一次怎麼查。

## MPU6050/GY-521 接線(階段 4a)

| GY-521 接腳 | 接到 | 備註 |
| --- | --- | --- |
| `VCC` | STM32 **5V** | MPU6050 晶片本身只吃 3.3V，但 GY-521 板子上通常還有一顆自己的穩壓晶片(常見標示 "KB33")，需要一定壓差才能穩定輸出，官方/社群普遍建議餵 5V 進 `VCC` 而不是直接給 3.3V，否則穩壓後實際到晶片的電壓可能不夠。原本用 3.3V 供電(推理是晶片規格 2.375V–3.46V、3.3V 最保守)，但這個假設沒考慮到板子上這顆穩壓晶片的壓差問題，已改成 5V。 |
| `GND` | STM32 GND | 共地。 |
| `SCL` | STM32 `PB6`(I2C1_SCL，AF4) | datasheet Table 9 查證，跟現有接線(`PA0`/`PA2`/`PA3`/`PA13`/`PA14`)都不衝突。 |
| `SDA` | STM32 `PB7`(I2C1_SDA，AF4) | 同上。 |
| `XDA`/`XCL`/`ADO`/`INT` | 不接 | 這次用不到(輔助 I2C 主機、位址選擇、中斷輸出)，保持空接。 |

**麵包板注意事項**：模組的 8 根針腳務必完全插到底——沒插緊會造成 I2C 匯流排卡在 `BUSY` 狀態，連 `START` 訊號都發不出去，症狀是韌體整個看起來像卡死。可以用 `openocd -f openocd.cfg -c "init" -c "halt" -c "mdw 0x40005418 1" -c "resume" -c "shutdown"` 直接讀 `I2C1_SR2`，`BUSY`(bit 1)如果是 1 就代表匯流排卡住。

`WHO_AM_I`(位址 0x75)官方文件寫死是 `0x68`，但實測這批 GY-521 板子(可能是相容/副廠晶片)穩定回報 `0x72`——韌體已經改成同時接受兩個值，不要看到不是 `0x68` 就假設接線有問題，先確認數值是否**穩定重複**(多次重新燒錄結果一致)。

**⚠️ 目前狀態(2026-08-18)：兩顆 GY-521 都無法通訊，問題已隔離在模組本身，5V 供電也無效。** 詳見 PRD.md 階段 4b 章節的完整排除記錄——STM32 端的 I2C1 韌體已經多次用 LCD1602(PCF8574，位址 `0x27`)交叉驗證完全正常(包含改成上面「單一電源」接法之後也重新驗證過)，但兩顆獨立購入的 MPU6050/GY-521 模組送出位址後都收不到 ACK，換成 5V 供電(見上方 `VCC` 那一列的原因)後結果依然相同。在拿到來源不同的新模組(已下單 Adafruit MPU-6050)之前，這條線暫時卡住。

## 階段 5a:EMG 抓放控制(`emg_grip_control`)

```sh
cmake --build build --target flash_emg_grip_control
```

不等 MPU6050，先驗證 Phase 3 拆分架構(見 PRD.md 第 3 節)裡不依賴 IMU 的那一半：沿用階段 3c/3d 已驗證的 TIM2+ADC1 硬體觸發 1kHz 讀取 MyoWare `ENV`，餵進 `include/edgeneuro/control/grip_state_machine.hpp` 的 `GripStateMachine` 再接 `slew_rate_limiter.hpp` 的 `SlewRateLimiter`。接線同「MyoWare 2.0 接線」+「USB-TTL 接線」兩節，`ENV → PA0`。

**用 `python3 tools/watch_myoware_uart.py` 即時監看**（已支援這個階段的輸出格式）。

**閾值校準（`kCalibrationEnabled`，預設關閉）**：`emg_grip_control_main.cpp` 開頭有一個編譯期開關。預設 `false`，開機直接用 `kFallbackThreshold`（目前是上次真實校準得到的 2037）進入正常運作，不需要任何人互動。想重新校準時改成 `true`、重新編譯燒錄，開機後會透過 UART 互動式引導：先印出「放鬆，準備好後送任意一個位元組」，等收到位元組才開始 3 秒取樣，握拳階段同理——**刻意不用固定延遲**，因為透過對話/腳本操作時的真人反應時間跟韌體自主計時對不上，之前踩過兩次坑。互動時可以用一小段 Python(`serial.Serial(port, 9600); ser.write(b'\n')`)在準備好的當下送出觸發位元組。校準完成後會印出 `relaxed_max`/`contracted_min`/算出的 `threshold`，記得把新數字更新回 `kFallbackThreshold`，下次開機才不用重新校準。

**實測結果(2026-08-18)**：真實電極訊號動態範圍遠比階段 3e 手指觸碰測試乾淨——放鬆 `~434-495`、持續握拳 `~3700+`，原本沿用階段 3e 舊資料設的 `threshold=1220` 意外地已經落在中間、不用調。反覆握拳/放鬆循環測試，轉態(`EDGE -> Gripping`/`EDGE -> Released`)都乾淨對應真實動作，放鬆時的自然訊號衰減也沒有被誤判成雜訊觸發。順便把先前延後處理的 MyoWare 增益調整(未知數 4)用真實資料收尾。

## 階段 5b:EMG+IMU 合併主迴圈(`phase3_control_loop`)

```sh
cmake --build build --target flash_phase3_control_loop
```

把階段 5a 的 EMG 迴圈跟 `ComplementaryFilter` 姿態融合合併成同一個 1kHz 主迴圈——這是真正接近 Phase 3 最終韌體的形狀。核心是 `ImuReader`：把一次完整 14-byte I2C 讀取拆成 10 個顯式狀態，`step()` 每次只檢查一個硬體旗標就返回，不會阻塞 EMG 取樣。接線同時需要「MyoWare 2.0 接線」（EMG，`ENV → PA0`）跟「MPU6050/GY-521 接線」兩節（IMU，`SCL → PB6`、`SDA → PB7`）。

**實測結果(2026-08-20/21，開發階段用 LCD1602 當替代品)**：第一版把 `imu_reader.step()` 綁在 `if (ADC1->SR & ADC_SR_EOC)` 裡,只跟著 EMG tick(1ms)呼叫一次,一秒只完成 47-48 次完整讀取——遠低於 100kHz 匯流排理論上限(約 500-650 次/秒)。原因是輪詢頻率被 tick 卡住,不是匯流排慢。把 `step()` 移到主迴圈最外層、跟 ADC EOC 判斷脫鉤,讓它利用兩次取樣之間的 CPU 閒置時間盡量多跑,改完後**躍升到 592-593 次/秒**,非常接近硬體上限。EMG tick 節奏全程沒受影響,證實不阻塞設計確實有效。

**指令軌跡平滑化(2026-08-21)**：`roll_smoother`/`pitch_smoother`(`IirFilter`,單極指數移動平均,`kSmoothAlpha=0.5`)已經接上,套用在 `roll`/`pitch` 輸出。`kSmoothAlpha` 是暫定值,不是推導出來的截止頻率。

**`ImuReader` 逾時自我修復**：逾時後除了 `STOP`,也會比照 `i2c1_init()` 做一次完整 `SWRST` + 重新設定 `CR2`/`CCR`/`TRISE`,讓暫時性的匯流排 `BUSY` 卡死能自動恢復,不用每次都手動重插線、重新燒錄。診斷用全域變數 `g_timeout_count`/`g_imu_state_at_timeout`/`g_sr1_at_timeout`/`g_sr2_at_timeout` 可以用 SWD 讀出來確認有沒有在重試。

## 真實 Adafruit MPU-6050 到貨,`kImuTargetAddr` 改回 `0x68`(2026-08-22)

接腳(跟舊的 GY-521 clone 不一樣,已查證 Adafruit 官方文件,見下方接線表)：`Vin → STM32 3.3V`(板上穩壓器官方支援 3-5V 都能穩定轉換,不用像 GY-521 那樣改接 5V)、`GND`、`SCL → PB6`、`SDA → PB7`。`AD0` 不接時預設拉低,位址 `0x68`,跟程式碼原本設定一致。

**接上完整迴圈後修好一個真實 bug**：這份韌體是先拿 LCD1602 開發的,從沒寫過喚醒感測器那一步(`PWR_MGMT_1`/`0x6B` 清 `SLEEP` 位元)——階段 4a、`complementary_filter_hello_main.cpp` 都有做,唯獨這個合併版本漏了。症狀：協定完全正常(`imu_completions` 一樣 591-593 次/秒),但 `roll`/`pitch` 永遠是 0。新增 `mpu6050_write_reg_blocking()`,開機時做一次(阻塞式沒關係,只跑一次,不影響主迴圈),比照階段 4a 已驗證的寫入序列喚醒感測器。

**實測結果**：`roll_x1000`/`pitch_x1000` 靜止時穩定,手動傾斜/旋轉板子時即時、正確連續變化(觀察到 roll 1150–1705、pitch 381–1218 隨動作變動),`roll_smoothed`/`pitch_smoothed` 正確跟隨。`imu_completions` 維持 591-593 次/秒,**跟先前 LCD1602 替代測試量到的速度一致**——證實輪詢脫鉤的速度分析換成真實感測器後依然成立。本專案第一次完整驗證：真實 IMU 資料 → `ComplementaryFilter` 融合 → `IirFilter` 平滑,全部在真實硬體、真實動作下正確運作。

**I2C Fast Mode(400kHz)測過了,晶片/算式都沒問題,但這組麵包板接線不穩定(2026-08-22)**：`CCR=14`/`TRISE=6`(RM0368 18.6.8/18.6.9 Fast mode 公式算出,不是隨便選的)第一次測完全成功,`imu_completions` 從 592/秒衝到 **2028/秒**(約 3.4 倍)。但接線稍微被碰過(搖晃測試抖動時)之後,**同一組已經在 100kHz 下驗證正常的接線,只改回 400kHz 就立刻 `BUSY` 卡死**——排除了單純接線鬆脫的解釋,結論是麵包板杜邦線的寄生電容撐不住 Fast mode 較嚴格的上升時間預算,不是計算錯誤。**目前預設維持 100kHz(可靠)**,400kHz 的常數(`kI2cCcrFastMode400k`/`kI2cTriseFastMode400k`)保留在程式碼裡但未使用——換成焊接短導線的正式板子後這個提速應該還能用。

## MPU6050 接線(Adafruit,階段 5c)

| Adafruit 接腳 | 接到 | 備註 |
| --- | --- | --- |
| `Vin` | STM32 **3.3V** | 板上穩壓器官方文件確認支援 3-5V 輸入都能穩定轉換,不像先前的 GY-521 clone 有壓差問題,不需要改接 5V。 |
| `GND` | STM32 GND | 共地。 |
| `SCL` | STM32 `PB6` | 跟現有 I2C1 設定一致。 |
| `SDA` | STM32 `PB7` | 同上。 |
| `3Vo`/`INT`/`AD0`/`FS`/`SCE`/`SDE`/`CLKIN` | 不接 | 這次用不到;`AD0` 不接時內部預設拉低,位址是 `0x68`。 |

## Stage 6：第二顆 MPU6050(前臂),同一條 I2C1 bus,不同位址(2026-08-22)

真實 Phase 3 硬體規劃確定要用兩顆 IMU(上臂+前臂,見 PRD.md 第 3 節架構修正),不是只有階段 5c 那一顆。兩顆共用同一條 I2C1 bus(`PB6`/`PB7`),靠位址區分——I2C 本來就是為了這個情境設計的,不需要為了「兩顆感測器」就多開一條實體匯流排(I2C2 的可用接腳跟除錯用的 JTDO 衝突,階段 4b 已經試過放棄,見上方)。

**第二顆(前臂)接線,跟階段 5c 的第一顆(上臂)幾乎一樣,只差一條線**：

| Adafruit 接腳 | 接到 | 備註 |
| --- | --- | --- |
| `Vin` | STM32 **3.3V** | 跟第一顆一樣。 |
| `GND` | STM32 GND | 共地,跟第一顆共用。 |
| `SCL` | STM32 `PB6` | 跟第一顆並聯在同一條線上,不是接到別的腳位。 |
| `SDA` | STM32 `PB7` | 同上,並聯。 |
| `AD0` | STM32 **3.3V** | **跟第一顆唯一的差異**——這條線決定位址從預設的 `0x68` 變成 `0x69`(MPU6050 資料手冊：`AD0` 是 7-bit 位址的最低位元)。第一顆的 `AD0` 維持不接(拉低,`0x68`)。 |

**韌體改動**：`ImuReader` 原本把目標位址寫死成一個全域常數(`kImuTargetAddr = 0x68u`),現在改成建構子參數,`main()` 建立兩個實例(`shoulder_reader(0x68u)`、`elbow_reader(0x69u)`)。因為兩顆共用同一個實體 I2C1 週邊,同一時間只能有一顆在進行傳輸——新增一個簡單的交替邏輯：目前作用中的 reader 完整讀完一次才換下一顆,逾時的話**留在同一顆重試**,不會跳過(跟原本單顆時的重試邏輯一致)。預期效應：單顆吞吐量會下降到階段 5c 量到的 591-593 次/秒的大約一半(~290/秒),因為兩顆共用同一條 bus,每次傳輸佔用的匯流排時間不會因為換位址而改變。**這是預期中的取捨,不是 bug**——之後應該用韌體新增的 per-IMU 計數器(`shoulder_completions`/`elbow_completions`)實測確認,而不是只靠推算。

**USART2 鮑率同時從 9600 提高到 115200**：新的輸出格式(`tick=<n> grip=<f> gripping=<0|1> shoulder_pitch=<f> shoulder_roll=<f> elbow=<f>`)每行約 95 bytes,9600 鮑率(~960 bytes/秒)只夠撐每秒 10 行左右,對即時追蹤手臂動作來說太卡。`BRR=0x008B` 是照 RM0368 19.3.4 公式,對應這個專案目前用的 16MHz HSI(整個 codebase 沒有任何檔案設定過 `RCC->CFGR`/`RCC->PLLCFGR`,用 grep 確認過,不是假設)算出來的,不是憑感覺選的——算式跟原本 9600 鮑率的 `BRR=0x0683` 用的是同一套,細節見 `phase3_control_loop_main.cpp` 裡 `usart2_init()` 的註解。改成每 10 個 EMG tick 印一行(100Hz,~950 bytes/秒,鮑率預算的 8% 左右),取代原本每 1000 tick 印一次的診斷輸出。

**下一步(尚未執行,待實體接線完成)**：先用 `python3 tools/check_hardware_ready.py --i2c-scan` 確認兩個位址都有 ACK(不要只靠肉眼看燈號),再用 `openocd ... mdw` 讀新增的 `g_wake_result_shoulder`/`g_wake_result_elbow` 確認兩顆喚醒都成功,最後才進到 Python/MuJoCo 端驗證——完整順序見 PRD.md Stage 6 章節。

## 已知問題

- **WeAct 官方的 HID bootloader 燒錄工具在這台 Apple Silicon Mac 上不能用**——追查到根因是 `hid_enumerate()` 回傳空的裝置路徑(這支 2019 年工具跟現今 IOKit 有深層相容性問題，不值得繼續修)。改用 ST-Link + OpenOCD 燒錄；完整調查過程見 PRD.md 的 Phase 1.5 章節。
- 階段 3(1kHz 取樣)已完成並實測通過(見上方階段 3a/3b/3c/3d)——Timer 預除頻器、ADC 觸發來源編碼全部先查證 RM0368 再寫，不是猜的。目前未用 DMA(輪詢 `EOC` 已足夠應付單通道 1kHz),未來若要同時取樣多通道才需要評估。

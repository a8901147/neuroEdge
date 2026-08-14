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
cmake --build build --target blink footprint_check heap_guard_check uart_hello -j
```

每個目標編譯完都會(透過 `objcopy`)產生對應的 `.bin`,並在每次編譯時印出 `arm-none-eabi-size` 的輸出——Flash/SRAM 用量不需要另外量測就看得到。

## 燒錄(需要 ST-Link 透過 SWD 連接)

```sh
cmake --build build --target flash_blink            # 燒完看板子上的 LED 是否慢速閃爍(~1Hz)
cmake --build build --target flash_footprint_check   # 只證明不會當機，目前還沒有過/不過的燈號
cmake --build build --target flash_heap_guard_check  # LED 恆亮 = 過，快閃 = 偵測到 malloc_count 違規
cmake --build build --target flash_uart_hello        # 接上 USB-TTL 後，@ 9600 baud 應該每秒收到一行文字
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

**這片板子的實測結果**：兩個階段都是第一次燒錄就通過——階段 0 慢速閃爍、階段 2 恆亮。這解決了什麼(未知數 1、2)、還剩什麼沒解決(未知數 3 Timer/ADC 時序、未知數 4 真實 MyoWare 訊號品質——這兩項都卡在需要 USB-TTL 轉接器做 UART 診斷)，詳見 PRD.md 的 Phase 1.5 章節。

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

## USB-TTL 接線(階段 3/4 診斷用 UART)

腳位選 **USART2**(`PA2`=TX、`PA3`=RX)——這不是猜的，是這片板子(WeAct Black Pill)在 Zephyr 官方 board 文件裡被指定為這片板子的標準 debug console UART(`STDIO_UART_TX`=PA2、`STDIO_UART_RX`=PA3)，且已確認跟目前用到的其他腳位不衝突：`PC13`=LED、`BOOT0`=開機模式按鈕(不是一般 GPIO，跟 UART 無關)、`PA13`/`PA14`=SWD(ST-Link 用)、`PA0`=MyoWare ADC。

| USB-TTL 接腳 | 接到 | 備註 |
| --- | --- | --- |
| `GND` | STM32 GND | 共地。 |
| `TXD` | STM32 `PA3`(USART2_RX) | 交叉接——轉接器的 TX 接到 MCU 的 RX。 |
| `RXD` | STM32 `PA2`(USART2_TX) | 交叉接——轉接器的 RX 接到 MCU 的 TX。 |
| `VCC`(3V3/5V) | **不要接** | 板子目前已經有電源(USB 或 ST-Link 供電)，USB-TTL 的 VCC 如果同時接上會變成兩個電源同時驅動 3.3V 軌，有把其中一顆穩壓晶片燒壞的風險。只接 GND + TXD + RXD 三條線就好。 |

板子插上後，Mac 端可以先確認有正確列舉出序列埠(`ls /dev/tty.usbserial-*` 或 `ls /dev/tty.SLAB_USBtoUART*`，依轉接器晶片而定)，這一步不需要 STM32 這邊已經有任何 UART 韌體，純粹確認轉接器本身有被系統認出來。

## 已知問題

- **WeAct 官方的 HID bootloader 燒錄工具在這台 Apple Silicon Mac 上不能用**——追查到根因是 `hid_enumerate()` 回傳空的裝置路徑(這支 2019 年工具跟現今 IOKit 有深層相容性問題，不值得繼續修)。改用 ST-Link + OpenOCD 燒錄；完整調查過程見 PRD.md 的 Phase 1.5 章節。
- 階段 3(1kHz 取樣)所需的 Timer/ADC/DMA 韌體**還沒開始寫**——診斷用的 UART 驅動已經查證、實作、實測通過(見上方階段 3a)，但 Timer 預除頻器、ADC 觸發來源編碼這些還沒對照 RM0368 查證過，刻意延後到查證完成或能在硬體上實測驗證，而不是先寫出「盡力猜測」的版本。

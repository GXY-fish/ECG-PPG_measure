# ADS1292R 三导联心电 —— ESP32-S3 / ESP-IDF 移植说明

> 本文档配合 `main/ads1292.c` / `main/ads1292.h` 使用。
> 代码由 STM32F103C8T6 版（`模块-STM32/STM32-V2.0`）移植而来，
> 并吸收了 STM32 版调试一整天才定位到的几个坑，**每一处都在代码注释里写了原因**。

---

## 1. 三导联接法

ADS1292R ECG Breakout（Protocentral v4）用 3.5mm 立体声插头引出三极：

| 插头位置 | 名称 | 贴法 |
|---|---|---|
| 尖端 tip | **LA** | 左臂，或左上胸 |
| 中环 ring | **RA** | 右臂，或右上胸 |
| 尾套 sleeve | **RL** | 右腿，或右下腹 |

芯片内部通道映射：

```
CH2 = IN2P/IN2N  <-- LA / RA      心电主通道（增益 6）
RLD 输出         <-- RL           右腿驱动参考极
CH1 = IN1P/IN1N  <-- 呼吸解调     本工程未用（增益 2）
```

**要点**

- **RL 极是抑制工频干扰的关键**，不接 RL 波形会明显变脏。
- 贴电极前用酒精擦拭皮肤并晾干；手腕可做快速测试，**胸贴幅度更大、呼吸信号也更明显**。
- 导联脱落状态**每帧都随状态字下发**（`ads1292_get_loff()`），不需要另外读寄存器——
  读寄存器必须先 `SDATAC` 退出连续转换，会打断数据流、产生错位帧。

---

## 2. 硬件连接（ESP32-S3-DevKitC-1）

| 信号 | ESP32-S3 | 模块 |
|---|---|---|
| SCLK | GPIO12 | SCK |
| MOSI | GPIO11 | MOSI |
| MISO | GPIO13 | MISO |
| CS0 | GPIO10 | CSB |
| PWDN/RST | GPIO21 | RST |
| START | GPIO47 | START |
| DRDY | GPIO14 | DRDY |
| VCC | 外部 5V | VCC（本开发板 5V 为 VIN 输入脚） |
| GND | GND | GND |
| CLK | 不接 | CLK（CLK SEL 跳线选内部时钟） |

---

## 3. 寄存器配置（已与 Protocentral 官方库逐位交叉验证）

| 寄存器 | 地址 | 值 | 含义 |
|---|---|---|---|
| CONFIG1 | 0x01 | `0x02` | DR=010 → 500 SPS 连续转换 |
| CONFIG2 | 0x02 | `0xE1` | bit7=1；导联脱落比较器开；内部基准缓冲开；2.42V；内部时钟 |
| LOFF | 0x03 | `0x10` | 导联脱落阈值/电流默认；[0]=0 → **DC 模式** |
| CH1SET | 0x04 | `0x20` | 通道1 上电，增益 2（呼吸） |
| CH2SET | 0x05 | `0x00` | 通道2 上电，增益 6，正常电极输入（**心电主通道**） |
| RLD_SENS | 0x06 | `0xEC` | RLD 输出开；RLD2N/RLD2P 使能；RLD 脱落检测关 |
| LOFF_SENS | 0x07 | `0x0C` | 仅通道2 正负端做导联脱落检测 |
| LOFF_STAT | 0x08 | `0x00` | 只读状态 |
| RESP1 | 0x09 | `0xF6` | 呼吸解调/调制开，相位 0x0D，内部时钟 |
| RESP2 | 0x0A | `0x03` | 内部 RLDREF，32kHz |
| GPIO | 0x0B | `0x0C` | 两个 GPIO 均为输入（复位默认值） |
| ID | 0x00 | `0x73` | ADS1292R 只读 ID，用于自检 |

**关于 `LOFF = 0x10`（DC 脱落电流）**：Protocentral 默认走 AC 模式（`FLEAD_OFF=1`，即 `0x11` 一档）。
DC 模式在电极脱落时更容易把输入顶到饱和。若观察到"电极一拔波形就贴满量程"，
可改成 `0x11` 试 AC 模式；本工程沿用 STM32 版已验证可用的 `0x10`。

---

## 4. 参考实现（GitHub / 官方）

| 来源 | 用途 |
|---|---|
| [Protocentral/protocentral-ads1292r-arduino](https://github.com/Protocentral/protocentral-ads1292r-arduino) | **权威参考**，寄存器序列/时序常量/DRDY 处理都以此交叉验证 |
| [Protocentral ADS1292R 官方文档](https://protocentral.com/product/ads1292r-ecg-respiration-breakout-kit/docs/getting-started/) | **三导联电极接法**、ESP32 接线示例、CLK SEL 跳线说明 |
| [esp32_ads1292r（Gitee）](http://m.gitee.com/Mr_tsura/esp32_ads1292r) | 从 Protocentral 移植的 ESP-IDF v5.4.1 版，附 PyQt 上位机 |
| [Protocentral/protocentral_healthypi_v4](https://github.com/Protocentral/protocentral_healthypi_v4) | ESP32 经典版 + ADS1292R 整机方案，信号处理管线可参考 |
| [card-io-ecg/card-io-fw](https://github.com/card-io-ecg/card-io-fw) | ESP32-S3/C6 + ADS129x，**Rust + embassy** 架构，算法组织可参考 |

以上三个 GitHub 工程已下载到桌面 `ESP32_ADS1292R_参考工程\` 目录，可离线对照。

本工程的 `host/ecg_host.py` 已实现 PC 上位机（二进制帧解析 + 波形显示）。

---

## 5. 踩过的坑（STM32 版实战，代码注释里均有标注）

### 5.1 `RDATAC` 之后 `START` 命令会被忽略 ★最关键

数据手册 8.5.2.7 原文：

> "If the device is in **RDATAC mode**, a SDATAC command must be issued before **any other commands** can be sent to the device."

STM32 版曾把顺序写成 `RDATAC → START`，结果 START 被当成数据流忽略、
**DRDY 永不来、一个样本都采不到**。正确顺序：

```c
START 命令（此时仍在 SDATAC）+ START 脚上升沿   /* 两条路互为备份 */
→ 最后才 RDATAC
```

### 5.2 `tPOR` 没等满，`RESET` 命令从不生效 → 表现为**间歇性故障**

`tPOR = 2^18 / fCLK ≈ 512 ms`（内部时钟 512kHz），**期间所有命令都被忽略**。
STM32 版释放复位后只等 10ms 就发 `SDATAC`/`RESET`，两条都在窗口内被忽略。
而手册说串口状态机一旦进入未知状态，**只能靠 RESET 或拉高 CS 恢复**——
RESET 失效意味着芯片一旦卡住就再也出不来，直到断电。
**卡不卡取决于上电时序，于是成了间歇性**：同样的固件，有时出波形有时不出。

本工程已按 `600ms` 等满 tPOR。

### 5.3 RDATAC 模式下 DRDY 会一直拉低 → 不能只认中断信号量 ★易死锁

数据手册：RDATAC 模式下 **DRDY 拉低后会一直保持到数据被读走**（读第一个 SCLK 时才回高）。

由此产生一个很容易栽的组合：`ads1292_init()` 的顺序是
`RDATAC+START` → 等 50ms 做 DRDY 诊断 → **最后才挂 DRDY 中断**。
500SPS 下第一个下降沿约 8ms（tSETTLE）就出现，早于中断挂载；
此后 DRDY 停在低电平、**不再产生新的下降沿** → 信号量永远拿不到
→ `ads1292_read_data()` 每次 20ms 超时返回 false **且不读帧**
→ DRDY 永远不清 → **数据流一帧都收不到，永久卡死**。
（任务若曾慢于 500SPS，也会落入同一状态。）

修法：就绪判定改成「**信号量 或 DRDY 电平**」双路，取不到信号量但 DRDY 为低照样读；
读完后若 DRDY 已回高，顺手清掉可能残留的事件，避免下次把旧事件当新数据重复读同一帧。

### 5.4 SPI 模式与时钟

- ADS1292R 要求 **SPI Mode 1**（CPOL=0, CPHA=1）。
- 配置寄存器时 **SCLK ≤ 1MHz**；命令字节与后续字节之间至少留 `tSDECODE = 4 tCLK ≈ 7.81µs`。
- ESP32 的 SPI **事务内字节是背靠背的，没有空隙可插**，所以本工程把 SCLK 降到
  **500kHz（一个字节 16µs > 7.81µs）** 来满足要求。数据帧连读 9 字节无此限制，
  144µs 读一帧对 500SPS 绰绰有余。

### 5.5 帧对齐校验

每帧 9 字节 = 3 状态 + CH1(24bit) + CH2(24bit)。状态字高 4 位恒为 `1100`：

```c
if ((rx[0] & 0xF0) != 0xC0 || (rx[1] & 0x1F) != 0x00 || rx[2] != 0x00) {
    s_bad_frames++;      /* 字节错位，丢弃 */
    return false;
}
```

### 5.6 导联脱落位在状态字里的位置

24bit 状态字 = `1100 | LOFF_STAT[4:0] | GPIO[1:0] | 13 个 0`，
所以 `LOFF_STAT[4:0] = ((rx[0] & 0x0F) << 1) | ((rx[1] >> 7) & 0x01)`。

### 5.7 显示侧的两个坑

- **包络必须设下限**：信号很小（电极没接好、只剩噪声）时自适应包络会被压到接近 0，
  任意噪声都被放大到满屏，看起来像"ECG 特别之大"。
- **采集任务里绝不做耗时 I2C**：OLED 整屏刷新约 20ms，放进采集任务会让 500SPS 丢点。
  本工程采集任务只更新数据，刷屏交给低优先级 display_task。

### 5.8 移植陷阱：显示通道选错（CH1 ↔ CH2）★2026-10-23 实修

从 STM32 版移植时，`main.c` 的波形与 BPM 误用了 `ch1`：

- **CH2 才是心电主通道**：Protocentral 板上 LA/RA 接 IN2P/IN2N，`CH2SET=0x00` 增益 6；
- CH1 是呼吸通道（`CH1SET=0x20` 增益 2，配 RESP 解调），输出以慢波/基线为主。

官方 Arduino 驱动头文件原话：*"On ProtoCentral boards CH1 carries respiration and
CH2 carries ECG."* 用错通道幅度差 3 倍、且基本看不到 QRS。已改为 `ch2`，
上位机 BPM 检测同步改到 CH2。

---

## 6. 快速排错流程

```
开机日志看不到 "ADS1292R 就绪，ID=0x73"
  └─ 把 ads1292.h 的 ADS1292_LINE_DIAG 改成 1，重编烧录
     万用表黑笔接模块 GND、红笔顶信号焊孔：
       · 随打印在 0V <-> 3.3V 跳  → 这根线通了
       · 电压一直不动              → 线断/插错孔/接触不良
       · MISO/DRDY 开上拉仍读 0    → 有东西把线拉低

ID 正常但没波形
  └─ 看初始化后那行 "[线路] 初始化后 DRDY(14)=?"
       · =1 → 芯片没在转换（查供电/RESET/START/DRDY 接线）
       · =0 → 芯片活着，查电极与导联脱落状态（ads1292_get_loff）

有波形但很脏
  └─ 确认 RL 极接了；贴电极前酒精擦皮肤；试胸贴替代腕贴
```

---

## 8. 编译与烧录

**推荐：VSCode + ESP-IDF 扩展**（本机已装好 IDF v5.5，工程 `.vscode/settings.json`
已指向 `C:\Users\coxfi\esp\esp-idf`）。用扩展的 "Build Project" / "Flash (UART)" 即可，
串口监视波特率 115200，数据走板载 USB-Serial-JTAG 口。

**命令行编译（Git Bash / MSYS 环境需注意）**：本机的 Bash 工具会向子进程注入
`MSYSTEM=MINGW64`，`env -u` 也剥不掉，而 IDF 的 `idf.py` 一见 MSYSTEM 就直接退出
（`idf.py` 的 `__main__` 分支只打印警告不干活；`idf_tools.py` 子进程更是直接
fatal）。绕法是把 `os.environ` 里的 `MSYSTEM/MSYS` 删掉后再 import 调用 `main()`，
并手工把工具链加进 PATH：

```
xtensa-esp-elf\esp-14.2.0_20260121\xtensa-esp-elf\bin
cmake\3.30.2\bin        ninja\1.12.1
（均位于 C:\Users\coxfi\esp\tools\tools\ 下）
```

**注意**：如果整个工程文件夹被移动过（比如从桌面移进"大创"），必须先
`idf.py fullclean` 再编译——CMakeCache.txt 里记着旧路径，不清理会直接报错。

---

## 9. 文件说明

| 文件 | 作用 |
|---|---|
| `main/ads1292.c` / `.h` | ADS1292R 驱动（SPI、寄存器、DRDY 中断、帧校验） |
| `main/main.c` | ECG/PPG/显示三任务 + USB-Serial-JTAG 二进制上报 |
| `host/ecg_host.py` | PC 上位机（帧解析、波形绘制、导联脱落/饱和提示） |
| `host/启动上位机.bat` | 上位机启动脚本 |

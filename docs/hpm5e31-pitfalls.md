# HPM5E31 开发避坑记录

本文只记录已经由源码、芯片状态或硬件运行结果支持的结论。详细时间线保留在
`hpm5e31-backlog.md` 和项目根目录 `handoff.md`。

## 1. 先复用本地基线

- 外设与传感器例子先搜索当前仓库、`ch32-hal`、Embassy、
  `embedded-drivers` 和 `old-embedded-drivers`。
- 自定义板使用自己的原理图、网络表和实测波形。官方 HPM5E00EVK 资料用于确认
  SoC pinmux、时钟、DMA request 和 SDK 调用方式。
- 当前已验证的分层基线为：`main_marker`、`rtt_defmt_probe`、`i2c_detect`、
  `i2c_shared_sensors`、`pms7003_uart3`、`monitor_usb_lite`。
- 综合例子发生故障时，从首个偏离上述基线的层开始定位。

## 2. HPM5E31 运行时与链接

- HPM5E31 压缩指令环境中，Embassy timer 曾通过 2 字节对齐的间接函数入口触发
  instruction fault。当前构建保留 `-C llvm-args=-align-all-functions=2`，使函数入口
  达到 4 字节对齐。
- `.uninit` 中的 trap record 会跨复位保留旧值。阶段标记、当前 ELF 地址和刚清零的
  exception record 共同构成有效证据。
- USB 中断压力下曾出现任务帧或返回地址损坏。外部中断入口当前显式保存并恢复
  `mepc`、`mstatus`、`MXSTATUS`、`MCCTLBEGINADDR` 和 `MCCTLDATA`。持续 USB 压力测试
  已通过 1000 个完整 monitor 协议周期和 PPOR 后 100 个周期验收。
- 外部 IRQ wrapper 放在 ILM `.fast.text`，与 HPM SDK 的 `.isr_vector` 执行位置一致。
  runtime 在开启 L1I 后执行 `fence.i`，清除 flash algorithm/复位链可能留下的 XPI
  instruction-cache 状态。加入该同步后，曾反复出现的 XPI PC 高位跳变停止复现。
- 2026-07-15 的综合传感器测试复现 `mcause=1`，`mepc/mtval=0x802070ba` 落在 ELF
  文本范围之外。检查发现 144 字节 trap frame 尾部仍有空间，而 wrapper 当时未保存
  `mepc/mstatus`。补齐这两个 CSR 后，相同压力测试保持 exception/panic record 为零，
  传感器任务继续运行；剩余故障收敛为 USB 响应内容损坏。

## 3. 烧录与复位

- HPM5E31 必须命中专用 HPMicro target sequence。缺失 vendor 注册时，probe-rs 会
  落到通用 RISC-V hart reset。
- 当前冷复位与 HPM SDK/OpenOCD 的 PPOR 路径一致：向 `0xF410001C` 写入
  `0x24000000`，等待 hart 重新启动。
- HPM5E31 Flash 算法需要 HPM5E ROM/XPI 地址语义、hybrid XPI 初始化、XPI/AHB
  buffer 清理和 cache 状态恢复。Boot ROM 冷启动会重新建立这些状态。
- CMSIS-DAP/JTAG 写入 RAM page buffer 曾出现可重复的少量字损坏。ROM program 前的
  4 KiB page-buffer readback/retry 和最终 programmed-range 完整 readback 都属于成功
  条件。
- `Finished` 只表示算法调用返回。完整成功包含算法成功、目标范围逐字节校验和应用
  自动启动验证。
- 物理复位、BOOT0 或第二次烧录属于恢复事件。记录恢复前的 DMI/SBA 状态、Flash
  mismatch 地址和 trap record，随后重新计算稳定性统计。
- 默认 `SPEED=500` kHz 与 `FLASH_TIMEOUT>=120s` 已完成大镜像烧录；USB 综合镜像使用
  `180s` 留出校验余量。
- 2026-07-15 的两次 OpenOCD 写入在 100 kHz 下分别触发 502 ms 和 10 s flash
  algorithm timeout，随后 hart 停留在 `dmstatus=0x00400ca2`，debug halt 请求失效。
  该恢复状态需要一次板载 RESET；本轮不计入 USB 固件稳定性结果。后续烧录回到已验证的
  500 kHz 基线，并继续要求 `Verified OK`、reset 和 runtime proof 同时成立。

## 4. USB device controller 与 Embassy driver

- USB DMA 描述符、endpoint list、qTD 和 payload buffer 放在 `.noncacheable`，并满足
  控制器对齐要求。CPU local alias 与 system-bus alias 的转换需要覆盖所有交给 USB
  控制器的指针。
- HPM5E SDK 将顶部 64 KiB AXI SRAM 配置成
  `MEM_TYPE_MEM_NON_CACHE_BUF`。当前链接布局为 cacheable
  `0x01200000..0x01230000` 和 non-cacheable
  `0x01230000..0x01240000`，与官方 HPM5E linker/PMA 配置一致。
- qTD descriptor word 必须一次计算完成并做完整 32 位 volatile write。旧实现先写
  buffer page pointer，再通过第二次 read-modify-write 写 current offset；现场捕获到期望
  `0x0123304a` 被截成 `0x0000304a`，主机随后收到该错误地址对应的指令字节。官方 C SDK
  对 buffer word 使用单次指针赋值。当前 Rust 实现对 token、next、buffer 和 expected
  length 均采用 full-word store，并在 prime 前执行 `fence iorw, iorw`；该修改已完成
  10/100/1000 轮板上验收。
- qTD token `0x001e8000` 的含义为：remaining bytes=30、IOC(bit 15)=1、
  ACTIVE(bit 7)=0。判断完成状态时直接按位域解码。
- `ENDPTPRIME` 使用 write-one-to-set 语义。提交 endpoint 使用直接写目标 bit，并在
  fence/readback 后观察 prime/stat/complete。
- `USBSTS`、`ENDPTSETUPSTAT`、`ENDPTCOMPLETE` 使用 W1C 语义。处理流程先保存快照，
  再写回需要清除的位，随后 fence/readback。
- qTD retire 与软件 future 完成之间存在 posted-write/late-writeback 窗口。描述符和
  payload 的生命周期必须覆盖硬件最终写回；复用 slot 前检查 token、complete 和
  endpoint state。
- ISR 负责读取并 W1C `ENDPTCOMPLETE`，随后发布软件 completion bit 并唤醒对应
  endpoint future。future 等待 qTD ACTIVE 清零以及 `ENDPTPRIME/ENDPTSTAT` 释放后返回。
  上层保持标准 Embassy `read().await` / `write().await` 流程，无需插入控制器专用延时。
- monitor bulk 协议解析、GETVALUE 取值和 CRC 计算期间保持机器中断开放。bulk task
  独占协议状态，传感器共享值使用原子访问；整帧关中断会扩大 USB IRQ latency。
- USB reset 使全部 endpoint generation 失效。ISR 发布 reset event、清空软件 completion
  并唤醒 bus future 后立即返回；bus future 重建 EP0。该顺序与当前 HPM SDK 一致，避免
  同一 IRQ 快照中的旧 UI completion 进入新枚举会话。
- `USBINTR.UE` 是全部 endpoint 共用的 completion interrupt gate。SETUPSTAT 置位期间关闭
  UE 会形成确定性死锁：控制 future 可能正在等待 EP0 的另一方向，随后 bulk qTD 已完成、
  `ENDPTCOMPLETE` 已置位，却没有中断发布 software completion。ISR 现在保持 UE 开启，
  EP0 setup 通过 setup tripwire 与 EP0 OUT waker 消费；bulk completion 始终可达。
- 单个逻辑 bulk 事务曾触发数百次 USB UI/SRI 中断。IRQ count 是稳定性测试的核心
  指标之一；中断风暴可进一步放大上下文保存、W1C 和 endpoint retire 缺陷。
- 单轮 CONNECT/GETINFO/GETVALUE 只构成 smoke 证据。稳定验收使用同一 USB 枚举会话
  连续执行协议周期，并同时检查 packet index、CRC、command、exception record、
  sensor refresh counter 和 USB IRQ 增长率。
- 每次构建都从当前 ELF 重新解析 monitor symbol 地址。链接布局会随代码和 LTO 改变。
- GETVALUE 处理器对 `payload_len` 做独立上界检查，并校验
  `payload_len == 12 + item_count * 12`。异常长度返回 `RESULT_PAYLOAD_ERR`，避免 slice
  panic 和越界响应构造。

## 5. I2C 三传感器

- 自定义板当前总线为 I2C0：PC08=SCL、PC09=SDA，Standard mode，7-bit 地址为
  BH1750 `0x23`、APDS-9930 `0x39`、BME680 `0x77`。
- 多设备例子使用 Embassy shared-bus `I2cDevice` 代理。每个驱动持有独立 device
  proxy，底层事务由同一个 critical-section mutex 串行化。
- BH1750 high-resolution 首次转换至少等待 120 ms。当前单位为
  `MON_BH1750_LUX_X100 / 100` lux。
- APDS-9930 身份寄存器返回 `0x39`。clear、IR、proximity 都是 ADC/raw count；lux
  和距离换算需要光学结构与校准参数。
- BME680 gas resistance 在 heater warm-up 期间持续变化。温湿压可立即显示，空气质量
  算法应增加预热与基线学习阶段。

## 6. PMS7003 UART

- 自定义板接收脚为 PC14/UART3_RXD ALT2，链路参数为 9600 8N1。PMS7003 TX 为
  3.3 V TTL，串口 idle-high。
- 已验证的 blocking UART + `0x42 0x4d 0x00 0x1c` 前缀同步器连续解析 100 个
  checksum-valid frame。
- 综合 USB 例子使用 blocking UART 实例的 `embedded-hal-nb::serial::Read` 轮询，每次
  最多消费 64 字节并每 1 ms 主动让出 executor。该路径保留已验证的 UART/FIFO配置，
  同时避免等待完整帧期间占用 USB task。
- 当前对外 PM1.0、PM2.5、PM10 使用 atmospheric 字段，单位为 `ug/m3`；version、
  sensor error、valid/invalid frame 和 UART error counter 同步暴露。

## 7. 当前验收命令

```sh
cd /Users/mono/Elec/HPM/hpm-hal/examples/hpm5e00evk
cargo build --offline --release --target riscv32imac-unknown-none-elf --bin monitor_usb_lite
BIN=monitor_usb_lite \
ELF=target/riscv32imac-unknown-none-elf/release/monitor_usb_lite \
SPEED=500 FLASH_TIMEOUT=180s ./monitor.sh flash
uv run --with pyusb python tools/monitor_smoke.py \
  --transport bulk \
  --elf target/riscv32imac-unknown-none-elf/release/monitor_usb_lite \
  --iterations 10 --timeout 3

cargo build --offline --release --target riscv32imac-unknown-none-elf --bin monitor_sensors
BIN=monitor_sensors \
ELF=target/riscv32imac-unknown-none-elf/release/monitor_sensors \
SPEED=500 FLASH_TIMEOUT=180s ./monitor.sh flash
uv run --with pyusb python tools/monitor_sensor_stress.py \
  --elf target/riscv32imac-unknown-none-elf/release/monitor_sensors \
  --iterations 60 --interval 1 --timeout 3
```

USB driver 按 1、10、100、1000 个完整协议周期逐级验收，每级要求零 timeout、零 CRC/
header/command-order 错误、零 panic、零 exception。综合例子还要求 I2C ready mask 为
`0x7`，三颗 I2C sample counter 与 PMS7003 valid frame 持续增加，传感器/UART error
counter 保持稳定。

## 8. HPM5E31 时钟必须使用本 family 配置

- HPM5E 曾直接复用 HPM6E 的 SYSCTL 实现，默认运行状态实测为 PLL0 VCO=600 MHz、
  CLK0=/1、CPU=600 MHz、DCDC=1200 mV。HPM SDK 的 HPM5E31 board 配置为 PLL0
  VCO=960 MHz、CLK0=/2=480 MHz、CLK1=/2.4=400 MHz、DCDC=1275 mV。
- HAL 现在先将 DCDC 提升到 1275 mV，再把 CPU 临时切到 24 MHz，配置 PLL0 与两个
  post-divider，最后把 CPU 切到 480 MHz。该流程消除了运行状态对 ROM、PPOR 和烧录前
  PLL 状态的依赖。
- SDK 的 `pllctlv2_pll_is_stable()` 将“PLL 当前关闭”视为配置完成。PLL 无消费者时等待
  `response=1` 会永久阻塞；CPU mux 选择 PLL0_CLK0 后，SYSCTL 自动启用 PLL 与 divider。
- 2026-07-16 板上 readback：MFI=40、MFN=0、DIV0=5、DIV1=7、DCDC=1275 mV，
  exception record 全零。

## 9. 2026-07-15 综合例子实测结果

`monitor_sensors` release+LTO 构建、完整烧录校验和自动 reset 已通过。首次 monitor
读取成功得到：

```text
BH1750:     21.66 lux
APDS-9930: clear=67, IR=22, proximity=60 raw count
BME680:     27.34 C, 48.80 %RH, 99201 Pa
PMS7003:    PM1.0=5, PM2.5=7, PM10=9 ug/m3
PMS7003 valid frames=21
USB IRQ count=303
```

当时同一 USB 会话在第二轮 bulk OUT 超时。JTAG 观察到 I2C cycle 从 15 增长到 79，
PMS7003 valid frame 从 21 增长到 110，exception record 保持清零，证明采集任务独立稳定。

补齐 trap `mepc/mstatus` 并加入 parser 长度防护后，最终失败表现为主机收到
`bad monitor header`；目标端 exception/panic 均为 0，I2C ready mask=`0x7`、cycle=41、
PMS7003 valid frame=58。当前四传感器数据路径已贯通并持续刷新，USB bulk monitor 的
该阶段的 response data/completion 边界仍未通过 10-cycle 稳定门槛。

上述失败状态已被 2026-07-16 修复取代。`monitor_usb_lite` 在完整 flash verify 和 PPOR 后
完成 1000/1000 连续协议周期，耗时 15.093 s；随后再次 PPOR，完成 100/100，耗时
1.511 s。两轮均为零 timeout、零 CRC/header/command 错误、零 panic、零 exception。

同一 runtime/driver 基线上的 `monitor_sensors` 随后完成完整 flash verify、PPOR 和
30/30 综合周期，耗时 33.963 s。I2C cycle 从 25 增至 54，PMS7003 valid frame 从
35 增至 74，USB IRQ 从 777 增至 2691，四颗传感器刷新检查全部通过。

## 10. 核心修复代码归属与诊断隔离（2026-07-17）

- L1I/XPI 同步属于 runtime 启动不变量，`fence.i` 保留在 L1I enable 的紧邻位置。
- external IRQ 的 ILM 放置和 Andes CSR 保存属于 runtime attribute 语义，统一位于
  `external_interrupt` 宏生成代码。
- PLIC 初始化使用 `andes-riscv` 类型化寄存器 API；CLAIM/COMPLETE 具有读副作用，清理
  pending source 时使用直接 `write`。
- HPM5E PLL/DCDC 属于 family SYSCTL 实现。默认 PLL 参数进入 `sysctl::Config`，安全
  切换函数消费配置并按 divider 公式更新 `CLOCKS`。
- qHD/qTD full-word store、DMA 地址转换、`fence iorw`、W1C ownership、software
  completion、endpoint generation 和 retire 判定属于 USB DCD 驱动状态机。
- CPU-only software completion 位于普通 BSS；DMA 可见的 qHD、qTD 和 payload 保持在
  `.noncacheable`。
- USB controller snapshot 与 qTD trace 集中到 `usb::diagnostics`，由
  `usb-diagnostics` feature 控制。默认构建不会执行额外寄存器读取或 atomic store。
- runtime 逐阶段打点已经清除；早期 trap recorder 由 `runtime-diagnostics` 控制，并在
  Rust startup 成功后清零。
- monitor 例子继续保留协议级 trace、CSR、panic、exception 和传感器健康字段，承担
  hardware qualification 职责。

整理后的板上验收：`monitor_usb_lite` 完成 1000/1000，耗时 15.081 s；
`monitor_sensors` 最终类型化 PLIC 镜像完成 30/30，耗时 34.683 s。I2C cycle 从 10
增至 39，PMS7003 valid frame 从 14 增至 54，USB IRQ 从 769 增至 2683，协议、panic
和 exception 错误均为零。

## 11. 代表性外设验证中的计时基准（2026-07-19）

- GPTMR1 配置频率为 100 MHz，1,000,000 tick 通过 24 MHz MCHTMR 交叉测量得到
  240,049 tick，对应 10.002 ms。连续 32 轮计时与 PD06 GPIO 输出锁存检查通过。
- 同一 10 ms 区间的 `mcycle` 增量约为 6.54M，与 HAL 上报的 480 MHz CPU 时钟缺少
  固定比例关系。板级外设墙钟验证统一使用已配置为 24 MHz OSC 的 MCHTMR；`mcycle`
  保留作 CPU 执行计数诊断值。
- PF27/ADC0_IN0 接地后连续采样 256 次，`min=0`、`max=226`、`avg=8`，换算平均值
  约 0 mV，ADC0 初始化、校准、pin analog 配置和单次转换路径通过。
- PF27 接入 ML8511 后连续采样 512 次，raw `min=19719`、`max=20314`、`avg=19951`，
  按 3.3 V 参考换算为 0.992–1.022 V、平均 1.004 V。外部模拟信号与接地基线形成
  明确区分，排除 ADC 恒零造成的验证假阳性。
- ROHM ML8511A 手册给出典型灵敏度 0.129 V/(mW/cm²) 和 10 mW/cm² 时 2.2 V
  的已知点。例程使用 `UV=(Vout-2.2)/0.129+10` 并将负值钳位为零，每 500 ms
  对 64 个 ADC 样本求平均。板上周期读数为 0.731–0.741 mW/cm²，典型约
  0.736 mW/cm²；该值用于相对趋势，绝对精度受器件零点、温度和 ADC 参考影响。
- HPM5E31 的 SPI 信号 pin trait 分布在 PA、PC。PC 成组映射包括 SPI0
  PC04/PC06/PC07、SPI1 PC10/PC12/PC13、SPI3 PC20/PC22/PC23、SPI2
  PC26/PC28/PC29，顺序均为 SCLK/MISO/MOSI；PB、PF 用于其他复用功能。
- SPI 构造函数已经在 SCLK IOC 配置中设置 `loop_back`，与 HPM SDK pinmux 初始化保持
  一致。SPI1 回环验证采用 PC13 MOSI 连接 PC12 MISO，PC10 输出 SCLK。
- 上述 SPI1 连接在 1 MHz 下完成 async DMA 回环 100/100 轮；每轮 32 字节模式随轮次
  变化，接收数据逐字节匹配，SPI pinmux、时钟、HDMA request、IRQ 和全双工传输路径通过。

## 12. 从唯一有效上游开始 family 支持

- HPM HAL 的有效上游为 `hpmicro-rs/hpm-hal`。仓库只保留这个 remote，开始新 family
  工作前先 fetch，并从最新 `origin/master` 建立分支。
- 本地与上游共同修改的文件按语义合并。HPM5E 的板上证据决定 PLL、DCDC、PLIC、
  MCHTMR 和 USB DCD 路径；上游版本决定 PWMV2、SPI、音频驱动和代码生成基线。
- 共享寄存器版本只说明 IP 接近，family 资源集合仍需独立表达。HPM5E/HPM6E SYSCTL
  使用两个 family 文件，避免 CPU1、RAM1、FEMC、I2S/PDM 等 HPM6E 资源进入 HPM5E。
- 可选外设使用生成器的 `peri_*` capability cfg。HPM6E50 缺 ADC3，family 级
  `hpm6e` cfg 会造成构建失败；`peri_adc3` 精确表达该约束。
- 每次合并运行代表 family 矩阵。此次矩阵覆盖 HPM53、HPM67、HPM68、HPM6E、HPM5E，
  并发现最新上游 HPM68 DAO/PDM 适配缺口。
- HAL 提交依赖 HPM5E metapac 与 runtime 提交。依赖先发布为可访问的 commit/tag，HAL
  再切换到不可变 git revision，保证 CI 和审查者能够复现。

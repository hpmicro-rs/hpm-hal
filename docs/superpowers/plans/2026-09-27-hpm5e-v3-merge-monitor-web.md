# hpm5e v1+v2 → v3 合并 & monitor-web 独立仓库 实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 把 hpm-hal 的 v1(hpm5e-support)与 v2(origin/hpm5e-support-v2)合并为 hpm5e-support-v3 并推送;监控主机工具独立为 hpmicro-rs/monitor-web 仓库。

**Architecture:** 两条工作流:Phase 1 在独立 worktree 里以 v2 为基底 cherry-pick v1 五提交(按文件级处置策略解决冲突),应用 WIP,全量构建+硬件 smoke 验证后推送 v3;Phase 2 把 v1 的 tools/ + monitor.sh 复制进新仓库 monitor-web,做路径参数化与打包适配,本地验证后推送。

**Tech Stack:** git worktree / cherry-pick,Rust nightly(build-std),probe-rs(fork 版),Python(pyserial/pyusb),uv。

**关键环境事实**(已存 memory):
- probe-rs 必须用 fork 编译版 `/Users/mono/Elec/probe-rs/target/debug/probe-rs`(stock 版与 CherryUSB CMSIS-DAP 不兼容)
- 烧录 ~7 分钟;验证用 `BIN=monitor_sensors ./monitor.sh smoke-bulk`(USB bulk,VID 1209 PID 5E31),不是 CDC smoke
- HPMicro.yaml 的 stack_size 已注释(任何 probe-rs 都不支持该字段)
- 磁盘:当前剩余 57G,构建峰值 ~6G,逐板构建并清理
- 主工作区 /Users/mono/Elec/HPM/hpm-hal 有 WIP(dma/ewdg/monitor_web 改动),**不可在其上切换分支**

**提交常量**(v1 五提交,旧→新):
- `3e9702e` hpm5e 脚手架(Cargo.toml、build.rs、HPMicro.yaml、hpm5e00evk 工程、monitor_blinky/monitor_uart_lite、tools/、monitor.sh)
- `1bbbce7` src/usb/endpoint.rs 小修(2+/7-)
- `9e07d2b` time_driver gptmr/mchtmr、dma v1/v2 小改、ewdg、sysctl v67/v6e、usb bus/control_pipe/diagnostics/endpoint/mod
- `2d190a0` monitor_sensors.rs(新)、monitor_usb_lite 扩展、tools 升级、HPMicro.yaml 改
- `a4eb03a` src/adc/mod.rs(446 行)+ hpm5e00evk adc_async.rs

v2 tip:`3b028e7`。v2 已有 hpm5e00evk 工程脚手架(c6e4eea)与 adc finite sampling(d66490c)。

---

## Phase 1:v3 分支

### Task 1:创建 v3 worktree

**Files:** 无(纯 git 操作)

- [ ] **Step 1:从 v2 创建 v3 分支的独立 worktree**

```bash
cd /Users/mono/Elec/HPM/hpm-hal
git worktree add /Users/mono/Elec/HPM/hpm-hal-v3 -b hpm5e-support-v3 origin/hpm5e-support-v2
```

Expected:worktree 检出完成,HEAD = 3b028e7。主工作区保持 hpm5e-support + WIP 不动。

- [ ] **Step 2:确认 v3 基线可编译(hpm5e00evk example)**

```bash
cd /Users/mono/Elec/HPM/hpm-hal-v3/examples/hpm5e00evk && cargo build --bin blinky 2>&1 | tail -2
```

Expected:编译成功(建立基线,后续构建失败可归因)。

### Task 2:只读审查,产出文件级处置清单(与用户确认)

**Files:** 无修改

- [ ] **Step 1:adc 对比(v1 a4eb03a vs v2 d66490c)**

```bash
cd /Users/mono/Elec/HPM/hpm-hal-v3
git diff d66490c a4eb03a -- src/adc/mod.rs | head -40
git diff d66490c a4eb03a --stat
```

判据:diff 为空或 v2 为超集 → 跳过 v1 的 a4eb03a;v1 有独有功能 → 记下差异行,进入 Task 6 补丁。

- [ ] **Step 2:usb 对比(v1 的 1bbbce7、9e07d2b 的 usb 部分 vs v2 现状)**

```bash
git show 1bbbce7 -- src/usb/endpoint.rs
git diff origin/hpm5e-support-v2 9e07d2b -- src/usb/ | head -60
```

判据:v2 已含等价修复(如 v2 e189485 "harden device transfer lifecycle")→ 跳过;独有逻辑(逐段)记入 Task 4 补丁清单。

- [ ] **Step 3:time driver 与 sysctl 对比**

```bash
git diff origin/hpm5e-support-v2 9e07d2b -- src/embassy/time_driver_gptmr.rs src/embassy/time_driver_mchtmr.rs | head -60
git diff origin/hpm5e-support-v2 9e07d2b -- src/sysctl/v67.rs src/sysctl/v6e.rs | head -40
```

判据:v2 有更新版本则保留 v2;v1 的 hpm5e 特定修复若 v2 缺失 → 记入补丁清单。

- [ ] **Step 4:dma WIP 审查(本地未提交精简版 vs v2)**

```bash
git diff origin/hpm5e-support-v2 -- src/dma/v1.rs | grep '^-' | head -30
git diff origin/hpm5e-support-v2 -- src/dma/v2.rs | grep '^-' | head -20
```

判据:删除的是死代码/调试残留 → 保留精简版;删的是 v2 功能(circular、channel API 等)→ 补回。结论记入 Task 7。

- [ ] **Step 5:汇总处置清单,发用户确认**

清单格式:文件 → v2 保留 / v1 覆盖 / 混合补丁(附差异摘要)。用户确认后进入 Task 3。

### Task 3:cherry-pick 3e9702e(脚手架,剔除工具部分)

**Files:** 冲突涉及根 Cargo.toml、build.rs、examples/hpm5e00evk/*

- [ ] **Step 1:cherry-pick 并解决冲突**

```bash
cd /Users/mono/Elec/HPM/hpm-hal-v3
git cherry-pick 3e9702e
```

冲突解决策略(逐文件):
- 根 `Cargo.toml`:v2 版本为准;若 v1 添加了 monitor 固件所需依赖(bosch-bme680、static_cell、embassy-usb 相关),追加到 v2 版
- `build.rs`:v2 为准(确认 v1 的 hpm5e 支持在 v2 已存在,缺失则补)
- `examples/hpm5e00evk/Cargo.toml`:v2 为准 + 追加 v1 的 monitor 依赖
- `examples/hpm5e00evk/.cargo/config.toml`、`memory.x`:v2 为准(保留 v2 的 runner/probe 配置;若 v2 无 runner 则用 v1 的,路径改 `PROBE_RS` 环境变量方式留待 monitor-web)
- `HPMicro.yaml`:取 v1 版(此时先不带 stack_size 注释,Task 7 统一处理)
- 固件 bins(`monitor_blinky.rs`、`monitor_uart_lite.rs`、`cpu_features.rs`):保留 v1 版
- `tools/`、`monitor.sh`:丢弃(不加入 v3)

```bash
git rm -r examples/hpm5e00evk/tools examples/hpm5e00evk/monitor.sh   # 若冲突后仍存在
git add -A && git commit --no-edit
```

- [ ] **Step 2:确认 v3 无 tools/**

```bash
ls examples/hpm5e00evk/   # 应无 tools/ 与 monitor.sh
```

### Task 4:cherry-pick usb 修复(按 Task 2 结论)

**Files:** src/usb/*(按清单)

- [ ] **Step 1:按处置清单执行**

若 Task 2 判"跳过":`git cherry-pick 1bbbce7 9e07d2b -n` 后只 add 需要的文件,其余 reset:

```bash
git cherry-pick -n 1bbbce7 9e07d2b
# 逐文件按清单决定 add / checkout --ours(即 v2)
git add <保留文件>
git checkout -- <丢弃文件>
git commit -m "fix(usb): merge hpm5e multi-qtd and runtime hardening fixes from v1"
```

若判"全部跳过":不 cherry-pick,直接进入 Task 5。

- [ ] **Step 2:构建 hpm5e00evk monitor 相关 bin 验证 usb 模块编译**

```bash
cd examples/hpm5e00evk && cargo build --bin monitor_blinky 2>&1 | tail -2
```

Expected:编译成功。

### Task 5:cherry-pick 2d190a0(传感器监控固件,剔除 tools)

**Files:** monitor_sensors.rs、monitor_usb_lite.rs、monitor_blinky.rs、HPMicro.yaml、hpm5e00evk Cargo.toml/config

- [ ] **Step 1:cherry-pick 并剔除工具**

```bash
git cherry-pick 2d190a0
# 冲突按 Task 3 同策略解决;tools/ 与 monitor.sh 一律丢弃
git rm -r examples/hpm5e00evk/tools examples/hpm5e00evk/monitor.sh 2>/dev/null || true
git add -A && git commit --no-edit
```

- [ ] **Step 2:确认 monitor_sensors.rs 在 v3 且 tools/ 不存在**

```bash
ls examples/hpm5e00evk/src/bin/monitor_sensors.rs && ls examples/hpm5e00evk/tools 2>&1 | head -1
```

Expected:前者存在,后者 No such file。

### Task 6:adc 处理(按 Task 2 结论)

**Files:** src/adc/mod.rs、examples/hpm5e00evk/src/bin/adc_async.rs(视结论)

- [ ] **Step 1:跳过或 cherry-pick**

跳过:`git cherry-pick a4eb03a` 不执行,记录理由(v2 d66490c 为超集)。
保留差异:手工把 v1 独有段应用到 src/adc/mod.rs,单独提交 `feat(adc): port v1 async sequence sampling additions`。

- [ ] **Step 2:构建 adc_async 验证**

```bash
cd examples/hpm5e00evk && cargo build --bin adc_async 2>&1 | tail -2
```

### Task 7:WIP 应用与收尾清理

**Files:** src/dma/v1.rs、src/dma/v2.rs、src/ewdg.rs、HPMicro.yaml、examples/hpm5e00evk/Cargo.toml(如缺依赖)

- [ ] **Step 1:从主工作区复制 WIP 文件(按 Task 2 dma 结论)**

```bash
cd /Users/mono/Elec/HPM/hpm-hal-v3
cp /Users/mono/Elec/HPM/hpm-hal/src/dma/v1.rs src/dma/v1.rs      # 若保留精简版
cp /Users/mono/Elec/HPM/hpm-hal/src/dma/v2.rs src/dma/v2.rs
cp /Users/mono/Elec/HPM/hpm-hal/src/ewdg.rs src/ewdg.rs
# dma 若需补回 v2 功能:在 v2 版本基础上手动合并,不整文件覆盖
```

- [ ] **Step 2:HPMicro.yaml 的 stack_size 注释**

```bash
sed -i '' 's/^    stack_size: 1024/    # stack_size: 1024/' HPMicro.yaml
grep -c "# stack_size" HPMicro.yaml   # 应为 6
```

- [ ] **Step 3:提交 WIP(逻辑分组)**

```bash
git add src/dma src/ewdg.rs && git commit -m "chore(hpm5e): carry v1 dma/ewdg WIP"
git add HPMicro.yaml && git commit -m "chore: comment unsupported stack_size in HPMicro.yaml"
```

### Task 8:全量构建验证(逐板,磁盘管理)

**Files:** 无

- [ ] **Step 1:逐板构建(每板完检查磁盘)**

```bash
cd /Users/mono/Elec/HPM/hpm-hal-v3/examples
for d in hpm5300evk hpm5e00evk hpm6750evkmini hpm6e00evk hpm6200evk hpm6300evk; do
  (cd $d && cargo build --release 2>&1 | tail -1)
  df -h /System/Volumes/Data | tail -1
done
```

Expected:6 板全部编译通过;磁盘剩余 ≥ 50G(峰值 ~6G)。

- [ ] **Step 2:构建失败则修复并重跑对应板**

### Task 9:硬件 smoke 验证

**Files:** 无

- [ ] **Step 1:debug 构建 monitor_sensors 并烧录(7 分钟,用 fork probe-rs,超时 600s)**

```bash
cd /Users/mono/Elec/HPM/hpm-hal-v3/examples/hpm5e00evk
cargo build --bin monitor_sensors 2>&1 | tail -1
PROBE_RS_RISCV_SYSBUS_WRITE_CHUNK_WORDS=32 PROBE_RS_CMSIS_DAP_JTAG_SEQUENCE_BATCH=1 \
  /Users/mono/Elec/probe-rs/target/debug/probe-rs download \
  --chip HPM5E31 --chip-description-path /Users/mono/Elec/HPM/hpm-hal-v3/HPMicro.yaml \
  --protocol jtag --probe 0d28:0204:214F44B4FEDADD10BB0B8044E000E80A \
  --speed 500 --connect-under-reset --disable-progressbars --disable-double-buffering --verify \
  target/riscv32imafc-unknown-none-elf/debug/monitor_sensors
```

Expected:`Finished in Ns`,exit 0。

- [ ] **Step 2:复位 + smoke-bulk 确认传感器数据**

```bash
/Users/mono/Elec/probe-rs/target/debug/probe-rs reset --chip HPM5E31 \
  --chip-description-path /Users/mono/Elec/HPM/hpm-hal-v3/HPMicro.yaml --protocol jtag \
  --probe 0d28:0204:214F44B4FEDADD10BB0B8044E000E80A --speed 500
cd /Users/mono/Elec/HPM/hpm-hal/examples/hpm5e00evk   # 主工作区(工具仍在这里)
BIN=monitor_sensors ELF=/Users/mono/Elec/HPM/hpm-hal-v3/examples/hpm5e00evk/target/riscv32imafc-unknown-none-elf/debug/monitor_sensors \
  ./monitor.sh smoke-bulk
```

Expected:BME680 温湿压 + PMS7003 PM1.0/2.5/10 有非零读数,INVALID_FRAMES=0。

### Task 10:归档 v1、推送 v3

**Files:** 无

- [ ] **Step 1:本地归档 v1(主工作区)**

```bash
cd /Users/mono/Elec/HPM/hpm-hal
git branch -m hpm5e-support backup/hpm5e-support
git branch --list 'backup/*'
```

Expected:`backup/hpm5e-support` 存在;主工作区继续留在该分支(工作区文件不动)。

- [ ] **Step 2:推送 v3(需用户确认后执行)**

```bash
cd /Users/mono/Elec/HPM/hpm-hal-v3
git push -u origin hpm5e-support-v3
```

---

## Phase 2:monitor-web 独立仓库

### Task 11:创建 hpmicro-rs/monitor-web 仓库

**Files:** 无

- [ ] **Step 1:创建仓库(org 操作,执行前与用户确认可见性 public/private)**

```bash
gh repo create hpmicro-rs/monitor-web --public --description "Host-side monitor tooling for HPM5E sensor monitor firmware"
```

- [ ] **Step 2:本地初始化**

```bash
mkdir -p /Users/mono/Elec/HPM/monitor-web && cd /Users/mono/Elec/HPM/monitor-web
git init -b main
```

### Task 12:复制工具并适配

**Files:** monitor-web 内 monitor_web/、monitor_smoke.py、monitor_sensor_stress.py、monitor_benchmark.py、monitor.sh、pyproject.toml、README.md

- [ ] **Step 1:从主工作区复制(含未提交改动)**

```bash
cd /Users/mono/Elec/HPM/monitor-web
cp -R /Users/mono/Elec/HPM/hpm-hal/examples/hpm5e00evk/tools/monitor_web .
cp /Users/mono/Elec/HPM/hpm-hal/examples/hpm5e00evk/tools/monitor_smoke.py \
   /Users/mono/Elec/HPM/hpm-hal/examples/hpm5e00evk/tools/monitor_sensor_stress.py \
   /Users/mono/Elec/HPM/hpm-hal/examples/hpm5e00evk/tools/monitor_benchmark.py .
cp /Users/mono/Elec/HPM/hpm-hal/examples/hpm5e00evk/monitor.sh .
rm -rf monitor_web/__pycache__ monitor_web/static/__pycache__ 2>/dev/null || true
find . -name __pycache__ -type d -exec rm -rf {} + 2>/dev/null || true
```

- [ ] **Step 2:server.py 适配(去 sys.path hack,改同目录 import)**

原:
```python
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
import monitor_smoke as monitor
```
改为:
```python
ROOT = Path(__file__).resolve().parent.parent
import monitor_smoke as monitor
```
并把 `DEFAULT_ELF` 改为环境变量优先:
```python
DEFAULT_ELF = Path(os.environ.get("MONITOR_ELF", ROOT.parent / "target/riscv32imafc-unknown-none-elf/debug/monitor_sensors"))
```
其他引用 `ROOT / "target..."` 的地方同步检查替换。

- [ ] **Step 3:monitor.sh 路径参数化检查**

确认全部外部路径已有 env var 覆盖:`PROBE_RS`、`CHIP_DESC`(默认值改为指向用户 hpm-hal 检查出的 HPMicro.yaml,README 说明)、`PROBE`、`BIN`、`ELF`、`HPM_SDK_BASE`、`OPENOCD`。若无则补 `CHIP_DESC="${CHIP_DESC:-}"` 并加缺失检测报错。

- [ ] **Step 4:新增 pyproject.toml 与 README.md**

pyproject.toml:
```toml
[project]
name = "hpm5e-monitor-tools"
version = "0.1.0"
requires-python = ">=3.10"
dependencies = ["pyserial>=3.5", "pyusb>=1.2"]
```
README.md:仓库定位、快速上手(`uv run --with pyserial --with pyusb python monitor_web/server.py`)、协议简述(MONITOR_START_CMD 0x4850 / MAGIC 0x4d44 / 查询 GETVALUE 0x00a0)、硬件要求(HPM5E00EVK 烧 monitor_sensors 固件)、env var 清单。

- [ ] **Step 5:提交**

```bash
git add -A && git commit -m "feat: extract HPM5E monitor host tooling from hpm-hal v1"
```

### Task 13:monitor-web 本地验证并推送

**Files:** 无

- [ ] **Step 1:smoke-bulk 用新仓库工具跑通(板子应仍在运行 v3 固件)**

```bash
cd /Users/mono/Elec/HPM/monitor-web
ELF=/Users/mono/Elec/HPM/hpm-hal-v3/examples/hpm5e00evk/target/riscv32imafc-unknown-none-elf/debug/monitor_sensors \
  uv run --with pyserial --with pyusb python monitor_smoke.py --transport bulk --elf "$ELF" --timeout 2
```

Expected:传感器读数输出同 Task 9。

- [ ] **Step 2:server.py 起服务验证**

```bash
MONITOR_ELF=<同上> uv run --with pyserial --with pyusb python monitor_web/server.py --port 8765 &
curl -s http://127.0.0.1:8765/ | head -5
```

Expected:HTTP 200 返回前端页面;数据端点能返回板子读数(视前端接口,至少服务无异常日志)。

- [ ] **Step 3:推送(需用户确认)**

```bash
git remote add origin git@github.com:hpmicro-rs/monitor-web.git
git push -u origin main
```

---

## 自检对照(spec → 任务)

- repo 名/范围 A/全新初始化 → Task 11、12
- v3 删除 tools/ + monitor.sh → Task 3、5
- rebase 结构(v2 基底 + v1 五提交) → Task 1、3-6
- dma 先审查再定 → Task 2 Step 4、Task 7 Step 1
- usb/adc 对比处置 → Task 2 Step 1-2、Task 4、6
- ewdg 保留、HPMicro.yaml 保留+注释 → Task 7
- 旧分支留档 backup/ → Task 10
- 验证:6 板构建 + 硬件 smoke + monitor-web 本地验证 → Task 8、9、13
- 磁盘管理(逐板+清理)→ Task 8 Step 1;master 同步 → 范围外(无任务,符合 spec)

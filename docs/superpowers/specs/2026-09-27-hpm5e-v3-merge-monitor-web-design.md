# hpm5e v1+v2 合并为 v3 & monitor-web 独立仓库 — 设计

日期:2026-09-27
状态:已与用户逐节确认

## 背景

hpm-hal 仓库存在两条同源分叉的开发线(merge base `620f58d fix(hpm53): fix examples`):

- **v1**(本地 `hpm5e-support`,无远程备份):5 个提交,HPM5E 传感器监控固件 + 主机侧工具(monitor_web / smoke / benchmark)+ USB 修复 + adc async sequence sampling;另有未提交 WIP(dma 精简、ewdg、HPMicro.yaml、monitor_web 前端改动)
- **v2**(`origin/hpm5e-support-v2`):72 个提交,更宽的 HAL 开发(sdxc、i2s、pdm、pwm v2、femc 重构、sysctl 拆分、adc finite sequence sampling、usb 硬化、CI 等),**没有**监控固件和工具

两者都不包含上游 master(femc PR #9 等 74 个提交),master 同步不在本次范围。

目标:
1. 监控主机工具独立为 `hpmicro-rs/monitor-web` 仓库
2. v1 + v2 合并为 v3 分支,推送到远程

## 决策记录(用户已确认)

| 决策点 | 结论 |
|---|---|
| 新 repo 名 | `hpmicro-rs/monitor-web` |
| repo 范围 | A:整个 tools/ 套件(monitor_web/、monitor_smoke.py、monitor_sensor_stress.py、monitor_benchmark.py)+ monitor.sh |
| hpm-hal 侧 | v3 中删除 tools/ + monitor.sh,只留固件 |
| 合并结构 | v2 为基底,v1 的 5 个提交逐个 rebase 上去 |
| dma WIP 冲突 | 先功能 review:精简是故意的则保留本地版,误删则补回 |
| 旧分支 | 留档;本地 v1 重命名 `backup/hpm5e-support`(沿用仓库 backup/ 惯例),远程 v2 保留作档,不再参与本地检查 |
| 验证磁盘 | 逐板构建→验证→清理,控制 ~6G 峰值 |

## 工作流 A:monitor-web 独立仓库

- **全新初始化**(不带 git 历史;工具代码仅数个提交,历史价值低)
- 内容取自 v1 工作区的 `examples/hpm5e00evk/tools/` + `monitor.sh`(含未提交的 monitor_web 前端改动和 monitor_smoke.py 的 PMS/BH 符号扩充)
- 适配改动:
  - 去掉 server.py 的 `sys.path` hack,同目录 import monitor_smoke
  - 路径参数化:默认 ELF、HPMicro.yaml、probe 串号、端口等走环境变量/CLI 参数(monitor.sh 已有大部分 env var,补齐 README 说明)
  - 新增 `pyproject.toml`(依赖 pyserial、pyusb)与顶层 README(协议说明、快速上手、硬件要求)
  - 清理 `__pycache__`
- 固件 `monitor_sensors.rs` 留在 hpm-hal 作参考实现,不在新 repo

## 工作流 B:v3 分支合并

- 基底:`origin/hpm5e-support-v2` tip(`3b028e7`)
- v1 五提交处置规则:
  1. `3e9702e monitor tooling` → 跳过(迁 monitor-web)
  2. `1bbbce7 multi-qtd usb transfer` → 与 v2 usb 修复 diff 对比:重复跳过,独有保留
  3. `9e07d2b harden runtime and USB` → 同上
  4. `2d190a0 qualified sensor monitor` → 保留(固件与 statics,**剔除 tools/ 部分**)
  5. `a4eb03a adc async sequence sampling` → 与 v2 `d66490c finite sequence sampling` 对比,确认超集则跳过,缺功能则补齐
- 未提交 WIP 处置:
  - dma/v1.rs、dma/v2.rs → 按 review 结论(见风险)
  - ewdg.rs → 保留(v2 的 wdg.rs 共存,不同模块)
  - HPMicro.yaml → 保留(stack_size 已注释,probe-rs 不支持该字段)
  - monitor_smoke.py、monitor_web/ → 删除(迁 monitor-web)
- 隔离方式:git worktree(主工作区有 WIP,不在主工作区切换分支)
- 完成形态:新分支 `hpm5e-support-v3` 推 origin;本地 v1 重命名 `backup/hpm5e-support`

## 验证

1. v3 全量构建:6 板 example 逐个 `cargo build --release`(等价 ci.sh),**构建一个验一个,验完可清缓存**,控制磁盘峰值 ~6G(当前剩余 57G)
2. 硬件验证:`monitor_sensors` 烧 HPM5E00EVK(probe-rs fork,预计 ~7 分钟),`smoke-bulk` 确认 BME680/PMS7003/BH1750 读数
3. monitor-web:本地起 server.py,确认经 USB bulk 读到板子实时数据,前端页面可开
4. tools/ 移除后 hpm5e00evk example 仍可编译

## 风险与范围外

- **dma review 是最大不确定点**:若发现本地精简版误删了 v2 功能需补回
- v1 的 USB 修复与 v2 usb 硬化可能语义重叠,以 diff + 硬件 smoke 验证为准
- **上游 master 同步不在本次范围**(后续单独处理)
- 远程 v2 分支保留作档,v3 稳定后再决定是否删除

## 交付物

1. `hpmicro-rs/monitor-web` 仓库(初始提交 + 可运行)
2. `origin/hpm5e-support-v3` 分支(构建全绿 + 硬件 smoke 通过)
3. 本地旧分支留档(backup/ 前缀)

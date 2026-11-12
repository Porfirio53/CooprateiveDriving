# HOCD + 当前 TiRex：T248P 首轮实验

工作目录为 `/home/patton/cooperative_driving`。外部 MyTirex 源码、当前 TiRex 和模型，以及 CARLA 二进制均只读复用。部署记录见 `docs/接入前准备验收.md`。

## 接入方向盘

1. 接好 T248P 电源与踏板，再接 USB，等待自校准结束。
2. 在方向盘 `MODE → PS/PC` 选择 `PC`，`ROT°` 选择 `360°`。
3. 在桌面终端运行轴检查，转动方向盘，按 Ctrl+C 退出：

   ```bash
   cd /home/patton/cooperative_driving
   bash scripts/probe_wheel.sh
   ```

4. 运行引导标定，依次回正、保持左端、保持右端、回正；程序自动确定轴号和符号并写入 `configs/wheel.json`：

   ```bash
   bash scripts/probe_wheel.sh --calibrate
   ```

   自动识别支持 `T248` 名称和 USB 标识 `044f:b696`；系统显示 `Thrustmaster Racing Wheel FFB` 也可识别。如果同时接了多个方向盘，可加 `--device-index N`。`calibrated:false` 时，真人启动命令会要求先完成测轴。首轮不接踏板控制、按钮换道或力反馈。

## 当前正式实验：单人三地图九组

当前只安排 P001 一人。Town03、Town04、Town05 三张地图各有固定障碍和绕行路线，每张各运行一次共享 Hold、共享 TiRex、真人独立转向 + TiRex。都注入 300 ms 延迟，纵向 PID 目标统一为 30 km/h，计分行驶约 30–45 秒，90 秒为硬上限。

**九条逐组启动指令及操作流程见 [正式实验_单人三地图.md](docs/正式实验_单人三地图.md)。每次由驾驶者手动启动一组，检查结果后再继续。** 例如：

```bash
cd /home/patton/cooperative_driving
bash scripts/run_formal.sh --map town03 --condition hold
```

`manual_tirex` 保留同样的 TiRex 补偿，由真人独立转向，不加载 MPC/PPO。每个物理 tick 只记录 CSV 数值，正式运行不保存图片序列或自动输出图片。完整轨迹为 `trajectory.csv`，参考路线、障碍坐标和原始控制数据随每趟保留。

## 原长路线诊断入口

```bash
cd /home/patton/cooperative_driving
bash scripts/carla.sh start
bash scripts/run_trial.sh --condition direct --participant PILOT
bash scripts/run_trial.sh --condition hold --participant PILOT
bash scripts/run_trial.sh --condition tirex --participant PILOT
```

先用无额外延迟的一趟确认方向与车辆一致、画面连续和正常结束，然后按顺序自己跑通 Hold、TiRex。每趟先停车积累至少 7 秒历史，TiRex 还会等首条可用预测。当前模型在桥接进程启动时加载和预热，之后整趟复用；首次准备可能需要几十秒，预热不计入驾驶时间。

`Esc`、关闭窗口或 `Ctrl+C` 结束并清理；中断也保存结果。Town04 固定路线约 613 m，目标速度 30 km/h，最长计分 90 秒。保留原 Frenet 局部规划、MPC、PID、PPO 和 0.2 秒预瞄，PPO 用 CPU。参考路线坐标已保存，运行时不使用旧 waypoint 索引定位。

当前单人九组的预排顺序在 `configs/participant_schedule.csv`。旧长路线命令仅用于已有部署记录和单独诊断；正式运行使用上面的 `run_formal.sh`。

```bash
bash scripts/run_trial.sh --condition hold --participant PILOT
bash scripts/run_trial.sh --condition tirex --participant PILOT
```

新趟自动生成独立目录。两条件都经 5004 → 5005 → 5006，只有转向数据注入 300 ms 延迟，同步消息即时返回。HOCD 在每个控制周期按计划下一物理步的时间重新插值中位轨迹，失效或无覆盖则使用已收到的 Hold。

## 数据与复查

每趟 `runs/<trial>/config.json`、`ticks.csv`、`summary.json` 保存配置、每 tick 数据和结束原因；同目录保存原始桥接输入、接收事件、推理记录、参考路线和 `trajectory.csv`。默认不保存相机图片，诊断时可用 `--save-preview` 显式保存一张末帧预览。`diagnostic_input:true` 的准备记录使用程序生成的输入，不能当作真人实验数据。

```bash
source scripts/env.sh
"$HOCD_PYTHON" -B scripts/analyze_run.py runs/<trial>
"$HOCD_PYTHON" -B -m unittest discover -s tests -v
"$HOCD_PYTHON" -B scripts/verify_external_resources.py
bash scripts/carla.sh status
bash scripts/carla.sh stop
```

分析输出 `metrics.json`、`scored_ticks.csv`；包括真实位姿的横向 RMSE、碰撞和完成情况、以实际物理 tick 的原始采样保持值计算的转向误差、实测 AoI、推理耗时、施加时间偏差及回退比例。物理作用时刻由下一次 `world.tick` 提交和返回包围，CSV 分别记录两者；不宣称能观测引擎内部的精确墙钟作用时刻。

## 环境与条件驱动

HOCD 用系统 Python 3.10 的独立 venv，路径 `envs/hocd`，与方案的解释器版本和依赖一致；重建用 `bash scripts/bootstrap_hocd.sh`，锁定清单为 `configs/requirements-hocd.lock`。TiRex 直接调用已有 `/home/patton/MyTirex/.venv/bin/python`，不安装或升级它的依赖。

pygame 测轴不依赖系统 joystick 包，备用 `jstest` 已解包在新目录，运行 `bash scripts/jstest.sh /dev/input/js0`。T248 社区驱动已下载并针对当前内核预编译，未安装或加载。只有接入后无法正常读轴时，才按原方案使用：

```bash
# 先拔掉方向盘 USB
cd /home/patton/cooperative_driving/vendor/hid-tmff2
sudo make install
sudo modprobe hid-tmff-new
# 重新接入，选择 PC，再测轴
```

驱动安装需要机器管理员密码。当前阶段没有触发安装；没有进行固件更新。LAIA、175 ms、协变量与扩展实验按方案留到真人系统跑通后。

## GitHub 仓库范围

仓库保留 `bridge/`、`scripts/`、`tests/`、`configs/`、`docs/` 和经过本项目修改的 `vendor/HOCD/` 源码。HOCD 以普通文件快照纳入版本管理，基线版本见 `configs/deployment_manifest.json`，原项目说明保留在 `vendor/HOCD/README.md`。

`.gitignore` 排除本地虚拟环境、缓存、编辑器数据库、运行日志、`runs/` 实验记录、`reviews/` 生成报告、`runtime/` 状态、`data/` 数据集、外部模型、可选方向盘驱动及编译产物。忽略规则不删除本地文件。配置中的路线 JSON、参与者顺序 CSV 和依赖锁文件仍纳入版本管理。

共享控制运行和测试所需的 `vendor/HOCD/rl/model/ppo/actor_model.pkl`、`critic_model.pkl` 很小，作为明确例外保留；新增训练权重和序列化数据默认忽略。上游 `utils/feasible_routes.pkl` 当前只是 Git LFS 指针，不随仓库提交；若使用上游强化学习训练入口，需另行从 HOCD 上游获取实际路线数据。

克隆后通过 `bash scripts/bootstrap_hocd.sh` 重建 HOCD 环境，CARLA、外部 MyTirex/TiRex 源码及 TiRex 模型按部署文档单独准备。现有脚本和配置包含 `/home/patton/...` 本机路径，换机器部署前需按文档核对并调整；`logs/` 中的历史验收基线、`runs/` 数据和 `reviews/` 报告也不随克隆提供。

`vendor/hid-tmff2/` 是可选的外部驱动，本机副本保持原状。新机器只有需要该驱动时才按部署文档单独克隆，历史部署版本记录在 `configs/deployment_manifest.json`。

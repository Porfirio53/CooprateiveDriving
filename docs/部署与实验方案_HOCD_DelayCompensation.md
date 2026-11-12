# HOCD + DelayCompensation 快速部署与首轮实验方案

> 2026-10-21 更新：当前正式实验已按用户要求改为单人、三地图、三条件共九组。具体场景、条件和启动指令以 [正式实验_单人三地图.md](正式实验_单人三地图.md) 为准；本文中的旧正式采集安排保留为部署背景。

本方案依据截至 2026 年 10 月 19 日的代码与公开资料编写；尚未在目标 Ubuntu 机器实装，真人输入桥接和预测轨迹回传需要补充少量代码。

先用一台 Ubuntu 22.04 电脑、现有 CARLA 0.9.16 和图马斯特 T248P 跑通横向共享驾驶，再比较固定 300 ms 延迟下的 Hold 与当前 TiRex 补偿。首轮选 Town04 的一段车道保持或简单弯道，无动态障碍。纵向沿用 HOCD 的 PID，真人只操纵方向盘。只预测一个连续转向信号，speed 协变量、175 ms 条件与公开数据回放随后补充。

## 准备目录与固定版本

以下命令在 Ubuntu 执行。协同驾驶独立工作目录统一为 `/home/patton/cooperative_driving`，新增代码、环境、配置、数据、模型运行缓存和实验记录全部放在这里。`/home/patton/MyTirex` 与 `/home/patton/carla_experiments` 仅作为外部资源读取，不在其中新增或修改文件。

| 内容 | Ubuntu 路径 | 本次操作 |
|---|---|---|
| 协同驾驶工作目录 | `/home/patton/cooperative_driving` | 全部新增工作放在此处 |
| 延迟补偿接口 | `/home/patton/MyTirex/src/mytirex` | 从外部导入，保持原文件不变 |
| 当前 TiRex 实现与模型 | `/home/patton/MyTirex/vendor/tirex`、`/home/patton/MyTirex/models/tirex2` | 只读加载 |
| CARLA 二进制与地图 | `/home/patton/carla_experiments/simulator` | 调用原二进制，运行目录与日志另行指定 |
| HOCD | `/home/patton/cooperative_driving/vendor/HOCD` | 克隆、适配与配置修改均在新目录内 |
| TiRex 桥接 | `/home/patton/cooperative_driving/bridge` | 新增外部适配和轨迹回传代码 |
| 实验记录 | `/home/patton/cooperative_driving/runs` | 保存每趟配置与 CSV |
| LAIA 小包 | `/home/patton/cooperative_driving/data/LAIA` | 真人系统跑通后再下载 |

目录安排如下：

```text
/home/patton/cooperative_driving/
├── vendor/HOCD/                 # 新克隆的 HOCD，含方向盘与控制端适配
├── bridge/                     # TiRex 外部适配、延迟队列及轨迹回传
├── configs/                    # 本实验配置及从旧项目读取的配置副本
├── scripts/                    # 启动脚本
├── envs/hocd/                  # 新建的 HOCD Python 环境
├── data/LAIA/                  # 辅助数据
├── runs/                       # 真人记录与回放结果
├── logs/                       # 进程日志
├── runtime/carla/              # CARLA 运行时 Saved、配置等
└── .cache/                     # 本次工作使用的编译与运行缓存
```

```bash
sudo apt-get update
sudo apt-get install -y git curl build-essential libgl1 libglib2.0-0 libvulkan1 joystick
mkdir -p /home/patton/cooperative_driving/{vendor,bridge,configs,scripts,envs,data/LAIA,runs,logs,runtime/carla,.cache}
cat > /home/patton/cooperative_driving/scripts/env.sh <<'SH'
export COOP_ROOT=/home/patton/cooperative_driving
export PYTHONDONTWRITEBYTECODE=1
export XDG_CACHE_HOME="$COOP_ROOT/.cache"
export HF_HOME="$COOP_ROOT/.cache/huggingface"
export TORCH_HOME="$COOP_ROOT/.cache/torch"
export CUDA_CACHE_PATH="$COOP_ROOT/.cache/cuda"
export MPLCONFIGDIR="$COOP_ROOT/.cache/matplotlib"
export CONDA_PKGS_DIRS="$COOP_ROOT/.cache/conda"
mkdir -p "$CUDA_CACHE_PATH" "$MPLCONFIGDIR"
SH
source /home/patton/cooperative_driving/scripts/env.sh
cd "$COOP_ROOT/vendor"
git clone https://github.com/i-Qin/HOCD.git HOCD
cd HOCD
git checkout 49b0189595f3dc525f4bc7194671742e203f7839
git switch -c thesis/carla-0916-delay
```

采用本次审阅的 HOCD 版本，仓库已包含 PPO 权重，无需先训练。每趟配置记录 HOCD 版本、TiRex 模型路径和实验参数即可。

后续每个运行终端先执行 `source /home/patton/cooperative_driving/scripts/env.sh`。原 MyTirex 的配置只读；需要调整的参数先复制到新目录再改。禁止 Python 在旧源码旁生成 `__pycache__`，TiRex 的内核缓存也由新适配器指向本工作目录。

## HOCD 单独建环境，TiRex 复用原环境

### HOCD：Python 3.10

CARLA 0.9.16 已提供 Python 3.10–3.12 的 Linux wheel。两个环境分别保留 HOCD 的旧 Torch 与当前 DelayCompensation 的新依赖。下面是 HOCD 的安装起点，不直接安装原 `requirements.txt` 中的 CARLA 0.9.14、CUDA Torch 和 `yaml~=0.2.5`。

```bash
source /home/patton/cooperative_driving/scripts/env.sh
conda create --prefix "$COOP_ROOT/envs/hocd" python=3.10 pip -y
conda activate "$COOP_ROOT/envs/hocd"
python -m pip install 'torch==1.13.1+cpu' --index-url https://download.pytorch.org/whl/cpu
python -m pip install 'carla==0.9.16' 'numpy==1.24.4' 'cvxopt==1.3.2' \
  'gym==0.26.2' 'matplotlib==3.7.5' 'networkx==3.1' 'PyYAML==6.0.2' \
  'omegaconf==2.3.0' 'pandas==2.0.3' 'seaborn==0.13.2' \
  'pygame==2.6.1' 'tabulate==0.9.0'
cd "$COOP_ROOT/vendor/HOCD"
```

PPO 权重记录了 `cuda:0`。在 HOCD 的 `rl/ppo.py` 两处加载语句加入设备映射，供 CPU 推理使用：

```python
self.actor.load_state_dict(torch.load(self._actor_path, map_location=self._device))
self.critic.load_state_dict(torch.load(self._critic_path, map_location=self._device))
```

### MyTirex：直接复用已有环境和模型

打开此前运行延迟补偿实验的终端，激活原来的 TiRex 环境，只复用其 Python 和已安装依赖，不重新安装或升级。当前项目使用 Python 3.12、Torch 2.8.0、CUDA/nvcc 12.8；与 HOCD 的 Python 3.10 环境分开运行。桥接脚本在新工作目录中运行，通过导入路径调用原来的 `mytirex` 和当前 TiRex 源码。下面两条复制配置的命令只在首次准备时执行，后续保留新目录内调整过的副本。

```bash
source /home/patton/cooperative_driving/scripts/env.sh
export PYTHONPATH="/home/patton/MyTirex/src:/home/patton/MyTirex/vendor/tirex/src${PYTHONPATH:+:$PYTHONPATH}"
cd "$COOP_ROOT"
cp /home/patton/MyTirex/configs/tirex_tuning.json "$COOP_ROOT/configs/"
cp /home/patton/MyTirex/configs/experiment.json "$COOP_ROOT/configs/"
```

模型始终以绝对路径 `/home/patton/MyTirex/models/tirex2` 只读加载，不复制模型或源码。新实验只修改 `$COOP_ROOT/configs` 中的副本。模型加载适配见“最小桥接实现”：将资源位置与运行位置分开，不能直接用原来的项目级启动命令，否则会把输出或内核缓存写回 MyTirex。

当前调参保留 256 点历史、32 点趋势窗、阈值 1.5、最小 R² 0.6、`tta_diff:true`、`pad_context:true` 和关闭的 sign-flip TTA。25 ms 采样下历史约 6.4 秒，先不重新调参。TiRex 首次调用需要编译内核，提前预热，正式驾驶时复用已加载模型。

## 先把 HOCD 真人共享驾驶开起来

### T248P：先读方向轴，必要时再装驱动

T248P 可以作为 HOCD 的真人转向设备。HOCD 的 `controller/g29.py` 实际使用 pygame joystick，未限制设备品牌；需要修改的是设备序号、方向轴和原来的 G29 按钮逻辑。只改配置文件还不够，因为上游 `G29.__init__(config)` 没有使用传入的配置。[输入源码](https://github.com/i-Qin/HOCD/blob/49b0189595f3dc525f4bc7194671742e203f7839/controller/g29.py)。

接好电源、踏板和 USB，等待方向盘自校准结束。按 `MODE`，用十字键上下选 `PS/PC`、左右选 `PC`，再按 `MODE` 确认；同样在 `ROT°` 项选 `360°` 并确认，直接在方向盘上设置即可。手册说明转角设置适用于应用未自动控制转角的情况，后面以实际轴读数为准。首轮只读转向，踏板控制、拨片换道和力反馈都不接入。[官方使用手册，MODE 章节](https://ts.thrustmaster.com/download/accessories/Manuals/T248/T248PS_User_manual.pdf)。

在 Ubuntu 桌面中激活 `/home/patton/cooperative_driving/envs/hocd` 环境，运行下面这段读轴程序，转动方向盘，记下设备序号、随转动变化的轴号和左右方向；按 Ctrl+C 结束。这里直接使用 pygame，轴号与 HOCD 一致。

```bash
python - <<'PY'
import time
import pygame

pygame.init()
pygame.display.set_mode((520, 100))
wheels = [pygame.joystick.Joystick(i) for i in range(pygame.joystick.get_count())]
print("设备数:", len(wheels))
for i, wheel in enumerate(wheels):
    wheel.init()
    print(i, wheel.get_name(), "axes=", wheel.get_numaxes())
try:
    while True:
        pygame.event.pump()
        for i, wheel in enumerate(wheels):
            print(i, [round(wheel.get_axis(j), 3) for j in range(wheel.get_numaxes())])
        time.sleep(0.2)
except KeyboardInterrupt:
    pass
finally:
    pygame.quit()
PY
```

中心应接近 0，左右端接近 −1 和 +1。能正常读轴就直接进入下一节。只有没有设备或方向轴读数明显异常时，再安装支持 T248 的社区驱动 [Kimplul/hid-tmff2](https://github.com/Kimplul/hid-tmff2)。先拔掉方向盘 USB，再执行其手动安装路线：

```bash
sudo apt-get install -y build-essential "linux-headers-$(uname -r)" joystick
git clone --recurse-submodules https://github.com/Kimplul/hid-tmff2.git \
  /home/patton/cooperative_driving/vendor/hid-tmff2
cd /home/patton/cooperative_driving/vendor/hid-tmff2
make
sudo make install
sudo modprobe hid-tmff-new
```

重新插入 USB，切到 PC 模式，再读一次方向轴。若系统明确提示 Secure Boot 拒绝加载模块，再按驱动项目说明处理模块签名。图马斯特官方支持页提供的是 Windows 驱动与固件工具，Ubuntu 不安装其 `.exe`；也不把固件更新或独立 `hid-tminit` 安装作为首轮前置步骤。[官方支持页](https://support.thrustmaster.com/en/product/t248-ps-en/)。

### 替换方向盘输入适配

为减少改动，保留内部类名 `G29` 和配置键 `driver_model: G29`；它们只是 HOCD 的历史命名，实际设备是 T248P。以下 HOCD 文件均指新克隆的 `/home/patton/cooperative_driving/vendor/HOCD` 内的文件。将其中的 `controller/g29.py` 替换为下面的首轮转向版：

```python
import pygame
from utils.data import LatDebugInfo


class G29:
    def __init__(self, config=None):
        config = config or {}
        pygame.init()
        pygame.joystick.init()
        self._joystick = pygame.joystick.Joystick(int(config.get("device_index", 0)))
        self._joystick.init()
        self._steer_idx = int(config.get("steering_axis", 0))
        self._steer_sign = float(config.get("steering_sign", 1))
        self.connected = True

    def control(self, path_list=None, vehicle_state=None, e_rr=None, min_index=None):
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                raise KeyboardInterrupt
            if event.type == pygame.KEYUP and event.key == pygame.K_ESCAPE:
                raise KeyboardInterrupt

        steer = self._steer_sign * float(self._joystick.get_axis(self._steer_idx)) * 3.14
        return steer, LatDebugInfo(
            intention="",
            g29_connected=self.connected,
            triangle=False,
            rect=False,
            circle=False,
            times=False,
        )
```

这个版本按配置选择设备和方向轴，移除了“只能接一个 joystick”的限制，也不再读取 G29 固定按钮号。关闭窗口或按 Esc 通过主入口的退出清理结束。首轮换道意图为空；在 `display/pygame_display.py` 的 `run_step` 中删除整个 `if ctrl.lat.g29_connected:` 条件块，连同其下随机亮灯、绘制按钮和反应时间任务的三行调用，避免进入原来的 G29 按键任务。

转向继续沿用上游 `axis × 3.14` 的内部尺度，不重新定义 PPO 输入。`steering_sign` 取 1 或 −1，让左右方向与车辆一致；两组使用完全相同的轴映射、转角设置和输出转换。首轮不发送力反馈指令；装好驱动也不会自动得到 CARLA 路感。

### 启动 CARLA 与共享控制

从新工作目录启动原 CARLA 的 Shipping 二进制，准备 Town04 地图。直接调用二进制，避免原 `CarlaUE4.sh` 中可能执行的权限修改；用 `-userdir` 将项目运行文件放到新目录，用 `-abslog` 指定日志位置：

```bash
source /home/patton/cooperative_driving/scripts/env.sh
cd "$COOP_ROOT"
/home/patton/carla_experiments/simulator/CarlaUE4/Binaries/Linux/CarlaUE4-Linux-Shipping \
  CarlaUE4 -quality-level=Low -carla-rpc-port=2000 \
  -userdir="$COOP_ROOT/runtime/carla" -abslog="$COOP_ROOT/logs/carla.log" \
  > "$COOP_ROOT/logs/carla-console.log" 2>&1
```

以上路径对应标准 CARLA 0.9.16 Linux 包，只读取原引擎、地图和资源。`-userdir` 对应 UE 的项目用户目录，项目 Saved 文件由其派生；`-abslog` 指定绝对日志文件。[CARLA 直接调用示例](https://github.com/carla-simulator/carla/issues/8079)，[UE 用户目录接口](https://dev.epicgames.com/documentation/en-us/unreal-engine/API/Runtime/Core/Misc/FPaths/CustomUserDirArgument?application_version=4.27)，[UE 命令参数](https://dev.epicgames.com/documentation/en-us/unreal-engine/command-line-arguments?application_version=4.27)。

在新 HOCD 的 `utils/config.yaml` 合并以下字段，保留其余设置。主入口从固定路径读取配置，没有 `--config` 参数。其他实验参数放在新工作目录的 `configs` 中，不改原 carla_experiments 的配置。

```yaml
simulator:
  town: Town04
  synchronous: true
  fixed_time: 0.01
  no_rendering: false
controller:
  pre_ts: 0.2
  dt: 0.01
  lateral:
    type: SharedControl
    SharedControl:
      driver_model: G29
      vehicle_model: MPC
      authority_model: PPO
      driver_state: 0.5
    G29:
      device_index: 0
      steering_axis: 0
      steering_sign: 1
rl:
  device: cpu
  train: false
  use_state_norm: false
```

`G29` 下的 0、0、1 是填写示例，改成上一节实测的设备序号、轴号和方向符号。必须用 `SharedControl + G29`；直接设 `lateral.type:G29` 会绕过共享控制。`driver_state:0.5` 是固定策略输入，不当作真人疲劳测量。

方向盘适配完成后，再做三项启动修改：

1. 选择能完成的 Town04 简单路线，记录起终点坐标；旧版 514/441/1648 等 waypoint 索引要在 0.9.16 重新确认。移除动态障碍；没有候选路径时停止或退出，避免空路径崩溃。
2. 将 `PygameDisplay.run_step` 内 `clock.tick(30)` 从控制循环移走，显示独立限流。唯一主循环按墙钟每 10 ms 执行一次 `world.tick()`，规划仍每 0.2 秒一次，预测异步进行；保留相机渲染供真人驾驶。
3. 主入口补碰撞传感器和 CSV 记录；退出时销毁车辆、相机并恢复 world 同步设置。

```bash
source /home/patton/cooperative_driving/scripts/env.sh
conda activate "$COOP_ROOT/envs/hocd"
cd "$COOP_ROOT/vendor/HOCD"
python -B main.py
```

先由自己开完一趟无额外延迟路线，确认可以转向、看到连续画面、正常结束，且仿真时间与墙钟基本一致。这是部署准备，不增加实验条件。先不重写规划器，也不默认重训 PPO。

## 接入转向：一个输入、一个小桥接模块

### 单目标配置与已有链路

在 `/home/patton/cooperative_driving/configs/hocd_signals.json` 新建单目标配置：

```json
{
  "sample_period_us": 25000,
  "targets": ["steer_h"],
  "past_covariates": [],
  "future_covariates": [],
  "units": {"steer_h": "hocd_steer"}
}
```

真人转向在 HOCD 端至少 100 Hz 采集，时间戳用 `time.monotonic_ns()//1000`。按 25 ms 网格取该时刻前已采到的最近值；`source_time_us` 写网格时刻，`native_source_time_us` 写所用原始采样时刻，不能填 CARLA 仿真时间。只有 `steer_h`，不加 speed。原始转向同时存日志供事后评分。

直接复用现有底层流式接口，不再额外运行合成源联通步骤。原 `mytirex stream-receive --root /home/patton/MyTirex` 会向旧目录写入 `outputs`，本方案由新目录内的桥接脚本调用相同的接收与预测组件，并自行指定记录位置。

### 最小桥接实现

只采用本机 UDP JSON，不引入消息队列或跨机服务。HOCD Python 3.10 不能导入要求 Python 3.12 的 `mytirex`；两端用 socket 传数值。在 `/home/patton/cooperative_driving/bridge` 新增两个小文件：`hocd_tirex_adapter.py` 处理外部模型加载，`tirex_bridge.py` 复用原来的 `SignalSource`、`CaptureSession` 和 `StreamingRuntime`，处理采样、固定延迟队列及结果回传。文件尚需实现；这里规定其接口与写入位置。

模型适配器采用 `ExternalTiRexModel(TiRexModel)`：只覆盖初始化、加载和记录元数据的部分，继承当前的信号上下文构造、趋势处理和预测方法。资源根固定为 `/home/patton/MyTirex`，运行根固定为 `/home/patton/cooperative_driving`。具体约定为：

- 从新 `configs` 副本读取配置，`batch_size=1`；通过已有环境导入旧源码，不向旧项目安装任何东西。
- 用 `configure_runtime(运行根)` 创建内核缓存，再从当前 `vendor/tirex` 调用 `tirex2.load_model`，模型参数为旧模型的绝对路径。保持现有 `device="cuda"`、`compile=False`、`use_flex_attention=False` 和趋势处理设置。
- 不调用父类中绑定项目目录的构造、`from_local()`、加载或元数据入口；这些入口把资源根和运行根绑定，会在旧目录创建缓存。直接把 `--root` 改成新目录也无法替代这层适配。
- `CaptureSession` 与 `StreamingRuntime` 继续使用原实现，日志和发布回调由新桥接统一写入新 `runs`、`logs`。以 `python -B` 启动桥接，保持旧源码目录无字节码写入。

这层适配只分开资源读取与运行输出，不修改预测算法，也不修改 MyTirex 中的任何代码。[模型加载](../../代码/DelayCompensation/src/mytirex/inference/model.py)，[内核缓存配置](../../代码/DelayCompensation/src/mytirex/inference/gpu.py)，[现有在线入口](../../代码/DelayCompensation/src/mytirex/streaming/live.py)。

1. HOCD 向本机桥接发送原始转向及采样时间；桥接形成 25 ms 网格值。
2. 只给转向数据排队 300 ms，保留原序号和时间戳；现有同步响应即时处理。接收器记录真正到达时间，因此 300 ms 注入值不直接代替实测 AoI。
3. 用已有单在途线程推理，最小请求间隔 25 ms；保留当前异步调度与 EWMA 计算预算。请求跨度在现有 AoI、计算和复用预算之外，再包含一个 10 ms 控制周期。
4. 回传最新已收到的 `last_observation`；每次新预测完成时另回传中位轨迹，HOCD 缓存两者。原 `stream-receive --stdout` 只输出标量，轨迹回传是这里新增的接口。

本机端口固定为：5004 接收 HOCD 原始转向，5005 沿用现有源与接收协议，5006 向 HOCD 返回观测和轨迹。消息带试验号、递增序号和消息类型，HOCD 每周期取最新一份；单目标轨迹限制在现有时域内，使用一个 UDP 报文即可。

轨迹消息保留：会话/试验号、forecast_id、固定源锚点 `s_f`、锚点转向值、25,000 μs 步长、中位预测数组、锚点映射到接收时钟的 `t_f`、映射速率 `r` 及有效状态。直接用已有 `ClockMapping.age(s_f,t_snapshot)` 返回的点估计计算映射点，保留代码中的区间投影：

$$
t_f=t_{snapshot}-age.estimate\_us.
$$

同机进程共享操作系统单调时钟，HOCD 直接使用该接收时钟坐标。沿用已有同步估计；映射更新时回传更新后的映射点和状态。旧轨迹始终绑定自己的 `s_f`，不能因新包到达而重锚。

### 每 10 ms 选值，再做 PPO 和融合

不能把每 25 ms 发布的一个预测标量持用多个控制 tick。HOCD 每次得到 frame f 的状态后，按已安排的下一物理步 f+1 的墙钟时间 `t_use` 重选轨迹：

$$
\delta=(t_{use}-t_f)/r,\qquad
\hat u_h=interp\bigl(\delta,[0,25000,\ldots,H\cdot25000],[u(s_f),median_1,\ldots,median_H]\bigr).
$$

只在轨迹范围内线性插值，不外推。计划生效时间在控制前确定，不能事后读取真实生效时间再选值。控制侧简单检查会话、有限值、有效状态和覆盖范围；不满足就用已收到的 Hold。记录原目标帧与实际施加帧，迟到也保留。

同一份 `u_h_est` 同时替换 PPO 状态中的真人转向和最终混合中的真人转向：

```text
PPO state = [driver_state, e_d, e_fi, a_y, u_h_est, u_a]
u_mix = lambda * u_a + (1-lambda) * u_h_est
```

T248P 每周期只采一次：前述方向盘适配器得到的转向值送入 5004 和原始日志，`SharedControl.control` 中原来直接取得的真人转向改从 5006 缓存读取。即把“采方向盘”和“向共享控制提供转向”拆开，接桥后不在 `driver_control.control` 内再次采样。Hold 条件也走同一条延迟链路，避免共享控制仍直接读到未延迟的方向盘值。

`e_fi` 实际为航向角差的正弦；输入顺序及原权重保持一致。原 `pre_ts=0.2 s` 是车辆状态预瞄，两组均保留，不再把 0.2 秒加进预测目标。最终仍由 HOCD 唯一调用 `apply_control`。

## 从自己跑通到首轮真人采集

按“无额外延迟开一趟 → 300 ms Hold 开一趟 → 300 ms TiRex 开一趟”顺序自己跑通。首次 CUDA 编译和模型预热先完成；每趟开始时积累约 7 秒历史，等第一条可用预测出现再开始计分即可。显示不等待模型，预测计算不阻塞 10 ms 控制循环。

首轮只保留两个条件，使用同一条路线、同一速度目标、同一 PPO 和输出转换：

| 条件 | 真人转向如何进入共享控制 |
|---|---|
| Hold | 最后收到的、已延迟 300 ms 的转向值 |
| TiRex | 相同延迟历史，由当前 TiRex 预测，每个 10 ms 控制 tick 重选轨迹 |

请 3–5 名真人参与，每种条件各开 2 趟，每趟约 60–90 秒。条件顺序交替或随机，先练习再采集，中间可休息。首轮用于获得可分析数据和判断效果方向。

每趟在 `/home/patton/cooperative_driving/runs` 保存一个配置 JSON 和一个 CSV 即可。JSON 记录参与者匿名编号、条件、延迟、路线、T248P 映射和版本；CSV 每 tick 记录墙钟时间、CARLA frame/仿真时间、原始转向、最新收到转向及其源时间/AoI、实际选中预测、forecast_id、推理耗时、计划目标时间、车辆位置、自动转向、混合转向及碰撞。原始真人转向只供评分，不能旁路进入延迟条件控制器。两组使用相同采样流程，暂停和未完成也记入结果。

先看四类结果：

- **横向 RMSE**：用车辆实际位姿对预先保存的同一参考路线计算。`ctrl.lat.e_d` 含 0.2 秒预瞄，只作诊断。
- **碰撞与完成情况**：每趟是否碰撞、是否完成、退出原因。
- **真人转向估计误差**：在实际使用时刻，将 Hold/TiRex 估计与记录的原始真人转向比较，报告 MAE 或 RMSE；真值按采样保持定义。
- **平均延迟**：收到转向的实际年龄、推理平均耗时、预测目标时间与实际施加时间差分别列出，不把 300 ms 当作全部延迟。

先按每人每条件两趟列结果，再汇总均值和离散程度。TiRex 无覆盖而回到 Hold 的 tick 仍计入结果，另记比例，避免只挑预测成功片段。

## 真人系统跑通后，再用 LAIA 辅助回放

辅助集选 [LAIA](https://www.cloningdcb.org/)，只取 CAN 和 steer，不先下载图像。论文为 CARLA 0.9.14、名义 25 Hz、44 名驾驶员；网站旧版介绍仍有 40 人，按实际版本记录。先取已核实的小包：

```bash
mkdir -p /home/patton/cooperative_driving/data/LAIA
cd /home/patton/cooperative_driving/data/LAIA
curl -L --fail -o D019_Town04_R03_CB.tar.gz https://datasets.cvc.uab.es/ted/D019_Town04_R03_CB.tar.gz
tar -xzf D019_Town04_R03_CB.tar.gz
```

随下载保存官网对应版本条款即可。

该包 CAN JSON 没有显式 timestamp，文件名索引多数相差 39/40/41，也有约 40040 的大缺口。先核对索引时间单位，再按名义 40 ms 导入通用 `SignalView`，遇大缺口切段。用 160 点历史约覆盖 6.4 秒；趋势窗先保留 32 点并记录其时长为 1.28 秒。steer 直接按原单位做单目标预测，不未经标定施加到 HOCD，不用单位未核实的 speed/calc_speed。

回放仍只比较同一轨迹、同一 300 ms 延迟下 Hold 与当前 TiRex 的转向误差。LAIA 补充真人动作证据，不阻塞前面的真人系统，也不替代 CARLA 0.9.16 闭环结果。

## 首轮以后再扩展

首轮结果可用后，再增加 175 ms 延迟、speed 历史协变量、更多驾驶员、换道、抖动/丢包、精确方向盘物理标定及规划器完善。跨机网络、全面故障压力、PPO 重训和更完整统计设计均放到此阶段。

实现参考：[HOCD 固定源码](https://github.com/i-Qin/HOCD/tree/49b0189595f3dc525f4bc7194671742e203f7839)、[HOCD 论文](https://arxiv.org/abs/2512.23220)、[CARLA 0.9.16 发布](https://carla.org/2025/09/16/release-0.9.16/)、[同步时间步](https://github.com/carla-simulator/carla/blob/0.9.16/Docs/adv_synchrony_timestep.md)、[LAIA 论文](https://arxiv.org/html/2607.25570v1)。本地接口见 [SignalSource](../../代码/DelayCompensation/src/mytirex/streaming/source.py)、[StreamingRuntime](../../代码/DelayCompensation/src/mytirex/streaming/runtime.py)、[轨迹插值](../../代码/DelayCompensation/src/mytirex/signals.py)和[时钟映射](../../代码/DelayCompensation/src/mytirex/timing/clock.py)。

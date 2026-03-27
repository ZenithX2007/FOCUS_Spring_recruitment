# INMP441 麦克风 + 离线语音 + FFT 低音检测 + LED（ESP32-S3）

本工程在 ESP32-S3 上通过 **I2S** 采集 INMP441 音频，一路数据同时送入 **乐鑫 ESP-SR**（AFE + WakeNet + MultiNet）做离线唤醒与中文命令词识别，并行做 **esp_dsp FFT** 低频能量检测（模拟「拍桌子」冲击），并驱动 **LEDC PWM** 控制 LED 亮度与闪烁。

## 📋 项目概述

### 🎯 核心功能

1. **离线语音识别**（ESP-SR）
   - 唤醒词：**「Hi,乐鑫」**（WakeNet9）
   - 中文命令词识别（MultiNet6）：
     - `开灯` → LED 常亮
     - `熄灯` → LED 关闭
     - `闪烁` → LED PWM 闪烁
     - `暗度调低一档` → 亮度降档

2. **FFT 低频能量检测**（模拟"拍桌子"效果）
   - 对音频末尾 128 点做 FFT 变换
   - 检测低频带（bin 1-4）能量
   - 自适应噪声底 + 相对阈值判断
   - 触发后执行"调暗一档"操作

3. **LED 状态机控制**
   - 4 档亮度调节（0, 400, 1600, 8191 duty）
   - 支持常亮、关闭、闪烁三种模式
   - 50ms 定时刷新 PWM 占空比

### 🏗️ 系统架构

```
INMP441 麦克风 (I2S) → BCLK=41, WS=40, DIN=42, 16kHz 单声道
    ↓
音频任务 (audio_task)
    ├─→ FFT 分支 (128 点 FFT + 汉宁窗 + 低频能量检测 + 冲击判断)
    └─→ ESP-SR AFE 分支 (音频前端 + WakeNet 唤醒 + MultiNet 识别)
            ↓
        LED 控制指令 → LEDC PWM (GPIO48, 5kHz, 13-bit)
```

## 运行前准备

| 项目 | 说明 |
|------|------|
| 芯片 | ESP32-S3（工程目标 `idf.py set-target esp32s3`） |
| Flash | 建议 **≥ 4MB**；分区表 `partitions.csv` 中已预留 `model` 分区存放语音模型 |
| ESP-IDF | 与 `main/idf_component.yml` 中 esp-sr / esp-dsp 版本兼容的 IDF（如 5.x） |
| 麦克风 | INMP441：BCLK、WS、DIN 与 `components/iis_audio/iis_audio.c` 中 GPIO 一致（默认 BCLK=41，WS=40，DIN=42），16 kHz 单声道 |
| LED | 默认 GPIO48 作为 LEDC PWM 输出（可在 `main/led_fsm.c` 中修改 `LED_GPIO_NUM`） |

若你之前用过 **2MB Flash** 或默认单分区表的 `sdkconfig`，请执行一次 **`idf.py fullclean`**，再按下方步骤重新配置，使 `sdkconfig.defaults` 中的 **4MB Flash + 自定义分区表** 生效。

## 🔧 技术细节

### I2S 音频采集
- **采样率**: 16kHz
- **数据格式**: 32-bit I2S → 转换为 16-bit
- **GPIO 配置**:
  - BCLK: 41
  - WS: 40
  - DIN: 42

### FFT 低音检测参数
```c
FFT_SIZE = 128              // FFT 点数
LOW_FREQ_BIN_MAX = 4        // 低频范围：bin 1~4
BASS_HIT_FRAMES = 3         // 连续帧确认
BASS_COOLDOWN_US = 400ms    // 冷却时间
BASS_RELATIVE_RATIO = 2.5   // 相对噪声底倍数
BASS_MIN_ABS = 35.0         // 最小绝对阈值
BASS_FLOOR_EMA_ALPHA = 0.97 // 噪声底 EMA 系数
```

### LED 亮度档位
| 档位 | Duty 值 | 亮度 |
|------|---------|------|
| 0 | 0 | 关闭 |
| 1 | 400 | 低亮 |
| 2 | 1600 | 中亮 |
| 3 | 8191 | 高亮 |

### 分区表布局
```
nvs:       0x9000,  24KB
phy_init:  0xf000,  4KB
factory:   0x10000, 1.5MB (代码)
model:     0x190000,2.4MB (语音模型)
```

## 编译与烧录

1. 进入工程目录，设置目标并编译：

   ```bash
   idf.py set-target esp32s3
   idf.py build
   ```

2. **`sdkconfig.defaults`** 中已启用 **WakeNet `wn9s_hilexin`（Hi,乐鑫）** 与 **MultiNet `mn6_cn`**。`managed_components/espressif__esp-sr` 会在开启自定义分区表时，用 `movemodel.py` 根据 `sdkconfig` 打包 **`srmodels.bin`**，并挂接到 **`idf.py flash`**，将模型写入名为 **`model`** 的分区。

3. 连接串口后烧录并监视日志：

   ```bash
   idf.py -p PORT flash monitor
   ```

4. 烧录成功后，串口会打印已加载的模型列表、AFE `feed_chunk` 等信息；若提示找不到 `model` 分区或未烧录模型，请确认分区表与 Flash 容量配置正确，并重新全量烧录。

## 上电后如何工作

1. **音频任务** 从 I2S 连续读取一整帧 **AFE 所需采样数**（`feed_chunk`），**同一缓冲** 的末尾 **128 点** 用于 FFT，整帧送入 **AFE**。
2. **WakeNet** 在 AFE 输出流上检测唤醒词 **「Hi,乐鑫」**（对应模型 `wn9s_hilexin`）。
3. 唤醒后进入 **MultiNet** 识别窗口，识别下列 **中文命令**（与 `main/commands_cn.txt` 及代码中 `command_id` 一致）：

   | command_id | 语音 | 效果 |
   |------------|------|------|
   | 0 | 开灯 | LED 常亮（当前亮度档位） |
   | 1 | 熄灯 | LED 关闭 |
   | 2 | 闪烁 | LED 以 PWM 闪烁 |
   | 3 | 暗度调低一档 | 亮度降一档（若当前为常亮则立即生效） |

4. **FFT 低频分支** 对低频带能量做相对阈值 + 连续帧确认 + 冷却，判定为「冲击」时执行 **与「暗度调低一档」相同的调光**（`ledFsmDimStepDown`）。
5. 在 **唤醒/命令识别进行中**，会自动提高低频触发门槛，减轻语音爆破音误触调光。

## 新功能操作说明（语音 / 拍桌 / LED）

### 语音控制

- 先清晰说出唤醒词 **「Hi,乐鑫」**，再在提示窗口内说出 **命令词**（如「开灯」「熄灯」「闪烁」「暗度调低一档」）。
- 识别结果会在串口打印 `命令 id=… prob=…`，便于确认是否触发。

### 拍桌 / 低音冲击（FFT）

- 在安静环境下用力拍桌或类似低频冲击，若能量超过自适应噪声底与相对阈值，串口会打印 **「低频冲击触发」**，并 **调暗一档**。
- 若与唤醒词冲突或环境嘈杂，可先以「语音互斥」逻辑降低误触；仍不理想时可在 `main/main.c` 顶部调整 `BASS_*` 常量（相对阈值、连续帧数、冷却时间等）。

### LED 与硬件

- 默认使用 **GPIO48** 作为 **LEDC PWM** 输出；若与你的开发板不一致，请修改 `main/led_fsm.c` 中的 **`LED_GPIO_NUM`**。
- **常亮** 下有多档亮度；**「暗度调低一档」** 与 **拍桌触发** 共用同一套亮度递减逻辑。

## 📁 目录结构

```
├── partitions.csv          # 自定义分区（含 model）
├── sdkconfig.defaults      # ESP-SR 模型与 Flash/分区默认选项
├── main/
│   ├── main.c              # 音频、AFE、MultiNet、FFT、与 LED 策略
│   ├── led_fsm.c / .h      # LEDC 亮度与闪烁状态机
│   └── commands_cn.txt     # 中文命令词说明（与代码注册一致）
├── components/iis_audio/   # I2S 读入 int16 单声道
│   ├── iis_audio.c
│   └── iis_audio.h
└── managed_components/     # esp-sr、esp-dsp（组件管理器拉取）
    ├── espressif__esp-sr/  # 语音识别库
    └── espressif__esp-dsp/ # DSP 库（FFT）


    ┌─────────────────────────────────────────────┐
│           INMP441 麦克风 (I2S)              │
│  BCLK=41, WS=40, DIN=42, 16kHz 单声道      │
└─────────────────┬───────────────────────────┘
                  │ int16_t 音频流
                  ▼
┌─────────────────────────────────────────────┐
│        音频任务 (audio_task)                │
│  ┌───────────────────────────────────────┐  │
│  │  I2S 读取 feed_chunk (512 样本)        │  │
│  └──────────────┬────────────────────────┘  │
│                 │                           │
│         ┌───────┴────────┐                  │
│         ▼                ▼                  │
│  ┌─────────────┐  ┌──────────────────┐     │
│  │ FFT 分支    │  │ ESP-SR AFE 分支  │     │
│  │ - 汉宁窗    │  │ - 音频前端处理   │     │
│  │ - 128 点 FFT │  │ - WakeNet 唤醒   │     │
│  │ - 低频能量  │  │ - MultiNet 识别  │     │
│  │ - 冲击检测  │  │ - 命令词匹配     │     │
│  └──────┬──────┘  └────────┬─────────┘     │
│         │                  │               │
│         ▼                  ▼               │
│  调光指令            LED 控制指令           │
└─────────┬──────────────┬───────────────────┘
          │              │
          ▼              ▼
   ┌──────────┐   ┌──────────┐
   │ LEDC PWM │   │ LED GPIO │
   │ 5kHz     │   │ GPIO48   │
   └──────────┘   └──────────┘
```

## 🛠️ 依赖组件

- [`espressif__esp-sr`](https://github.com/espressif/esp-sr): 离线语音识别（WakeNet + MultiNet）
- [`espressif__esp-dsp`](https://github.com/espressif/esp-dsp): 数字信号处理库（FFT）
- [`iis_audio`](components/iis_audio): 自定义 I2S 音频采集组件

## ⚡ 性能特点

✅ **双路并行处理**: 同一段音频同时送 FFT 和 AFE  
✅ **内存优化**: AFE 使用内部 SRAM (`MALLOC_CAP_INTERNAL`)  
✅ **实时性**: 音频任务优先级 5，使用 `taskYIELD()` 让出 CPU  
✅ **抗误触**: 语音识别时 FFT 触发门槛提升 4 倍  
✅ **自适应**: FFT 噪声底自动学习环境背景音  
✅ **模块化**: I2S、LED FSM 独立封装  

## 常见问题

### 1. 提示「compile_commands.json 中未找到 main.c」或 IntelliSense 报错

说明当前 **`build/compile_commands.json` 与工程路径不一致**（例如工程从别的电脑拷贝过来，`compile_commands` 里仍是旧机器的绝对路径）。

**处理：**

1. 在工程根目录执行 **`idf.py fullclean`**，然后 **`idf.py build`**，用本机路径重新生成 `build/compile_commands.json`。
2. 确认 **ESP-IDF 插件** 使用的构建目录为当前工程下的 `build`（本仓库已在 `.vscode/settings.json` 中设置 `idf.buildPath` 为 `${workspaceFolder}/build`）。
3. 重新加载窗口：命令面板执行 **Developer: Reload Window**，或重启 VS Code / Cursor。

若从未成功编译过，需先完整 **`idf.py set-target esp32s3`** 再 **`idf.py build`**，否则不会有有效的 `compile_commands.json`。

### 2. CMake：`include could not find ... /tools/cmake/project.cmake`

表示 **`IDF_PATH` 环境变量未设置**（常见于在未激活 ESP-IDF 的环境下直接点「配置」或运行 `cmake`）。

**处理：** 使用 **ESP-IDF 自带终端** 或先执行 **`export.ps1` / `export.bat`**，再 **`idf.py build`**；或在系统里把 **`IDF_PATH`** 设为本地 **esp-idf 根目录**（与 `.vscode/settings.json` 里 `idf.espIdfPathWin` 一致即可）。

### 3. 真实编译失败（gcc / ld 报错）

请把 **终端里第一条 `error:` 那几行** 复制出来排查。常见项：

- **Flash / 分区**：`sdkconfig` 仍为 2MB 或旧分区表 → 执行 `fullclean` 后重新配置，保证与 `sdkconfig.defaults`、`partitions.csv` 一致。
- **模型分区**：`model` 分区过小 → 看编译时 `movemodel.py` 打印的 **Recommended model partition size**，必要时加大 `partitions.csv` 中 `model` 的 Size。
- **组件未找到**：确认已执行 **`idf.py reconfigure`** 或完整 build，且 `managed_components` 中 `espressif__esp-sr`、`espressif__esp-dsp` 存在。

## 参考

- [ESP-SR 文档](https://docs.espressif.com/projects/esp-sr/en/latest/esp32s3/index.html)
- [ESP-IDF 分区表](https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/partition-tables.html)
- [ESP-DSP 文档](https://docs.espressif.com/projects/esp-dsp/en/latest/)

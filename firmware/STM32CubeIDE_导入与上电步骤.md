# STM32CubeIDE 使用与安全切换

本目录已按 STM32CubeIDE 的 GCC 工作流准备：`FOC.ioc` 的 Toolchain/IDE 已设为 `STM32CubeIDE`，并保留 SWD（PA13/PA14）和 HSE 8 MHz 配置。

## 今天：虚拟轨迹验证（不驱动功率级）

`Bsp/Inc/motor_config.h` 中的以下配置是当前默认值：

```c
#define MOTOR_POWER_STAGE_ENABLED   0U
#define MOTOR_AUTORUN_DEMO          1U
```

在该配置下，固件每毫秒执行一次“正向 180 度/3 秒、停 1 秒、反向 180 度/3 秒、停 1 秒”的目标轨迹；但 PB1 始终为低，TIM3 的 CH1/CH2/CH3 也不会被启动或更新，SimpleFOCmini 的 MOS 功率桥不会被使能。

1. 打开 STM32CubeIDE。
2. 选择 **File > New > STM32 Project from an Existing STM32CubeMX Configuration File (.ioc)**。
3. 选择本目录下的 `FOC.ioc`，项目名保持 `FOC`；项目位置选择 `PicoFOC-main/firmware`，并取消创建子目录（项目根目录必须就是现有 `firmware`）。
4. 第一次弹出代码生成提示时选择 **Generate Code**。如果 CubeIDE 提示将删除已有文件，取消操作；保留现有 `App`、`Bsp`、`Drv`、`Lib`、`Core` 与 `Drivers` 目录。
5. 在项目属性的 **C/C++ General > Paths and Symbols** 中，把 `App/Src`、`Bsp/Src`、`Drv/Src`、`Lib/Src` 加为源码目录；把相应的 `Inc` 目录加为 include paths。保留 `Core/Src` 与 `Drivers` 的默认路径。
6. 编译、下载。若只接 ST-LINK 供电，程序可以运行虚拟时序和读取 AS5600；电机不应施力或转动。

CubeIDE/CubeMX 会保留 `USER CODE BEGIN/END` 区域；不要在重新生成代码时覆盖 `App`、`Bsp`、`Drv`、`Lib` 下的文件。

## 明天：接 12 V 后启用真实低电压动作

上电前先断开 12 V，确认三相线、AS5600、PB1、地线都无松动。确保电源极性正确，电源限流先设置为 **0.25 A**。

仅将 `motor_config.h` 中这一行改为：

```c
#define MOTOR_POWER_STAGE_ENABLED   1U
```

重新编译下载后，固件会先以 `0.5 V` d 轴电压对齐约 1.5 秒并保存编码器零点，再开始 180 度/3 秒往返。位置/速度环交轴电压也限制为 `0.5 V`；按图片中的约 `2.55 ohm` 相电阻估算，静止相电流约 `0.20 A`。这是保守起始限制，不能替代电源限流。

成功标准：对齐阶段电机只轻微定位；随后每 3 秒平稳转过约 180 度，停 1 秒后反向；电源电流不持续超过 0.3 A，绕组和驱动板在 1 分钟内不明显发热。任何抖动、尖叫、堵转或电流达到限流时，立即断开 12 V，并将 `MOTOR_POWER_STAGE_ENABLED` 改回 `0U`。

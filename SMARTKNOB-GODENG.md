# 咯噔版本

这是 2026-11-14 的 SmartKnob 连续卡点试验版本，基于当前 V2 固件源码。

- 控制曲线：整圈连续正弦型虚拟卡点，避免卡点捕获与滑动牵引之间的力矩跳变。
- 卡点间距：30°；最大 `Uq`：0.80 V；单次最长通电：30 s。
- 基础滑动阻尼：0.002 V/rpm；卡点附近附加动态阻尼：0.012 V/rpm，按卡点邻域平滑增加，针对大惯性越过卡点后的持续抖动。
- AS5600 健康失败立即关闭 PB1/PWM。
- 编程复位不会自动驱动；备份寄存器布防后，下一次手动 Reset 才运行一次。
- S2E 校准记录保持不变：`F0C0F0C0 534B0001 44435640 0000FF07 B2CB3737`。

## v6 动态阻尼归档

- 构建产物：`v2/SmartKnob-torque-v6-damped-2026-11-14/FOC-SmartKnob-torque-v6-damped.bin`
- SHA-256：`83C6DD251A8E383B3CEDCC0198865D9024C34213337E72F2D5B1DFB001E75C51`
- CSV 归档：`逻辑分析仪软件/数据/SmartKnob-v5-2026-11-14.csv`，SHA-256：`29882844A6C2563C4C481B3BB0F8214EFD7E6DA4ACE888260DA81E3B8E508989`
- 定量观察：AS5600 约 200 Hz；稳定段 10--14 s 在卡点附近约 ±4.5° 摆动，后续衰减至约 ±2.7°，判断为欠阻尼而非角度采样噪声。

当前可复现构建入口：`firmware/Makefile`，工具链为 STM32CubeIDE GNU Arm Embedded。真实下载必须使用 `mode=UR reset=HWrst freq=4000`；HOTPLUG 仅用于只读验证。


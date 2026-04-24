# 三轮智能车（无舵机）统一整合工程

这是基于你当前上传的 **LS2K0300 精简移植版** 继续整理出的完整工程版本，
目标是把 **图像算法、双轮差速控制、识别串口、逐飞助手图传、IPS200 显示** 统一到一套注释清晰的工程里。

## 工程定位

这套工程适用于：

- 三轮智能车
- 无舵机结构
- 左右驱动轮差速转向
- 第三轮为随动轮/万向轮
- 以图像循迹为主，识别串口为辅

## 和 RunACCM2025 的关系

RunACCM2025 原仓库是完整竞赛车架，目录中包含 `capture / include / param / thread / track`，
并在 `track` 下继续细分为 `basic / imgprocess / special / standard`，同时还包含桥、障碍、充电区、临停区等比赛特定元素处理。  
本工程并没有把这些强比赛绑定的逻辑全部硬搬，而是提炼出更适合当前车型调试的基础主链版本：

- 相机采图
- 灰度循迹
- 模糊 PD 差速
- 左右轮速度 PID
- 电机输出
- 编码器反馈
- 串口识别接收
- IPS200 显示
- 逐飞助手图传

## 数据流总览

```text
UVC 摄像头
    ↓
灰度图 frame.gray
    ↓
line_track_process()
    ↓
track.error / track.error_filtered
    ↓
fuzzy_pd_update()
    ↓
左右轮目标速度 left_target_rps / right_target_rps
    ↓
pid_update()
    ↓
MotorDriver8701::set_cmd()
    ↓
DRV8701 双电机输出
```

## 当前目录说明

- `app_config.hpp`：全部宏参数，建议只在这里调参
- `app_types.hpp`：公共结构体与基础工具函数
- `uvc_camera.*`：UVC 摄像头采集封装
- `vision_uart.*`：识别串口协议解析
- `line_track.*`：多扫描线循迹算法
- `control_utils.*`：PID 与模糊 PD
- `seekfree_hal.*`：逐飞硬件适配层（DRV8701 + IPS200）
- `main.cpp`：统一调度主循环
- `OPEN_SOURCE_INTEGRATION.md`：本次整合思路说明

## 这次补强的地方

相比你上传的版本，这次重点补了 4 件事：

1. **把散落在 `main.cpp` 的 TCP / 图传相关宏收回 `app_config.hpp`**
2. **修正 `uvc_camera.cpp` 中设备路径使用不统一的问题**
3. **把 IPS200 状态显示真正接回主循环**
4. **保持变量、宏和函数注释完整，方便后续继续扩展**

## 你最常改的地方

### 1) 硬件与速度参数
优先修改 `app_config.hpp`：

- `MOTOR_BASE_SPEED`
- `MOTOR_TURN_SPEED_LIMIT`
- `SPEED_PID_*`
- `STEER_PD_*`
- `ENCODER_COUNTS_PER_WHEEL_REV`

### 2) 图像循迹参数
也是优先改 `app_config.hpp`：

- `TRACK_ROI_Y0 / TRACK_ROI_Y1`
- `TRACK_LOCAL_SEARCH_MARGIN`
- `TRACK_MIN_LANE_WIDTH / TRACK_MAX_LANE_WIDTH`
- `TRACK_SLOWDOWN_ERROR / TRACK_BRAKE_ERROR`

### 3) 识别结果触发动作
当前默认只是接收和显示识别结果。
如果你后续要加“看到某类别就减速/停车/切状态机”，建议直接在 `main.cpp` 里扩展决策函数。

## 编译说明

这个工程默认假设你已经有逐飞 LS2K0300 / SDK 环境：

- `zf_common_headfile.hpp`
- `zf_device_uvc`
- `zf_driver_encoder`
- `zf_driver_pwm`
- `zf_driver_gpio`
- `zf_device_ips200`
- 以及对应底层库

基础步骤：

```bash
mkdir build
cd build
cmake ..
make -j
```

## 注意事项

1. `CMakeLists.txt` 当前只编译本工程自己的源码；
2. 如果你的 `zf_driver_encoder` 不是预编译库，而是源码文件，请把它的 `.cpp` 补到 CMake 里；
3. 如果你的 UVC 设备路径不是 `/dev/video0`，修改 `CAM_DEV_PATH`；
4. 如果编码器方向反了，修改 `ENCODER_LEFT_SIGN` / `ENCODER_RIGHT_SIGN`；
5. 如果左右电机接线相反，优先检查 `SMARTCAR_DIR_LEFT / RIGHT` 绑定和电机方向级性。

## 建议调试顺序

1. 先不跑高速，只确认左右轮方向正确；
2. 确认编码器反馈符号正确；
3. 确认图像误差方向正确；
4. 先调差速控制，再调速度环；
5. 最后再恢复识别触发动作和元素区状态机。

# 三轮智能车（无舵机）十字 + 环岛工程

这是在上一版“三轮无舵机统一整合工程”基础上继续补强的版本。

这次重点不是再改控制框架，而是把你已经上传过的 `image_track` 思路，真正整合进当前差速车工程里，让图像侧除了基础循迹之外，还能处理：

- 十字识别与补线
- 左环岛识别与补线
- 右环岛识别与补线

## 当前主链

```text
UVC 摄像头
    ↓
灰度图 frame.gray
    ↓
自适应阈值二值化（暗赛道转白）
    ↓
底部最长白列找种子
    ↓
左右边界搜索
    ↓
十字 / 环岛 判定与状态机补线
    ↓
中心线输出
    ↓
模糊 PD 差速
    ↓
左右轮速度 PID
    ↓
DRV8701 双电机输出
```

## 相比上一版新增的能力

1. `line_track.cpp` 不再只做基础多扫描线误差提取；
2. 增加了十字、左环、右环的图像统计判定；
3. 增加了元素区状态保持与退出逻辑；
4. 元素区下会自动补线，控制仍然走统一差速主链；
5. 屏幕会直接显示 `NORMAL / CROSS / RING_L / RING_R`。

## 当前目录说明

- `app_config.hpp`：全部参数入口，含十字 / 环岛阈值
- `app_types.hpp`：数据结构，已加入 element / state
- `line_track.*`：图像循迹 + 十字 / 环岛识别 + 补线
- `control_utils.*`：PID 与模糊 PD
- `seekfree_hal.*`：DRV8701 + IPS200
- `uvc_camera.*`：UVC 采图
- `vision_uart.*`：识别串口解析
- `main.cpp`：统一调度主循环

## 你最应该先调的参数

### 基础图像
- `TRACK_ROI_Y0 / TRACK_ROI_Y1`
- `TRACK_LOCAL_SEARCH_MARGIN`
- `TRACK_MIN_LANE_WIDTH / TRACK_MAX_LANE_WIDTH`
- `TRACK_BINARY_MIN / TRACK_BINARY_MAX`

### 十字 / 环岛
- `TRACK_CROSS_BOTH_LOST_MID_MIN`
- `TRACK_CROSS_WIDTH_DELTA_MIN`
- `TRACK_RING_SIDE_LOST_MID_MIN`
- `TRACK_RING_OTHER_SIDE_LOST_MID_MAX`
- `TRACK_CROSS_HOLD_FRAMES`
- `TRACK_RING_HOLD_FRAMES`

### 控制
- `MOTOR_BASE_SPEED`
- `TRACK_CROSS_SPEED_SCALE`
- `TRACK_RING_SPEED_SCALE`
- `STEER_PD_*`
- `SPEED_PID_*`

## 现阶段仍未做的事

- 没有把桥、障碍区、临停区、斑马线等其他比赛元素继续接进来
- 没有根据识别串口类别触发任务动作
- 没有专门做“出环二阶段状态机”“环岛方向记忆 + 二次确认”这类更强比赛策略

## 编译

```bash
mkdir build
cd build
cmake ..
make -j
```

> 前提仍然是你本机 / 板端已经有逐飞 LS2K0300 相关 SDK 与底层库。

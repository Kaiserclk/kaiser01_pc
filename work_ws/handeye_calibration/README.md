# 手眼标定程序使用说明

## 📋 功能概述

这是一个完整的手眼标定数据采集和计算程序,基于 `moveit_calibration_plugins` 实现。

**主要功能**:
- ✅ 实时检测标定板(ArUco/Charuco)位姿
- ✅ 自动获取机械臂末端位姿
- ✅ 交互式数据采集(通过 ROS2 服务)
- ✅ 自动计算手眼标定矩阵
- ✅ 支持 Eye-in-Hand 和 Eye-to-Hand 两种模式
- ✅ 标定结果自动保存为 YAML 文件

---

## 🚀 快速开始

### 1. 编译功能包

```bash
cd ~/ros2-humble/work_ws
colcon build --packages-select handeye_calibration
source install/setup.bash
```

### 2. 启动标定程序

**Eye-in-Hand 模式**(相机安装在机械臂末端):
```bash
ros2 launch handeye_calibration handeye_calibration.launch.py
```

**Eye-to-Hand 模式**(相机固定在外部):
```bash
ros2 launch handeye_calibration handeye_calibration.launch.py \
  mount_type:=eye_to_hand
```

### 3. 数据采集流程

#### 步骤 1: 准备标定板
- 打印 ArUco 标定板(程序会自动生成)
- 确保标定板平整放置
- 确保相机能看到标定板

#### 步骤 2: 移动机械臂并采集样本

```bash
# 方法1: 通过命令行调用服务
ros2 service call /handeye_calibration/sample std_srvs/srv/Trigger

# 方法2: 使用 Python 脚本(见下方)
```

**推荐采集流程**:
1. 移动机械臂到位置 1
2. 等待机械臂稳定(2-3秒)
3. 调用 sample 服务
4. 移动到位置 2(改变姿态,不仅是位置)
5. 重复步骤 2-4
6. 至少采集 10 组数据

#### 步骤 3: 执行标定

```bash
# 调用标定服务
ros2 service call /handeye_calibration/calibrate std_srvs/srv/Trigger
```

程序会自动:
- 检查数据量是否足够
- 调用 OpenCV 手眼标定求解器
- 显示标定结果(平移、旋转、变换矩阵)
- 保存结果到 `/tmp/handeye_calibration_result.yaml`

#### 步骤 4: 重置数据(可选)

```bash
# 清除所有采集的样本,重新开始
ros2 service call /handeye_calibration/reset std_srvs/srv/Trigger
```

---

## 🔧 参数配置

### Launch 文件参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `base_frame` | string | `base_link` | 机械臂基座坐标系 |
| `end_effector_frame` | string | `tool0` | 末端执行器坐标系 |
| `camera_frame` | string | `camera_color_optical_frame` | 相机光学坐标系 |
| `target_type` | string | `Aruco` | 标定板类型(Aruco/Charuco) |
| `mount_type` | string | `eye_in_hand` | 安装模式(eye_in_hand/eye_to_hand) |
| `min_samples` | int | `10` | 最小样本数量 |
| `solver_method` | string | `Tsai1989` | 标定算法 |
| `output_file` | string | `/tmp/handeye_calibration_result.yaml` | 输出文件路径 |

### 支持的标定算法

- `Tsai1989` (推荐,默认)
- `Park1994`
- `Horaud1995`
- `Andreff1999`
- `Daniilidis1998`

---

## 📊 标定结果示例

程序会在终端显示:

```
===========================================
Calibration Result:
===========================================
Translation:
  X: 0.012345
  Y: -0.023456
  Z: 0.123456
Rotation (Euler angles - XYZ):
  Roll:  0.012345 rad (0.71 deg)
  Pitch: -0.023456 rad (-1.34 deg)
  Yaw:   0.034567 rad (1.98 deg)
Transformation Matrix:
0.999876 0.012345 -0.009876 0.012345
-0.012234 0.999890 0.008765 -0.023456
0.009987 -0.008654 0.999912 0.123456
0.000000 0.000000 0.000000 1.000000
===========================================
```

并保存到 YAML 文件:

```yaml
calibration_result:
  mount_type: eye_in_hand
  solver_method: Tsai1989
  num_samples: 15
  base_frame: base_link
  end_effector_frame: tool0
  camera_frame: camera_color_optical_frame
  translation:
    x: 0.012345
    y: -0.023456
    z: 0.123456
  rotation_matrix:
    - 0.999876
    - 0.012345
    - ...
  euler_angles:
    roll: 0.012345
    pitch: -0.023456
    yaw: 0.034567
  quaternion:
    w: 0.999912
    x: 0.006173
    y: -0.011728
    z: 0.003456
```

---

## 💡 辅助脚本

### 便捷采样脚本

创建 `sample.sh`:

```bash
#!/bin/bash
echo "调用采样服务..."
ros2 service call /handeye_calibration/sample std_srvs/srv/Trigger
```

使用方法:
```bash
chmod +x sample.sh
# 每次移动到位置后运行
./sample.sh
```

### 一键标定脚本

创建 `calibrate.sh`:

```bash
#!/bin/bash
echo "开始执行手眼标定..."
ros2 service call /handeye_calibration/calibrate std_srvs/srv/Trigger
```

---

## 🎯 提高标定精度的建议

1. **样本数量**: 至少 10 组,推荐 15-30 组
2. **姿态多样性**: 
   - 不仅改变位置,更要改变姿态(旋转)
   - 避免所有位姿在同一平面
   - 覆盖相机视野的不同区域
3. **标定板质量**:
   - 打印要清晰,不要模糊
   - 平整放置,不要弯曲
   - 精确测量实际尺寸
4. **环境光照**:
   - 均匀光照,避免强烈反光
   - 避免阴影遮挡标定板
5. **机械臂稳定性**:
   - 移动后等待 2-3 秒再采样
   - 避免机械臂振动

---

## 🔍 故障排查

### 问题 1: "Target not detected"
**原因**: 相机看不到标定板或标定板参数不正确
**解决**:
- 检查相机画面中标定板是否清晰可见
- 确认标定板参数(标记大小、间距)与实际一致
- 检查相机内参是否已标定

### 问题 2: "TF lookup failed"
**原因**: TF 树不完整或坐标系名称错误
**解决**:
```bash
# 检查 TF 树
ros2 run tf2_tools view_frames
evince frames.pdf

# 检查坐标系是否存在
ros2 topic echo /tf
```

### 问题 3: "Camera info not received"
**原因**: 相机内参话题未发布
**解决**:
```bash
# 检查话题是否存在
ros2 topic list | grep camera_info

# 手动查看话题内容
ros2 topic echo /camera/color/camera_info
```

### 问题 4: 标定结果不准确
**原因**: 样本质量差或数量不足
**解决**:
- 增加样本数量(15-30组)
- 提高样本姿态多样性
- 重新采集数据并重置

---

## 📝 使用流程总结

```
1. 启动程序
   ↓
2. 确保相机能看到标定板
   ↓
3. 移动机械臂到位置 1
   ↓
4. 等待稳定后调用 sample 服务
   ↓
5. 重复步骤 3-4 (至少10次)
   ↓
6. 调用 calibrate 服务
   ↓
7. 查看终端输出的标定结果
   ↓
8. 检查 /tmp/handeye_calibration_result.yaml
```

---

## 🛠️ 集成到 URDF

标定完成后,将结果添加到机器人的 URDF 或 TF 配置中:

```xml
<!-- Eye-in-Hand: 相机在末端 -->
<joint name="camera_joint" type="fixed">
  <parent link="tool0"/>
  <child link="camera_link"/>
  <origin xyz="0.012345 -0.023456 0.123456" 
          rpy="0.012345 -0.023456 0.034567"/>
</joint>
```

---

## 📞 支持

如有问题,请检查:
1. 终端输出的错误信息
2. ROS2 日志: `ros2 node info /handeye_calibrator`
3. TF 树是否完整
4. 相机话题是否正常发布

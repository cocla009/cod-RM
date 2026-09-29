# armor_detector：纯神经网络装甲板识别

当前链路：`image_raw → RobotPilots 0526 / OpenVINO → 四角点 + 颜色 + 编号 → 四点 IPPE PnP → yaw 优化 → Armors`。
灯条提取、灯条配对、PCA 修正、数字 ROI 和 LeNet 分类器已删除；检测失败发布空结果。

部署参考 `sp_vision_25` 的 YOLOv5 输入输出约定。只支持 [RobotPilots 0526](https://github.com/broalantaps/RobotDetectionModel) 这一张量契约；普通 YOLO 检测权重不能直接替换。
模型来源、版本、哈希与使用条件见 [model/README.md](model/README.md)。

## 安装和构建

需要 ROS 2 Humble、OpenCV、Eigen，以及 **OpenVINO Runtime 2024.6 或兼容版本**（含 C API、ONNX frontend、CPU plugin）。
OpenVINO 为外部 SDK，不能只依靠当前 package.xml 的 ROS 依赖自动安装。
以下命令从 `cod_fyt2024_vision` 目录执行，使用 Bash：

```bash
source /opt/ros/humble/setup.bash
python3 -m venv --system-site-packages "$HOME/.venvs/cod-openvino"
"$HOME/.venvs/cod-openvino/bin/pip" install 'openvino==2024.6.0'
export OpenVINO_DIR="$("$HOME/.venvs/cod-openvino/bin/python" -c 'import pathlib, openvino; print(pathlib.Path(openvino.__file__).parent / "cmake")')"
export LD_LIBRARY_PATH="$(dirname "$OpenVINO_DIR")/libs:${LD_LIBRARY_PATH:-}"
python3 rm_auto_aim/armor_detector/model/fetch_model.py
colcon build --packages-up-to armor_detector --cmake-args \
  -DOpenVINO_DIR="$OpenVINO_DIR" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
```

也可使用已有的 OpenVINO SDK，将 `OpenVINO_DIR` 指向其 CMake 目录，并按 SDK 要求设置运行时库路径。
代码通过 OpenVINO C API 调用推理，避免 Python wheel 的 C++ ABI 与 ROS/OpenCV 不一致。
不要把 OpenVINO 的旧 C++ ABI 编译宏加到整个 ROS 工作空间。

已删除的 `DebugLight(s)` / `DebugArmor(s)` 消息、旧头文件和旧模型不会被增量安装自动清理。
从旧安装迁移时，建议使用新的构建/安装目录并重新构建下游包：

```bash
colcon build --build-base build_nn --install-base install_nn \
  --packages-up-to armor_detector armor_solver \
  --cmake-args -DOpenVINO_DIR="$OpenVINO_DIR" -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install_nn/setup.bash
```

在启动节点的终端或自启动服务中也要配置 OpenVINO 运行时库路径。
完整 bringup 的相机、串口等依赖仍按项目主 README 安装。新检测参数位于源码的
`rm_bringup/config/node_params/armor_detector_params.yaml`；使用 bringup 前也要重新安装 `rm_bringup`，
不要沿用旧安装目录内的检测参数。已有其依赖的工作空间可执行 `colcon build --packages-select rm_bringup`。
能量机关包保留原实现及其 OpenVINO C++ SDK 配置；若构建能量机关，请使用与 ROS/OpenCV 的
C++ ABI 一致的完整 OpenVINO SDK，上述 pip 方案仅用于本次装甲板包。

单独启动识别节点（需外部提供图像、标定、TF）：

```bash
ros2 run armor_detector armor_detector_node --ros-args \
  --params-file rm_bringup/config/node_params/armor_detector_params.yaml
```

模型必须在构建安装前下载；模型路径也可通过 `nn.model_path` 指向绝对路径、`file://` 或 `package://`。
启动时若模型缺失、设备不可用、张量形状/类型不匹配，则节点启动失败并给出原因。运行时不会联网下载。

## ROS 接口

| 类型 | 名称 | 内容 |
| --- | --- | --- |
| 订阅 | `image_raw` | 相机图像；队列深度 1，转换为 RGB8 |
| 订阅 | `camera_info` | 与图像同 frame、同分辨率的完整图像针孔标定；支持 plumb_bob/rational_polynomial，不接受未折算内参的 ROI/binning |
| 发布 | `armor_detector/armors` | 原有 `rm_interfaces/msg/Armors`，时间戳和 frame 沿用输入图像，位姿在相机坐标系 |
| 发布 | `armor_detector/marker` | 通过 PnP 筛选后的装甲板；每帧清理旧标记 |
| 发布 | `armor_detector/result_img` | `debug=true` 且有订阅时发布网络检测角点、编号、置信度和帧龄；不代表全部通过 PnP |
| 服务 | `armor_detector/set_mode` | 0 红方自瞄、1 蓝方自瞄；2–5 暂停装甲板识别；非法模式返回失败 |

心跳接口沿用原有实现。`Armors` / `Armor` 消息结构不变，后端跟踪和规划接口不变。
原 `debug_lights`、`debug_armors`、`binary_img`、`number_img` 话题已删除。

没有匹配的标定、TF 查询失败、图像转换或推理失败、帧过期时发送当前帧的空 `Armors`；不复用上一帧检测。
`image_raw` 需提供有效的采集时间戳和光学坐标系 frame；回放 bag 时需配置一致的 ROS 仿真时间。

## 参数

配置文件：[armor_detector_params.yaml](../../rm_bringup/config/node_params/armor_detector_params.yaml)。

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `nn.model_path` | `package://armor_detector/model/rp_0526.onnx` | 0526 ONNX 文件 |
| `nn.device` | `CPU` | 可选 OpenVINO 支持的设备，如 AUTO/GPU；需要对应驱动 |
| `nn.confidence_threshold` | 0.7 | sigmoid(objectness) 阈值，范围 (0,1) |
| `nn.nms_threshold` | 0.4 | 重叠抑制 IoU 阈值，范围 (0,1) |
| `detect_color` | 0 | 启动时敌方颜色；运行时通过 set_mode 服务切换 |
| `max_frame_age` | 0.2 s | 包含推理/解算的最大帧龄；超过后只发空结果 |
| `max_reprojection_error` | 5 px | 四角点重投影 RMSE 上限 |
| `geometry.small_width/height` | 0.135 / 0.055 m | 小装甲板灯条端点之间的尺寸 |
| `geometry.large_width/height` | 0.230 / 0.055 m | 大装甲板灯条端点之间的尺寸 |
| `use_ba` | true | 使用固定俯仰先验进行 yaw 重投影优化 |
| `pnp_solution_selection` | true | 在满足重投影约束的 IPPE 解中按俯仰先验排序 |
| `target_frame` | odom | 姿态先验采用的参考坐标系 |
| `debug` | false | 发布网络检测叠加图 |

`debug` 及两个 `nn.*threshold` 参数可动态修改；其余为启动参数，修改需重启。
例如：`ros2 param set /armor_detector nn.confidence_threshold 0.75`，使用命名空间时替换节点路径。

## 四点与位姿约定

- 输入保持比例缩放至 640 范围，图像放左上角，右侧和底部补黑；RGB /255，NCHW。
- 网络点序是 `左上、左下、右下、右上`；PnP 点序转换为 `左下、左上、右上、右下`。
- 还原角点时使用实际缩放后的宽高，避免整数取整误差。不按屏幕坐标重新排序点。
- 物体系 `x` 垂直板面、`y` 向左、`z` 向上；保留后端使用的姿态轴约定。
- 凹四边形、反向点序、退化/越界角点直接丢弃；不通过裁剪角点伪造完整板面。
- PnP 与 yaw 优化使用同一套物理尺寸。yaw 搜索先去畸变，最终位姿用带畸变投影重新检查。
- 同一敌方颜色的重复框跨编号执行 NMS；灰/紫类别不进入自瞄结果。

编号映射：`G → sentry`，`O → outpost`，`Bs/Bb → base`。按本项目规则，
只有 1 号和基地装甲板使用大板尺寸；包括 `Bs` 和 `Bb` 在内的两个基地标签均按大板处理。
其余编号统一使用小板尺寸，不再根据图像中的宽高比推断大小板。

## 部署前尚需确认

- 按实际装甲板校对 `geometry.*` 和相机标定。新默认尺寸与旧 PCA 链路的补偿尺寸不同。
- 在目标计算机上测量推理耗时与帧龄，检查 `max_frame_age` 是否适合实际帧率。
- 用本机图像评估红蓝识别、编号、远距离角点误差、运动模糊和遮挡；核对基地标签的实际使用场景。
- 当前迁移没有提供本机数据集训练结果或实车精度结论；旧传统检测专用测试已随旧 API 删除。

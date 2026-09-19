# rm_serial_driver

FYT视觉24赛季串口通信模块

## fyt::SerialDriverNode

串口驱动节点

### 发布话题

*  `serial/receive` (`rm_interfaces/msg/SerialReceiveData`) - 下位机发送到上位机的数据
*  `tf` (`geometry_msgs/msg/TransformStamped`) - 云台的tf变换
  
### Subscribed Topics

* `cmd_gimbal` (`rm_interfaces/msg/GimbalCmd`) - 云台控制信息
* `cmd_chassis` (`rm_interfaces/msg/ChassisCmd`) - 底盘控制信息

### 参数

* `target_frame` (string, default: "odom") - 下位机欧拉角的相对坐标系
* `timestamp_offset` (double, default: 0.0) - tf数据的时间戳补偿
* `port_name` (string, default: "/dev/ttyUART") - 串口设备对应的文件名
* `protocol` (string, default: "infantry") - 协议类型
* `enable_data_print` (bool, default: false) - 是否打印串口读出的原始数据

### 云台控制帧兼容约定

位置模式继续使用既有 16 字节帧，字段布局不变。`GimbalCmd.control_mode == 1`
时发送版本化 32 字节 jerk 帧，帧头后的字段为：

* `1`: magic `0xA2`
* `2`: mode（`1` = jerk）
* `3`: fire（`0/1`）
* `4/8`: pitch/yaw，单位 deg，`float32`
* `12/16`: yaw/pitch jerk，单位 deg/s^3，`float32`
* `20`: command dt，单位 s，`float32`
* `24`: command sequence 低 32 位，`uint32`

索引 `0` 为帧头，`30` 为校验字节，`31` 为帧尾。默认、步兵和哨兵协议
均保留旧位置帧；下位机必须识别 magic 和 mode 后才执行 jerk，不能把 32
字节帧按旧 16 字节布局解释。

## fyt::VirtualSerial

仿真串口驱动节点

### 发布话题

*  `serial/receive` (`rm_interfaces/msg/SerialReceiveData`) - 下位机发送到上位机的数据（固定数据）
*  `tf` (`geometry_msgs/msg/TransformStamped`) - 云台的tf变换（固定数据）
  
### 参数

* `pitch` (double, default: 0.0) - 固定的pitch角度 
* `yaw` (double, default: 0.0) - 固定的yaw角度 
* `vision_mode` (int, default: 0) - 视觉模式 

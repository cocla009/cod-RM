### 改动内容
- /rm_auto_aim/armor_detector
1. 使用LeNet-5网络结构(卷积神经网络)进行数字分类，训练过程中加入椒盐噪声
2. 矫正灯条角点以提高pnp解算准确性，由旋转矩形上顶点做角点->主成分分析（PCA）获取灯条对称轴后沿对称轴方向寻找上下两个亮度梯度变化最大的点作灯条角点
3. BA优化求取装甲板朝向角yaw（具体思路可看2023年上海交通大学青工会）
- /rm_auto_aim/armor_solver
1. 增加选板
选板函数为Solver::selectBestArmor,选板逻辑根据目标的偏航角速度target_v_yaw设置角度阈值theta，如果目标的偏航角速度target_v_yaw的绝对值小于min_switching_v_yaw_，将theta设置为0
2. 增加火控
除armor_solver.cpp文件外/rm_utils/src/math/trajectory_compensator.cpp文件也为重点（建议通读math文件夹下文件）
3. 参数调试修改
- solver.max_tracking_v_yaw 最大跟踪速度偏航角
- solver.prediction_delay 预测延迟，影响选板，用于计算时间延迟
- solver.controller_delay 控制器延迟，是否调整位置
- solver.side_angle 侧向角度，计算跳到下一装甲板角度
- solver.min_switching_v_yaw 最小切换速度偏航角，避免两装甲板间频繁转换
- ekf中参数xyz方差分开
- solver.gravity 重力加速度
- solver.resistance 空气阻力，测试
- iteration_times 计算角度迭代次数
4.修改相机驱动
增加相机为Bayer8像素格式时的最优化选项，可在高帧率情况下，得到较好的图像

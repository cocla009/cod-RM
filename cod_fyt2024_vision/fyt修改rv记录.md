### 改动内容
- /rm_auto_aim/armor_detector
1. 使用LeNet-5网络结构(卷积神经网络)进行数字分类，训练过程中加入椒盐噪声
2. 矫正灯条角点以提高pnp解算准确性，由旋转矩形上顶点做角点->主成分分析（PCA）获取灯条对称轴后沿对称轴方向寻找上下两个亮度梯度变化最大的点作灯条角点
3. 使用不依赖初值的全局粗到细重投影搜索求取装甲板朝向角 yaw，移除 G2O/Sophus 依赖
- /rm_auto_aim/armor_solver
1. 增加选板
选板函数为 Solver::selectBestArmor：低速选择最接近视线的装甲板并保持短时锁定，高速根据旋转方向使用 coming/leaving 非对称窗口。
2. 增加火控
最终 yaw/pitch 使用动态几何开火窗口统一判断；中心跟踪不会沿用装甲板分支的旧 fire_advice。
3. 参数调试修改
- solver.max_tracking_v_yaw 最大跟踪速度偏航角
- solver.prediction_delay 预测延迟，影响选板，用于计算时间延迟
- solver.controller_delay 控制器延迟，是否调整位置
- solver.coming_angle / solver.leaving_angle 高速旋转时的选板窗口
- solver.min_switching_v_yaw 最小切换速度偏航角，避免两装甲板间频繁转换
- ekf中参数xyz方差分开
- solver.gravity 重力加速度
- solver.resistance 空气阻力，测试
- iteration_times 计算角度迭代次数
- compensator_type 弹道补偿模型（注意不是旧配置中的 `compenstator_type`）
- fire_margin、min_fire_tolerance、max_fire_tolerance 动态火控窗口
4.修改相机驱动
增加相机为Bayer8像素格式时的最优化选项，可在高帧率情况下，得到较好的图像

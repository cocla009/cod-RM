# ros2_hik_camera

海康威视USB3.0工业摄像头ros包

## 使用方式

```
ros2 launch hik_camera hik_camera.launch.py
```

## 参数

- exposure_time 曝光时间
- gain          伽马值

## 相机像素格式设置
- 36行MV_CC_SetPixelFormat 
```
RGB8:0x02180014     BGR8:0x02180015
Bayer8:0x01080009   
Bayer_RG_10:0x0110000d
Bayer_RG_10_Packed:0x010C0027
Bayer_RG_12:0x01100011  
Bayer_RG_12_Packed:0x010C002B
```
- 40行MV_CC_SetBayerCvtQuality
```
0: 最邻近
1: 均衡
2: 最优化
```





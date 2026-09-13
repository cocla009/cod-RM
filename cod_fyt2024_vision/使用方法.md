当前并为添加导航模块，因此只需要启动该节点即可
``` 
source install/setup.bash
ros2 launch rm_bringup bringup.launch.py
```
上位机给下位机发送，一共16位。    
第0位帧头0xff    
第1位Fire   
2-5位pitch   
6-9位yaw   
10-13位distance 
第14位 为空 
第15位帧尾0x0d

下位机给上位机发送：一共16位。   
第0位帧头0xff    
第1位mode    
2-5位roll    
6-9位pitch    
10-13位yaw   
第14位 为空 
第15位帧尾0x0d
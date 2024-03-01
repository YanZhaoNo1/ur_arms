# Some problems encountered during the learning process

## 2024.2.29
```xml
xacro：
<hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
 <!-- <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface> -->
```

这块要注意！卡了好久没看出来.......

## 2024.3.01
```shell
rostopic pub -1 /urbot/wrist_1_joint_position_controller/command std_msgs/Float64 "data: -0.5" 
```
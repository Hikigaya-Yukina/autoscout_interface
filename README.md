# Autoware 22 scout interface 的简单说明

## 1. 位置&内容

目前计划放在 src/vehicle/external 中

包含：

- scout自带的消息包
- 模型文件包
- 定义的类和基础功能包
- ugv_sdk(通讯包)
- 自己编写的interface（AUTOWARE的消息与ROS2基础消息互相转换的接口）

## 2.编译&使用

暂时没有经过测试

这4～5个包不要和Autoware的源代码部分同步编译。可能会报错。

最后把他们重新放入对应位置，并重新编译。

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select scout_base scout_msgs scout_description ugv_sdk autoscout_interface
```



通讯节点已经彻底放入 vehicle_interface.launch.xml中了

需要另外启动socut_ros2的节点

```bash
$ ros2 launch scout_base scout_base.launch.py
```

ugv_sdk只是单纯被引用的包

# autoPilot
autoPilot

使用说明

使用多个终端

```
roscore  //启动ros
```

```
catkin_make //编译
source devel/setup.bash 
rosrun autoPilot tier1 //运行接收器
```

```
source devel/setup.bash 
rosbag play point_cloud_projection.bag
```

```
rviz rviz //打开rviz，之后add 一个topic
```


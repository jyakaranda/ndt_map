# NDT_MAP

## Introduction

SLAM package using NDT registration library of Autoware with loop-closure detection (odometry based) referenced from lego_loam.

loop-closure enabled map cloud
![loop_closure_enabled_1](./img/loop_closure_enabled_1.png)
loop-closure disabled map cloud
![loop_closure_disabled_1](./img/loop_closure_disabled_1.png)
loop-closure enabled trajectory
![loop_closure_enabled_2](./img/loop_closure_enabled_2.png)
loop-closure disabled trajectory
![loop_closure_disabled_2](./img/loop_closure_disabled_2.png)

## Dependency

- [gtsam](https://github.com/borglab/gtsam/releases)(Georgia Tech Smoothing and Mapping library, 4.0.0-alpha2)
- [ndt_cpu](https://github.com/autowarefoundation/autoware/tree/master/ros/src/computing/perception/localization/lib/ndt_cpu)
- [ndt_gpu](https://github.com/autowarefoundation/autoware/tree/master/ros/src/computing/perception/localization/lib/ndt_gpu)(not used currently)

## Usage

### Input

- Point Cloud(`/lslidar_point_cloud`)
- Odometry(`/odom/imu`)
- Imu(`/imu/data`)
- TF: /base_link -> /laser, /odom -> /base_link(`/tf`)

### Output

- (`/laser_cloud_surround`)

### Run the package

1. Run the launch file:

```shell
roslaunch ndt_map test.launch
```

2. Play existing bag files [test_0515.bag](https://drive.google.com/file/d/1Y6KR9FUQggcyhvGsnkv7zpYQGvc7dQR_/view?usp=sharing):

```shell
rosbag play test_0515.bag --clock
```

## Issues

- thread-safe
- optimize initial guess in point cloud registration
- add other initial guess support(use_odom need to be set true currently)
- error estimation of pitch will cause accumulated error of height estimation
- a little bit shake in pose estimation when playing test_0515.bag (especially in z axis), haven't check it out yet

## TODOs

- save/load NDT map using $(\mu,\Sigma)$ format
- add appearence based loop-closure detection support
- add gpu support (based on modified ndt_gpu lib)
- learning engineering tricks of NDT localization implementation in Apollo

## Reference

1. [autowarefoundation/autoware](https://github.com/autowarefoundation/autoware)
2. [RobustFieldAutonomyLab/LeGO-LOAM](https://github.com/RobustFieldAutonomyLab/LeGO-LOAM)
3. [NDT 公式推导及源码解析（1）](https://blog.csdn.net/u013794793/article/details/89306901)
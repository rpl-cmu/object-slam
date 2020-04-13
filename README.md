# Object SLAM

Object SLAM is an implementation of semantic RGB-D SLAM that employs recognized objects in the scene as landmarks. As a RGBD camera moves through a scenecontaining objects, a Deep Neural Network (DNN) produces instance segmentations of the observed objects. These instances are tracked simultaneously and volumetrically integrated in realtime to represent the scene in a sparse object map.

## TODO

- [x] RGBD odometry test using Open3D
- [x] Transfer and run DNN inference on RGB image in parallel
- [ ] Launch PointRend and C++ program at once, using build script
- [ ] Read dataset in separate thread with buffer similar to ElasticFusion

## Requirements and Dependencies

- C++17 (Compiler: GCC 7+, CMake: 3.15+)
- Open3D : Make sure to compile and install my fork of Open3D
- GTSAM library
- Eigen > 3.3
- Boost 1.65+
- Conan package manager (docopt, fmt, spdlog, cppzmq, protobuf)

## Dataset requirement

The dataset is required to have the following folder structure:
```bash
.
├── camera-intrinsics.json
├── color [723 entries]
├── depth [723 entries]
└── preprocessed [1446 entries]
```




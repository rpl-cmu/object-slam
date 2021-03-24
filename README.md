# Compositional and Scalable Object SLAM

## Introduction
Compositional and Scalable Object SLAM is an implementation of semantic RGB-D SLAM that employs recognizable objects in the scene as landmarks. As a RGBD camera moves through a scene containing objects, a pre-trained Deep Neural Network (DNN) (PointRend) produces instance segmentations of the observed objects. These instances are tracked simultaneously and volumetrically integrated in realtime to represent the scene in a sparse object map.

<center><img src="misc/object-slam.gif" width="480" style="center"></center>

## Requirements and Dependencies

- C++17 (Compiler: GCC 7+, CMake: 3.15+) (std::optional, make_unique, etc)
- Open3D CUDA branch: My fork of CUDA branch preferable
- GTSAM library: (Not yet used)
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





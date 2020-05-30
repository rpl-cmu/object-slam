# Object SLAM

Object SLAM is an implementation of semantic RGB-D SLAM that employs recognized objects in the scene as landmarks. As a RGBD camera moves through a scenecontaining objects, a Deep Neural Network (DNN) produces instance segmentations of the observed objects. These instances are tracked simultaneously and volumetrically integrated in realtime to represent the scene in a sparse object map.

## TODO

- [x] RGBD odometry test using Open3D
- [x] Refactor datareader to read in separate thread
- [x] Transfer and run DNN inference on RGB image in parallel (doesn't work well)
- [x] Create Pipeline abstract class (similar to Kimera (don't forget license)) to streamline async tasks
- [x] Run Segmentation as a separate task properly (at lower rate)
- [x] Implement Frame to SDF Tracking
- [x] Implement coarse to fine / gaussian pyramid tracking (done for single object)
- [ ] Move raycasting into mapping thread
- [ ] Design Mapping thread
- [ ] Obtain APIs from Wei for tracker thread
- [ ] Launch PointRend and C++ program at once, using bash script (low priority)

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




# Object SLAM

Object SLAM is an implementation of semantic RGB-D SLAM that employs recognized objects in the scene as landmarks. As a RGBD camera moves through a scenecontaining objects, MASK RCNN produces instance segmentations of the observed objects. These instances are volumetrically integrated to produce objects, which form a sparse representation of the scene.

Currently an offline pre-processing and reconstruction pipeline is implemented

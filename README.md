# NO ROS version of Traversability Analysis

- reads point clouds from .bin files directly (and also images): no more dependent on rosbags
- loads SVM model and config files
- computes features from 3D point cloud
- predicts traversability
- opencv imshows the pointclou(semantickitti-labeled) and the traversability grid


## Problems
 - no odometry concatenation of pointclouds
    - leads to incorrect predictions by the SVM model (not enough points to predict)

 - moving to projecting branch to project the grid (with the features) over the image
    - target: produce a feature-augmented image tensor to use with semantic segmentation 
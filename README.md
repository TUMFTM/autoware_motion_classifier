# Probabilistic Motion Classifier for Autoware

Accurate motion classification is essential for safe autonomous driving, as false dynamic predictions of static objects can lead to unnecessary planner interventions and reduced ride comfort. Conventional velocity-based filtering often fails in real-world conditions where noisy detections cause “jitter” — small frame-to-frame variations in object position that tracking systems misinterpret as motion.

This repository provides an uncertainty-aware motion classifier for the Autoware autonomous driving stack. The module augments a 3D LiDAR object detector (CenterPoint) with aleatoric uncertainty estimation and applies a two-sample z-test over short observation windows to statistically distinguish true motion from perception jitter.

The implementation is designed for deployment efficiency:

Reuses Autoware’s existing data association from the tracker

Passes uncertainty estimates via standard covariance fields

Adds minimal computational overhead and requires no retraining on large datasets

Empirical evaluation shows parity with velocity thresholding on nuScenes, but a significant reduction in false dynamic predictions and unnecessary stops during real-world test drives — especially in the “jitter band” of ambiguous detections that speed-only rules misclassify.

Paper:

## Usage
Exchange the following two perception modules of Autoware's original softwarestack with the modules found in this repository.

### Lidar CenterPoint
[autoware_lidar_centerpoint](autoware_lidar_centerpoint/README.md)

### Motion Classifier
[autoware_motion_classifier](autoware_motion_classifier/README.md)

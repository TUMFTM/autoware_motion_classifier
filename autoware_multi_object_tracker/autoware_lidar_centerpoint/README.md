# autoware_lidar_centerpoint

## Purpose

autoware_lidar_centerpoint is a package for uncertainty aware detection of dynamic 3D objects. In addition to the bounding box parameters it estimates the heteroscedastic aleatoric uncertainty of x and y center location, velocity in x and y, as well as yaw. It provides these estimates together with the bounding box to the multi object tracker and motion classifier. This module is only compatible with object detectors that estimate the aforementioned uncertainties. It replaces the corresponding autoware_lidar_centerpoint in the original autoware softwarestack

## Inner-workings / Algorithms

In this implementation, CenterPoint [1] uses a PointPillars-based [2] network to inference with TensorRT.

## Inputs / Outputs

### Input

| Name                 | Type                            | Description      |
| -------------------- | ------------------------------- | ---------------- |
| `~/input/pointcloud` | `sensor_msgs::msg::PointCloud2` | input pointcloud |

### Output

| Name                       | Type                                             | Description          |
| -------------------------- | ------------------------------------------------ | -------------------- |
| `~/output/objects`         | `autoware_perception_msgs::msg::DetectedObjects` | detected objects     |
| `debug/cyclic_time_ms`     | `tier4_debug_msgs::msg::Float64Stamped`          | cyclic time (msg)    |
| `debug/processing_time_ms` | `tier4_debug_msgs::msg::Float64Stamped`          | processing time (ms) |

## Parameters

### ML Model Parameters

Note that these parameters are associated with ONNX file, predefined during the training phase. Be careful to change ONNX file as well when changing this parameter. Also, whenever you update the ONNX file, do NOT forget to check these values.

| Name                                   | Type         | Default Value                                    | Description                                                           |
| -------------------------------------- | ------------ | ------------------------------------------------ | --------------------------------------------------------------------- |
| `model_params.class_names`             | list[string] | ["CAR", "TRUCK", "BUS", "BICYCLE", "PEDESTRIAN"] | list of class names for model outputs                                 |
| `model_params.point_feature_size`      | int          | `4`                                              | number of features per point in the point cloud                       |
| `model_params.max_voxel_size`          | int          | `40000`                                          | maximum number of voxels                                              |
| `model_params.point_cloud_range`       | list[double] | [-76.8, -76.8, -4.0, 76.8, 76.8, 6.0]            | detection range [min_x, min_y, min_z, max_x, max_y, max_z] [m]        |
| `model_params.voxel_size`              | list[double] | [0.32, 0.32, 10.0]                               | size of each voxel [x, y, z] [m]                                      |
| `model_params.downsample_factor`       | int          | `1`                                              | downsample factor for coordinates                                     |
| `model_params.encoder_in_feature_size` | int          | `9`                                              | number of input features to the encoder                               |
| `model_params.has_variance`            | bool         | `false`                                          | true if the model outputs pose variance as well as pose for each bbox |
| `model_params.has_twist`               | bool         | `false`                                          | true if the model outputs velocity as well as pose for each bbox      |

### Core Parameters

| Name                                             | Type         | Default Value             | Description                                                   |
| ------------------------------------------------ | ------------ | ------------------------- | ------------------------------------------------------------- |
| `encoder_onnx_path`                              | string       | `""`                      | path to VoxelFeatureEncoder ONNX file                         |
| `encoder_engine_path`                            | string       | `""`                      | path to VoxelFeatureEncoder TensorRT Engine file              |
| `head_onnx_path`                                 | string       | `""`                      | path to DetectionHead ONNX file                               |
| `head_engine_path`                               | string       | `""`                      | path to DetectionHead TensorRT Engine file                    |
| `build_only`                                     | bool         | `false`                   | shutdown the node after TensorRT engine file is built         |
| `trt_precision`                                  | string       | `fp16`                    | TensorRT inference precision: `fp32` or `fp16`                |
| `post_process_params.score_threshold`            | double       | `0.4`                     | detected objects with score less than threshold are ignored   |
| `post_process_params.yaw_norm_thresholds`        | list[double] | [0.3, 0.3, 0.3, 0.3, 0.0] | An array of distance threshold values of norm of yaw [rad].   |
| `post_process_params.iou_nms_search_distance_2d` | double       | -                         | If two objects are farther than the value, NMS isn't applied. |
| `post_process_params.iou_nms_threshold`          | double       | -                         | IoU threshold for the IoU-based Non Maximum Suppression       |
| `post_process_params.has_twist`                  | boolean      | false                     | Indicates whether the model outputs twist value.              |
| `densification_params.world_frame_id`            | string       | `map`                     | the world frame id to fuse multi-frame pointcloud             |
| `densification_params.num_past_frames`           | int          | `1`                       | the number of past frames to fuse with the current frame      |

### The `build_only` option

The `autoware_lidar_centerpoint` node has `build_only` option to build the TensorRT engine file from the ONNX file.
Although it is preferred to move all the ROS parameters in `.param.yaml` file in Autoware Universe, the `build_only` option is not moved to the `.param.yaml` file for now, because it may be used as a flag to execute the build as a pre-task. You can execute with the following command:

```bash
ros2 launch autoware_lidar_centerpoint lidar_centerpoint.launch.xml model_name:=centerpoint_tiny model_path:=/home/autoware/autoware_data/lidar_centerpoint model_param_path:=$(ros2 pkg prefix autoware_lidar_centerpoint --share)/config/centerpoint_tiny.param.yaml build_only:=true
```

## Assumptions / Known limits

- The `object.existence_probability` is stored the value of classification confidence of a DNN, not probability.

## Trained Models

The trained model is available here:


`Centerpoint` was trained in `nuScenes` (~28k lidar frames) [3] for 20 epochs and finetuned on the institute's internal database (~1.4k lidar frames) for 20 epochs.

Model performance:

| Object Class |   AP   |
|---------------|--------|
| car           | 0.871  |
| truck         | 0.397  |
| bus           | 0.375  |
| pedestrian    | 0.427  |
| bicycle       | 0.318  |

## Deploying CenterPoint model to Autoware

#### Convert CenterPoint PyTorch model to ONNX Format

The autoware_lidar_centerpoint implementation requires two ONNX models as input the voxel encoder and the backbone-neck-head of the CenterPoint model. Other aspects of the network,
such as preprocessing operations, are implemented externally. 

#### Create the config file for the custom model

Create a new config file named **centerpoint.param.yaml** under the config file directory of the autoware_lidar_centerpoint node. Sets the parameters of the config file like
point_cloud_range, point_feature_size, voxel_size, etc. according to the training config file.

```yaml
/**:
  ros__parameters:
    # weight files
    encoder_onnx_path: "$(var model_path)/pts_voxel_encoder_$(var model_name).onnx"
    encoder_engine_path: "$(var model_path)/pts_voxel_encoder_$(var model_name).engine"
    head_onnx_path: "$(var model_path)/pts_backbone_neck_head_$(var model_name).onnx"
    head_engine_path: "$(var model_path)/pts_backbone_neck_head_$(var model_name).engine"
    trt_precision: fp16
    cloud_capacity: 2000000
    post_process_params:
      # post-process params
      circle_nms_dist_threshold: 0.5
      iou_nms_search_distance_2d: 10.0
      iou_nms_threshold: 0.1
      # CAR, TRUCK, BUS, BICYCLE, PEDESTRIAN
      score_thresholds: [0.35, 0.35, 0.35, 0.35, 0.35]
      yaw_norm_thresholds: [0.3, 0.3, 0.3, 0.3, 0.0]
    densification_params:
      world_frame_id: map
      num_past_frames: 1
```

#### Build the lidar_centerpoint node

```bash
source /opt/ros2/humble/setup.bash
cd /YOUR/AUTOWARE/PATH/Autoware
source install/setup.bash
colcon build --packages-select autoware_lidar_centerpoint
```

#### Launch the lidar_centerpoint node

```bash
cd /YOUR/AUTOWARE/PATH/Autoware
source install/setup.bash
ros2 launch autoware_lidar_centerpoint lidar_centerpoint.launch.xml  model_name:=centerpoint_custom  model_path:=/PATH/TO/ONNX/FILE/
```

## References/External links

[1] Yin, Tianwei, Xingyi Zhou, and Philipp Krähenbühl. "Center-based 3d object detection and tracking." arXiv preprint arXiv:2006.11275 (2020). <https://github.com/tianweiy/CenterPoint>

[2] Lang, Alex H., et al. "PointPillars: Fast encoders for object detection from point clouds." Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition. 2019.

[3] <https://www.nuscenes.org/nuscenes>


## Legal Notice

_The nuScenes dataset is released publicly for non-commercial use under the Creative
Commons Attribution-NonCommercial-ShareAlike 4.0 International Public License.
Additional Terms of Use can be found at <https://www.nuscenes.org/terms-of-use>.
To inquire about a commercial license please contact <nuscenes@motional.com>._

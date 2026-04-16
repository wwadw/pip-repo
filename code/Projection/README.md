# Projection Tools

用于 LiDAR-相机投影检查的 Rerun 工作台。

## 安装

建议在已 `source` ROS 环境后安装 wheel：

```bash
uv pip install /home/ww/pip-repo/dist/Projection/projection_tools-0.1.0-py3-none-any.whl
```

## 启动

最简单的使用方式：

```bash
source /opt/ros/noetic/setup.bash
projection-rerun \
  --bag /path/to/input.bag \
  --yaml /path/to/camera.yaml
```

默认会启动本地 Web 服务：

- `http://127.0.0.1:8765`

如果需要，也可以额外覆盖 topic：

```bash
projection-rerun \
  --bag /path/to/input.bag \
  --yaml /path/to/camera.yaml \
  --image-topic /camera/image_semantic \
  --overlay-image-topic /camera/image_raw \
  --cloud-topic /lidar/points
```

## 配置文件格式

程序会从 YAML 中读取以下字段：

### 顶层字段

- `semantic_image_topic`
  语义图像 topic
- `overlay_image_topic`
  叠加显示图像 topic
- `pointcloud_topic`
  点云 topic

说明：

- 如果没有 `pointcloud_topic`，程序还会尝试读取 `raw_pointcloud_topic` 或 `compensated_pointcloud_topic`
- 也支持从 `input_sources.semantic_pointcloud.topic` 中提取点云 topic

### `semantic_camera` 字段

- `image_width`
  图像宽度
- `image_height`
  图像高度
- `camera_matrix.fx`
  相机内参 `fx`
- `camera_matrix.fy`
  相机内参 `fy`
- `camera_matrix.cx`
  相机内参 `cx`
- `camera_matrix.cy`
  相机内参 `cy`
- `distortion_coeffs`
  畸变参数数组
- `lidar_to_camera_transform`
  `4 x 4` 外参矩阵，含旋转和平移

## 示例配置

下面是一份可直接参考的 YAML 示例：

```yaml
semantic_image_topic: /usb_cam/image_semantic_id
overlay_image_topic: /usb_cam/image_raw
pointcloud_topic: /mfla/frame_cloud

semantic_camera:
  image_width: 1280
  image_height: 720

  camera_matrix:
    fx: 923.128
    fy: 921.447
    cx: 640.0
    cy: 360.0

  distortion_coeffs:
    - 0.0123
    - -0.0345
    - 0.0001
    - -0.0002
    - 0.0

  lidar_to_camera_transform:
    - [0.0, -1.0, 0.0, 0.12]
    - [0.0, 0.0, -1.0, 0.03]
    - [1.0, 0.0, 0.0, 0.25]
    - [0.0, 0.0, 0.0, 1.0]
```

## 启动示例

```bash
source /opt/ros/noetic/setup.bash
projection-rerun \
  --bag /home/ww/bags/demo.bag \
  --yaml /home/ww/config/minimal_semantic_robot.yaml
```

## 运行前提

- 可用的 ROS 环境
- 能访问的 `rosbag`
- 正确的 YAML 配置

# calib-snap

用于标定采集的图像 + 点云配对工具。支持三种输入方式：

- `rtsp`：从 RTSP 拉流读图像，再和 ROS `PointCloud2` 配对保存
- `camera`：从本地摄像头或 `/dev/video*` 读图像，再和 ROS `PointCloud2` 配对保存
- `dual_ros`：图像和点云都直接从 ROS 话题读取

安装后可直接使用 `calib-snap` 命令。

## 依赖

- Python >= 3.8
- `numpy`
- `opencv-python`
- ROS 环境（提供 `rospy`、`sensor_msgs`、`cv_bridge`）

## 安装

建议在已 `source` ROS 环境后安装 wheel：

```bash
pip install https://pip.wgists.me/dist/calib-snap/calib_snap-0.1.0-py3-none-any.whl
```

如果要直接在仓库里用 `uv` 跑别名命令，建议先建一个可复用系统 ROS 包的环境：

```bash
cd /home/ww/pip-repo/code
uv venv --python "$(which python3)" --system-site-packages
source .venv/bin/activate
source /opt/ros/noetic/setup.bash
uv pip install .
```

## 使用

RTSP 模式：

```bash
calib-snap \
  --input-mode rtsp \
  --rtsp-uri rtsp://127.0.0.1:8554/test \
  --pointcloud-topic /livox/lidar
```

摄像头模式：

```bash
calib-snap \
  --input-mode camera \
  --camera-source 0 \
  --pointcloud-topic /livox/lidar
```

也可以直接传设备路径：

```bash
calib-snap \
  --input-mode camera \
  --camera-source /dev/video0 \
  --pointcloud-topic /livox/lidar
```

双 ROS 话题模式：

```bash
calib-snap \
  --input-mode dual_ros \
  --source-image-topic /g1/camera/0/color/image_raw \
  --pointcloud-topic /livox/lidar
```

## 热键

- `s`：保存当前图像和最近一帧点云
- `q`：退出

## 输出

默认输出目录为当前目录下的 `data/`：

- `images/`：采集图片
- `pointclouds/`：对应点云 PCD

在 `rtsp` 和 `camera` 模式下，图像还会额外发布到：

- `/calib/image_raw`
- `/calib/image_raw/compressed`

## 常用参数

- `--output-dir`：输出目录
- `--image-ext`：图片格式，支持 `png/jpg/jpeg`，默认 `png`
- `--pcd-fields`：保存到 PCD 的字段列表，默认 `x,y,z,intensity`
- `--max-pointcloud-age`：允许配对的最大点云时延，默认 `0.5`
- `--fps`：RTSP/Camera 采集和发布频率

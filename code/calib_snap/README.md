# calib-snap

用于标定采集的图像 + 点云配对工具，也支持不带点云的纯图片采集。支持三种图像输入方式：

- `rtsp`：从 RTSP 拉流读图像
- `camera`：从本地摄像头或 `/dev/video*` 读图像
- `dual_ros`：直接从 ROS 图像话题读取

安装后可直接使用 `calib-snap` 命令。

## 依赖

- Python >= 3.8
- `numpy`
- `opencv-python`
- ROS 环境（提供 `rospy`、`sensor_msgs`、`cv_bridge`）

## 安装

建议在已 `source` ROS 环境后安装 wheel：

```bash
uv pip install /home/ww/pip-repo/dist/calib-snap/calib_snap-0.1.3-py3-none-any.whl
```

如果要直接在源码目录里用 `uv` 跑别名命令，建议先建一个可复用系统 ROS 包的环境：

```bash
cd /home/ww/pip-repo/code
uv venv --python "$(which python3)" --system-site-packages
source .venv/bin/activate
source /opt/ros/noetic/setup.bash
uv pip install .
```

## 纯图片采集

不指定 `--pointcloud-topic` 时，默认进入纯图片采集模式。

ROS 图像话题自动隔帧保存：

```bash
calib-snap \
  --input-mode dual_ros \
  --source-image-topic /camera/image_raw \
  --save-dir ./images \
  --image-save-mode interval \
  --image-interval 10 \
  --prefix img \
  --image-ext jpg
```

ROS 图像话题只手动保存：

```bash
calib-snap \
  --input-mode dual_ros \
  --source-image-topic /camera/image_raw \
  --save-dir ./images \
  --image-save-mode manual \
  --prefix img \
  --image-ext jpg
```

RTSP 视频流只手动保存：

```bash
calib-snap \
  --input-mode rtsp \
  --rtsp-uri rtsp://127.0.0.1:8554/test \
  --save-dir ./images \
  --image-save-mode manual \
  --prefix rtsp \
  --image-ext jpg
```

本地摄像头自动保存并允许手动补拍：

```bash
calib-snap \
  --input-mode camera \
  --camera-source 0 \
  --camera-mjpg \
  --camera-width 1280 \
  --camera-height 720 \
  --camera-fps 30 \
  --save-dir ./images \
  --image-save-mode both \
  --image-interval 5 \
  --prefix usb \
  --image-ext png
```

## 图像 + 点云配对采集

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
  --camera-mjpg \
  --camera-width 1280 \
  --camera-height 720 \
  --camera-fps 30 \
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

- `s`：配对模式下保存当前图像和最近一帧点云；纯图片 `manual` 或 `both` 模式下保存当前图像
- `q`：退出

## 输出

配对模式默认输出目录为当前目录下的 `data/`：

- `images/`：采集图片
- `pointclouds/`：对应点云 PCD

纯图片模式使用 `--save-dir` 指定保存目录，文件名格式为：

```text
<prefix>_YYYYmmdd_HHMMSS_micro.<image-ext>
```

在 `rtsp` 和 `camera` 模式下，图像还会额外发布到：

- `/calib/image_raw`
- `/calib/image_raw/compressed`

## 常用参数

- `--capture-mode`：采集模式，支持 `auto/paired/image`，默认 `auto`
- `--camera-width` / `--camera-height`：本地摄像头采集分辨率，只在 `--input-mode camera` 时支持，必须成对设置
- `--camera-mjpg`：请求本地摄像头使用 MJPG 采集格式，只在 `--input-mode camera` 时支持
- `--camera-fps`：请求本地摄像头硬件采集帧率，只在 `--input-mode camera` 时支持，和控制程序循环频率的 `--fps` 不同
- `--output-dir`：配对模式输出目录
- `--save-dir`：纯图片模式保存目录
- `--image-ext`：图片格式，支持 `png/jpg/jpeg`，默认 `png`
- `--image-save-mode`：纯图片保存触发方式，支持 `interval/manual/both`，默认 `interval`
- `--image-interval`：纯图片自动保存间隔帧数，默认 `10`
- `--prefix`：纯图片文件名前缀，默认 `img`
- `--image-quality`：纯图片保存质量，默认 `95`
- `--pcd-fields`：保存到 PCD 的字段列表，默认 `x,y,z,intensity`
- `--max-pointcloud-age`：允许配对的最大点云时延，默认 `0.5`
- `--fps`：RTSP/Camera 采集和发布频率

## 版本新增内容

### 0.1.3

- 新增本地摄像头采集格式参数：`--camera-mjpg`，对应 `cv2.CAP_PROP_FOURCC = MJPG`。
- 新增本地摄像头硬件采集帧率参数：`--camera-fps`，对应 `cv2.CAP_PROP_FPS`。
- 摄像头采集配置按 MJPG、宽度、高度、FPS 的顺序写入 OpenCV `VideoCapture`。

### 0.1.2

- 新增本地摄像头采集尺寸参数：`--camera-width`、`--camera-height`。
- 摄像头宽高必须成对设置，只在 `--input-mode camera` 下支持。
- RTSP 和 ROS 图像话题模式保持原始输入尺寸不变。

### 0.1.1

- 新增纯图片采集模式。不指定 `--pointcloud-topic` 时，`--capture-mode auto` 默认只保存图片。
- 新增 `--capture-mode auto|paired|image`，将采集行为和图像来源分开。
- 新增 `--image-save-mode interval|manual|both`，支持自动隔帧保存、只手动保存、自动加手动保存。
- 新增纯图片参数：`--save-dir`、`--image-interval`、`--prefix`、`--image-quality`。
- `rtsp` 和 `camera` 纯图片模式继续发布 `/calib/image_raw` 和 `/calib/image_raw/compressed`。
- 保留原有图像 + 点云配对采集行为。

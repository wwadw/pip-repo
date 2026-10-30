# calib-snap

用于标定采集的图像 + 点云配对工具，支持单帧点云、多帧累加的稠密点云和纯图片采集。支持三种图像输入方式：

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

在已加载 ROS 环境的 Python 环境中安装 wheel；`0.1.4` 起支持稠密采集：

```bash
pip install https://pip.wgists.me/dist/calib-snap/calib_snap-0.1.5-py3-none-any.whl
```

也可以在仓库中创建可复用系统 ROS 包的环境，从源码安装：

```bash
cd /home/ww/pip-repo/code/calib_snap
source /opt/ros/noetic/setup.bash
uv venv --python "$(which python3)" --system-site-packages
source .venv/bin/activate
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

## 稠密点云采集

显式指定 `--capture-mode dense`，并在帧数和时长中选择一个结束条件。每次按 `s` 固定当前图像，开始接收后续点云；达到目标后自动保存一张图像和一个合并 PCD，随后等待下一轮。

按帧数累加，例如每轮接收 50 帧，保存前使用 5 mm 体素降采样：

```bash
calib-snap \
  --input-mode dual_ros \
  --source-image-topic /camera/image_raw \
  --pointcloud-topic /livox/lidar \
  --capture-mode dense \
  --accumulate-frames 50 \
  --voxel-size 0.005 \
  --output-dir ./data_dense
```

按时长累加，例如每轮采集 3 秒：

```bash
calib-snap \
  --input-mode camera \
  --camera-source 0 \
  --pointcloud-topic /livox/lidar \
  --capture-mode dense \
  --accumulate-seconds 3 \
  --accumulate-timeout 5 \
  --accumulate-max-mb 256 \
  --output-dir ./data_dense
```

RTSP 输入同样支持稠密模式，使用 `--input-mode rtsp --rtsp-uri <地址>` 指定图像来源。

- `--accumulate-frames N`：正整数，接收到 N 帧通过校验的非空点云后保存，默认不设置。
- `--accumulate-seconds T`：有限正数，从按下 `s` 开始计时，到 T 秒保存，默认不设置。
- `--accumulate-timeout T`：等待首帧或下一帧可接收点云的最长时间，默认 `5.0` 秒。
- `--accumulate-max-mb M`：正整数，原始点云数据缓存上限，默认 `256` MiB，不是进程总内存上限。
- `--voxel-size V`：保存前对整轮点云降采样的体素边长，单位米，默认 `0` 表示关闭；`0.005` 表示 5 mm。仅接受有限非负数。
- `--cancel-key KEY`：取消尚未开始写盘的轮次，默认 `c`。

`--accumulate-frames` 和 `--accumulate-seconds` 必须且只能设置一个；这些累加参数、`--voxel-size` 和 `--cancel-key` 仅适用于稠密模式，稠密模式必须提供 `--pointcloud-topic`。

从 `0.1.5` 起可选择体素降采样。所有帧共用以坐标原点为基准的固定网格，每个占用体素输出一个点：XYZ 取该体素内所有有效点的质心，其他字段（包括 intensity、ring、向量字段等）保留第一条有效观测的原值。XYZ 必须为浮点字段；PCD 字段顺序、类型和元素数量保持不变。体素越大，合并的空间范围越大；标定采集可先用 `0.005`，再比较 `0.002` 和 `0.01` 的边缘与标定误差。

降采样在后台保存阶段执行，减少输出 PCD 的点数，不降低采集阶段的原始缓存峰值。保存时还需要与占用体素数相关的额外内存；`--accumulate-max-mb` 继续只限制原始消息缓存。

采集期间保持雷达、相机、标定板及场景静止。程序在相同 `frame_id` 下直接合并点云，不执行配准或运动补偿；保存的图像始终为按下 `s` 时的图像。按键前缓存的点云不计入本轮。

空点云、过期点云、重复或倒退的非零时间戳不计帧，也不刷新断流超时；零时间戳按每次接收计帧。`--max-pointcloud-age` 在每帧接收时检查，默认 `0.5` 秒，负值关闭检查。时长到期不依赖下一帧到来。

断流超时、缓存超限、坐标系或所选字段结构改变、所选字段布局非法时，本轮失败并释放缓存，不保存不完整样本。写盘时过滤 XYZ 中包含 NaN/Inf 的点，最终必须至少有一个有效点；输出为 `HEIGHT 1` 的 ASCII PCD。

## 热键

- `s`：单帧配对模式保存当前图像和最近一帧点云；稠密模式启动一轮累加；纯图片 `manual` 或 `both` 模式保存当前图像
- `c`：稠密模式取消当前尚未写盘的轮次
- `q` 或 Esc：退出；稠密模式丢弃未写盘的轮次，已开始写盘的轮次完成后退出

稠密模式采集或写盘期间重复按 `s` 只显示进度，不重启或排队；开始写盘后 `c` 不再取消。热键需要在启动程序的终端中输入。

## 输出

单帧配对和稠密模式默认输出目录为当前目录下的 `data/`，可通过 `--output-dir` 修改：

- `images/`：采集图片
- `pointclouds/`：对应点云 PCD

每对图像和点云使用相同的文件名主干。稠密模式每轮输出一对文件，终端显示累加帧数、原始点数、缓存占用、体素尺寸和最终保存点数。

纯图片模式使用 `--save-dir` 指定保存目录，文件名格式为：

```text
<prefix>_YYYYmmdd_HHMMSS_micro.<image-ext>
```

在 `rtsp` 和 `camera` 模式下，图像还会额外发布到：

- `/calib/image_raw`
- `/calib/image_raw/compressed`

## 常用参数

- `--capture-mode`：采集模式，支持 `auto/paired/image/dense`，默认 `auto`；`auto` 在有点云话题时选择单帧配对，否则只采集图片
- `--camera-width` / `--camera-height`：本地摄像头采集分辨率，只在 `--input-mode camera` 时支持，必须成对设置
- `--camera-mjpg`：请求本地摄像头使用 MJPG 采集格式，只在 `--input-mode camera` 时支持
- `--camera-fps`：请求本地摄像头硬件采集帧率，只在 `--input-mode camera` 时支持，和控制程序循环频率的 `--fps` 不同
- `--output-dir`：单帧配对和稠密模式输出目录
- `--save-dir`：纯图片模式保存目录
- `--image-ext`：图片格式，支持 `png/jpg/jpeg`，默认 `png`
- `--image-save-mode`：纯图片保存触发方式，支持 `interval/manual/both`，默认 `interval`
- `--image-interval`：纯图片自动保存间隔帧数，默认 `10`
- `--prefix`：纯图片文件名前缀，默认 `img`
- `--image-quality`：纯图片保存质量，默认 `95`
- `--pcd-fields`：保存到 PCD 的字段列表，默认 `x,y,z,intensity`
- `--max-pointcloud-age`：最大点云年龄，默认 `0.5` 秒；稠密模式逐帧检查，负值关闭检查
- `--fps`：RTSP/Camera 采集和发布频率

## 版本新增内容

### 0.1.5

- 新增 `--voxel-size`，支持稠密点云在保存前按体素降采样，默认关闭。
- XYZ 使用体素内质心，其他字段保留第一条有效观测；保存日志包含体素尺寸和输出点数。

### 0.1.4

- 新增 `dense` 稠密采集模式，可按帧数或时长累加点云。
- 支持冻结起拍图像、取消本轮、断流超时、缓存上限和多帧 PCD 保存。

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

# fov-filter

基于 ROS1 的 PointCloud2 FOV 过滤器。它直接读取 bag 包中的点云话题，按多个水平/垂直 FOV 区域以及距离范围做实时过滤，并把结果发布到 ROS 话题。

这个包自带 bag 播放控制，不依赖 `rosbag play`，因此支持：

- 暂停 / 继续
- 单步前进 / 单步后退
- 动态新增、更新、删除 FOV 区域
- 参数变化后立即对当前帧重新过滤并重发
- 可选把被过滤掉的点云标成红色单独可视化

## 依赖

- Python >= 3.8
- `numpy`
- `PyYAML`
- ROS1 环境（提供 `rospy`、`rosbag`、`sensor_msgs`、`std_msgs`、`rosgraph_msgs`）

## 安装

建议在已 `source` ROS 环境后安装：

```bash
cd /home/ww/test/scripts/fov_filter
uv venv --python "$(which python3)" --system-site-packages
source .venv_ros/bin/activate
source /opt/ros/noetic/setup.bash
uv pip install -e .
```

如果你的 `python3` 来自 conda 或其它自定义环境，这种方式会把当前 Python 环境里的 ROS 相关运行时依赖一并复用进去，通常比裸 `uv venv` 更稳。

构建 wheel：

```bash
uv build --wheel
```

从 wheel 安装：

```bash
uv pip install dist/fov_filter-0.1.0-py3-none-any.whl
```

构建结果在 `dist/` 下，安装后可直接使用三个命令：

- `fov-filter`
- `fov-filter-ctl`
- `fov-filter-ui`

## 启动

```bash
source /opt/ros/noetic/setup.bash
roscore
```

另开终端运行：

```bash
source /opt/ros/noetic/setup.bash
fov-filter \
  --bag /home/ww/bags/ren/go7.bag \
  --topic /mfla/frame_cloud \
  --paint-rejected \
  --region front:-45:45:-15:20 \
  --region side_left:45:110:-20:25
```

默认发布的话题：

- `/fov_filter/cloud`：过滤后保留的点云
- `/fov_filter/rejected`：被过滤掉的点云
- `/fov_filter/visualized`：可选的彩色可视化点云，过滤掉的点为红色
- `/fov_filter/state`：当前播放与区域状态，JSON 字符串
- `/fov_filter/command`：控制命令输入，JSON 字符串

## 动态控制

如果你不想手动敲命令，直接开桌面滑块面板：

```bash
fov-filter-ui
```

建议的使用方式：

1. 先启动 `fov-filter`
2. 再启动 `fov-filter-ui`
3. 在 UI 里拖动帧滑块，并结合区域数值输入做精调

UI 提供：

- 播放 / 暂停 / 前进一帧 / 后退一帧
- bag 帧位置滑块
- 播放倍率步进输入器
- 区域列表
- 水平 / 垂直角 / 距离的滑块 + 手动输入
- 动态新增 / 删除区域
- `paint_rejected`、`publish_rejected`、`loop` 开关
- 配置文件加载 / 导出按钮
- 橙红主按钮 + 暖色滑块的统一配色

暂停：

```bash
fov-filter-ctl pause
```

继续播放：

```bash
fov-filter-ctl play
```

单步前进和后退：

```bash
fov-filter-ctl next
fov-filter-ctl prev
```

动态新增区域：

```bash
fov-filter-ctl add \
  --name center \
  --h-min -30 --h-max 30 \
  --v-min -10 --v-max 15 \
  --d-min 0.0 --d-max 1.6
```

更新区域：

```bash
fov-filter-ctl update \
  --name center \
  --h-min -20 --h-max 20 \
  --v-min -8 --v-max 12 \
  --d-min 0.0 --d-max 1.2
```

删除区域：

```bash
fov-filter-ctl remove --name center
```

查看当前状态：

```bash
fov-filter-ctl status
```

从 TOML/YAML 重新加载配置：

```bash
fov-filter-ctl load-config /home/ww/test/scripts/fov_filter/config.example.toml
```

导出当前启用区域为 `filter_regions` YAML：

```bash
fov-filter-ctl export-config ./filter_regions.yaml
```

## 配置文件

可使用 `--config` 指定 TOML/YAML 文件。示例 TOML 见 [config.example.toml](/home/ww/test/scripts/fov_filter/config.example.toml)。UI 和 `fov-filter-ctl export-config` 导出的 YAML 顶层为 `filter_regions:`。

## 参数说明

- `--bag`：bag 文件路径
- `--topic`：需要读取的 PointCloud2 话题
- `--region`：启动时添加的区域，格式 `name:hmin:hmax:vmin:vmax[:enabled][:dmin:dmax]`
- `--rate`：播放倍率，默认 `1.0`
- `--start-paused`：启动后先暂停
- `--loop`：播到末尾后循环
- `--paint-rejected`：发布彩色可视化点云并将过滤点标红
- `--publish-clock`：同步发布 `/clock`

水平角定义为 `atan2(y, x)`，内部按 `[0, 360)` 归一化比较，因此同时支持 `-45..45` 和 `315..45` 这两种写法。垂直角定义为 `atan2(z, hypot(x, y))`，范围 `[-90, 90]` 度。距离使用三维欧氏距离，单位米，当前默认编辑范围为 `0~2m`。

若未配置任何启用中的区域，则默认不过滤，直接保留所有有效点。

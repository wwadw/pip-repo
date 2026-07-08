# fov-filter

当前版本：`0.1.13`

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
source /opt/ros/noetic/setup.bash
python -m pip install --force-reinstall --no-deps https://pip.wgists.me/dist/fov_filter/fov_filter-0.1.13-py3-none-any.whl
hash -r
```

检查当前环境中的版本和安装位置：

```bash
python -m pip show fov-filter
which fov-filter-ui
```

如果 `which fov-filter-ui` 指向 `~/.local/bin/fov-filter-ui`，请确认它没有继续指向旧的 Python3.8 用户包；否则可能出现 conda 环境和 `~/.local/lib/python3.8/site-packages` 混用的问题。

构建 wheel：

```bash
cd /home/ww/pip-repo/code/fov_filter
conda activate env1
python -m pip wheel --no-deps -w dist .
```

重新安装当前 wheel：

```bash
conda activate env1
python -m pip install --force-reinstall --no-deps https://pip.wgists.me/dist/fov_filter/fov_filter-0.1.13-py3-none-any.whl
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
  --topic-prefix /fov_filter/mfla_frame_cloud \
  --paint-rejected \
  --region front:-45:45:-15:20 \
  --region side_left:45:110:-20:25
```

UI 中配置的 FOV 区域表示“要删除/过滤掉的区域”。默认发布的话题。建议多路点云或多开播放器时使用 `--topic-prefix`，例如
`--topic-prefix /fov_filter/mfla_frame_cloud`，这样整组输出和控制话题都能按数据源区分：

- `/fov_filter/points_kept`：过滤后保留的点云
- `/fov_filter/points_removed`：被过滤掉的点云
- `/fov_filter/points_colored`：可选的彩色可视化点云，保留点和移除点用不同颜色显示
- `/fov_filter/fov_regions`：FOV 区域线框，RViz 中添加 `MarkerArray` 可直接查看
- `/fov_filter/state`：当前播放与区域状态，JSON 字符串
- `/fov_filter/command`：控制命令输入，JSON 字符串

RViz 可视化建议：

1. 添加 `PointCloud2`，话题选 `/fov_filter/points_kept` 或 `/fov_filter/points_colored`
2. 添加 `MarkerArray`，话题选 `/fov_filter/fov_regions`
3. 如果只想看被删掉的点，添加 `PointCloud2` 并选择 `/fov_filter/points_removed`
4. 如果启用了 `--publish-clock` 并使用 bag 时间，记得设置 ROS 参数 `/use_sim_time=true`

从 `0.1.11` 开始，播放器内部的自动播放循环使用墙钟调度，不再依赖 `rospy.Rate.sleep()`。因此即使 `/use_sim_time=true` 但 `/clock` 没有推进，UI 的播放按钮也不会被 ROS 仿真时间卡住。

## 动态控制

可以只打开桌面滑块面板，在 UI 中选择 bag 和点云话题后直接启动后台 `fov-filter`：

```bash
fov-filter-ui
```

UI 一站式启动流程：

1. 点击“选择 bag”并选择 `.bag` 文件
2. 等待 UI 自动扫描 `PointCloud2` 话题
3. 在下拉框选择需要过滤的点云话题
4. 点击“加载 / 启动 bag”
5. 启动后继续在同一个 UI 中播放、单步、调 FOV、导出配置

如果播放器使用了自定义前缀，UI 和控制命令也使用同一个前缀：

```bash
fov-filter-ui --topic-prefix /fov_filter/mfla_frame_cloud
fov-filter-ctl --topic-prefix /fov_filter/mfla_frame_cloud status
```

外部终端启动播放器仍然支持：

1. 先启动 `fov-filter`
2. 再启动 `fov-filter-ui`
3. 在 UI 里拖动帧滑块，并结合区域数值输入做精调

UI 提供：

- 播放 / 暂停 / 前进一帧 / 后退一帧
- 在 UI 中选择 bag 文件，自动扫描其中的 `PointCloud2` 话题，并通过下拉框选择后加载
- 如果当前没有可用播放器，UI 会自动启动后台 `fov-filter`
- 如果当前已有同前缀播放器，UI 会通过 `load_bag` 命令热加载新的 bag 和话题
- bag 帧位置滑块
- 播放倍率步进输入器
- 区域列表
- 点击“新增区域”直接创建区域，双击列表中的区域名称可行内重命名
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
fov-filter-ctl load-config /home/ww/pip-repo/code/fov_filter/config.example.toml
```

导出当前启用区域为 `filter_regions` YAML：

```bash
fov-filter-ctl export-config ./filter_regions.yaml
```

如果要把 UI/RViz 里按发布点云角度标出来的区域导出给 RSHELIOS 驱动使用，导出时加 `--lidar-model rshelios`：

```bash
fov-filter-ctl export-config ./filter_regions.yaml --lidar-model rshelios
```

RSHELIOS 解码器在发布点云前用雷达内部水平角过滤，而发布点云的 `y` 轴符号会让水平角近似变成 `(360 - 驱动内部角度) % 360`。因此这个选项只在导出 `filter_regions` 时把水平角转换为驱动内部角度；UI、Marker 和实时过滤仍然按 RViz 中看到的发布点云坐标工作。导出的 YAML 缩进与 rslidar_sdk 常用配置保持一致。

## 配置文件

可使用 `--config` 指定 TOML/YAML 文件。示例 TOML 见 [config.example.toml](/home/ww/pip-repo/code/fov_filter/config.example.toml)。UI 和 `fov-filter-ctl export-config` 导出的 YAML 顶层为 `filter_regions:`。

## 参数说明

- `--bag`：bag 文件路径
- `--topic`：需要读取的 PointCloud2 话题
- `--topic-prefix`：整组输出/控制话题前缀，默认 `/fov_filter`
- `--region`：启动时添加的区域，格式 `name:hmin:hmax:vmin:vmax[:enabled][:dmin:dmax]`
- `--rate`：播放倍率，默认 `1.0`
- `--start-paused`：启动后先暂停
- `--loop`：播到末尾后循环
- `--paint-rejected`：发布彩色可视化点云并将过滤点标红
- `--publish-clock`：同步发布 `/clock`
- `--marker-topic`：RViz FOV 区域 `MarkerArray` 话题，默认随 `--topic-prefix` 生成为 `.../fov_regions`

水平角定义为 `atan2(y, x)`，内部按 `[0, 360)` 归一化比较，因此同时支持 `-45..45` 和 `315..45` 这两种写法。垂直角定义为 `atan2(z, hypot(x, y))`，范围 `[-90, 90]` 度。距离使用三维欧氏距离，单位米，当前默认编辑范围为 `0~2m`。命中启用区域的点会发布到 `/fov_filter/points_removed`，未命中的点会保留到 `/fov_filter/points_kept`。

若未配置任何启用中的区域，则默认不过滤，直接保留所有有效点。

## bag 加载排查

播放器现在会更宽容地读取 bag：会尝试打开未索引 bag，使用 bag 自身时间做播放调度，并在状态消息中的 `load_warnings` 给出加载提示。若指定话题不存在，会直接列出 bag 内可用的 `PointCloud2` 话题，方便确认 `--topic` 是否写错。

UI 启动的后台播放器日志会写到：

```bash
/tmp/fov_filter_ui_player_<UI进程号>.log
```

查看最新日志：

```bash
tail -f "$(ls -t /tmp/fov_filter_ui_player_*.log | head -1)"
```

如果日志中能看到 `收到命令 op=play`，但没有 `播放推进到 index=...`，优先检查是否运行的是旧版本：

```bash
conda activate env1
python -m pip show fov-filter
which fov-filter-ui
```

当前 `0.1.13` 已避免 `/use_sim_time` 卡住播放循环；正常播放时日志会持续打印 `播放推进到 index=...`。

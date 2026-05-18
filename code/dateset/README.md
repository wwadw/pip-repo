# dataset

ROS 图像数据采集工具的 Python 包。安装后可用 `dataset` 命令直接启动采集。

## 依赖

- Python >= 3.8
- `opencv-python`
- `numpy`
- ROS 环境（提供 `rospy`、`sensor_msgs`、`cv_bridge`）

## 安装

建议在已 `source` ROS 环境后安装：

```bash
uv venv
uv pip install -e /home/ww/test/scripts/dateset
```

## 运行

```bash
dataset /camera/image_raw 10 ./images
```

或使用配置文件：

```bash
dataset --config config.toml
```

参数说明：

- `topic`：ROS 图像话题名称
- `interval`：采集间隔帧数（每隔多少帧保存一张图片）
- `save_dir`：图片保存目录
- `-f, --format`：图片格式（jpg/png），默认 jpg
- `-p, --prefix`：图片文件名前缀，默认 img
- `-q, --quality`：图片质量（1-100），默认 95

## 直接用 Python 运行

```bash
python /home/ww/test/scripts/dateset/dataset.py /camera/image_raw 10 ./images
```

## 配置文件示例

```toml
[collector]
topic = "/camera/image_raw"
interval = 10
save_dir = "./images"
format = "jpg"
prefix = "img"
quality = 95
```

## 常见问题

- 如果提示 `rospy 未安装或 ROS 环境未加载`，请先 `source` ROS setup（例如 `source /opt/ros/noetic/setup.bash`）。

# 编译教程

本文档说明如何在 `code/Projection` 目录下编译：

- 前端静态网页
- Python wheel 包

## 前提

建议先准备 ROS 环境和 Python 虚拟环境：

```bash
cd /home/ww/pip-repo/code/Projection
source /opt/ros/noetic/setup.bash
uv venv .venv --python python3 --system-site-packages
env UV_CACHE_DIR=/tmp/uv-cache uv pip install --python .venv/bin/python -e ".[dev]"
```

如果第一次在当前目录构建前端，还需要安装前端依赖：

```bash
cd /home/ww/pip-repo/code/Projection/web
npm install
```

## 编译静态网页

在 `web/` 目录执行：

```bash
cd /home/ww/pip-repo/code/Projection/web
npm run build
```

当前项目的前端产物不会输出到 `web/dist`，而是输出到：

```bash
/home/ww/pip-repo/code/Projection/rerun_projection/web_dist
```

可以用下面的命令快速检查：

```bash
find /home/ww/pip-repo/code/Projection/rerun_projection/web_dist -maxdepth 2 -type f | sort
```

## 编译 wheel

在项目根目录执行：

```bash
cd /home/ww/pip-repo/code/Projection
.venv/bin/python -m build --no-isolation
```

生成后的文件会输出到：

```bash
/home/ww/pip-repo/code/Projection/dist
```

常见产物为：

- `projection_tools-0.1.0-py3-none-any.whl`
- `projection_tools-0.1.0.tar.gz`

可以用下面的命令检查：

```bash
find /home/ww/pip-repo/code/Projection/dist -maxdepth 1 -type f | sort
```

## 推荐顺序

如果要发布完整包，建议始终按这个顺序执行：

```bash
cd /home/ww/pip-repo/code/Projection/web
npm run build

cd /home/ww/pip-repo/code/Projection
.venv/bin/python -m build --no-isolation
```

原因是 wheel 会打包 `rerun_projection/web_dist` 下的静态资源。先编译前端，再编译 wheel，才能确保包内网页资源是最新的。

## 安装编译结果

wheel 编译完成后，可以直接安装：

```bash
uv pip install /home/ww/pip-repo/code/Projection/dist/projection_tools-0.1.0-py3-none-any.whl
```

安装后可用以下方式启动：

```bash
source /opt/ros/noetic/setup.bash
projection-rerun --bag /path/to/input.bag --yaml /path/to/camera.yaml
```

# Projection Tools

LiDAR-camera projection utilities. The legacy `Projection.py` Open3D/OpenCV viewer is still available, and the new Rerun workbench adds a web control panel for parameter editing and 2D/3D linked selection.

## Rerun Workbench

Install in a ROS-capable environment with Python 3.10 or newer for the Rerun workbench:

```bash
cd /home/ww/pip-repo/.worktrees/rerun-projection-workbench/code/Projection
source /opt/ros/noetic/setup.bash
uv venv .venv --python python3 --system-site-packages
env UV_CACHE_DIR=/tmp/uv-cache uv pip install --python .venv/bin/python -e ".[dev]"
cd web
npm install
npm run build
cd ..
.venv/bin/projection-rerun --bag /path/to/input.bag
```

You can also pass initial values that will be pre-filled in the web panel:

```bash
.venv/bin/projection-rerun \
  --bag /path/to/input.bag \
  --yaml /path/to/camera.yaml \
  --image-topic /camera/image_semantic \
  --cloud-topic /lidar/points
```

Example against the local smoke-test bag:

```bash
.venv/bin/projection-rerun \
  --bag /home/ww/bags/ren/go7.bag \
  --image-topic /camera/color/image_mask \
  --cloud-topic /mfla/frame_cloud
```

## Web Panel Behavior

- `Apply Source` reloads `bag`, image topic, point cloud topic, and YAML source settings. It clears the current selection and locked pairs because point indices may no longer match.
- `Apply Projection` updates camera intrinsics, distortion coefficients, `lidar_to_camera`, image size, and minimum depth. It keeps locked point pairs and reprojects them with the new parameters.
- Clicking in the 2D panel sends a pixel to Python, which highlights the closest projected point.
- Selecting `world/points` in the Rerun viewer sends the point instance back to Python, which highlights its 2D projection.
- `Lock Pair`, `Delete Last`, and `Clear All` manage persistent correspondence markers.

## Notes

- The workbench uses `rerun-sdk==0.31.3` and `@rerun-io/web-viewer==0.31.3` so the Web Viewer exposes `selection_change` events.
- The Rerun workbench requires Python 3.10+ because the newer Rerun SDK and web viewer are needed for bidirectional selection. This is separate from the legacy ROS Noetic-era script.
- The legacy Open3D/OpenCV script can still be run directly with `python Projection.py --bag /path/to/input.bag`.

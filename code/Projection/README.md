# Projection Tools

LiDAR-camera projection utilities. The legacy `Projection.py` Open3D/OpenCV viewer is still available, and the new Rerun workbench adds a nuScenes-style Rerun layout with a web control panel for parameter editing.

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
.venv/bin/projection-rerun
```

Build an installable package after the frontend bundle has been generated:

```bash
cd /home/ww/pip-repo/.worktrees/rerun-projection-workbench/code/Projection
cd web
npm run build
cd ..
.venv/bin/python -m build --no-isolation
.venv/bin/python -m pip install dist/projection_tools-0.1.0-py3-none-any.whl
projection-rerun
```

The default launch matches the current legacy debugging command:

```bash
.venv/bin/projection-rerun
```

You can also pass overrides that will be pre-filled in the web panel:

```bash
.venv/bin/projection-rerun \
  --bag /path/to/input.bag \
  --yaml /path/to/camera.yaml \
  --image-topic /camera/image_semantic \
  --overlay-image-topic /camera/image_raw \
  --cloud-topic /lidar/points
```

Example against the local smoke-test bag:

```bash
.venv/bin/projection-rerun \
  --yaml /home/ww/elevation_ws/src/elevation_mapping/elevation_mapping_demos/config/robots/minimal_semantic_robot.yaml \
  --bag /home/ww/bags/b2biaopin/louti/2026-04-14-20-18-10_semantic.bag \
  --image-topic /usb_cam/image_semantic_id \
  --overlay-image-topic /usb_cam/image_raw \
  --cloud-topic /mfla/frame_cloud
```

## Web Panel Behavior

- The main Rerun view uses a large 3D panel on top and two camera panels on the bottom. Each camera panel is rooted at the native Rerun camera entity, so the 2D views and 3D image planes share the same sensor path.
- The 3D panel only shows the overlay camera image plane. The semantic camera stays available in 2D without adding a second overlapping frustum in 3D.
- `Apply Source` reloads `bag`, semantic image topic, detection overlay topic, point cloud topic, and YAML source settings. It clears the current selection and locked pairs because point indices may no longer match.
- `Apply Projection` updates camera intrinsics, distortion coefficients, `lidar_to_camera`, image size, and minimum depth. It keeps locked point pairs and reprojects them with the new parameters.
- Selecting `world/ego_vehicle/lidar` in the Rerun viewer sends the point instance back to Python, which highlights the corresponding point in 3D plus both camera views.
- The camera panels only show the image plane plus lightweight selection and locked-pair markers; they do not render the full projected point set.
- `Lock Pair`, `Delete Last`, and `Clear All` manage persistent correspondence markers.

## Notes

- The workbench uses `rerun-sdk==0.31.3` and `@rerun-io/web-viewer==0.31.3` so the Web Viewer exposes `selection_change` events.
- The Rerun workbench requires Python 3.10+ because the newer Rerun SDK and web viewer are needed for bidirectional selection. This is separate from the legacy ROS Noetic-era script.
- The installed command alias is `projection-rerun`; the Python package includes the built frontend assets from `rerun_projection/web_dist`.
- The legacy Open3D/OpenCV script can still be run directly with `python Projection.py --bag /path/to/input.bag`.

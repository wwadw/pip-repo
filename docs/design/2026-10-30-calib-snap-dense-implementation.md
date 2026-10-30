# calib-snap Dense Capture Implementation Plan

> Execute inline with test-driven-development and verification-before-completion. The user authorized implementation on 2026-10-30.

**Goal:** Add key-triggered image and accumulated point-cloud capture.

**Architecture:** Keep the current single-module package. A clock-independent session owns each capture; a node worker checks deadlines and saves frozen snapshots outside the node lock.

**Tech Stack:** Python >= 3.8, ROS 1 sensor_msgs, OpenCV, NumPy, unittest.

**Spec:** `docs/design/2026-10-30-calib-snap-dense.md`

## Global Constraints

- Preserve auto/paired/image selection and all three image inputs.
- Require exactly one frame or duration target for dense capture.
- Defaults: 5.0-second silence timeout and 256 MiB raw message cache.
- Use monotonic receipt time for windows, ROS time only for message age.
- Freeze the trigger image, exclude previously cached clouds, clear each round.
- Preserve selected field order, field counts, byte order and row padding.
- Save finite XYZ points as flattened ASCII PCD with matching counts.
- Keep callbacks free of point decoding and file I/O.
- Fail and clean the round on schema changes, overflow, timeout or save errors.
- Add no runtime dependencies. Publication and push were authorized separately after implementation, for release 0.1.4.

## Task 1: Parameters And Accumulation

Files: `code/calib_snap/calib_snap.py`, `code/calib_snap/tests/test_dense_capture.py`.

Produces: `DenseCaptureSession.start(image, image_stamp, now)`, `add_cloud(msg, received_at, ros_now)`, `take_ready(now)`, `cancel()`, `finish_save(session_id)`, and read-only state/progress.

- [x] Write tests for target selection, invalid values, frame limits, time boundaries, deduplication, cache limits, schema changes and cancellation.
- [x] Run `python3 -m unittest discover -s tests -p test_dense_capture.py -v` and confirm missing dense behavior fails.
- [x] Implement CLI validation, snapshot data and the session.
- [x] Run the CLI/session tests and all existing image tests.

Representative contract:

```python
session.start(image, image_stamp, now=10.0)
session.add_cloud(cloud, received_at=10.1, ros_now=100.0)
snapshot = session.take_ready(now=13.0)
```

The returned snapshot must contain only admitted clouds, the copied trigger image, output fields, completion reason and capture timing.

## Task 2: Point-Cloud And Pair Output

Files: `code/calib_snap/calib_snap.py`, `code/calib_snap/tests/test_dense_capture.py`.

Consumes: frozen session snapshots.
Produces: `save_pointclouds_as_pcd(clouds, output_path, requested_fields)` and `save_dense_capture_pair(snapshot, image_dir, pointcloud_dir, image_ext, capture_index)`.

- [x] Write failing output tests with hand-checked binary fixtures and read back the PCD headers and values.
- [x] Cover reordered/vector fields, endian changes, row padding, finite XYZ filtering and zero-point rejection.
- [x] Implement two-pass streaming PCD writing and temporary pair files with non-overwriting publication.
- [x] Inject image, PCD and commit failures and verify cleanup and preservation of old samples.
- [x] Run the writer tests and existing tests.

Representative output for three two-point clouds:

```text
WIDTH 6
HEIGHT 1
POINTS 6
DATA ascii
```

## Task 3: Node Integration And Lifecycle

Files: `code/calib_snap/calib_snap.py`, `code/calib_snap/tests/test_dense_capture.py`.

Consumes: the session and pair writer.
Produces: dense hotkey handling, deadline worker, progress logging and idempotent `close()`.

- [x] Write failing tests for frozen images, old-cloud exclusion, repeated save keys, cancellation, worker deadlines and close.
- [x] Wire both input loops through the same dense key handling.
- [x] Start a single event-driven worker only for dense mode; save and decode outside the lock.
- [x] Register ROS shutdown and close all main-loop exits; finish in-progress writes and discard unfinished rounds.
- [x] Run all tests and a real ROS message/reader integration check when the dependencies are available.

## Task 4: Documentation And Final Checks

Files: `code/calib_snap/README.md`, `docs/calib-snap.md`, generated `docs/calib-snap.html`, design status.

- [x] Document both target flags, cancel/quit behavior, silence timeout, raw cache limit and the static-scene requirement.
- [x] Generate the affected HTML page through `scripts/generate_indexes.py` rendering functions without unrelated output churn.
- [x] Run all calib-snap tests, Python 3.8 syntax checks and `git diff --check`.
- [x] Review the final changes against the spec, resolve concrete findings and report any unavailable hardware validation.

## Initial Implementation Validation

Completed on 2026-10-30:

- 55 tests passed with real ROS messages and the official `sensor_msgs.point_cloud2` reader enabled.
- Without the extracted ROS packages, 54 tests passed and the real ROS integration test skipped.
- Python 3.8.10 compilation passed.
- Local wheel built successfully under `/tmp/calib-snap-dense-wheel`; repository release artifacts and package version remain unchanged.
- Only `docs/calib-snap.html` was regenerated; `git diff --check` passed.
- Code review findings about failed-read hotkeys and overlapping selected fields were reproduced with failing tests, fixed and re-reviewed.
- Physical camera/LiDAR capture and projection alignment were not tested; no ROS master or hardware was available.

## Release 0.1.4 Validation

- Updated the package version and both installation guides to 0.1.4.
- Built `dist/calib-snap/calib_snap-0.1.4-py3-none-any.whl` from the final source with Python 3.8.10.
- Re-ran all 55 tests with the real ROS reader enabled; all passed.
- Installed the wheel into an isolated temporary directory and verified its version, console entry point and both accumulation targets with ROS stubs.
- Verified the installed Python module matches the source.
- Generated the root page, total download index, calib-snap download index and calib-snap documentation with the existing renderer.
- Checked generated pages against the renderer and verified every local wheel download target exists.
- Kept old wheels and the existing static repository format.

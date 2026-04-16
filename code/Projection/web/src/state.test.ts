import { describe, expect, it } from "vitest";
import { applyBootstrap, createDraftState } from "./state";

describe("state", () => {
  it("hydrates form drafts from backend bootstrap", () => {
    const state = createDraftState();
    applyBootstrap(state, {
      config: {
        bag_file: "/bags/demo.bag",
        yaml_path: "",
        image_topic: "/camera/image",
        pointcloud_topic: "/lidar/points",
        image_width: 1280,
        image_height: 720,
        camera_matrix: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        distortion_coeffs: [0, 0, 0, 0, 0],
        lidar_to_camera: [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]],
        min_depth: 0.05
      },
      current_selection: null,
      locked_pairs: []
    });

    expect(state.draftSource.bag_file).toBe("/bags/demo.bag");
    expect(state.draftSource.image_topic).toBe("/camera/image");
    expect(state.draftProjection.image_width).toBe(1280);
  });
});

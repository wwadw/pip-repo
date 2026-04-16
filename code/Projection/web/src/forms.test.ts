import { describe, expect, it, vi } from "vitest";
import { buildSourceActions } from "./forms";

describe("forms", () => {
  it("posts Apply Source with the current draft payload", async () => {
    const api = { applySource: vi.fn().mockResolvedValue({ ok: true }) };
    const actions = buildSourceActions(api);

    await actions.apply({
      bag_file: "/bags/demo.bag",
      yaml_path: "",
      image_topic: "/camera/image",
      pointcloud_topic: "/lidar/points"
    });

    expect(api.applySource).toHaveBeenCalledWith({
      bag_file: "/bags/demo.bag",
      yaml_path: "",
      image_topic: "/camera/image",
      pointcloud_topic: "/lidar/points"
    });
  });
});

import { readFileSync } from "node:fs";
import { resolve } from "node:path";
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
      overlay_image_topic: "/camera/raw",
      pointcloud_topic: "/lidar/points"
    });

    expect(api.applySource).toHaveBeenCalledWith({
      bag_file: "/bags/demo.bag",
      yaml_path: "",
      image_topic: "/camera/image",
      overlay_image_topic: "/camera/raw",
      pointcloud_topic: "/lidar/points"
    });
  });

  it("renders playback and grouped projection controls without the selection panel", () => {
    const source = readFileSync(resolve(__dirname, "forms.ts"), "utf8");

    expect(source).toContain("Viewer Timeline");
    expect(source).toContain("Camera Matrix");
    expect(source).toContain("LiDAR To Camera");
    expect(source).not.toContain("Play");
    expect(source).not.toContain("Pause");
    expect(source).not.toContain("<h2>Selection</h2>");
  });
});

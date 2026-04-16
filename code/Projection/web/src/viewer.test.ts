import { expect, test } from "vitest";

import { selectionItemKey } from "./viewer";

test("selection item key is stable for repeated 3d point selections", () => {
  const key = selectionItemKey({
    type: "entity",
    entity_path: "world/ego_vehicle/lidar",
    instance_id: 42,
    position: [1, 2, 3]
  });

  expect(key).toBe("world/ego_vehicle/lidar:42:1,2,3");
});

test("selection item key accepts projected points from either camera view", () => {
  const semanticKey = selectionItemKey({
    type: "entity",
    entity_path: "world/ego_vehicle/semantic_camera/projected_points",
    instance_id: 7,
    position: [11, 12]
  });
  const overlayKey = selectionItemKey({
    type: "entity",
    entity_path: "world/ego_vehicle/overlay_camera/projected_points",
    instance_id: 7,
    position: [11, 12]
  });

  expect(semanticKey).toBe("world/ego_vehicle/semantic_camera/projected_points:7:11,12");
  expect(overlayKey).toBe("world/ego_vehicle/overlay_camera/projected_points:7:11,12");
});

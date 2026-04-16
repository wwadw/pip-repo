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

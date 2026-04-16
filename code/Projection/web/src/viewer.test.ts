import { expect, test } from "vitest";

import { mapLayerPointToImagePixel, selectionItemKey } from "./viewer";

test("selection item key is stable for repeated 3d point selections", () => {
  const key = selectionItemKey({
    type: "entity",
    entity_path: "world/ego_vehicle/lidar",
    instance_id: 42,
    position: [1, 2, 3]
  });

  expect(key).toBe("world/ego_vehicle/lidar:42:1,2,3");
});

test("maps click-layer coordinates back into overlay image pixels", () => {
  const pixel = mapLayerPointToImagePixel({
    x: 150,
    y: 75,
    layerWidth: 300,
    layerHeight: 150,
    imageWidth: 1200,
    imageHeight: 600
  });

  expect(pixel).toEqual([600, 300]);
});

test("ignores clicks that land in the click-layer letterbox margin", () => {
  const pixel = mapLayerPointToImagePixel({
    x: 10,
    y: 10,
    layerWidth: 400,
    layerHeight: 300,
    imageWidth: 400,
    imageHeight: 100
  });

  expect(pixel).toBeNull();
});

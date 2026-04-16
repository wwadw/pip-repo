import { readFileSync } from "node:fs";
import { resolve } from "node:path";
import { test, expect } from "vitest";

const root = resolve(__dirname, "..");

test("workbench uses one rerun viewer so the blueprint owns the 3d and camera layout", () => {
  const html = readFileSync(resolve(root, "index.html"), "utf8");

  expect(html).toContain('<button id="imageClickLayer"');
  expect(html).toContain('id="rerunViewer"');
  expect(html).not.toContain('id="viewer2d"');
});

test("page shell does not let wheel events scroll the whole workbench", () => {
  const css = readFileSync(resolve(__dirname, "styles.css"), "utf8");

  expect(css).toMatch(/html,\s*body\s*{[^}]*overflow:\s*hidden/s);
  expect(css).toMatch(/\.sidebar\s*{[^}]*overflow-y:\s*auto/s);
});

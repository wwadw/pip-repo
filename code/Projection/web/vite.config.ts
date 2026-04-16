import { defineConfig } from "vite";

export default defineConfig({
  build: {
    outDir: "../rerun_projection/web_dist",
    emptyOutDir: true
  },
  test: {
    environment: "node"
  }
});

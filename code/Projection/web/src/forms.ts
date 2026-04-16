import type { ProjectionDraft, SourceDraft, WorkbenchState } from "./state";

type ApiShape = {
  applySource: (payload: SourceDraft) => Promise<unknown>;
  applyProjection: (payload: ProjectionDraft) => Promise<unknown>;
  lockPair: () => Promise<unknown>;
  deleteLastPair: () => Promise<unknown>;
  clearPairs: () => Promise<unknown>;
};

export function buildSourceActions(api: Pick<ApiShape, "applySource">) {
  return {
    apply: (draft: SourceDraft) => api.applySource(draft)
  };
}

function matrixToMultiline(matrix: number[][]): string {
  return matrix.map((row) => row.join(", ")).join("\n");
}

function parseMatrix(raw: string, fallback: number[][]): number[][] {
  try {
    const rows = raw
      .split("\n")
      .map((line) => line.trim())
      .filter(Boolean)
      .map((line) => line.split(",").map((value) => Number(value.trim())));
    return rows.length > 0 ? rows : fallback;
  } catch {
    return fallback;
  }
}

function renderLockedPairs(state: WorkbenchState): string {
  if (state.lockedPairs.length === 0) {
    return `<div class="empty-state">No locked pairs yet.</div>`;
  }
  return state.lockedPairs
    .map((pair: any) => {
      const color = `rgb(${pair.color_rgb.join(",")})`;
      return `<div class="pair-row"><span class="pair-chip" style="--chip:${color}">P${pair.pair_id}</span><span>${pair.projected_pixel.map((value: number) => value.toFixed(1)).join(", ")}</span></div>`;
    })
    .join("");
}

export function renderForms(
  container: HTMLElement,
  state: WorkbenchState,
  api: ApiShape,
  onRefresh: (payload: any) => void
) {
  container.innerHTML = `
    <div class="sidebar-shell">
      <section class="sidebar-section">
        <h2>Data Source</h2>
        <label>Bag Path<input id="bag_file" value="${state.draftSource.bag_file}" /></label>
        <label>YAML Path<input id="yaml_path" value="${state.draftSource.yaml_path}" /></label>
        <label>Image Topic<input id="image_topic" value="${state.draftSource.image_topic}" /></label>
        <label>Point Cloud Topic<input id="pointcloud_topic" value="${state.draftSource.pointcloud_topic}" /></label>
        <button id="applySource">Apply Source</button>
      </section>
      <section class="sidebar-section">
        <h2>Projection Params</h2>
        <label>Image Width<input id="image_width" type="number" value="${state.draftProjection.image_width}" /></label>
        <label>Image Height<input id="image_height" type="number" value="${state.draftProjection.image_height}" /></label>
        <label>Min Depth<input id="min_depth" type="number" step="0.01" value="${state.draftProjection.min_depth}" /></label>
        <label>Camera Matrix<textarea id="camera_matrix">${matrixToMultiline(state.draftProjection.camera_matrix)}</textarea></label>
        <label>Distortion Coeffs<input id="distortion_coeffs" value="${state.draftProjection.distortion_coeffs.join(", ")}" /></label>
        <label>LiDAR To Camera<textarea id="lidar_to_camera">${matrixToMultiline(state.draftProjection.lidar_to_camera)}</textarea></label>
        <button id="applyProjection">Apply Projection</button>
      </section>
      <section class="sidebar-section">
        <h2>Selection</h2>
        <div class="status-card">${state.currentSelection ? JSON.stringify(state.currentSelection, null, 2) : "No current selection"}</div>
        <div class="button-row">
          <button id="lockPair">Lock Pair</button>
          <button id="deleteLastPair">Delete Last</button>
          <button id="clearPairs">Clear All</button>
        </div>
        <div class="pairs-list">${renderLockedPairs(state)}</div>
      </section>
    </div>
  `;

  const readSourceDraft = (): SourceDraft => ({
    bag_file: (container.querySelector("#bag_file") as HTMLInputElement).value,
    yaml_path: (container.querySelector("#yaml_path") as HTMLInputElement).value,
    image_topic: (container.querySelector("#image_topic") as HTMLInputElement).value,
    pointcloud_topic: (container.querySelector("#pointcloud_topic") as HTMLInputElement).value
  });

  const readProjectionDraft = (): ProjectionDraft => ({
    image_width: Number((container.querySelector("#image_width") as HTMLInputElement).value),
    image_height: Number((container.querySelector("#image_height") as HTMLInputElement).value),
    min_depth: Number((container.querySelector("#min_depth") as HTMLInputElement).value),
    camera_matrix: parseMatrix((container.querySelector("#camera_matrix") as HTMLTextAreaElement).value, state.draftProjection.camera_matrix),
    distortion_coeffs: (container.querySelector("#distortion_coeffs") as HTMLInputElement).value.split(",").map((value) => Number(value.trim())),
    lidar_to_camera: parseMatrix((container.querySelector("#lidar_to_camera") as HTMLTextAreaElement).value, state.draftProjection.lidar_to_camera)
  });

  (container.querySelector("#applySource") as HTMLButtonElement).onclick = async () => {
    const payload = await api.applySource(readSourceDraft());
    onRefresh(payload);
  };
  (container.querySelector("#applyProjection") as HTMLButtonElement).onclick = async () => {
    const payload = await api.applyProjection(readProjectionDraft());
    onRefresh(payload);
  };
  (container.querySelector("#lockPair") as HTMLButtonElement).onclick = async () => {
    const payload = await api.lockPair();
    onRefresh(payload);
  };
  (container.querySelector("#deleteLastPair") as HTMLButtonElement).onclick = async () => {
    const payload = await api.deleteLastPair();
    onRefresh(payload);
  };
  (container.querySelector("#clearPairs") as HTMLButtonElement).onclick = async () => {
    const payload = await api.clearPairs();
    onRefresh(payload);
  };
}

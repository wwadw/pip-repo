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
  const isBuilding = state.startup.state === "building";
  const canLock = state.currentSelection !== null;
  const startupNotice = isBuilding
    ? `<div class="status-banner status-banner-loading">Preparing recording. The page is up now, and the full bag is still being indexed in the background.</div>`
    : state.startup.state === "error"
      ? `<div class="status-banner status-banner-error">Recording failed to initialize: ${state.startup.error ?? "Unknown error"}</div>`
      : "";

  container.innerHTML = `
    <div class="sidebar-shell">
      <section class="sidebar-hero">
        <div class="hero-kicker">Projection Workbench</div>
        <h1>Parameters</h1>
        <p>Use the Rerun time bar under the viewer as the single playback control. The right sidebar only handles bag source and calibration edits.</p>
        ${startupNotice}
      </section>

      <section class="sidebar-section">
        <div class="section-heading">
          <div>
            <div class="section-kicker">Viewer Timeline</div>
            <h2>Timeline</h2>
          </div>
        </div>
        <div class="empty-state">Use the built-in Rerun transport controls below the viewer to scrub, play, and pause the bag timeline.</div>
      </section>

      <section class="sidebar-section">
        <div class="section-heading">
          <div>
            <div class="section-kicker">Data Source</div>
            <h2>ROS Inputs</h2>
          </div>
        </div>
        <label>Bag Path<input id="bag_file" value="${state.draftSource.bag_file}" /></label>
        <label>YAML Path<input id="yaml_path" value="${state.draftSource.yaml_path}" /></label>
        <label>Semantic Image Topic<input id="image_topic" value="${state.draftSource.image_topic}" /></label>
        <label>Detection Overlay Topic<input id="overlay_image_topic" value="${state.draftSource.overlay_image_topic}" /></label>
        <label>Point Cloud Topic<input id="pointcloud_topic" value="${state.draftSource.pointcloud_topic}" /></label>
        <button id="applySource" class="button-primary" ${isBuilding ? "disabled" : ""}>Apply Source</button>
      </section>

      <section class="sidebar-section">
        <div class="section-heading">
          <div>
            <div class="section-kicker">Projection Params</div>
            <h2>Calibration</h2>
          </div>
        </div>

        <div class="param-group">
          <h3>Camera Matrix</h3>
          <div class="compact-grid">
            <label>Image Width<input id="image_width" type="number" value="${state.draftProjection.image_width}" /></label>
            <label>Image Height<input id="image_height" type="number" value="${state.draftProjection.image_height}" /></label>
          </div>
          <label>Intrinsic Matrix<textarea id="camera_matrix">${matrixToMultiline(state.draftProjection.camera_matrix)}</textarea></label>
        </div>

        <div class="param-group">
          <h3>Distortion</h3>
          <div class="compact-grid">
            <label>Min Depth<input id="min_depth" type="number" step="0.01" value="${state.draftProjection.min_depth}" /></label>
            <label>Coefficients<input id="distortion_coeffs" value="${state.draftProjection.distortion_coeffs.join(", ")}" /></label>
          </div>
        </div>

        <div class="param-group">
          <h3>LiDAR To Camera</h3>
          <label>Extrinsic Matrix<textarea id="lidar_to_camera">${matrixToMultiline(state.draftProjection.lidar_to_camera)}</textarea></label>
        </div>

        <button id="applyProjection" class="button-primary" ${isBuilding ? "disabled" : ""}>Apply Projection</button>
      </section>

      <section class="sidebar-section">
        <div class="section-heading">
          <div>
            <div class="section-kicker">Locked Pairs</div>
            <h2>Comparisons</h2>
          </div>
          <div class="section-meta">${state.lockedPairs.length} saved</div>
        </div>
        <div class="button-row">
          <button id="lockPair" class="button-primary" ${canLock && !isBuilding ? "" : "disabled"}>Lock Pair</button>
          <button id="deleteLastPair" class="button-secondary" ${isBuilding ? "disabled" : ""}>Delete Last</button>
          <button id="clearPairs" class="button-danger" ${isBuilding ? "disabled" : ""}>Clear All</button>
        </div>
        <div class="pairs-list">${renderLockedPairs(state)}</div>
      </section>
    </div>
  `;

  const readSourceDraft = (): SourceDraft => ({
    bag_file: (container.querySelector("#bag_file") as HTMLInputElement).value,
    yaml_path: (container.querySelector("#yaml_path") as HTMLInputElement).value,
    image_topic: (container.querySelector("#image_topic") as HTMLInputElement).value,
    overlay_image_topic: (container.querySelector("#overlay_image_topic") as HTMLInputElement).value,
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

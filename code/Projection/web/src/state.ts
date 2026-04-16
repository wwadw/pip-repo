export type SourceDraft = {
  bag_file: string;
  yaml_path: string;
  image_topic: string;
  overlay_image_topic: string;
  pointcloud_topic: string;
};

export type ProjectionDraft = {
  image_width: number;
  image_height: number;
  camera_matrix: number[][];
  distortion_coeffs: number[];
  lidar_to_camera: number[][];
  min_depth: number;
};

export type CurrentFrame = {
  frame_index: number;
  image_stamp: number;
  cloud_stamp: number;
  overlay_stamp: number | null;
  image_count: number;
  cloud_count: number;
  visible_points: number;
  total_points: number;
};

export type StartupStatus = {
  state: "idle" | "building" | "ready" | "error";
  error: string | null;
};

export type WorkbenchState = {
  draftSource: SourceDraft;
  draftProjection: ProjectionDraft;
  currentFrame: CurrentFrame | null;
  currentSelection: unknown;
  lockedPairs: unknown[];
  rerunGrpcUrl: string;
  startup: StartupStatus;
};

export function createDraftState(): WorkbenchState {
  return {
    draftSource: {
      bag_file: "",
      yaml_path: "",
      image_topic: "",
      overlay_image_topic: "",
      pointcloud_topic: ""
    },
    draftProjection: {
      image_width: 0,
      image_height: 0,
      camera_matrix: [],
      distortion_coeffs: [],
      lidar_to_camera: [],
      min_depth: 0.05
    },
    currentFrame: null,
    currentSelection: null,
    lockedPairs: [],
    rerunGrpcUrl: "",
    startup: {
      state: "idle",
      error: null
    }
  };
}

export function applyBootstrap(state: WorkbenchState, payload: any): void {
  state.draftSource = {
    bag_file: payload.config.bag_file,
    yaml_path: payload.config.yaml_path ?? "",
    image_topic: payload.config.image_topic,
    overlay_image_topic: payload.config.overlay_image_topic,
    pointcloud_topic: payload.config.pointcloud_topic
  };
  state.draftProjection = {
    image_width: payload.config.image_width,
    image_height: payload.config.image_height,
    camera_matrix: payload.config.camera_matrix,
    distortion_coeffs: payload.config.distortion_coeffs,
    lidar_to_camera: payload.config.lidar_to_camera,
    min_depth: payload.config.min_depth
  };
  state.currentFrame = payload.current_frame ?? null;
  state.currentSelection = payload.current_selection ?? null;
  state.lockedPairs = payload.locked_pairs ?? [];
  state.rerunGrpcUrl = payload.rerun_grpc_url ?? "";
  state.startup = payload.startup ?? { state: "ready", error: null };
}

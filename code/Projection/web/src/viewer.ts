import { WebViewer, type RecordingOpenEvent, type SelectionChangeEvent, type TimeUpdateEvent } from "@rerun-io/web-viewer";

type ViewerApi = {
  select3d: (payload: unknown) => Promise<unknown>;
};

type SelectionItem = {
  type?: string;
  entity_path?: string;
  instance_id?: number | null;
  position?: number[] | null;
};

export type ViewerCallbacks = {
  initialFrameIndex: number;
  getFrameIndex: () => number;
  onTimeUpdate: (frameIndex: number) => void;
  onPause: (frameIndex: number) => Promise<void>;
  ensureFrameSynced: (frameIndex: number) => Promise<void>;
};

export type ViewerHandle = {
  stop: () => void;
};

export function selectionItemKey(item: SelectionItem): string | null {
  const selectablePaths = new Set([
    "world/ego_vehicle/lidar",
    "world/ego_vehicle/overlay_camera/projected_points"
  ]);
  if (item.type !== "entity" || !item.entity_path || !selectablePaths.has(item.entity_path)) {
    return null;
  }
  const instance = item.instance_id ?? "none";
  const position = item.position?.join(",") ?? "none";
  return `${item.entity_path}:${instance}:${position}`;
}

export async function mountViewer(
  container: HTMLElement,
  grpcUrl: string,
  onSelection: (payload: any) => void,
  api: ViewerApi,
  callbacks: ViewerCallbacks
): Promise<ViewerHandle> {
  const viewer = new WebViewer();
  await viewer.start(grpcUrl, container, {
    hide_welcome_screen: true,
    width: "100%",
    height: "100%"
  });

  let lastSelectionKey: string | null = null;
  let activeRecordingId: string | null = null;

  viewer.on("recording_open", (event: RecordingOpenEvent) => {
    activeRecordingId = event.recording_id;
    viewer.set_active_timeline(event.recording_id, "frame");
    viewer.set_current_time(event.recording_id, "frame", callbacks.initialFrameIndex);
  });

  viewer.on("time_update", (event: TimeUpdateEvent) => {
    callbacks.onTimeUpdate(Math.max(0, Math.round(event.time)));
  });

  viewer.on("pause", async () => {
    await callbacks.onPause(callbacks.getFrameIndex());
  });

  viewer.on("selection_change", async (event: SelectionChangeEvent) => {
    const item = event.items[0];
    if (!item) {
      return;
    }
    const key = selectionItemKey(item);
    if (key === null || key === lastSelectionKey) {
      return;
    }
    lastSelectionKey = key;
    const frameIndex = callbacks.getFrameIndex();
    await callbacks.ensureFrameSynced(frameIndex);
    const payload = await api.select3d({
      frame_index: frameIndex,
      entity_path: item.entity_path,
      instance_id: item.instance_id ?? null,
      position: item.position ?? null
    });
    onSelection(payload);
  });

  return {
    stop: () => {
      if (activeRecordingId !== null) {
        try {
          viewer.close(grpcUrl);
        } catch {
          // Ignore close errors during teardown.
        }
      }
      viewer.stop();
    }
  };
}

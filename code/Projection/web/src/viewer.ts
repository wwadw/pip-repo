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

type LayerPointArgs = {
  x: number;
  y: number;
  layerWidth: number;
  layerHeight: number;
  imageWidth: number;
  imageHeight: number;
};

type ImageClickLayerBindings = {
  getImageSize: () => { imageWidth: number; imageHeight: number };
  submit: (pixel: [number, number]) => Promise<void>;
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

export function mapLayerPointToImagePixel(args: LayerPointArgs): [number, number] | null {
  if (args.layerWidth <= 0 || args.layerHeight <= 0 || args.imageWidth <= 0 || args.imageHeight <= 0) {
    return null;
  }

  const scale = Math.min(args.layerWidth / args.imageWidth, args.layerHeight / args.imageHeight);
  if (scale <= 0) {
    return null;
  }

  const renderedWidth = args.imageWidth * scale;
  const renderedHeight = args.imageHeight * scale;
  const offsetX = (args.layerWidth - renderedWidth) / 2;
  const offsetY = (args.layerHeight - renderedHeight) / 2;

  if (
    args.x < offsetX ||
    args.x > offsetX + renderedWidth ||
    args.y < offsetY ||
    args.y > offsetY + renderedHeight
  ) {
    return null;
  }

  const imageX = Math.round((args.x - offsetX) / scale);
  const imageY = Math.round((args.y - offsetY) / scale);
  return [
    Math.max(0, Math.min(args.imageWidth - 1, imageX)),
    Math.max(0, Math.min(args.imageHeight - 1, imageY))
  ];
}

export function bindImageClickLayer(element: HTMLElement, bindings: ImageClickLayerBindings): () => void {
  const handleClick = (event: MouseEvent) => {
    const rect = element.getBoundingClientRect();
    const { imageWidth, imageHeight } = bindings.getImageSize();
    const pixel = mapLayerPointToImagePixel({
      x: event.clientX - rect.left,
      y: event.clientY - rect.top,
      layerWidth: rect.width,
      layerHeight: rect.height,
      imageWidth,
      imageHeight
    });
    if (pixel === null) {
      return;
    }
    void bindings.submit(pixel);
  };

  element.addEventListener("click", handleClick);
  return () => {
    element.removeEventListener("click", handleClick);
  };
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

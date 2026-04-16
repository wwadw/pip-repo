import { WebViewer, type SelectionChangeEvent } from "@rerun-io/web-viewer";

type ViewerApi = {
  select2d: (payload: unknown) => Promise<unknown>;
  select3d: (payload: unknown) => Promise<unknown>;
};

export async function mountViewer(
  container: HTMLElement,
  grpcUrl: string,
  onSelection: (payload: any) => void,
  api: ViewerApi
) {
  const viewer = new WebViewer();
  await viewer.start(grpcUrl, container, {
    hide_welcome_screen: true,
    width: "100%",
    height: "100%"
  });

  viewer.on("selection_change", async (event: SelectionChangeEvent) => {
    const item = event.items[0];
    if (!item || item.type !== "entity" || item.entity_path !== "world/points") {
      return;
    }
    const payload = await api.select3d({
      frame_index: 0,
      entity_path: item.entity_path,
      instance_id: item.instance_id ?? null,
      position: item.position ?? null
    });
    onSelection(payload);
  });

  return viewer;
}

export function bindImageClickLayer(element: HTMLElement, submit: (pixel: [number, number]) => Promise<void>) {
  element.addEventListener("click", (event: MouseEvent) => {
    const rect = element.getBoundingClientRect();
    const x = event.clientX - rect.left;
    const y = event.clientY - rect.top;
    void submit([x, y]);
  });
}

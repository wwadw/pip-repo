import * as api from "./api";
import { renderForms } from "./forms";
import { applyBootstrap, createDraftState } from "./state";
import "./styles.css";
import { mountViewer, type ViewerHandle } from "./viewer";

const state = createDraftState();
let viewerHandle: ViewerHandle | null = null;
let activeGrpcUrl = "";
let viewerFrameIndex = 0;
let syncedFrameIndex = 0;

async function syncFrame(frameIndex: number): Promise<any | null> {
  if (frameIndex === syncedFrameIndex) {
    return null;
  }
  const payload = await api.setFrame(frameIndex);
  syncedFrameIndex = frameIndex;
  return payload;
}

async function remountViewer(grpcUrl: string, frameIndex: number): Promise<void> {
  viewerHandle?.stop();
  viewerHandle = await mountViewer(
    document.querySelector("#rerunViewer") as HTMLElement,
    grpcUrl,
    rerender,
    api,
    {
      initialFrameIndex: frameIndex,
      getFrameIndex: () => viewerFrameIndex,
      onTimeUpdate: (frame) => {
        viewerFrameIndex = frame;
      },
      onPause: async (frame) => {
        viewerFrameIndex = frame;
        const payload = await syncFrame(frame);
        if (payload) {
          await applyPayload(payload);
        }
      },
      ensureFrameSynced: async (frame) => {
        viewerFrameIndex = frame;
        const payload = await syncFrame(frame);
        if (payload) {
          await applyPayload(payload);
        }
      }
    }
  );
}

function renderSidebar(): void {
  renderForms(document.querySelector("#sidebar") as HTMLElement, state, api, rerender);
}

async function applyPayload(payload: any): Promise<void> {
  const nextFrameIndex = payload.current_frame?.frame_index ?? viewerFrameIndex;
  applyBootstrap(state, payload);
  syncedFrameIndex = state.currentFrame?.frame_index ?? syncedFrameIndex;
  viewerFrameIndex = nextFrameIndex;

  const nextGrpcUrl = state.rerunGrpcUrl;
  if (nextGrpcUrl && nextGrpcUrl !== activeGrpcUrl) {
    activeGrpcUrl = nextGrpcUrl;
    await remountViewer(nextGrpcUrl, viewerFrameIndex);
  }

  renderSidebar();
}

function rerender(payload: any): void {
  void applyPayload(payload);
}

async function bootstrap() {
  const payload = await api.fetchBootstrap();
  await applyPayload(payload);
}

void bootstrap();

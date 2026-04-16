import * as api from "./api";
import { renderForms } from "./forms";
import { applyBootstrap, createDraftState } from "./state";
import "./styles.css";
import { bindImageClickLayer, mountViewer } from "./viewer";

const state = createDraftState();

function rerender(payload: any) {
  applyBootstrap(state, payload);
  renderForms(document.querySelector("#sidebar") as HTMLElement, state, api, rerender);
}

async function bootstrap() {
  const payload = await api.fetchBootstrap();
  rerender(payload);

  if (state.rerunGrpcUrl3d || state.rerunGrpcUrl) {
    await mountViewer(document.querySelector("#viewer3d") as HTMLElement, state.rerunGrpcUrl3d || state.rerunGrpcUrl, rerender, api);
  }
  if (state.rerunGrpcUrl2d || state.rerunGrpcUrl) {
    await mountViewer(document.querySelector("#viewer2d") as HTMLElement, state.rerunGrpcUrl2d || state.rerunGrpcUrl, rerender, api);
  }

  bindImageClickLayer(document.querySelector("#imageClickLayer") as HTMLElement, async (pixel) => {
    const response = await api.select2d({ frame_index: 0, pixel });
    rerender(response);
  });
}

void bootstrap();

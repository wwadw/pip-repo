export async function fetchBootstrap() {
  const response = await fetch("/api/bootstrap");
  return response.json();
}

export async function applySource(payload: unknown) {
  const response = await fetch("/api/config/source", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload)
  });
  return response.json();
}

export async function applyProjection(payload: unknown) {
  const response = await fetch("/api/config/projection", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload)
  });
  return response.json();
}

export async function select2d(payload: unknown) {
  const response = await fetch("/api/select/2d", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload)
  });
  return response.json();
}

export async function select3d(payload: unknown) {
  const response = await fetch("/api/select/3d", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(payload)
  });
  return response.json();
}

export async function lockPair() {
  const response = await fetch("/api/pairs/lock", { method: "POST" });
  return response.json();
}

export async function deleteLastPair() {
  const response = await fetch("/api/pairs/delete-last", { method: "POST" });
  return response.json();
}

export async function clearPairs() {
  const response = await fetch("/api/pairs/clear", { method: "POST" });
  return response.json();
}

export async function nextFrame() {
  const response = await fetch("/api/frame/next", { method: "POST" });
  return response.json();
}

export async function prevFrame() {
  const response = await fetch("/api/frame/prev", { method: "POST" });
  return response.json();
}

export async function setFrame(frameIndex: number) {
  const response = await fetch("/api/frame/set", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ frame_index: frameIndex })
  });
  return response.json();
}

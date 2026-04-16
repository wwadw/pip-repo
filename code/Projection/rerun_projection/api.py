from typing import Any, Dict

from fastapi import APIRouter
from pydantic import BaseModel


class ProjectionConfigPayload(BaseModel):
    image_width: int
    image_height: int
    camera_matrix: list
    distortion_coeffs: list
    lidar_to_camera: list
    min_depth: float


class SourceConfigPayload(BaseModel):
    bag_file: str
    yaml_path: str = ""
    image_topic: str
    overlay_image_topic: str
    pointcloud_topic: str


class Select2DPayload(BaseModel):
    frame_index: int
    pixel: list[float]


class Select3DPayload(BaseModel):
    frame_index: int
    entity_path: str
    instance_id: int | None = None
    position: list[float] | None = None


class FrameIndexPayload(BaseModel):
    frame_index: int


def _model_dump(model: BaseModel) -> Dict[str, Any]:
    return model.model_dump()


def build_router(runtime) -> APIRouter:
    router = APIRouter()

    @router.get("/api/bootstrap")
    def bootstrap():
        if getattr(runtime, "startup_state", None) == "idle" and not getattr(runtime, "test_mode", False):
            runtime.reload_source()
        return runtime.bootstrap_payload()

    @router.post("/api/config/projection")
    def apply_projection(payload: ProjectionConfigPayload):
        runtime.apply_projection(_model_dump(payload))
        return runtime.bootstrap_payload()

    @router.post("/api/config/source")
    def apply_source(payload: SourceConfigPayload):
        runtime.apply_source(_model_dump(payload))
        return runtime.bootstrap_payload()

    @router.post("/api/select/2d")
    def select_2d(payload: Select2DPayload):
        runtime.select_2d(_model_dump(payload))
        return runtime.bootstrap_payload()

    @router.post("/api/select/3d")
    def select_3d(payload: Select3DPayload):
        runtime.select_3d(_model_dump(payload))
        return runtime.bootstrap_payload()

    @router.post("/api/pairs/lock")
    def lock_pair():
        runtime.lock_current_pair()
        return runtime.bootstrap_payload()

    @router.post("/api/pairs/delete-last")
    def delete_last_pair():
        runtime.delete_last_pair()
        return runtime.bootstrap_payload()

    @router.post("/api/pairs/clear")
    def clear_pairs():
        runtime.clear_pairs()
        return runtime.bootstrap_payload()

    @router.post("/api/frame/next")
    def next_frame():
        runtime.next_frame()
        return runtime.bootstrap_payload()

    @router.post("/api/frame/prev")
    def prev_frame():
        runtime.prev_frame()
        return runtime.bootstrap_payload()

    @router.post("/api/frame/set")
    def set_frame(payload: FrameIndexPayload):
        runtime.set_frame(payload.frame_index)
        return runtime.bootstrap_payload()

    return router

from pathlib import Path

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles

from rerun_projection.api import build_router
from rerun_projection.runtime import build_runtime


def resolve_frontend_dist() -> Path:
    packaged_dist = Path(__file__).resolve().parent / "web_dist"
    if packaged_dist.exists():
        return packaged_dist
    return Path(__file__).resolve().parents[1] / "web" / "dist"


def create_app(test_mode: bool = False, runtime=None) -> FastAPI:
    app = FastAPI(title="Rerun Projection Workbench")
    runtime = runtime or build_runtime(test_mode=test_mode)
    app.include_router(build_router(runtime))

    frontend_dist = resolve_frontend_dist()
    if frontend_dist.exists():
        app.mount("/", StaticFiles(directory=str(frontend_dist), html=True), name="web")
    return app

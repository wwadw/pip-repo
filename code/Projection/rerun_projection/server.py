from pathlib import Path

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles

from rerun_projection.api import build_router
from rerun_projection.runtime import build_runtime


def create_app(test_mode: bool = False, runtime=None) -> FastAPI:
    app = FastAPI(title="Rerun Projection Workbench")
    runtime = runtime or build_runtime(test_mode=test_mode)
    app.include_router(build_router(runtime))

    frontend_dist = Path(__file__).resolve().parents[1] / "web" / "dist"
    if frontend_dist.exists():
        app.mount("/", StaticFiles(directory=str(frontend_dist), html=True), name="web")
    return app

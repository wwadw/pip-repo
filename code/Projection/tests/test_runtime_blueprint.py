from types import SimpleNamespace

import rerun_projection.runtime as runtime_module


class _FakeBlueprintNode:
    def __init__(self, *children, **kwargs):
        self.children = list(children)
        self.kwargs = kwargs


class _FakeBlueprint(_FakeBlueprintNode):
    pass


class _FakeVertical(_FakeBlueprintNode):
    pass


class _FakeGrid(_FakeBlueprintNode):
    pass


class _FakeSpatial3DView(_FakeBlueprintNode):
    pass


class _FakeSpatial2DView(_FakeBlueprintNode):
    pass


class _FakePinhole:
    def __init__(self, **kwargs):
        self.kwargs = kwargs

    @classmethod
    def from_fields(cls, **kwargs):
        return cls(**kwargs)


class _FakeBlueprintModule:
    Blueprint = _FakeBlueprint
    Vertical = _FakeVertical
    Grid = _FakeGrid
    Spatial3DView = _FakeSpatial3DView
    Spatial2DView = _FakeSpatial2DView


def test_workbench_blueprint_uses_camera_entities_as_2d_origins(monkeypatch):
    fake_rerun = SimpleNamespace(blueprint=_FakeBlueprintModule, Pinhole=_FakePinhole)
    monkeypatch.setitem(__import__("sys").modules, "rerun", fake_rerun)

    blueprint = runtime_module._workbench_blueprint()

    layout = blueprint.children[0]
    top_view = layout.children[0]
    camera_grid = layout.children[1]
    semantic_view, overlay_view = camera_grid.children

    assert "world/ego_vehicle/overlay_camera" in top_view.kwargs["contents"]
    assert "world/ego_vehicle/semantic_camera" not in top_view.kwargs["contents"]
    assert "world/ego_vehicle/semantic_camera_rig" not in top_view.kwargs["contents"]
    assert len(top_view.kwargs["defaults"]) == 1
    assert isinstance(top_view.kwargs["defaults"][0], _FakePinhole)
    assert top_view.kwargs["defaults"][0].kwargs["image_plane_distance"] == 1.5
    assert semantic_view.kwargs["origin"] == "world/ego_vehicle/semantic_camera"
    assert semantic_view.kwargs["contents"] == [
        "$origin/**",
        "selection/semantic_camera/**",
        "pairs/semantic_camera/**",
    ]
    assert overlay_view.kwargs["origin"] == "world/ego_vehicle/overlay_camera"
    assert overlay_view.kwargs["contents"] == [
        "$origin/**",
        "selection/overlay_camera/**",
        "pairs/overlay_camera/**",
    ]

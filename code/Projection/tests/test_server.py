from pathlib import Path

from rerun_projection import server


def test_frontend_dist_prefers_packaged_web_dist(monkeypatch, tmp_path):
    package_dir = tmp_path / "rerun_projection"
    package_dir.mkdir()
    packaged_dist = package_dir / "web_dist"
    packaged_dist.mkdir()
    monkeypatch.setattr(server, "__file__", str(package_dir / "server.py"))

    assert server.resolve_frontend_dist() == packaged_dist


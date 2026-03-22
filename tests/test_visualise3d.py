"""Tests for 3D visualisation helpers."""

from car_solver.visualise3d import make_iteration_snapshot_callback


def test_make_iteration_snapshot_callback_captures_configured_frames(monkeypatch, tmp_path):
    calls = []

    def fake_plot(densities, nelx, nely, nelz, save_path=None, threshold=0.3):
        calls.append((nelx, nely, nelz, save_path, threshold, tuple(densities)))

    monkeypatch.setattr("car_solver.visualise3d.plot_density_3d", fake_plot)

    callback, paths = make_iteration_snapshot_callback(
        nelx=4,
        nely=3,
        nelz=2,
        filename_prefix="frame",
        every=2,
        threshold=0.25,
        output_dir=str(tmp_path),
    )

    callback(0, [0.1, 0.2], 10.0, 1.0)
    callback(1, [0.3, 0.4], 9.0, 0.5)
    callback(2, [0.5, 0.6], 8.0, 0.25)

    assert len(paths) == 2
    assert paths[0].endswith("frame_0000.png")
    assert paths[1].endswith("frame_0002.png")
    assert len(calls) == 2
    assert calls[0][0:3] == (4, 3, 2)
    assert calls[0][4] == 0.25


def test_make_iteration_snapshot_callback_calls_base_callback(monkeypatch, tmp_path):
    seen = []

    def fake_plot(densities, nelx, nely, nelz, save_path=None, threshold=0.3):
        return None

    def base_callback(iteration, densities, compliance, change):
        seen.append((iteration, compliance, change))

    monkeypatch.setattr("car_solver.visualise3d.plot_density_3d", fake_plot)

    callback, _ = make_iteration_snapshot_callback(
        nelx=2,
        nely=2,
        nelz=2,
        filename_prefix="frame",
        every=3,
        output_dir=str(tmp_path),
        base_callback=base_callback,
    )

    callback(1, [0.1], 5.0, 0.2)

    assert seen == [(1, 5.0, 0.2)]

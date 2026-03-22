"""Tests for thresholded export helpers."""

import sys
import types

import numpy as np

from car_solver.export import (
    export_thresholded_density_stl,
    export_thresholded_density_vtk,
)


class _FakeSurface:
    def __init__(self, store):
        self.store = store

    def save(self, path):
        self.store.append(("surface", path))


class _FakeThresholded:
    def __init__(self, store):
        self.store = store

    def save(self, path):
        self.store.append(("threshold", path))

    def extract_surface(self):
        self.store.append(("extract_surface", None))
        return _FakeSurface(self.store)


class _FakeImageData:
    def __init__(self, dimensions, store):
        self.dimensions = dimensions
        self.cell_data = {}
        self.store = store

    def threshold(self, value, scalars):
        self.store.append(("threshold_call", self.dimensions, value, scalars))
        self.store.append(("cell_data", dict(self.cell_data)))
        return _FakeThresholded(self.store)


def test_export_thresholded_density_vtk_uses_thresholded_grid(monkeypatch, tmp_path):
    store = []

    def image_data(*, dimensions):
        return _FakeImageData(dimensions, store)

    monkeypatch.setitem(sys.modules, "pyvista", types.SimpleNamespace(ImageData=image_data))

    path = export_thresholded_density_vtk(
        np.array([0.2, 0.9]),
        nelx=1,
        nely=1,
        nelz=2,
        save_path=str(tmp_path / "density.vtk"),
        threshold=0.4,
    )

    assert path == str(tmp_path / "density.vtk")
    assert ("threshold_call", (2, 2, 3), 0.4, "density") in store
    assert ("threshold", str(tmp_path / "density.vtk")) in store


def test_export_thresholded_density_stl_extracts_surface(monkeypatch, tmp_path):
    store = []

    def image_data(*, dimensions):
        return _FakeImageData(dimensions, store)

    monkeypatch.setitem(sys.modules, "pyvista", types.SimpleNamespace(ImageData=image_data))

    path = export_thresholded_density_stl(
        np.array([0.1, 0.8]),
        nelx=1,
        nely=1,
        nelz=2,
        save_path=str(tmp_path / "density.stl"),
        threshold=0.5,
    )

    assert path == str(tmp_path / "density.stl")
    assert ("extract_surface", None) in store
    assert ("surface", str(tmp_path / "density.stl")) in store

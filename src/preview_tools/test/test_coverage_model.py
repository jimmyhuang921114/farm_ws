import math

import pytest

from preview_tools.coverage_model import CoverageModel


def test_unique_cells_distance_and_area():
    model = CoverageModel(0.5)
    assert model.add(0.1, 0.1)
    assert not model.add(0.2, 0.2)
    assert model.add(0.6, 0.1)
    assert model.pose_count == 3
    assert len(model.cells) == 2
    assert model.area == pytest.approx(0.5)
    assert model.distance == pytest.approx(
        math.hypot(0.1, 0.1) + math.hypot(0.4, -0.1))


def test_nonfinite_and_reset():
    model = CoverageModel(1.0)
    assert not model.add(float('nan'), 0.0)
    assert model.pose_count == 0
    assert model.add(-0.1, -0.1)
    assert (-1, -1) in model.cells
    model.reset()
    assert model.pose_count == 0
    assert model.area == 0.0
    assert model.last_position is None


def test_cell_size_must_be_positive():
    with pytest.raises(ValueError):
        CoverageModel(0.0)

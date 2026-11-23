"""Vertical surface heights must come only from triangles actually under the point."""
import pytest

from isaac.surface import box_triangles, fan_triangles, heights_at

FLOOR = ((-1.0, -1.0, 0.0), (1.0, -1.0, 0.0), (1.0, 1.0, 0.0)), ((-1.0, -1.0, 0.0), (1.0, 1.0, 0.0), (-1.0, 1.0, 0.0))
SLOPE = (((0.0, -1.0, 2.0), (2.0, -1.0, 4.0), (2.0, 1.0, 4.0)),)


def test_fan_triangulation_matches_face_counts():
    assert list(fan_triangles([3, 4], [0, 1, 2, 3, 4, 5, 6])) == [(0, 1, 2), (3, 4, 5), (3, 5, 6)]
    assert list(fan_triangles([], [])) == []


def test_highest_surface_first_and_misses_are_empty():
    assert heights_at(0.75, -0.5, FLOOR + SLOPE) == pytest.approx([2.75, 0.0])
    assert heights_at(1.5, 0.0, FLOOR + SLOPE) == pytest.approx([3.5])
    assert heights_at(-0.5, 0.0, FLOOR + SLOPE) == [0.0]
    assert heights_at(3.0, 0.0, FLOOR + SLOPE) == []


def test_shared_edge_and_vertical_triangles():
    assert heights_at(0.0, 0.0, FLOOR) == [0.0, 0.0]
    wall = (((0.0, 0.0, 0.0), (0.0, 0.0, 5.0), (1.0, 0.0, 5.0)),)
    assert heights_at(0.5, 0.0, wall) == []
    degenerate = (((0.0, 0.0, 1.0), (0.0, 0.0, 1.0), (0.0, 0.0, 1.0)),)
    assert heights_at(0.0, 0.0, degenerate) == []


def test_box_top_and_bottom():
    box = box_triangles((10.0, -2.0, 1.0), (4.0, 2.0, 2.0))
    assert len(box) == 12
    assert heights_at(10.0, -2.0, box) == pytest.approx([2.0, 2.0, 0.0, 0.0])
    assert heights_at(12.5, -2.0, box) == []

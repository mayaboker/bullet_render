"""Camera-axis migration must preserve the Bullet body convention."""
import math

import numpy as np
import pytest

from isaac.camera import usd_camera_orientation
from isaac.runtime import isaac_directory


def rotate(q, v):
    w, x, y, z = q
    vector = np.array([x, y, z])
    return v + 2 * np.cross(vector, np.cross(vector, v) + w * v)


@pytest.mark.parametrize('body', [(1, 0, 0, 0),
                                  (math.sqrt(.5), 0, 0, math.sqrt(.5)),
                                  (math.sqrt(.5), 0, 0, -math.sqrt(.5)),
                                  (0, 0, 0, 1),
                                  (math.sqrt(.5), math.sqrt(.5), 0, 0),
                                  (math.sqrt(.5), 0, math.sqrt(.5), 0),
                                  (.5, .5, -.5, .5)])
def test_usd_camera_axes_match_world_body_axes(body):
    camera = usd_camera_orientation(body)
    for camera_axis, body_axis in [([0, 0, -1], [1, 0, 0]),
                                   ([1, 0, 0], [0, -1, 0]),
                                   ([0, 1, 0], [0, 0, 1])]:
        np.testing.assert_allclose(rotate(camera, np.array(camera_axis)),
                                   rotate(body, np.array(body_axis)), atol=1e-14)


def test_runtime_respects_sdk_environment(monkeypatch, tmp_path):
    monkeypatch.setenv('ISAAC_SIM_DIR', str(tmp_path))
    assert isaac_directory() == tmp_path
    monkeypatch.delenv('ISAAC_SIM_DIR')
    assert isaac_directory().name == 'isaacsim6.2'

"""Geometry and encoding checks without a camera or GPU."""
import math
from types import SimpleNamespace

import numpy as np
from std_msgs.msg import Header
from camera.camera import CameraNode, image_message


def test_image_encoding_and_stamp():
    header = Header(frame_id='camera_optical_frame')
    header.stamp.sec = 12
    data = np.arange(24, dtype=np.float32).reshape(4, 6)[:, ::2]
    msg = image_message(data, '32FC1', header)
    assert (msg.width, msg.height, msg.step) == (3, 4, 12)
    assert msg.header == header
    assert np.array_equal(np.frombuffer(msg.data, dtype=np.float32).reshape(4, 3), data)


def test_calibration_resized_and_rectified():
    node = SimpleNamespace(frame_id='camera_optical_frame', calibration={
        'image_width': 1280, 'image_height': 960,
        'camera_matrix': {'data': [1000., 0., 640., 0., 1000., 480., 0., 0., 1.]},
        'distortion_coefficients': {'data': [0.1, 0., 0., 0., 0.]},
    })
    info = CameraNode.prepare_info(node, 640, 480)
    assert np.array_equal(info.k, [500., 0., 320., 0., 500., 240., 0., 0., 1.])
    assert np.array_equal(info.d, [0.] * 5)
    assert node.maps[0].shape == (480, 640)
    assert node.intrinsics == (500., 500., 320., 240.)


def test_no_calibration_does_not_invent_intrinsics():
    node = SimpleNamespace(frame_id='camera_optical_frame', calibration=None,
                           approximate_horizontal_fov_deg=0.0)
    info = CameraNode.prepare_info(node, 640, 480)
    assert info.k[0] == 0
    assert node.intrinsics is None


def test_approximate_camera_info_matches_image_geometry():
    node = SimpleNamespace(frame_id='camera_optical_frame', calibration=None,
                           approximate_horizontal_fov_deg=70.0)
    info = CameraNode.prepare_info(node, 640, 480)
    focal = 640 / (2 * math.tan(math.radians(35)))
    assert (info.width, info.height) == (640, 480)
    assert np.allclose(info.k, [focal, 0, 320, 0, focal, 240, 0, 0, 1])
    assert np.allclose(info.p, [focal, 0, 320, 0, 0, focal, 240, 0, 0, 0, 1, 0])
    assert node.intrinsics == (focal, focal, 320, 240)

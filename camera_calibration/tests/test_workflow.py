"""Offline logic tests; no physical camera calibration or persistent result."""
import contextlib
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import cv2
import numpy as np
import yaml

from camera_utils import PROJECT_DIR,ChessboardDetector,load_config,load_calibration,make_object_points,open_camera
from calibrate_camera import calibrate_samples,save_raw_frame


class WorkflowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        (PROJECT_DIR/'.tmp').mkdir(exist_ok=True)

    def test_object_points_mm_and_inner_corner_count(self):
        cfg=load_config()
        points=make_object_points(cfg['chessboard'])
        self.assertEqual(points.shape,(54,3))
        np.testing.assert_array_equal(points[1],[25,0,0])
        np.testing.assert_array_equal(points[9],[0,25,0])

    def test_sb_and_classic_detect_generated_board(self):
        board={'cols':9,'rows':6,'square_size_mm':25}
        gray=np.full((360,480),255,np.uint8)
        for row in range(7):
            for col in range(10):
                gray[40+row*40:40+(row+1)*40,40+col*40:40+(col+1)*40]=255 if (row+col)%2 else 0
        detector=ChessboardDetector(board)
        for use_sb in [True,False]:
            detector.use_sb=use_sb and hasattr(cv2,'findChessboardCornersSB')
            found,corners=detector.detect(gray)
            self.assertTrue(found)
            self.assertEqual(corners.shape,(54,1,2))

    def test_raw_save_never_overwrites_or_mutates_frame(self):
        with tempfile.TemporaryDirectory(dir=PROJECT_DIR/'.tmp') as tmp:
            raw=np.full((80,100,3),120,np.uint8)
            original=raw.copy()
            first=save_raw_frame(tmp,raw)
            first_bytes=first.read_bytes()
            second=save_raw_frame(tmp,raw)
            self.assertEqual(first.name,'000001.jpg')
            self.assertEqual(second.name,'000002.jpg')
            self.assertEqual(first.read_bytes(),first_bytes)
            np.testing.assert_array_equal(raw,original)
            self.assertEqual(cv2.imread(str(first)).shape,raw.shape)

    def test_mean_error_is_per_corner_distance_not_norm_divided_by_count(self):
        cfg=load_config()
        points=[np.zeros((54,1,2),np.float32) for _ in range(3)]
        k=np.array([[1000,0,960],[0,1000,540],[0,0,1]],np.float64)
        poses=[np.zeros((3,1)) for _ in range(3)]
        projected=np.tile(np.array([3,4],np.float32),(54,1)).reshape(54,1,2)
        with patch('calibrate_camera.cv2.calibrateCamera',return_value=(5.,k,np.zeros((1,5)),poses,poses)), patch('calibrate_camera.cv2.projectPoints',return_value=(projected,None)), contextlib.redirect_stdout(io.StringIO()):
            result=calibrate_samples(cfg,points,['a.jpg','b.jpg','c.jpg'],(1920,1080))
        self.assertEqual(result['calibration']['mean_reprojection_error'],5.)
        self.assertEqual(result['distortion_coefficients']['cols'],5)
        # Test artifact exists only inside a temporary test directory, never at
        # output/camera_calibration.yaml; these are mocked, not real intrinsics.
        with tempfile.TemporaryDirectory(dir=PROJECT_DIR/'.tmp') as tmp:
            path=Path(tmp)/'mock_fixture.yaml'
            path.write_text(yaml.safe_dump(result))
            actual,_=load_calibration(path,(1920,1080))
            np.testing.assert_array_equal(actual,k)
            with self.assertRaisesRegex(ValueError,'resolution'):
                load_calibration(path,(640,480))

    def test_solver_accepts_normalized_points_synthetic_only(self):
        cfg=load_config()
        obj=make_object_points(cfg['chessboard'])
        k=np.array([[1000,0,960],[0,1000,540],[0,0,1]],np.float64)
        points=[]
        for i in range(8):
            r=np.array([-.2+.06*i,.15-.04*i,.02*i])
            t=np.array([-100.+10*i,-50.+5*i,600.+30*i])
            projected,_=cv2.projectPoints(obj,r,t,k,np.zeros(5))
            points.append(projected.reshape(-1,1,2).astype(np.float32))
        with contextlib.redirect_stdout(io.StringIO()):
            result=calibrate_samples(cfg,points,[f'synthetic_{i}' for i in range(8)],(1920,1080))
        self.assertLess(result['calibration']['mean_reprojection_error'],.01)
        self.assertAlmostEqual(result['intrinsics']['fx'],1000,delta=1)

    def test_no_camera_gives_clear_error(self):
        with patch('camera_utils.cv2.VideoCapture') as constructor:
            constructor.return_value.isOpened.return_value=False
            with self.assertRaisesRegex(RuntimeError,'Cannot open camera'):
                open_camera(load_config()['camera'])
            constructor.return_value.release.assert_called_once()


if __name__ == '__main__': unittest.main()

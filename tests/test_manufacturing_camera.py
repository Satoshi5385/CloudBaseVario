import time
import unittest
from unittest import mock

from manufacturing_tools.camera_qr import CameraDevice, CameraQRWorker


class FakeCapture:
    def __init__(self, opened=True):
        self.opened = opened
        self.released = False

    def isOpened(self):
        return self.opened

    def read(self):
        return False, None

    def release(self):
        self.released = True


class FakeCV2:
    CAP_MSMF = 1400
    CAP_DSHOW = 700

    def __init__(self):
        self.capture = FakeCapture()

    def VideoCapture(self, _index, _backend):
        return self.capture

    @staticmethod
    def QRCodeDetector():
        return object()


class ManufacturingCameraTests(unittest.TestCase):
    def test_worker_reports_repeated_capture_failure_and_releases(self) -> None:
        fake = FakeCV2()
        worker = CameraQRWorker(CameraDevice(0, fake.CAP_MSMF, "MSMF", 0, 0))
        with mock.patch(
            "manufacturing_tools.camera_qr._import_cv2", return_value=fake
        ):
            worker.start()
            deadline = time.monotonic() + 2.0
            result = None
            while result is None and time.monotonic() < deadline:
                result = worker.latest()
                time.sleep(0.02)
            worker.stop()
        self.assertIsNotNone(result)
        self.assertIn("取得できません", result.fatal_error)
        self.assertTrue(fake.capture.released)

    def test_stop_is_safe_before_start(self) -> None:
        worker = CameraQRWorker(CameraDevice(0, 1400, "MSMF", 0, 0))
        worker.stop()


if __name__ == "__main__":
    unittest.main()


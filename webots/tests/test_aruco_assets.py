"""Los marcadores renderizados deben ser detectables por el mismo diccionario."""

import unittest
from pathlib import Path

import cv2


ASSETS = Path(__file__).resolve().parents[1] / "assets" / "aruco"


class ArucoAssetsTest(unittest.TestCase):
    def test_each_texture_has_its_declared_id(self):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        detector = cv2.aruco.ArucoDetector(dictionary, cv2.aruco.DetectorParameters())
        for marker_id in (0, 1, 2, 3, 10, 11):
            image = cv2.imread(str(ASSETS / "id{}.png".format(marker_id)), cv2.IMREAD_GRAYSCALE)
            corners, ids, _ = detector.detectMarkers(image)
            self.assertEqual(len(corners), 1)
            self.assertEqual(ids.flatten().tolist(), [marker_id])


if __name__ == "__main__":
    unittest.main()

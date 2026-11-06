import cv2
import numpy as np
import pytest
from std_msgs.msg import Header

object_finder = pytest.importorskip('amr_perception.object_finder')


def pixels(h, s, v, n=50):
    return cv2.cvtColor(np.full((n, 1, 3), (h, s, v), np.uint8), cv2.COLOR_HSV2BGR).reshape(-1, 3)


def test_pale_bluish_surface_is_not_blue():
    # カメラの色味が青寄りで、白っぽいラグがこの値に写った。
    assert object_finder.color_name(pixels(105, 51, 186)) == 'white'
    assert object_finder.color_name(pixels(112, 230, 150)) == 'blue'


def test_mask_image_marks_object_pixels():
    mask = np.zeros((4, 6), bool)
    mask[1:3, 2:5] = True
    image = object_finder.mask_image(Header(frame_id='camera_color_0'), mask)
    assert (image.height, image.width, image.step, image.encoding) == (4, 6, 6, 'mono8')
    data = np.frombuffer(bytes(image.data), np.uint8).reshape(4, 6)
    assert np.array_equal(data == 255, mask) and set(np.unique(data)) == {0, 255}

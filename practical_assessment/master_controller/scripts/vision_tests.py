#!/usr/bin/env python
import numpy as np

from vision_node import DetectableObject

class TestDetectableObject():

    def test_hsv_thresh_1(self):
        '''
        Standard Case
        '''
        obj = DetectableObject('', 20, 255, 255, 10, 2, 2)
        
        input_img = np.array([
            [[20, 255, 255], [18, 254, 255]],
            [[11, 255, 255], [22, 230, 254]]
        ])
        assert input_img.shape == (2, 2, 3)
        result = obj.hsv_threshold(input_img)

        assert result.shape == (2, 2)
        assert result[0, 0] == 255
        assert result[0, 1] == 255
        assert result[1, 0] == 0
        assert result[1, 1] == 0

    def test_hsv_thresh_2(self):
        '''
        Case where we have to wrap around 180/0
        '''
        obj = DetectableObject('', 5, 255, 255, 20, 100, 100)
        
        input_img = np.array([
            [[5, 255, 255], [0, 255, 255]],
            [[178, 255, 255], [175, 230, 254]],
            [[173, 255, 255], [16, 230, 254]],
        ])
        assert input_img.shape == (3, 2, 3)
        result = obj.hsv_threshold(input_img)

        assert result.shape == (3, 2)
        assert result[0, 0] == 255
        assert result[0, 1] == 255
        assert result[1, 0] == 255
        assert result[1, 1] == 255
        assert result[2, 0] == 0
        assert result[2, 1] == 0



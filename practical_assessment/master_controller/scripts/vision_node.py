#!/usr/bin/env python
import rospy
import os
import json
import numpy as np

from followbot.msg import DetectedObject
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv

DEBUG = False
DEBUG_SAMPLE_RADIUS = 10  # Radius not accurate as its a square


class DetectableObject(object):

    def __init__(self, name, h, s, v, h_range, s_range, v_range):
        self.name = name
        self.h = h
        self.s = s
        self.v = v
        self.h_range = h_range
        self.s_range = s_range
        self.v_range = v_range

        self.hsv_lo = np.array([(self.h - self.h_range / 2) % 180,
                                max(self.s - self.s_range / 2, 0),
                                max(self.v - self.v_range / 2, 0)])
        self.hsv_hi = np.array([(self.h + self.h_range / 2) % 180,
                                min(self.s + self.s_range / 2, 255),
                                min(self.v + self.v_range / 2, 255)])

    def find_in_image(self, img, config):
        # Smooth edges by blurring a bit
        blurred = cv.GaussianBlur(img, (config['gauss_kernel_size'],
                                        config['gauss_kernel_size']), 0)

        mask = None
        if self.hsv_lo[0] > self.hsv_hi[0]:  # We need to wrap around the hue spectrum
            mask1 = cv.inRange(blurred.copy(), self.hsv_lo, np.array([128,
                                                                      self.hsv_hi[1],
                                                                      self.hsv_hi[2]]))
            mask2 = cv.inRange(blurred, np.array([0,
                                                  self.hsv_lo[1],
                                                  self.hsv_lo[2]]), self.hsv_hi)
            mask = mask1 + mask2
        else:
            mask = cv.inRange(blurred, self.hsv_lo, self.hsv_hi)

        closed = cv.morphologyEx(mask, cv.MORPH_OPEN, (11, 11))
        # closed = cv.morphologyEx(mask, cv.MORPH_CLOSE, (5, 5))

        _, contours, _ = cv.findContours(closed,
                                         cv.RETR_TREE,
                                         cv.CHAIN_APPROX_SIMPLE)

        if DEBUG:
            cv.imshow('Mask: {0}'.format(self.name), closed)

        matches = 0
        best_contour = None
        best_area = 0

        # print('{0} {1}'.format(self.name, len(contours)))
        for contour in contours:
            moments = cv.moments(contour)
            area = moments['m00']
            if area >= config['contour_size_threshold']:
                matches += 1
                if area > best_area:
                    cx = int(moments['m10']/moments['m00'])
                    cy = int(moments['m01']/moments['m00'])
                    best_contour = (cx, cy)
                    best_area = area

        if matches > 1:
            warnmsg = 'Found {0} (>1) contour was found for {1}, consider \
                       tweaking the config!'.format(matches, self.name)
            if DEBUG:
                rospy.logwarn(warnmsg)
            else:
                rospy.logdebug(warnmsg)
        return best_contour


class Classifier():

    def __init__(self):
        self.targets = []
        self.last_detected = set()

        roshome = os.getenv('ROS_HOME') or os.path.expanduser('~/.ros')
        self.load_config('{}/vision_config.json'.format(roshome))

    def load_config(self, path):
        '''
        Loads the json config for the classifier.
        '''
        with open(path, 'r') as jsonfile:
            self.config = json.load(jsonfile)
            try:
                for target in self.config['targets']:
                    self.targets.append(DetectableObject(
                        target['name'],
                        target['h'],
                        target['s'],
                        target['v'],
                        target['h_range'],
                        target['s_range'],
                        target['v_range']
                    ))
                self.scale_down = self.config['scale_down']
            except KeyError as err:
                rospy.logerr('Key was not present in the config file: %s', err)

            rospy.logdebug('Loaded classifier config file succesfully')

    def classify(self, img):
        '''
        This is the main method of the classifier class. It takes an image as
        input and output the location of any objects detected in the scene.

        PARAMS:
         - img : OpenCV Image (Numpy Array)

        RETURNS:
         - A list of tuples, each tuple will contain the keys:
             - object_name: str
             - x: float
             - y: float
        '''
        detected = []
        for target in self.targets:
            result = target.find_in_image(img, self.config)
            if result:
                x, y = result
                if DEBUG:
                    rospy.loginfo('I can see {0} at x={1} y={2}'.format(
                        target.name, *result))
                detected.append((target.name, x, y))
        return detected


class ROSNode():

    def __init__(self):
        rospy.init_node('ObjectDetection', anonymous=True)

        rospy.on_shutdown(self.shutdown)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw',
                                          Image,
                                          self.image_callback)
        self.detected_pub = rospy.Publisher("/objects",
                                            DetectedObject,
                                            queue_size=5)

        self.classifier = Classifier()

    def image_callback(self, data):
        scale_down = self.classifier.scale_down
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # Nothing good ever happens in the bottom half of the image
            h = cv_image.shape[0]
            cv_image = cv_image[:int(h / 2), :, :]

            cv_image_resized = cv.resize(cv_image,
                                         (cv_image.shape[1] / scale_down,
                                          cv_image.shape[0] / scale_down))

            image_hsv = cv.cvtColor(cv_image_resized, cv.COLOR_BGR2HSV)

            if DEBUG:
                self.debug_colours(image_hsv.copy())

            detected = self.classifier.classify(image_hsv)
            for name, x, y in detected:
                msg = DetectedObject()
                msg.object_name = name
                msg.x, msg.y = x, y
                msg.size_y, msg.size_x, _ = image_hsv.shape
                self.detected_pub.publish(msg)

        except CvBridgeError as err:
            rospy.logerr('CvBridgeError in image_callback: %s', err)

    def debug_colours(self, img):
        height, width, _ = img.shape
        l, r = width / 2 - DEBUG_SAMPLE_RADIUS, width / 2 + DEBUG_SAMPLE_RADIUS
        t, b = height / 2 - DEBUG_SAMPLE_RADIUS, height / 2 + DEBUG_SAMPLE_RADIUS

        subarr = img[t:b, l:r]
        subarr = subarr.reshape(-1, subarr.shape[2])
        means = np.mean(subarr, axis=0)
        avg_text = 'h:{0:6.2f} s:{1:6.2f} v:{2:6.2f}'.format(*means)
        img = cv.rectangle(img, (l, t), (r, b), (0, 255, 255), 1)
        img = cv.putText(img, avg_text, (10, height - 10),
                         cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        cv.imshow("Sample", cv.cvtColor(img, cv.COLOR_HSV2BGR))
        cv.waitKey(3)

    def shutdown(self):
        rospy.loginfo("Goodbye!")


if __name__ == '__main__':
    rosnode = ROSNode()
    rospy.spin()

#!/usr/bin/env python

import rospy
import cv2
import collections
from std_msgs.msg import Header
from sensor_msgs.msg import Image
from numpy.random import uniform
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ImageModulation():

    def __init__(
            self, input_topic_name,
            output_topic_name, avg_time_stamp_delay,
            time_stamp_jitter, blur_factor,
            data_loss_percent, rate,
            start_with_full_queue):
        self.input_topic_name = input_topic_name
        self.output_topic_name = output_topic_name
        self.avg_time_stamp_delay = avg_time_stamp_delay
        self.time_stamp_jitter = time_stamp_jitter
        self.blur_factor = blur_factor
        self.data_loss_percent = data_loss_percent
        self.rate = rospy.Rate(rate)
        self.start_with_full_queue = start_with_full_queue

        self.im_datas = collections.deque(maxlen=blur_factor)
        self.headers = collections.deque(maxlen=blur_factor)
        self.curr_image = None
        self.curr_header = None
        self.last_image = None
        self.stamp = None
        self.float_time = None
        self.new_data = False
        self.bridge = CvBridge()

        self.image_sub = rospy.Subscriber(
            self.input_topic_name, Image, self.im_callback)
        self.image_pub = rospy.Publisher(
            self.output_topic_name, Image, queue_size=1)
        self.timer = rospy.Timer(
            rospy.Duration(1.0/rate), self.timer_cb)


    def timer_cb(self, data):
        if (len(self.im_datas) >= self.blur_factor):

            if self.new_data:
                self.curr_header = self.headers[0]
                #self.curr_image = self.im_datas[0]
                self.modulate_headers()
                self.modulate_image()
                self.new_data = False
            self.publish()

    def modulate_headers(self):
        self.stamp_to_float(self.curr_header.stamp)
        jitter = uniform(
            self.avg_time_stamp_delay - self.time_stamp_jitter,
            self.avg_time_stamp_delay + self.time_stamp_jitter)
        self.float_time += jitter
        self.float_to_stamp()
        head = Header()
        head.seq = self.curr_header.seq
        head.frame_id = self.curr_header.frame_id
        head.stamp = self.stamp
        self.curr_header = head

    def modulate_image(self):
        # Blending the images with 0.5 and 0.5
        im = self.im_datas[0]
        for i in range(self.blur_factor - 1):
            im = cv2.addWeighted(
                im, 0.5,
                self.im_datas[i], 0.5, 0)
        self.curr_image = im

    def float_to_stamp(self):
        self.stamp = rospy.Time().from_sec(self.float_time)

    def stamp_to_float(self, stamp):
        self.float_time = stamp.secs + stamp.nsecs / 1000000000.0

    def publish(self):

        rand = uniform(0.0,1.0)
        if rand > self.data_loss_percent:
            im = None
            try:
                im = self.bridge.cv2_to_imgmsg(self.curr_image, "bgr8")
            except CvBridgeError as e:
                print (e)
            if im:
                im.header = self.curr_header
                self.image_pub.publish(im)
        self.last_image = self.curr_image

    def im_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        if self.start_with_full_queue:
            for i in range(self.blur_factor):
                self.im_datas.appendleft(cv_image)
                self.headers.appendleft(data.header)
            self.start_with_full_queue = False
        self.im_datas.appendleft(cv_image)
        self.headers.appendleft(data.header)
        self.new_data = True

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

def main():
    rospy.init_node("ImageModulation")

    default_rate = 20
    default_blur_factor = 2
    default_avg_time_stamp_delay = 0.02
    default_time_stamp_jitter = 0.01
    default_data_loss_percent = 0.1
    default_input_topic_name = "/camera/rgb/image_raw";
    default_output_topic_name = "/camera/rgb/modified";
    default_start_with_filled_queue = True

    rate = rospy.get_param(
        "rate", default_rate)
    blur_factor = rospy.get_param(
        'blur_factor', default_blur_factor)
    avg_time_stamp_delay = rospy.get_param(
        'avg_time_stamp_delay',
        default_avg_time_stamp_delay)
    time_stamp_jitter = rospy.get_param(
        'time_stamp_jitter',
        default_time_stamp_jitter)
    data_loss_percent = rospy.get_param(
        'data_loss_percent',
        default_data_loss_percent)
    input_topic_name = rospy.get_param(
        'input_topic_name',
        default_input_topic_name)
    output_topic_name = rospy.get_param(
        'output_topic_name',
        default_output_topic_name)
    start_with_full_queue = rospy.get_param(
        'start_with_full_queue',
        default_start_with_filled_queue)


    image_mod = ImageModulation(
        input_topic_name,
        output_topic_name,
        avg_time_stamp_delay,
        time_stamp_jitter,
        blur_factor,
        data_loss_percent,
        rate,
        start_with_full_queue)

    image_mod.run()

if __name__ == "__main__":
    main()

import threading
import time

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2


class Kinect:
    """
    A class for interacting with a RealSense cameras.
    """

    def __init__(self):
        self.points = None

        rospy.init_node("kinect_point_cloud_listener", anonymous=True)
        rospy.Subscriber("/kinect2/qhd/points", PointCloud2, self.grab_frames)
        _ = rospy.wait_for_message("/kinect2/qhd/points", PointCloud2, timeout=1)
        self.thread = threading.Thread(target=self._run_grab_frames)
        self.thread.start()

    def _run_grab_frames(self):
        rospy.spin()

    def grab_frames(self, msg):
        """
        Grabs frames from the Kinect camera and stores them in instance variables.
        """
        gen = pc2.read_points(msg, skip_nans=True)
        int_data = list(gen)

        xyz = []

        for x in int_data:
            xyz.append(np.array([[x[0]], [x[1]], [x[2]]]))

        self.points = np.concatenate(xyz, axis=1)

    def close(self):
        rospy.signal_shutdown("Terminated Program")
        self.thread.join()

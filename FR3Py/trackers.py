from pupil_apriltags import Detector
import numpy as np
import cv2

class ApriltagTracker:
    """
    A class for tracking AprilTags in images.

    Attributes:
        intrinsic_matrix (numpy.ndarray): The camera's intrinsic matrix.
        distortion_coeffs (numpy.ndarray): The camera's distortion coefficients.
        tag_size (float): The size of the AprilTags in meters.
        family (str): The family of AprilTags to detect (default is 'tag36h11').
        nthreads (int): The number of threads to use for detection (default is 4).
        detector (pupil_apriltags.Detector): The AprilTag detector.
        detected_ids (list): A list of detected AprilTag IDs.

    Methods:
        process(self, image):
            Process an image to detect AprilTags.

        getTag(self, tag_id):
            Get information about a specific detected AprilTag.

        getAllTags(self):
            Get information about all detected AprilTags.

        undistortImage(self, img):
            Undistort an input image using camera calibration parameters.
    """

    def __init__(self,
                 tag_size,
                 intrinsic_matrix=None,
                 distortion_coeffs=None,
                 family='tag36h11',
                 nthreads=4):
        """
        Initialize the ApriltagTracker.

        Args:
            tag_size (float): The size of the AprilTags in meters.
            intrinsic_matrix (numpy.ndarray): The camera's intrinsic matrix.
            distortion_coeffs (numpy.ndarray): The camera's distortion coefficients.
            family (str): The family of AprilTags to detect (default is 'tag36h11').
            nthreads (int): The number of threads to use for detection (default is 4).
        """
        self.intrinsic_matrix = intrinsic_matrix
        self.distortion_coeffs = distortion_coeffs
        self.tag_size = tag_size
        self.family = family
        self.nthreads = nthreads
        self.detector = Detector(
            families='tag36h11',
            nthreads=nthreads
        )
        self.detected_ids = []

    def process(self, image):
        """
        Process an image to detect AprilTags.

        Args:
            image (numpy.ndarray): The input image to be processed.
        """
        if image.shape[-1] == 3:
            image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        if self.distortion_coeffs is not None:
            image = self.undistortImage(image)

        if self.intrinsic_matrix is not None:
            fx = self.intrinsic_matrix[0, 0]
            fy = self.intrinsic_matrix[1, 1]
            cx = self.intrinsic_matrix[0, 2]
            cy = self.intrinsic_matrix[1, 2]
            self.detections = self.detector.detect(image, estimate_tag_pose=True,
                                                   camera_params=[fx, fy, cx, cy],
                                                   tag_size=self.tag_size)
        else:
            self.detections = self.detector.detect(image, estimate_tag_pose=True)

        self.detected_ids = [d.tag_id for d in self.detections]

    def getTag(self, tag_id):
        """
        Get information about a specific detected AprilTag.

        Args:
            tag_id (int): The ID of the AprilTag to retrieve information about.

        Returns:
            dict or None: Information about the detected AprilTag or None if not found.
        """
        if tag_id in self.detected_ids:
            info = self.detections[self.detected_ids.index(tag_id)]
            corners = info.corners
            center = info.center
            if self.intrinsic_matrix is not None:
                R = info.pose_R
                t = info.pose_t
                T = np.vstack([np.hstack((R, t)), np.array([[0, 0, 0, 1]])])
                return {'corners': corners, 'center': center, 'pose': T}
            else:
                return {'corners': corners, 'center': center}
        else:
            return None

    def getAllTags(self):
        """
        Get information about all detected AprilTags.

        Returns:
            dict: A dictionary where keys are tag IDs and values are information about the detected AprilTags.
        """
        return {id: self.getTag(id) for id in self.detected_ids}

    def undistortImage(self, img):
        """
        Undistorts an input image using camera calibration parameters.

        Args:
            img (numpy.ndarray): The input image to be undistorted.

        Returns:
            numpy.ndarray: The undistorted image.
        """
        # Undistort the image
        undistorted_image = cv2.undistort(img,
                                          self.intrinsic_matrix,
                                          self.distortion_coeffs)
        return undistorted_image
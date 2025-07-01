#!/usr/bin/env python3

import rospy
import logging
import cv2
import cv2.aruco as aruco

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class ArucoDetector:
    """
    Subscribes to a ZED2 camera image stream and detects ArUco markers in the frame.
    Detected markers are drawn and logged for debugging or processing.
    """

    def __init__(self) -> None:
        rospy.loginfo(f"Starting aruco_detector.py")

        # Conver ROS Image messages to OpenCV images
        self.bridge = CvBridge()

        # Subscribe to raw color images from ZED2i
        self.image_sub = rospy.Subscriber(
            "/zed2i/zed_node/rgb_raw/image_raw_color", 
            Image, 
            self.image_callback
        )

        # Preload ArUco dictionary and detection parameters for efficiency
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
        self.aruco_params = aruco.DetectorParameters_create()

    def image_callback(self, msg: Image) -> None:
        """
        Callback for processing incoming images. Detects and annotates ArUco markers.
        """
        try:
            # Convert incoming ROS image message to OpenCV format (BGR)
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge conversion failed: {e}")
            return

        # Detect ArUco markers in the frame
        corners, ids, _ = aruco.detectMarkers(
            frame, 
            self.aruco_dict, 
            parameters=self.aruco_params
        )

        # If any marker were detected, draw them and log their IDs
        if ids is not None:
            ids = ids.flatten()
            for (marker_corner, marker_id) in zip(corners, ids):
                # Each marker has 4 corners: top-left, top-right, bottom-right, bottom-left
                reshaped = marker_corner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = map(
                    lambda pt: (int(pt[0]), int(pt[1])), reshaped
                )

                # Draw bounding box around the marker
                cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)

                # Draw the center and ID of the marker
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
                cv2.putText(frame, str(marker_id), (topLeft[0], topLeft[1] - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                rospy.loginfo(f"Detected ArUco marker ID: {marker_id}")

        # Display the annotated frame
        cv2.imshow("Aruco Detection", frame)
        cv2.waitKey(1)

if __name__ == '__main__':
    rospy.init_node('aruco_detector', anonymous=True)
    
    # Instantiate the detector
    ad = ArucoDetector()
    # Keep the node alive
    rospy.spin()
    # Clean up OpenCV windows on shutdown
    cv2.destroyAllWindows()        

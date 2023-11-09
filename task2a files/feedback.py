#! /usr/bin/env python3

'''
*****************************************************************************************
*
*        		===============================================
*           		Hologlyph Bots (HB) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 2A of Hologlyph Bots (HB) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''
################### IMPORT MODULES #######################
import rclpy
from rclpy.node import Node

# Import the required modules
##############################################################
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image

class ArUcoDetector(Node):

    def __init__(self):
        super().__init__('ar_uco_detector')
        # Subscribe the topic /camera/image_raw
        self.sub_camera=self.create_subscription(Image,'/camera/image_raw',self.image_callback,10)

        self.pub_det_aruco=self.create_publisher(Pose,'/detected_aruco',10)

        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
        self.cv_bridge=CvBridge()


    def image_callback(self, msg):
        #convert ROS image to opencv image
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #Detect Aruco marker
        corners,ids,_ = self.detector.detectMarkers(cv_image)

        if len(corners) > 0:
            ids=ids.flatten()

            if 1 in ids:
                idx = ids.tolist().index(1)

                corners = corners[idx].reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners

                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))

                cv2.line(cv_image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(cv_image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(cv_image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(cv_image, bottomLeft, topLeft, (0, 255, 0), 2)

                centerX = float((topLeft[0] + bottomRight[0]) / 2.0)
                centerY = float((topLeft[1] + bottomRight[1]) / 2.0)

                cv2.putText(cv_image, str(ids[idx]),
                            (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 2)
                print("[INFO] ArUco marker ID: {}".format(ids[idx]))

                cv2.imshow("Image",cv_image)

                currPos = Pose()

                currPos.position.x=centerX
                currPos.position.y=centerY
                currPos.position.z=0.0

                currPos.orientation.x=0.0
                currPos.orientation.y=0.0
                currPos.orientation.z=0.0
                currPos.orientation.w=0.0

                self.pub_det_aruco.publish(currPos)

                cv2.waitKey(0)
            # Publish the bot coordinates to the topic  /detected_aruco



def main(args=None):
    rclpy.init(args=args)

    aruco_detector = ArUcoDetector()

    rclpy.spin(aruco_detector)

    aruco_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

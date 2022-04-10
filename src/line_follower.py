#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Twist

class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw/compressed', CompressedImage, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.twist = Twist()
        self.logcount = 0
        self.lostcount = 0
        cv2.namedWindow("band", cv2.WINDOW_NORMAL)
        cv2.namedWindow("image", cv2.WINDOW_NORMAL)
        
        # cv2 window show not be occupying the entire screen as we want to view both windows simultaneously
        cv2.resizeWindow("band", 400, 300)
        cv2.resizeWindow("image", 400, 300)
        self.has_started = False

    def image_callback(self, msg):

        # get the compressed image from camera
        image = self.bridge.compressed_imgmsg_to_cv2(msg)

        # filter out everything that's not yellow
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([ 40, 0, 0])
        upper_yellow = numpy.array([ 120, 255, 255])
        mask = cv2.inRange(hsv,  lower_yellow, upper_yellow)

        h, w, _ = image.shape
        
        # only look at the portion of the line that is immediately visible to the robot (the 200 pixel down from a quarter above the bottom of the image) so that
        # it doesn't get distracted the curve ahead
        mask_filtered = mask.copy()
        search_top = int(3* h /4)
        search_bot = search_top + 200
        mask_filtered[0:search_top, 0:w] = 0
        mask_filtered[search_bot:h, 0:w] = 0

        M = cv2.moments(mask)
        MF = cv2.moments(mask_filtered)

        # always use MF, which only contains the edge of the line immeidately visible to the robot unless no line can be found
        # in the image or the robot has already been following the line before 
        if MF['m00'] > 0 or self.has_started:
            # the first time we are able to detect edge is when we set self.has_started to be True
            self.has_started = True
            cv2.imshow("band", cv2.resize(mask_filtered, (400, 300)))
            M = MF
        # sometimes the robot might start at a place where the line not immediately invisible (the line might be far away). In that case
        # we don't want to focus just on the porition line that is immeidately insivible cause it probably doesn't exist. It should try to move
        # toward the line that might be far away. 
        else:
            cv2.imshow("band", cv2.resize(mask, (400, 300)))

        print("M00 %d %d" % (M['m00'], self.logcount))

        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

            # the error is how close the centroid of the line is to the center width of the image
            err = cx - w/2
            
            self.twist.linear.x = 0.3
            self.twist.angular.z = -float(err) / 1000
            self.cmd_vel_pub.publish(self.twist)
        else:
            #rotate until the line is visible, this allows the robot to make a u turn or locate the line if the line is not immediatly visible
            self.twist.linear.x = 0
            self.twist.angular.z = 0.5
            self.cmd_vel_pub.publish(self.twist)

        cv2.imshow("image", cv2.resize(image, (400, 300)))
        cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
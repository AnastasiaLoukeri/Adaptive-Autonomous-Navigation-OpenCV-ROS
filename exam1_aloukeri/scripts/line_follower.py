#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import numpy as np
import math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from gazebo_msgs.msg import ModelStates


class LineFollower:

    def __init__(self):
        self.bridge       = CvBridge()
        self.enabled      = True
        self.last_err     = 0.0
        self.lost_counter = 0

        # Συντεταγμένες τερματισμού (από param server ή default)
        self.goal_x = rospy.get_param('/exam1/goal_x', -1.7)
        self.goal_y = rospy.get_param('/exam1/goal_y',  1.07)
        self.current_x = 0.0
        self.current_y = 0.0

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/camera/rgb/image_raw', Image,       self.camera_cb)
        rospy.Subscriber('/line_follower/enable', Bool,        self.enable_cb)
        rospy.Subscriber('/gazebo/model_states',  ModelStates, self.model_states_cb)
        rospy.loginfo("[LineFollower] Ξεκίνησε. Goal: (%.2f, %.2f)", self.goal_x, self.goal_y)

    
    def model_states_cb(self, msg):
        try:
            idx = msg.name.index('turtlebot3_waffle_pi')
            self.current_x = msg.pose[idx].position.x
            self.current_y = msg.pose[idx].position.y
        except ValueError:
            pass

    def enable_cb(self, msg):
        self.enabled = msg.data
        if not self.enabled:
            self.cmd_pub.publish(Twist())

    
    def camera_cb(self, msg):
        # Έλεγχος αν έφτασε στο goal ( απόσταση < 0.2)
        dist = math.sqrt((self.current_x - self.goal_x)**2 +
                         (self.current_y - self.goal_y)**2)
        if dist < 0.2:
            self.cmd_pub.publish(Twist())
            self.enabled = False
            rospy.loginfo("[LineFollower goal reached! dist=%.3f  pos=(%.3f, %.3f)",
                          dist, self.current_x, self.current_y)
            rospy.signal_shutdown("Goal reached")
            return

        rospy.loginfo_throttle(2, "LineFollower pos=(%.3f, %.3f)  dist_to_goal=%.3f",
                               self.current_x, self.current_y, dist)

        # Βήμα 1
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv   = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h, w  = image.shape[:2]

        # Βήμα 2
        mask_yellow = cv2.inRange(hsv, np.array([20, 100, 100]), np.array([35, 255, 255]))
        mask_white  = cv2.inRange(hsv, np.array([0,    0, 200]), np.array([179, 40, 255]))
        

        # Βήμα 3
        def find_centroid(mask, top_cut):
            roi = mask.copy()
            roi[:top_cut, :] = 0
            M = cv2.moments(roi)
            if M['m00'] > 300:
                return int(M['m10'] / M['m00']), int(M['m01'] / M['m00']), M['m00']
            return None, None, 0

        cx, cy, area = find_centroid(mask_yellow, int(2 * h / 3))
        if area < 300:
            cx, cy, area = find_centroid(mask_yellow, h // 2)
        if area < 300:
            cx, cy, area = find_centroid(mask_white, int(2 * h / 3))

        # Debug
        debug = image.copy()
        if cx is not None:
            cv2.circle(debug, (cx, cy), 10, (0, 0, 255), -1)
            cv2.line(debug, (w // 2, h), (cx, cy), (255, 0, 0), 2)
        cv2.putText(debug, "dist:%.2f" % dist, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.namedWindow("window", 1)
        cv2.imshow("window", debug)
        cv2.waitKey(1)

        if not self.enabled:
            return

        twist = Twist()

        #  Adaptive P controller
        if area > 300:
            err = cx - w / 2.0
            self.last_err     = err
            self.lost_counter = 0
            angular           = -err / 220.0
            twist.linear.x    = max(0.06, 0.14 - abs(angular) * 0.4)
            twist.angular.z   = angular
        else:
            self.lost_counter += 1
            twist.linear.x    = 0.0
            twist.angular.z   = -0.35 if self.last_err > 0 else 0.35
            if self.lost_counter > 30:
                twist.angular.z *= 0.5
            rospy.logwarn_throttle(1, " (lost=%d)", self.lost_counter)

        self.cmd_pub.publish(twist)


if __name__ == '__main__':
    rospy.init_node('line_follower')
    LineFollower()
    rospy.spin()






    # Επεξεργασία εικόνας κάμερας
    # Δέχεται ως όρισμα την εικόνα από το topic rgb/image_raw.
    # Τη μετατρέπει από ROS Image σε εικόνα OpenCV και αλλάζει τα χρώματα 
    # για καλύτερη επεξεργασία BGR → HSV με imgmsg_to_cv2 στο βήμα 1.
    # 2. Mask κίτρινης γραμμής (H=20-35, S>100, V>100), δηλαδή στα σημεία όπου η εικόνα 
    # ειναι κιτρινη,την κανει ασπρη ενω μαυριζει την υπόλοιπη. Το ίδιο κάνει για τη λευκή.
    #    (S<40, V>200) με cv2.inRange() (αυτες ειναι οι γραμμές του autorace) στο βήμα 2.
    # Στο βήμα 3 ορίζουμε την περιοχή ενδιαφέροντός μας απ'ο
    # την εικόνα, με εστίαση στο κάτω 1/3 (ευθεία) ή κάτω 1/2 (στροφή) ανάλογα,
    #καθώς δεν μας ενδιαφέρουν οι γραμμές που είναι πολύ μακρυά. 
    # Έπειτα βρίσκουμε το κέντρο/μέσο της γραμμής.


    #  Adaptive P controller
        # Υπολογίζουμε το σφάλμα στη διαδρομή , πόσο αποκκλίνει από την ευθεία
        #και αποθηκεύει το τελευταίο σφάλμα, σε περίπτωση που χάσει το φρειμ
        # αλλάζει αναλόγως την κατρυθυνση περιστροφης ωστε να ευθυγραμμιστει
        # Επειτα , αναλογως αν παει ευθεια ή στριβει , μειωνει ταχυτητα (σαν φρενο)
        # Αν δε προλαβει το φρειμ, σταματα να κινειται εως οτου περιστρεφεται ξανα, αναλογως τελικης 
        # προηγουμενης κατευθυνσης. Σε μερικά σημεια οπου η στροφή ειναι απότομη, σταματαειν να 
        #κινειται αλλα αυτοδιορθωνεται. Αν δε φτανει σε εσας εγκαιρως, αυξηστε linear x
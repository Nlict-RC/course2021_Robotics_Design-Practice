#!/usr/bin/env python
import rospy
#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Twist
from threading import Lock,Thread
import math
import time
class Tracking:
    def __init__(self):
        self.arrive_threshold = 0.2
        self.vx = 0.0
        self.vw = 0.0

        self.lock = Lock()
        self.path = Path()
        self.tf = tf.TransformListener()
        self.path_sub = rospy.Subscriber('/course_agv/global_path',Path,self.pathCallback)
        self.vel_pub = rospy.Publisher('/course_agv/velocity',Twist, queue_size=1)
        self.midpose_pub = rospy.Publisher('/course_agv/mid_goal',PoseStamped,queue_size=1)
        self.tracking_thread = None
        pass
    def updateGlobalPose(self):
        try:
            self.tf.waitForTransform("/map", "/robot_base", rospy.Time(), rospy.Duration(4.0))
            (self.trans,self.rot) = self.tf.lookupTransform('/map','/robot_base',rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("get tf error!")
        euler = tf.transformations.euler_from_quaternion(self.rot)
        roll,pitch,yaw = euler[0],euler[1],euler[2]
        self.x = self.trans[0]
        self.y = self.trans[1]
        self.yaw = yaw

        p = self.path.poses[self.goal_index].pose.position
        dis = math.hypot(p.x-self.x,p.y-self.y)
        if dis < self.arrive_threshold and self.goal_index < len(self.path.poses)-1:
            self.goal_index = self.goal_index + 1
        self.midpose_pub.publish(self.path.poses[self.goal_index])

    def pathCallback(self,msg):
        print("get path msg!!!!!",msg)
        self.path = msg
        self.lock.acquire()
        self.initTracking()
        self.lock.release()
        if self.tracking_thread == None:
            self.tracking_thread = Thread(target=self.trackThreadFunc)
            self.tracking_thread.start()
        pass
    def initTracking(self):
        self.goal_index = 0
        self.updateGlobalPose()
        pass
    def trackThreadFunc(self):
        print("running track thread!!")
        # while self.plan_lastIndex > self.plan_target_ind:
        while True:
            self.planOnce()
            time.sleep(0.001)
        print("exit track thread!!")
        self.lock.acquire()
        self.publishVel(True)
        self.lock.release()
        self.tracking_thread = None
        pass
    def planOnce(self):
        self.lock.acquire()

        self.updateGlobalPose()

        target = self.path.poses[self.goal_index].pose.position

        dx = target.x - self.x
        dy = target.y - self.y

        target_angle = math.atan2(dy, dx)

        self.vx = 0.2
        self.vw = (target_angle-self.yaw)/1.0
        if self.vw > 0.5:
            self.vw = 0.5
        if self.vw > 0.2 :
            self.vx = 0

        self.publishVel()

        self.lock.release()
        pass
    def publishVel(self,zero = False):
        cmd = Twist()
        cmd.linear.x = self.vx
        cmd.angular.z = self.vw
        if zero:
            cmd.linear.x = 0
            cmd.angular.z = 0
        self.vel_pub.publish(cmd)

def main():
    rospy.init_node('stupid_tracking')
    t = Tracking()
    rospy.spin()

if __name__ == '__main__':
    main()

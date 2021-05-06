#!/usr/bin/env python
import rospy
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped,Twist
from pure_pursuit import *
from threading import Lock,Thread
import time
class Tracking:
    def __init__(self):
        self.arrive = 0.1
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
        self.goal_dis = math.hypot(self.x-self.path.poses[-1].pose.position.x,self.y-self.path.poses[-1].pose.position.y)

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
        self.updateGlobalPose()
        self.plan_cx = []
        self.plan_cy = []
        for pose in self.path.poses:
            self.plan_cx.append(pose.pose.position.x)
            self.plan_cy.append(pose.pose.position.y)

        self.plan_target_speed = 0.3
        self.plan_dt = 0.01

        self.plan_state = State(x=self.x,y=self.y,yaw=self.yaw,v=0.0)
        self.plan_lastIndex = len(self.plan_cx) -1
        self.plan_time = 0.0
        self.plan_target_course = TargetCourse(self.plan_cx,self.plan_cy)
        self.plan_target_course.old_nearest_point_index = None
        self.plan_target_ind,_ = self.plan_target_course.search_target_index(self.plan_state,True)
        pass
    def trackThreadFunc(self):
        print("running track thread!!")
        # while self.plan_lastIndex > self.plan_target_ind:
        while True:
            self.lock.acquire()
            self.planOnce()
            self.lock.release()
            time.sleep(0.001)
            if self.goal_dis < self.arrive:
                print("arrive goal!")
                break
        print("exit track thread!!")
        self.lock.acquire()
        self.publishVel(True)
        self.lock.release()
        self.tracking_thread = None
        pass
    def planOnce(self):
        self.updateGlobalPose()
        self.plan_state.x = self.x
        self.plan_state.y = self.y
        self.plan_state.yaw = self.yaw
        self.plan_ai = proportional_control(self.plan_target_speed, self.plan_state.v)
        self.plan_di, self.plan_target_ind = pure_pursuit_steer_control(self.plan_state, self.plan_target_course, self.plan_target_ind)
        self.plan_state.update(self.plan_ai,self.plan_di)
        goal = self.path.poses[self.plan_target_ind]
        self.midpose_pub.publish(goal)
        self.plan_time += self.plan_dt
        self.publishVel()
        pass

    def publishVel(self,zero = False):
        cmd = Twist()
        cmd.linear.x = self.plan_state.v
        cmd.angular.z = self.plan_di
        if zero:
            cmd.linear.x = 0
            cmd.angular.z = 0
        self.vel_pub.publish(cmd)

def main():
    rospy.init_node('path_tracking')
    t = Tracking()
    rospy.spin()
    pass

def test(t):
    rx = [0,1,2,3,4,5,6,7]
    ry = [1,0,1,0,1,0,1,0]
    path = Path()
    path.header.seq = 0
    path.header.stamp = rospy.Time(0)
    path.header.frame_id = 'map'
    for i in range(len(rx)):
        pose = PoseStamped()
        pose.header.seq = i
        pose.header.stamp = rospy.Time(0)
        pose.header.frame_id = 'map'
        pose.pose.position.x = rx[i]
        pose.pose.position.y = ry[i]
        pose.pose.position.z = 0.01
        pose.pose.orientation.x = 0#self.rot[0]
        pose.pose.orientation.y = 0#self.rot[1]
        pose.pose.orientation.z = 0#self.rot[2]
        pose.pose.orientation.w = 1#self.rot[3]
        path.poses.append(pose)
    pub = rospy.Publisher('/course_agv/global_path',Path,queue_size = 10)
    time.sleep(0.5)
    pub.publish(path)
    print("publish path!!!!!")
    # t.pathCallback(path) 

if __name__ == '__main__':
    main()
#!/usr/bin/env python
# license removed for brevity
import rospy
import tf
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import cv2 # I THINK THATs how that works?
import numpy as np

global twist_pub
twist_pub = Twist()
global stop_flag
stop_flag = False

def objectCallback(object):
   global twist_pub
   global stop_flag
   FOLLOW = 1 #red cheese its
   STOP = 2 #laptop cover
   REPEL = 3 #dad's garage
   RESTART = 4 #sour punch bag


   id_var = 0;
   set_vel = Twist()

   camera_center = 320
   max_ang_vel = 0.6
   min_ang_vel = 0.4
   ang_vel = 0

   set_vel.linear.z = 0
   set_vel.angular.x = 0
   set_vel.angular.y = 0

   
   if (len(object.data) > 0): # Not sure if this is how python works
      for i in range(0,len(object.data),12):

         id_var = object.data[i]
         # ros info object id print
         # rospy.loginfo("object_id")
         # rospy.loginfo(id_var)
         objectWidth = object.data[i+1]
         objectHeight = object.data[i+2]
         speed_coefficient = camera_center / max_ang_vel

         if (id_var == RESTART):
            stop_flag = False
            break

         if (id_var == STOP):
            set_vel.linear.x = 0
            set_vel.angular.z = 0
            stop_flag = True
            break

         if (id_var == FOLLOW):
            if (stop_flag):
               break
            
            cvHomography = np.zeros((3, 3, 1), dtype = "float")
            cvHomography[0,0] = object.data[i + 3]
            cvHomography[1,0] = object.data[i + 4]
            cvHomography[2,0] = object.data[i + 5]
            cvHomography[0,1] = object.data[i + 6]
            cvHomography[1,1] = object.data[i + 7]
            cvHomography[2,1] = object.data[i + 8]
            cvHomography[0,2] = object.data[i + 9]
            cvHomography[1,2] = object.data[i + 10]
            cvHomography[2,2] = object.data[i + 11]
         
            inPts = np.array([[[0,0]], [[objectWidth, 0]], [[0, objectHeight]], [[objectWidth, objectHeight]]], dtype=np.float32)
            outPts = cv2.perspectiveTransform(inPts, cvHomography);
            

            x_pos = (outPts[0,0,0] + outPts[1,0,0] + outPts[2,0,0] + outPts[3,0,0])/4
         
            ang_vel = -(x_pos - camera_center)/ speed_coefficient
           
            if (ang_vel >= -(min_ang_vel/3)) and (ang_vel <= (min_ang_vel/3)):
               set_vel.angular.z = 0
               set_vel.linear.x = 0.5
            else:
               set_vel.angular.z = ang_vel
               set_vel.linear.x = 0.5
            # twist_pub.publish(set_vel)

         if (id_var == REPEL):
            if (stop_flag):
               break
            
            cvHomography = np.zeros((3, 3, 1), dtype = "float")
            cvHomography[0,0] = object.data[i + 3]
            cvHomography[1,0] = object.data[i + 4]
            cvHomography[2,0] = object.data[i + 5]
            cvHomography[0,1] = object.data[i + 6]
            cvHomography[1,1] = object.data[i + 7]
            cvHomography[2,1] = object.data[i + 8]
            cvHomography[0,2] = object.data[i + 9]
            cvHomography[1,2] = object.data[i + 10]
            cvHomography[2,2] = object.data[i + 11]
         
            inPts = np.array([[[0,0]], [[objectWidth, 0]], [[0, objectHeight]], [[objectWidth, objectHeight]]], dtype=np.float32)
            outPts = cv2.perspectiveTransform(inPts, cvHomography);
            

            x_pos = (outPts[0,0,0] + outPts[1,0,0] + outPts[2,0,0] + outPts[3,0,0])/4
         
            ang_vel = -(x_pos - camera_center)/ speed_coefficient
           
            if (ang_vel >= -(min_ang_vel/2)) and (ang_vel <= (min_ang_vel/2)):
               set_vel.angular.z = 0
               set_vel.linear.x = -0.5
            else:
               set_vel.angular.z = ang_vel
               set_vel.linear.x = -0.5

   else:
      # default: other object
      set_vel.angular.z = 0
      set_vel.linear.x = 0
   twist_pub.publish(set_vel)
      
   
# def odomCallback(msg):


def listener():
   rospy.init_node('objr_node', anonymous=True)
   rospy.Subscriber("objects", Float32MultiArray, objectCallback)
   # rospy.Subscriber("odom", Odometry, odomCallback)

def object_dt():
   global twist_pub

   twist_pub = rospy.Publisher('cmd_vel', Twist, queue_size=50)

   rate = rospy.Rate(50)

   while not rospy.is_shutdown():

      # twist_pub.publish(set_vel)
      rospy.spin()
      rate.sleep()
   # ros::Subscriber sub = n.subscribe("/objects", 1, objectCallback);
   
   # action_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
   

if __name__ == '__main__':
   try:
      listener()
      object_dt()
   except rospy.ROSInterruptException:
      pass

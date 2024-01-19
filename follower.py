#!/usr/bin/env python3  
import rospy
import actionlib
import cv2
from cv_bridge import CvBridge, CvBridgeError
import sys
import numpy as np
import http.client
import json
import datetime

from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

bridge = CvBridge()
twist = Twist()
def havadurumu():
  today_date = datetime.date.today().strftime("%Y-%m-%d")
  gun="Cuma"
  
  conn = http.client.HTTPSConnection("api.collectapi.com")
  headers = {
    'content-type': "application/json",
    'authorization': "apikey 1CI6VgCxkOGLV8lGYXFZq6:0XPezv3ukjYGfwU7tQ7UGg"
    }
  city="Konya"
  conn.request("GET", f"/weather/getWeather?data.lang=tr&data.city={city}&data.date={gun}", headers=headers)

  res = conn.getresponse()
  data = res.read()

  weather_data = json.loads(data.decode("utf-8"))
  conn.close()
  return weather_data["result"]

  
def callback(video):
    global bridge
    global twist
    global cmd_vel_pub
    global point_pub

    # convert ros_image into an opencv-compatible image
    try:
        cv_video = bridge.imgmsg_to_cv2(video, "bgr8")
        cv_video2 = cv2.resize(cv_video, (640, 480))
        roi = cv_video2.copy()[360:480, 120:500]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([23, 90, 95])
        upper_yellow = np.array([35, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        son = cv2.bitwise_and(roi, roi, mask=mask)
        h, w, d = roi.shape
        search_top = 3 * h / 4
        search_bot = 3 * h / 4 + 20
        mask[0:int(search_top), 0:w] = 0
        mask[int(search_bot):h, 0:w] = 0
        M = cv2.moments(mask)

        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(roi, (cx, cy), 20, (0, 0, 255), -1)
            # BEGIN CONTROL
            err = cx - w / 2
            twist.linear.x = 0.4
            twist.angular.z = -float(err) / 100
            cmd_vel_pub.publish(twist)

            # Publish cx and cy as a Point message
            point_msg = Point()
            point_msg.x = cx
            point_msg.y = cy
            point_msg.z = 0.0  # You can set this to another value if needed
            point_pub.publish(point_msg)

        else:
            twist.linear.x = -0.4
            cmd_vel_pub.publish(twist)
            
        weather_msg = String()
        weather_msg.data = json.dumps(havadurumu())
        weather_pub.publish(weather_msg)
        
        cv2.imshow("Roi", roi)
        cv2.imshow("Renk tespit", son)
        cv2.imshow("Video", cv_video2)
        cv2.waitKey(1)

    except CvBridgeError as e:
        print(e)
       
def weather_callback(weather_msg):
    rospy.loginfo("HAVA DURUMU BİLGİSİ: {}".format(weather_msg.data))
    
def listener_callback(point_msg):
    rospy.loginfo("ALINAN KONUM BİLGİSİ: x={}, y={}, z={}".format(point_msg.x, point_msg.y, point_msg.z))        

if __name__ == '__main__':
    rospy.init_node('map_navigation', anonymous=False)
    image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, callback)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    point_pub = rospy.Publisher('/cx_cy_point', Point, queue_size=1)
    weather_pub = rospy.Publisher('/weather_info', String, queue_size=1)
    rospy.Subscriber('/cx_cy_point', Point, listener_callback)
    rospy.Subscriber('/weather_info', String, weather_callback)
    
    rospy.spin()

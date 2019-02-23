#!/usr/bin/env python

import os
import cv2
import numpy as np

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image,PointCloud2
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pcl2


from visualization_msgs.msg import Marker

FRAME_ID='map'

def publish_camera(cam_pub,bridge,image):
    cam_pub.publish(bridge.cv2_to_imgmsg(image,"bgr8"))


def publish_point_cloud(pcl_pub,point_cloud):
    header=Header()
    header.stamp=rospy.Time.now()
    header.frame_id=FRAME_ID
    pcl_pub.publish(pcl2.create_cloud_xyz32(header,point_cloud[:,:3]))


def publish_ego_car(ego_car_pub):
    maker=Maker()
    maker.header.frame.id=FRAME_ID
    maker.stamp=rospy.Time.now()

    maker.id=0
    maker.action=Marker.ADD
    maker.lifttime = rospy.Duration()
    marker.type=Maker.LINE_STRIP

    marker.color.r=0.0
    marker.color.g=1.0
    marker.color.b=0.0
    marker.color.a=1.0
    marker.scale.x=0.2

    marker.points=[]
    marker.points.append(Point(10,-10,0))
    marker.points.append(Point(0,0,0))
    marker.points.append(Point(10,10,0))

    maker_array.makers.apped(marker)

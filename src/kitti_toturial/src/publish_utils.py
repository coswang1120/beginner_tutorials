#!/usr/bin/env python

import os
import cv2
import numpy as np

import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image,PointCloud2,Imu,NavSatFix
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pcl2
from geometry_msgs.msg import Point

import tf
from visualization_msgs.msg import Marker,MarkerArray

FRAME_ID='map'
LIFETIME=0.1

LINES = [[0,1],[1,2],[2,3],[3,0]]  #draw box left below corner is zero  clockwise below to top
LINES +=[[4,5],[5,6],[6,7],[7,4]]
LINES +=[[0,4],[1,5],[2,6],[3,7]] 
LINES +=[[4,1],[5,0]]

def publish_camera(cam_pub,bridge,image,boxes,types):
    for typ,box in zip(types,boxes):
        top_left=int(box[0]),int(box[1])
        bottom_right=int(box[2]),int(box[3])
        if np.array_equal(typ,['Car']):
            cv2.rectangle(image,top_left,bottom_right,(255,255,0),2)
        elif np.array_equal(typ,['Pedestrian']):
            cv2.rectangle(image,top_left,bottom_right,(0,226,0),2)
        else:
            cv2.rectangle(image,top_left,bottom_right,(141,255,0),2)
    cam_pub.publish(bridge.cv2_to_imgmsg(image,"bgr8"))


def publish_point_cloud(pcl_pub,point_cloud):
    header=Header()
    header.stamp=rospy.Time.now()
    header.frame_id=FRAME_ID
    pcl_pub.publish(pcl2.create_cloud_xyz32(header,point_cloud[:,:3]))


def publish_ego_car(ego_car_pub):

    marker_array=MarkerArray()

    marker=Marker()
    marker.header.frame_id=FRAME_ID
    marker.header.stamp=rospy.Time.now()

    marker.id=0
    marker.action=Marker.ADD
    marker.lifetime = rospy.Duration()
    marker.type=Marker.LINE_STRIP

    marker.color.r=0.0
    marker.color.g=1.0
    marker.color.b=0.0
    marker.color.a=1.0
    marker.scale.x=0.2

    marker.points=[]
    marker.points.append(Point(10,-10,0))
    marker.points.append(Point(0,0,0))
    marker.points.append(Point(10,10,0))

    marker_array.markers.append(marker)


    #3D car
    mesh_marker=Marker()
    mesh_marker.header.frame_id=FRAME_ID
    mesh_marker.header.stamp=rospy.Time.now()

    mesh_marker.id=-1
    mesh_marker.lifetime = rospy.Duration()
    mesh_marker.type=Marker.MESH_RESOURCE
    mesh_marker.mesh_resource="package://kitti_toturial/bmw_x5/BMW X5 4.dae"
    
    mesh_marker.pose.position.x=0.0
    mesh_marker.pose.position.y=0.0
    mesh_marker.pose.position.z=-1.73
    
    q=tf.transformations.quaternion_from_euler(np.pi/2,0,np.pi)
    mesh_marker.pose.orientation.x=q[0]
    mesh_marker.pose.orientation.y=q[1]
    mesh_marker.pose.orientation.z=q[2]
    mesh_marker.pose.orientation.w=q[3]
   
    mesh_marker.color.r=1.0
    mesh_marker.color.g=1.0
    mesh_marker.color.b=1.0
    mesh_marker.color.a=1.0

    mesh_marker.scale.x=0.9
    mesh_marker.scale.y=0.9
    mesh_marker.scale.z=0.9

    marker_array.markers.append(mesh_marker)
    ego_car_pub.publish(marker_array)

def publish_car_model(model_pub):
    mesh_marker=Marker()
    mesh_marker.header.frame_id=FRAME_ID
    mesh_marker.header.stamp=rospy.Time.now()

    mesh_marker.id=-1
    mesh_marker.lifetime = rospy.Duration()
    mesh_marker.type=Marker.MESH_RESOURCE
    mesh_marker.mesh_resource="package://kitti_toturial/bmw_x5/BMW X5 4.dae"
    
    mesh_marker.pose.position.x=0.0
    mesh_marker.pose.position.y=0.0
    mesh_marker.pose.position.z=-1.73
    
    q=tf.transformations.quaternion_from_euler(np.pi/2,0,np.pi)
    mesh_marker.pose.orientation.x=q[0]
    mesh_marker.pose.orientation.y=q[1]
    mesh_marker.pose.orientation.z=q[2]
    mesh_marker.pose.orientation.w=q[3]
   
    mesh_marker.color.r=1.0
    mesh_marker.color.g=1.0
    mesh_marker.color.b=1.0
    mesh_marker.color.a=1.0

    mesh_marker.scale.x=0.9
    mesh_marker.scale.y=0.9
    mesh_marker.scale.z=0.9
   

    model_pub.publish(mesh_marker) 


def publish_imu(imu_pub,imu_data):       
    imu=Imu()
    imu.header.frame_id=FRAME_ID
    imu.header.stamp=rospy.Time.now()
    
    q=tf.transformations.quaternion_from_euler(float(imu_data.roll),float(imu_data.pitch),float(imu_data.yaw))
    imu.orientation.x=q[0]
    imu.orientation.y=q[1]
    imu.orientation.z=q[2]
    imu.orientation.w=q[3]
    imu.linear_acceleration.x=imu_data.af
    imu.linear_acceleration.x=imu_data.al
    imu.linear_acceleration.x=imu_data.au
    imu.angular_velocity.x=imu_data.wf
    imu.angular_velocity.y=imu_data.wl
    imu.angular_velocity.z=imu_data.wu

    imu_pub.publish(imu)

def publish_gps(gps_pub,imu_data):       
    gps=NavSatFix()
    gps.header.frame_id=FRAME_ID
    gps.header.stamp=rospy.Time.now()
    
    gps.latitude=imu_data.lat
    gps.longitude=imu_data.lon
    gps.altitude=imu_data.alt

    gps_pub.publish(gps)  
def publish_3dbox(box3d_pub,corners_3d_velos,types,track_ids):
    marker_array=MarkerArray()  
    for i,corners_3d_velo in enumerate(corners_3d_velos):
        marker=Marker()
        marker.header.frame_id=FRAME_ID
        marker.header.stamp=rospy.Time.now()

        marker.id=i
        marker.action=Marker.ADD
        marker.lifetime=rospy.Duration(LIFETIME)
        marker.type=Marker.LINE_LIST

        if np.array_equal(types[i],['Car']):
            b=255
            g=255
            r=0   
        elif np.array_equal(types[i],['Pedestrian']):
            b=0
            g=226
            r=0           
        else:
            b=141
            g=255
            r=0 
            
        marker.color.r=r/255.0
        marker.color.g=g/255.0
        marker.color.b=b/255.0

        marker.color.a=1.0

        marker.scale.x=0.1

        marker.points=[]
       
        for l in LINES:
            
            p1=corners_3d_velo[l[0]]            
            marker.points.append(Point(p1[0],p1[1],p1[2]))
            p2=corners_3d_velo[l[1]]
            marker.points.append(Point(p2[0],p2[1],p2[2]))
        marker_array.markers.append(marker)
          
    
    text_marker=Marker()
    text_marker.header.frame_id=FRAME_ID
    text_marker.header.stamp=rospy.Time.now()

    text_marker.id=i+1000
    text_marker.action=Marker.ADD
    text_marker.lifetime=rospy.Duration(LIFETIME)
    text_marker.type=Marker.TEXT_VIEW_FACING

    p=np.mean(corners_3d_velo,axis=0)  
    text_marker.pose.position.x = p[0]
    text_marker.pose.position.y = p[1]
    text_marker.pose.position.z = p[2]+1

    text_marker.text=str(track_ids[i])

    text_marker.scale.x=1
    text_marker.scale.y=1
    text_marker.scale.z=1

    
    text_marker.scale.x=1
    text_marker.scale.y=1
    text_marker.scale.z=1
    
    if np.array_equal(types[i],['Car']):
        b=255
        g=255
        r=0   
    elif np.array_equal(types[i],['Pedestrian']):
        b=0
        g=226
        r=0           
    else:
        b=141
        g=255
        r=0 
            
    text_marker.color.r=r/255.0
    text_marker.color.g=g/255.0
    text_marker.color.b=b/255.0
    text_marker.color.a=1.0

    marker_array.markers.append(text_marker)
    box3d_pub.publish(marker_array) 
        
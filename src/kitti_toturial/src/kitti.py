#!/usr/bin/env python


from collections import deque
from data_utils import *
from publish_utils import *
from kitti_utils import *
from misc import *

EGOCAR=np.array([[2.15,0.9,-1.73],[2.15,-0.9,-1.73],[-1.95,-0.9,-1.73],[-1.95,0.9,-1.73],
                 [2.15,0.9,-0.23],[2.15,-0.9,-0.23],[-1.95,-0.9,-0.23],[-1.95,0.9,-0.23]])

DATA_PATH='/mnt/data/prog/Kitti/2011_09_26 /2011_09_26_drive_0005_sync/'

class Object():
    def __init__(self,center):
        self.locations=deque(maxlen=20)
        self.locations.appendleft(center)

    def update(self,center,displacement,yaw):
        for i in range(len(self.locations)):
            x0,y0=self.locations[i]
            x1=x0*np.cos(yaw_change)+y0*np.sin(yaw_change)-displacement
            y1=-x0*np.sin(yaw_change)+y0*np.cos(yaw_change)
            self.locations[i]=np.array([x1,y1])  
       
        if center is not None:
            self.locations.appendleft(center)  
    def reset(self):
        self.locations=deque(maxlen=20)


    

if __name__ == '__main__':
    frame = 0
    rospy.init_node('kitti_node',anonymous=True) 
    cam_pub=rospy.Publisher('kitti_cam',Image,queue_size=10)
    pcl_pub=rospy.Publisher('kitti_point_cloud',PointCloud2,queue_size=10)
    #ego_pub=rospy.Publisher('kitti_ego_car',Marker,queue_size=10)
    ego_pub=rospy.Publisher('kitti_ego_car',MarkerArray,queue_size=10)
    #model_pub=rospy.Publisher('kitti_car_model',Marker,queue_size=10)
    imu_pub=rospy.Publisher('kitti_imu',Imu,queue_size=10)
    gps_pub=rospy.Publisher('kitti_gps',NavSatFix,queue_size=10)
    box3d_pub=rospy.Publisher('kitti_3d',MarkerArray,queue_size=10)
    loc_pub=rospy.Publisher('kitti_loc',MarkerArray,queue_size=10)
    dist_pub=rospy.Publisher('kitti_dist',MarkerArray,queue_size=10)
    bridge=CvBridge()

    rate=rospy.Rate(10)

    df_tracking=read_tracking('/mnt/data/prog/Kitti/label_02/0000.txt')
    calib=Calibration('/mnt/data/prog/Kitti/2011_09_26 /2011_09_26_drive_0005_sync/',from_video=True)
    
    #ego_car=Object()
    tracker = {}
    pre_imu_data=None

    while not rospy.is_shutdown():
        df_tracking_frame=df_tracking[df_tracking.frame==frame]
        
        boxes_2d=np.array(df_tracking[df_tracking.frame==frame][['bbox_left','bbox_top','bbox_right','bbox_bottom']])
        types=np.array(df_tracking[df_tracking.frame==frame][['type']])
        boxes_3d=np.array(df_tracking_frame[['height','width','length','pos_x','pos_y','pos_z','rot_y']])
        track_ids=np.array(df_tracking_frame['track_id'])
        
        corners_3d_velos=[]
        centers={}
        minPQDs=[]
        for track_id,box_3d in zip(track_ids,boxes_3d):
            corners_3d_cam2= compute_3d_box_cam2(*box_3d)
            corners_3d_velo=calib.project_rect_to_velo(corners_3d_cam2.T)
            minPQDs += [min_distnace_cubolds(EGOCAR,corners_3d_velo)]
            corners_3d_velos += [corners_3d_velo]
            centers[track_id]=np.mean(corners_3d_velo,axis=0)[:2]
        
        corners_3d_velos+=[EGOCAR]
        types=np.append(types,'Car')
        track_ids=np.append(track_ids,-1)
        centers[-1]=np.array([0,0])    

        image=read_camera(os.path.join(DATA_PATH,'image_02/data/%010d.png'%frame))
        point_cloud=read_point_cloud(os.path.join(DATA_PATH,'velodyne_points/data/%010d.bin'%frame))
        imu_data=read_imu(os.path.join(DATA_PATH,'oxts/data/%010d.txt'%frame))
        
        if pre_imu_data is None:
            for track_id in centers:
                tracker[track_id] = Object(centers[track_id])
        else:
            displacement=0.1*np.linalg.norm(imu_data[['vf','vl']])
            yaw_change=float(imu_data.yaw-pre_imu_data.yaw)
            for track_id in centers:
                if track_id in tracker:
                    tracker[track_id].update(centers[track_id],displacement,yaw_change)           
                else:
                    tracker[track_id] = Object(centers[track_id])
            for track_id in tracker:
                if track_id not in centers:
                    tracker[track_id].update(None,displacement,yaw_change)                    




        pre_imu_data=imu_data
   
        publish_camera(cam_pub,bridge,image,boxes_2d,types)
        publish_point_cloud(pcl_pub,point_cloud[::2]) 
        publish_3dbox(box3d_pub,corners_3d_velos,types,track_ids)
        publish_ego_car(ego_pub)
        publish_imu(imu_pub,imu_data)
        publish_gps(gps_pub,imu_data)
        publish_loc(loc_pub,tracker,centers)
        publish_dist(dist_pub,minPQDs)

        rospy.loginfo("published")
        rate.sleep()
        frame+=1
        if frame==154:
            frame=0
            for track_id in tracker:
                tracker[track_id].reset()


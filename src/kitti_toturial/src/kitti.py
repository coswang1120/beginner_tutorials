#!/usr/bin/env python



from data_utils import *
from publish_utils import *
from kitti_utils import *

DATA_PATH='/mnt/data/prog/Kitti/2011_09_26 /2011_09_26_drive_0005_sync/'

class Object():
    def __init__(self):
        seld.locations=[]
        
    def update(self,displacement,yaw):
        for i in range(len(locations)):
            x0,y0=locations[i]
            x1=x0*np.cos(yaw_change)+y0*np.sin(yaw_change)-displacement
            y1=-x0*np.sin(yaw_change)+y0*np.cos(yaw_change)
            locations[i]=np.array([x1,y1])     

def compute_3d_box_cam2(h,w,l,x,y,z,yaw):
    R=np.array([[np.cos(yaw),0,np.sin(yaw)],[0,1,0],[-np.sin(yaw),0,np.cos(yaw)]])
    x_corners = [l/2, l/2, -l/2, -l/2, l/2, l/2, -l/2, -l/2]
    y_corners = [0, 0, 0, 0, -h, -h, -h, -h ]
    z_corners = [w/2, -w/2, -w/2, w/2, w/2, -w/2, -w/2, w/2 ]
    corners_3d_cam2=np.dot(R,np.vstack([x_corners,y_corners,z_corners]))
    corners_3d_cam2+=np.vstack([x,y,z])
    return corners_3d_cam2
    

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
    
    
    
    bridge=CvBridge()


    df_tracking=read_tracking('/mnt/data/prog/Kitti/label_02/0000.txt')
    calib=Calibration('/mnt/data/prog/Kitti/2011_09_26 /2011_09_26_drive_0005_sync/',from_video=True)
    
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        df_tracking_frame=df_tracking[df_tracking.frame==frame]
        
        boxes_2d=np.array(df_tracking[df_tracking.frame==frame][['bbox_left','bbox_top','bbox_right','bbox_bottom']])

        types=np.array(df_tracking[df_tracking.frame==frame][['type']])
        boxes_3d=np.array(df_tracking_frame[['height','width','length','pos_x','pos_y','pos_z','rot_y']])
        track_ids=np.array(df_tracking_frame['track_id'])
        corners_3d_velos=[]
        for box_3d in boxes_3d:
            corners_3d_cam2= compute_3d_box_cam2(*box_3d)
            corners_3d_velo=calib.project_rect_to_velo(corners_3d_cam2.T)
            corners_3d_velos+=[corners_3d_velo]
        
        image=read_camera(os.path.join(DATA_PATH,'image_02/data/%010d.png'%frame))
        publish_camera(cam_pub,bridge,image,boxes_2d,types)
        
        point_cloud=read_point_cloud(os.path.join(DATA_PATH,'velodyne_points/data/%010d.bin'%frame))
        publish_point_cloud(pcl_pub,point_cloud)
        publish_ego_car(ego_pub)
        #publish_car_model(model_pub)
        publish_3dbox(box3d_pub,corners_3d_velos,types,track_ids)
        imu_data=read_imu(os.path.join(DATA_PATH,'oxts/data/%010d.txt'%frame))
        publish_imu(imu_pub,imu_data)
        publish_gps(gps_pub,imu_data)
        rospy.loginfo("published")
        rate.sleep()
        frame+=1
        frame%=154


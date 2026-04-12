from email.header import Header

from geometry_msgs import PoseStamped,PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path,OccupancyGrid,Odometry
import numpy as np
from tf2_ros import TransformListener,Buffer
from collections import deque
from queue import Queue

class mapper(Node):
    def __init__(self):
        super().__init__('mapper_node')
        self.map_sub=self.create_subscription(OccupancyGrid,'/map',self.map_callback,10)
        self.scan_sub=self.create_subscription(PoseWithCovarianceStamped,'/scan',self.scan_callback,10)
        self.odom_sub=self.create_subscription(PoseWithCovarianceStamped,'/odom',self.odom_callback,10)
        self.tf_sub=TransformListener(Buffer(),self)

        self.map=None
        self.width=None
        self.height=None
        self.resolution=None
        self.origin=None
        scan_pose_x=[]
        scan_pose_y=[]
        for i in range():
            if i>0:
                scan_pose_x.append(scan_sub.position.x)
                scan_pose_y.append(scan_sub.position.y)


    

        def bresenham(x0,y0,x1,y1):
            for i in range(len(scan_pose_x)):
                x0=int(scan_pose_x[i]/self.resolution)
                y0=int(scan_pose_y[i]/self.resolution)
                x1=int(scan_pose_x[i+1]/self.resolution)
                y1=int(scan_pose_y[i+1]/self.resolution)
                dx=abs(x1-x0)
                dy=abs(y1-y0)
                sx=1 if x0<x1 else -1
                sy=1 if y0<y1 else -1
                err=dx-dy
                while True:
                    index=y0*self.width + x0
                    self.map[index]=100
                    if x0==x1 and y0==y1:
                        break
                    err2=err*2
                    if err2>-dy:
                        err-=dy
                        x0+=sx
                    if err2<dx:
                        err+=dx
                        y0+=sy

                


        
        def map_callback(self,msg):
            self.map=msg.data
            self.width=msg.info.width
            self.height=msg.info.height
            self.logger().info('Received map with width %d height %d'%(self.width,self.height))

        
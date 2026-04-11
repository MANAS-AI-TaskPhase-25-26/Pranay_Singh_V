from email.header import Header

from geometry_msgs import PoseStamped,PoseWithCovarianceStamped
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path,OccupancyGrid,Odometry
import numpy as np
from tf2_ros import TransformListener,Buffer
from collections import deque
from queue import Queue


class WFD(Node):
    def __init__(self):
        super().__init__('wavefront_frontier_detection')
        self.path_sub=self.create_subscription(Path,'/global_path',self.path_callback,10)
        self.cmd_pub=self.create_publisher(PoseStamped,'/cmd_pose',10)
        self.current_path=None
        self.current_index=0



# PoseWithCovarianceStamped:This expresses an estimated pose with a reference coordinate frame and timestamp
# Header header
# PoseWithCovariance pose
 

        self.scan_sub=self.create_subscription(PoseWithCovarianceStamped,'/scan',self.scan_callback,10)
        self.scan_pub = self.create_publisher(PoseWithCovarianceStamped,'/map',10)
        self.map_sub=self.create_subscription(OccupancyGrid,'/map',self.map_callback,10)

        self.odom_sub=self.create_subscription(PoseWithCovarianceStamped,'/odom',self.odom_callback,10)
        self.tf_sub=TransformListener(Buffer(),self)


        def path_callback(self,msg):
            self.current_path=msg.poses
            self.current_index=0
            self.get_logger().info('Received new path with %d poses'%len(self.current_path))

        def scan_callback(self,scan):
            if self.current_path is None:
                return
            if self.current_index>=len(self.current_path):
                self.get_logger().info('Reached end of path')
                return
            target_pose=self.current_path[self.current_index].pose
            cmd_pose=PoseStamped()
            cmd_pose.header=scan.header
            cmd_pose.pose=target_pose
            self.cmd_pub.publish(cmd_pose)
            self.get_logger().info('Published cmd pose to move towards target')

        def map_callback(self,msg):
            self.map=msg.data
            self.width=msg.info.width
            self.height=msg.info.height
            self.resolution=msg.info.resolution
            self.origin=msg.info.origin

            sef.get_logger().info('Received map with width %d height %d'%(self.width,self.height))

        tf_pose=self.tf_sub.buffer.lookup_transform('map','base_link',rclpy.time.Time(),timeout=rclpy.duration.Duration(seconds=1.0))
        

#Using queue data structure to implement breadth first search for local path planning as it uses a FIFO structure
q_map=Queue(maxsize=-1)#Fill in the queue with the current position in map coordinates
q_frontier=Queue(maxsize=q_map.qsize())#For extracting the next frontier to explore

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        self.pose_sub=self.create_subscription(PoseStamped,'/pose',self.pose_callback,10)
        self.current_pose=None

        def pose_callback(self,msg):
            self.current_pose=msg.pose
            q_map.append(msg.pose)
            self.get_logger().info('Added current pose: (%.2f,%.2f)'%(self.current_pose.position.x,self.current_pose.position.y))


        
        



import sys
import os

import rclpy 
from rclpy.node import Node
import sensor_msgs.msg as sensor_msgs

import lidar_pkg.Detection_Pipeline as pcb

import numpy as np
import open3d as o3d

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class PCDListener(Node):

    def __init__(self):
        super().__init__('pcd_subsriber_node')

        self.visEnable = False

        ## This is for visualization of the received point cloud.
        if self.visEnable:
            self.vis = o3d.visualization.Visualizer()
            self.vis.create_window()
            self.o3d_pcd = o3d.geometry.PointCloud()


        # Set up a subscription to the 'pcd' topic with a callback to the 
        # function `listener_callback`
        self.pcd_subscriber = self.create_subscription(
            sensor_msgs.PointCloud2,    # Msg type
            '/ch128x1/lslidar_point_cloud',                      # topic
            self.listener_callback,         # Function to call
            10                          # QoS
        )

        self.marker_pub = self.create_publisher(Marker, "/visualization_marker", 10)
        self.marker_array_pub = self.create_publisher(MarkerArray, "/visualization_marker_array", 10)
        self.y = -10.0
                
    def listener_callback(self, msg):
        # Here we convert the 'msg', which is of the type PointCloud2.
        # I ported the function read_points2 from 
        # the ROS1 package. 
        # https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/src/sensor_msgs/point_cloud2.py

        #data = list(read_points(msg))
        #print(len(data))
        pcd_as_numpy_array = np.array(list(read_points(msg)))
        #print (pcd_as_numpy_array.shape)
        v3d = o3d.utility.Vector3dVector(pcd_as_numpy_array[:,:3])
        self.o3d_pcd = o3d.geometry.PointCloud(v3d)

        visuals = pcb.detection_pipeline(self.o3d_pcd, debug=False)

        #self.y = self.y + 0.5
        #if self.y > 10:
        #    self.y = -10.0
        #self.publish_test_marker()
        markerArray = MarkerArray()
        markerId = 0
        for vi in visuals:
            #print (vi.max_bound - vi.min_bound)
            boxsize = (vi.max_bound - vi.min_bound)
            if (boxsize[0] < 20) and (boxsize[1] < 20) and (boxsize[2] < 20):
                markerArray.markers.append(self.buildMarker(id=markerId, scale=boxsize, position=vi.get_center()))
                markerId = markerId + 1
        self.marker_array_pub.publish(markerArray)
        

        if self.visEnable:
            for vi in visuals:
                self.vis.add_geometry(vi)
                self.vis.update_geometry(vi)

            self.vis.update_renderer()
            self.vis.poll_events()
            self.vis.get_view_control().rotate(0.0, 20.0)

            # viz.capture_screen_image("temp_%04d.png" % idx)
            for vi in visuals:
                self.vis.remove_geometry(vi)

            # The rest here is for visualization.
            #self.vis.remove_geometry(self.o3d_pcd)
            #self.o3d_pcd = o3d.geometry.PointCloud(v3d)
            #self.vis.add_geometry(self.o3d_pcd)
            #self.vis.poll_events()
            #self.vis.update_renderer()
    
    def buildMarker(self, id, scale, position):
        marker = Marker()

        marker.header.frame_id = "laser_link"
        #marker.header.stamp = self.get_clock().now()  #rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 1
        marker.id = id

        # Set the scale of the marker
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]

        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5

        # Set the pose of the marker
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        return marker

    def publish_test_marker(self):
        marker = Marker()

        marker.header.frame_id = "laser_link"
        #marker.header.stamp = self.get_clock().now()  #rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 1
        marker.id = 0

        # Set the scale of the marker
        marker.scale.x = 2.0
        marker.scale.y = 4.0
        marker.scale.z = 2.0

        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5

        # Set the pose of the marker
        marker.pose.position.x = 0.0
        marker.pose.position.y = self.y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        self.marker_pub.publish(marker)


## The code below is "ported" from 
# https://github.com/ros/common_msgs/tree/noetic-devel/sensor_msgs/src/sensor_msgs
# I'll make an official port and PR to this repo later: 
# https://github.com/ros2/common_interfaces
import sys
from collections import namedtuple
import ctypes
import math
import struct
from sensor_msgs.msg import PointCloud2, PointField

_DATATYPES = {}
_DATATYPES[PointField.INT8]    = ('b', 1)
_DATATYPES[PointField.UINT8]   = ('B', 1)
_DATATYPES[PointField.INT16]   = ('h', 2)
_DATATYPES[PointField.UINT16]  = ('H', 2)
_DATATYPES[PointField.INT32]   = ('i', 4)
_DATATYPES[PointField.UINT32]  = ('I', 4)
_DATATYPES[PointField.FLOAT32] = ('f', 4)
_DATATYPES[PointField.FLOAT64] = ('d', 8)

def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a L{sensor_msgs.PointCloud2} message.

    @param cloud: The point cloud to read from.
    @type  cloud: L{sensor_msgs.PointCloud2}
    @param field_names: The names of fields to read. If None, read all fields. [default: None]
    @type  field_names: iterable
    @param skip_nans: If True, then don't return any point with a NaN value.
    @type  skip_nans: bool [default: False]
    @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    @type  uvs: iterable
    @return: Generator which yields a list of values for each point.
    @rtype:  generator
    """
    assert isinstance(cloud, PointCloud2), 'cloud is not a sensor_msgs.msg.PointCloud2'
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, math.isnan
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset)
                    offset += point_step

def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print('Skipping unknown PointField datatype [%d]' % field.datatype, file=sys.stderr)
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt    += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt



def main(args=None):
    # Boilerplate code.
    rclpy.init(args=args)
    pcd_listener = PCDListener()
    rclpy.spin(pcd_listener)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pcd_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

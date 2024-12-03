#!/usr/bin/env python3
import numpy as np
import rospy 
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import yaml
import os
import open3d as o3d

class TopicCompressor:
    def __init__(self):
        rospy.init_node('topic_compressor')
        
        self.status_pub = rospy.Publisher('/topic_compressor/status', Float32MultiArray, queue_size=1)
        self.load_config()
        
        self.subscribers = {}
        self.publishers = {}
        self.topic_stats = {} 
        
        self.setup_topics()
        
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_status)

    def load_config(self):
        config_file = rospy.get_param('~config_file', '')
        if not config_file:
            rospy.logerr("Config file path not provided!")
            return
            
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)['topic_compressor']['topics']

    def setup_topics(self):
        for topic in self.config:
            topic_name = topic['name']
            from_topic = topic['from']
            to_topic = topic['to']
            compression_ratio = topic['compression_ratio']
            
            self.topic_stats[topic_name] = {
                'original_size': 0,
                'compressed_size': 0,
                'compression_ratio': compression_ratio,
                'from_topic': from_topic,
                'to_topic': to_topic
            }
            
            self.publishers[topic_name] = rospy.Publisher(
                to_topic, 
                PointCloud2, 
                queue_size=1
            )
            
            self.subscribers[topic_name] = rospy.Subscriber(
                from_topic,
                PointCloud2,
                self.compress_callback,
                callback_args=topic_name
            )
            
            rospy.loginfo(f"Set up compression for {topic_name}: {from_topic} -> {to_topic}")

    def compress_callback(self, msg, topic_name):
        try:
            # rospy.loginfo(f"Starting compression for {topic_name}")
            self.topic_stats[topic_name]['original_size'] = len(msg.data)
            
            points = pc2.read_points(msg, skip_nans=True)
            points_array = np.array(list(points))
            
            # rospy.loginfo(f"Original points shape: {points_array.shape}")
            # rospy.loginfo(f"Fields: {msg.fields}")
            
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points_array[:, :3])
            
            voxel_size = float(self.topic_stats[topic_name]['compression_ratio'])
            
            downsampled_pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
            
            compressed_points = np.asarray(downsampled_pcd.points)
            
            field_types = [(field.name, field.datatype) for field in msg.fields]
            
            if points_array.shape[1] > 3:
                from scipy.spatial import cKDTree
                tree = cKDTree(points_array[:, :3])
                _, indices = tree.query(compressed_points, k=1)
                additional_fields = points_array[indices, 3:]
                
                additional_fields[:, 1] = additional_fields[:, 1].astype(np.int32)
                
                compressed_points = np.hstack((compressed_points, additional_fields))
            
            # rospy.loginfo(f"Compressed points shape: {compressed_points.shape}")
            
            point_list = []
            for point in compressed_points:
                point_list.append([
                    float(point[0]),  # x
                    float(point[1]),  # y
                    float(point[2]),  # z
                    float(point[3]),  # intensity
                    int(point[4])     # ring
                ])
            
            compressed_msg = pc2.create_cloud(
                msg.header,
                msg.fields,
                point_list
            )
            
            self.topic_stats[topic_name]['compressed_size'] = len(compressed_msg.data)
            self.publishers[topic_name].publish(compressed_msg)
            
        except Exception as e:
            rospy.logerr(f"Error compressing {topic_name}: {str(e)}")
            rospy.logerr(f"Exception type: {type(e)}")
            import traceback
            rospy.logerr(f"Traceback: {traceback.format_exc()}")

    def publish_status(self, event):
        status_msg = Float32MultiArray()
        
        data = []
        layout = []
        
        for topic_name, stats in self.topic_stats.items():
            data.extend([
                float(stats['original_size']),
                float(stats['compressed_size'])
            ])
            
            dim = MultiArrayDimension()
            dim.label = topic_name
            dim.size = 2
            dim.stride = 2
            layout.append(dim)
        
        status_msg.data = data
        status_msg.layout.dim = layout
        
        self.status_pub.publish(status_msg)

if __name__ == '__main__':
    try:
        compressor = TopicCompressor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
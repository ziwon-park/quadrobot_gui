#!/usr/bin/env python3
import numpy as np
import rospy 
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import yaml
import os

class TopicCompressor:
    def __init__(self):
        rospy.init_node('topic_compressor')
        
        self.status_pub = rospy.Publisher('/topic_compressor/status', Float32MultiArray, queue_size=1)
        self.load_config()
        
        # 토픽별 구독자와 발행자 생성
        self.subscribers = {}
        self.publishers = {}
        self.topic_stats = {}  # 토픽별 통계 저장
        
        self.setup_topics()
        
        # 상태 발행 타이머
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
            self.topic_stats[topic_name]['original_size'] = len(msg.data)
            
            points = pc2.read_points(msg, skip_nans=True)
            points_array = np.array(list(points))
            compression_ratio = self.topic_stats[topic_name]['compression_ratio']
            compressed_points = points_array[::compression_ratio]
            
            compressed_msg = pc2.create_cloud(
                msg.header,
                msg.fields,
                compressed_points.tolist()
            )
            
            self.topic_stats[topic_name]['compressed_size'] = len(compressed_msg.data)
            
            self.publishers[topic_name].publish(compressed_msg)
            
        except Exception as e:
            rospy.logerr(f"Error compressing {topic_name}: {str(e)}")

    def publish_status(self, event):
        status_msg = Float32MultiArray()
        
        # 데이터 구성 [토픽1_원본크기, 토픽1_압축크기, 토픽2_원본크기, 토픽2_압축크기, ...]
        data = []
        layout = []
        
        for topic_name, stats in self.topic_stats.items():
            data.extend([
                float(stats['original_size']),
                float(stats['compressed_size'])
            ])
            
            # 레이아웃 정보 추가
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
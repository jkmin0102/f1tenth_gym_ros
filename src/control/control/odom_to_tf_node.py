#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')

        self.declare_parameter('odom_topic', '/ego_racecar/odom')
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'ego_racecar/base_link')

        self.odom_topic = self.get_parameter('odom_topic').value
        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value

        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, self.odom_topic, self.cb, 10)

        self.get_logger().info(f'Broadcasting TF {self.parent_frame} -> {self.child_frame} from {self.odom_topic}')

    def cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        # odom 메시지의 frame_id가 map이든 뭐든, 우리는 RViz fixed frame(map)에 맞춰 parent_frame을 사용
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = OdomToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

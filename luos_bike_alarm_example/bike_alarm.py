

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from rclpy.timer import Clock

class BikeExampleNode(Node):
    TOPIC_NAME = "/bike_alarm/markers"
    def __init__(self):
        super().__init__('bike_alarm')
        self.publisher_ = self.create_publisher(Marker, self.TOPIC_NAME, 10)
        self.timer = self.create_timer(1, self._cb_publish_markers)
        self.clock = Clock()

    def _cb_publish_markers(self):
        bike = Marker()
        bike.action = Marker.ADD
        bike.text = "Luos bike"
        bike.type = Marker.MESH_RESOURCE
        #bike.header.stamp = self.clock.now()
        bike.header.frame_id = "/world"
        # FIXME: from ament_index_python import get_resources
        # TODO: How does this resource-thing now work?
        bike.mesh_resource = "file:///home/yoan/ros2_ws/src/luos_bike_alarm_example/assets/bike.stl"
        bike.scale.x = 1e-3
        bike.scale.y = 1e-3
        bike.scale.z = 1e-3
        bike.color.r = 1.
        bike.color.g = 1.
        bike.color.b = 1.
        bike.color.a = 1.
        bike.pose.position.z = 3.
        bike.pose.orientation.w = 1.
        self.publisher_.publish(bike)
        self.get_logger().info("Publishing bike")

def main():
    rclpy.init()
    rclpy.spin(BikeExampleNode())

if __name__ == '__main__': main()
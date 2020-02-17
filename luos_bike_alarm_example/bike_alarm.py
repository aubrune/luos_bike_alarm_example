

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from visualization_msgs.msg import Marker
from rclpy.timer import Clock
from os.path import join
from ament_index_python.packages import get_package_prefix
from ament_index_python.resources import RESOURCE_INDEX_SUBFOLDER

class BikeExampleNode(Node):
    TOPIC_NAME_MARKERS = "/bike_alarm/markers"
    TOPIC_NAME_LUOS_IMU = "/gps/imu"
    def __init__(self):
        super().__init__('bike_alarm')
        self.publisher_ = self.create_publisher(Marker, self.TOPIC_NAME_MARKERS, 10)
        self.subscriber = self.create_subscription(Imu, self.TOPIC_NAME_LUOS_IMU, self._cb_imu_received, 10)
        self.timer = self.create_timer(0.1, self._cb_publish_markers)
        self.clock = Clock()

        # Here is the bike to be published in RViz
        self.bike = Marker()
        self.bike.text = "Luos bike"
        self.bike.type = Marker.MESH_RESOURCE
        #self.bike.header.stamp = self.clock.now()
        self.bike.header.frame_id = "/world"
        self.bike.mesh_resource = "file://" + join(get_package_prefix('luos_bike_alarm_example'),
                                       RESOURCE_INDEX_SUBFOLDER, "packages", 'bike.stl')
        self.bike.scale.x = 1e-3
        self.bike.scale.y = 1e-3
        self.bike.scale.z = 1e-3
        self.bike.color.r = 1.
        self.bike.color.g = 1.
        self.bike.color.b = 1.
        self.bike.color.a = 1.
        self.bike.pose.position.z = 3.
        self.bike.pose.orientation.w = 1.

    def _cb_imu_received(self, imu):
        # This callback is called when an Imu message is received on the specified topic 
        self.bike.pose.orientation = imu.orientation

    def _cb_publish_markers(self):
        # This publisher is called at a regular rate
        self.publisher_.publish(self.bike)
        self.get_logger().info("Publishing bike")

def main():
    rclpy.init()
    rclpy.spin(BikeExampleNode())

if __name__ == '__main__': main()
import rclpy
from rclpy.node import Node
from time import time
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import Imu
from luos_msgs.msg import State
from visualization_msgs.msg import Marker
from os.path import join
from ament_index_python.packages import get_package_prefix
from ament_index_python.resources import RESOURCE_INDEX_SUBFOLDER

TOPIC_NAME_LUOS_IMU = "/Imu_mod/imu"
TOPIC_NAME_LUOS_STATE = "/button_mod/events/changed"
TOPIC_NAME_LUOS_COLOR = "/rgb_led_mod/variables/color/write"
TOPIC_NAME_MARKERS = "/bike_alarm/markers"

class BikeColor:
    # Represent the color of the bike marker in RViz according to its riding state
    GREEN = [0.2, 1., 0.2]
    RED = [1., 0.2, 0.2]
    WHITE = [1., 1., 1.]
    BLINK_FAST = 0.4
    BLINK_SLOW = 1.
    BLINK_NONE = float('inf')

    def __init__(self):
        self._color = self.GREEN
        self._update_time = time()
        self._on = True
        self._blink = self.BLINK_NONE

    @property
    def color(self):
        now = time()
        if self._blink == self.BLINK_NONE:
            self._on = True
        elif now - self._update_time > self._blink:
            self._on = not self._on
            self._update_time = now
        return self._color if self._on else self.WHITE

    def set_color(self, color="white", blink="none"):
        self._color = getattr(self, color.upper())
        self._blink = getattr(self, "BLINK_" + blink.upper())
        self._update_time = 0


class BikeExampleNode(Node):
    # The actual node of the bike riding application
    def __init__(self):
        super().__init__('bike_alarm')
        self.publisher_ = self.create_publisher(Marker, TOPIC_NAME_MARKERS, 10)
        self.subscriber_imu = self.create_subscription(Imu, TOPIC_NAME_LUOS_IMU, self._cb_imu_received, 10)
        self.subscriber_button = self.create_subscription(State, TOPIC_NAME_LUOS_STATE, self._cb_state_received, 10)
        self.timer_bike = self.create_timer(0.1, self._cb_timer_bike)
        self.bike_color = BikeColor()
        self.state = "idle"   # The bike current state : ["idle", "ride", "alarm"]

        # Here is the bike to be published in RViz
        self.bike = Marker()
        self.bike.type = Marker.MESH_RESOURCE
        self.bike.header.frame_id = "/world"
        self.bike.mesh_resource = "file://" + join(get_package_prefix('luos_bike_alarm_example'),
                                       RESOURCE_INDEX_SUBFOLDER, "packages", 'bike.stl')
        self.bike.scale.x = 1e-3
        self.bike.scale.y = 1e-3
        self.bike.scale.z = 1e-3
        self.bike.color.a = 1.
        self.bike.pose.position.z = 3.
        self.bike.pose.orientation.w = 1.

        self.publisher_color = self.create_publisher(ColorRGBA, TOPIC_NAME_LUOS_COLOR, 10)
        self.get_logger().info("This is the Luos bike sharing example. \
Shake the IMU {} or press the State button {}".format(TOPIC_NAME_LUOS_IMU, TOPIC_NAME_LUOS_STATE))

    def _cb_imu_received(self, msg):
        # This callback is called when an Imu message is received on the specified topic 
        self.bike.pose.orientation = msg.orientation
        acc = msg.linear_acceleration
        if abs(acc.x) + abs(acc.y) + abs(acc.z) > 20 and self.state == "idle":
            self.get_logger().error("Alarm triggered!")
            self.state = "alarm"
            self.bike_color.set_color("red", "fast")

    def _cb_state_received(self, msg):
        # This callback is called when the state button is pressed
        if not msg.old_value and msg.new_value:
            if self.state in ["ride", "alarm"]:
                self.get_logger().info("Bike is idle")
                self.state = "idle"
                self.bike_color.set_color("green", "none")
            else:
                self.get_logger().warn("Riding...")
                self.state = "ride"
                self.bike_color.set_color("green", "slow")

    def _cb_timer_bike(self):
        # This publisher is called at a regular rate
        colour = self.bike_color.color
        # Invert full black and full white for the Luos LED:
        if colour == self.bike_color.WHITE:
            luos_color = ColorRGBA() 
        else:
            luos_color = ColorRGBA(r=colour[0]*64, g=colour[1]*64, b=colour[2]*64)
        self.publisher_color.publish(luos_color)   # Publish colour to the Luos color module
        self.bike.color.r, self.bike.color.g, self.bike.color.b = colour
        self.publisher_.publish(self.bike)         # Publish colour to the RViz marker

def main():
    rclpy.init()
    rclpy.spin(BikeExampleNode())

if __name__ == '__main__': main()
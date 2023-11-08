# Python
import time
import rclpy
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from rclpy.node import Node
from mavros_msgs.srv import WaypointPush
from mavros_msgs.msg import Waypoint
from mavros_msgs.msg import WaypointList



def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('mavros_service_caller')

    set_mode_client = node.create_client(SetMode, '/mavros/set_mode')
    arming_client = node.create_client(CommandBool, '/mavros/cmd/arming')
    takeoff_client = node.create_client(CommandTOL, '/mavros/cmd/takeoff')
    land_client = node.create_client(CommandTOL, '/mavros/cmd/land')

    set_mode_request = SetMode.Request(base_mode=0, custom_mode='4')
    arming_request = CommandBool.Request(value=True)
    takeoff_request = CommandTOL.Request(altitude=5.0)
    land_request = CommandTOL.Request()

    future_set_mode = set_mode_client.call_async(set_mode_request)
    time.sleep(5)
    future_arming = arming_client.call_async(arming_request)
    time.sleep(5)
    future_takeoff = takeoff_client.call_async(takeoff_request)
    time.sleep(10)

    # control the drone fly 1m/s in x axis and 0m/s in y axis and 0m/s in z axis for 1 seconds, use setpoint_velocity
    
    time.sleep(15)

    future_land = land_client.call_async(land_request)
    time.sleep(5)

    rclpy.spin_until_future_complete(node, future_set_mode)
    print(future_set_mode.result())

    rclpy.spin_until_future_complete(node, future_arming)
    print(future_arming.result())

    rclpy.spin_until_future_complete(node, future_takeoff)
    print(future_takeoff.result())

    rclpy.spin_until_future_complete(node, future_land)
    print(future_land.result())

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
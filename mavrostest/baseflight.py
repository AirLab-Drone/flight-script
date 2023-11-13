# Python
import threading
import time
import rclpy
from mavros_msgs.srv import SetMode, CommandBool, CommandTOL
from rclpy.node import Node
from mavros_msgs.srv import WaypointPush
from mavros_msgs.msg import Waypoint
from mavros_msgs.msg import WaypointList, PositionTarget
import flight_control


def main():
    rclpy.init()
    flightController = flight_control.FlightControl()
    time.sleep(5)
    isTakeoffSuccess = False
    while(isTakeoffSuccess == False):
        isTakeoffSuccess = flightController.armAndTakeoff()
    threading.Thread(target=flightController.initTimer).start()
    flightController.setVelocity(-0.5, -0.5, 0.0)
    print(f'Velocity: {flightController.msg.velocity}')
    time.sleep(10)
    flightController.setZeroVelocity()
    flightController.land()
    flightController.destroy()


if __name__ == '__main__':
    main()